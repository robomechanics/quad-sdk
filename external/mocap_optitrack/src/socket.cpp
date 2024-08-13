/* 
 * Copyright (c) 2011 University of Bonn, Computer Science Institute, 
 * Kathrin Gräve
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "mocap_optitrack/socket.h"
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <rclcpp/logging.hpp>

UdpMulticastSocket::UdpMulticastSocket(
  rclcpp::Node::SharedPtr &node, 
  const int local_port, 
  const std::string multicast_ip ) : node(node)
{
  remote_ip_exist = false;

  // Create a UDP socket
  RCLCPP_INFO(node->get_logger(), "Creating socket..." );
  m_socket = socket( AF_INET, SOCK_DGRAM, 0 );
  if( m_socket < 0 )
    throw SocketException( strerror( errno ) );

  // Allow reuse of local addresses
  RCLCPP_INFO(node->get_logger(), "Setting socket options..." );
  int option_value = 1;
  int result = setsockopt( m_socket, SOL_SOCKET, SO_REUSEADDR, (void*)&option_value, sizeof( int ) );
  if( result == -1 )
  {
    std::stringstream error;
    error << "Failed to set socket option: ";
    switch( errno )
    {
      case EBADF:
        error << "EBADF";
        break;
      case EFAULT:
        error << "EFAULT";
        break;
      case EINVAL:
        error << "EINVAL";
        break;
      case ENOPROTOOPT:
        error << "ENOPROTOOPT";
        break;
      case ENOTSOCK:
        error << "ENOTSOCK";
        break;
      default:
        error << "unknown error";
        break;
    }
    throw SocketException( error.str().c_str() );    
  }
  
  // Fill struct for local address
  memset ( &m_local_addr, 0, sizeof ( m_local_addr ) );
  m_local_addr.sin_family = AF_INET;
  m_local_addr.sin_addr.s_addr = htonl( INADDR_ANY );
  m_local_addr.sin_port = htons( local_port );
  RCLCPP_INFO(node->get_logger(), "Local address: %s:%i", inet_ntoa( m_local_addr.sin_addr ), ntohs( m_local_addr.sin_port ) );

  // Bind the socket
  RCLCPP_INFO(node->get_logger(), "Binding socket to local address..." );
  result = bind( m_socket, (sockaddr*)&m_local_addr, sizeof( m_local_addr ) );
  if( result == -1 )
  {
    std::stringstream error;
    error << "Failed to bind socket to local address:" << strerror( errno );
    throw SocketException( error.str().c_str() );
  }
  
  // Join multicast group
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr( multicast_ip.c_str() );
  mreq.imr_interface = m_local_addr.sin_addr;
  RCLCPP_INFO(node->get_logger(), "Joining multicast group %s...", inet_ntoa( mreq.imr_multiaddr ) );

  result = setsockopt(m_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq));
  if( result == -1 )
  {
    std::stringstream error;
    error << "Failed to set socket option: ";
    switch( errno )
    {
      case EBADF:
        error << "EBADF";
        break;
      case EFAULT:
        error << "EFAULT";
        break;
      case EINVAL:
        error << "EINVAL";
        break;
      case ENOPROTOOPT:
        error << "ENOPROTOOPT";
        break;
      case ENOTSOCK:
        error << "ENOTSOCK";
        break;
      default:
        error << "unknown error";
        break;
    }
    throw SocketException( error.str().c_str() );    
  }
    
  // Make socket non-blocking
  RCLCPP_INFO(node->get_logger(), "Enabling non-blocking I/O" );
  int flags = fcntl( m_socket, F_GETFL , 0 );
  result = fcntl(m_socket, F_SETFL, flags | O_NONBLOCK);
  if( result == -1 )
  {
    std::stringstream error;
    error << "Failed to enable non-blocking I/O: " << strerror( errno );
    throw SocketException( error.str().c_str() );
  }
}

UdpMulticastSocket::~UdpMulticastSocket()
{
  close( m_socket );
}

int UdpMulticastSocket::recv()
{
  memset ( buf, 0, MAXRECV + 1 );

  sockaddr_in remote_addr;
  int addr_len = sizeof(struct sockaddr);
  int status = recvfrom(
    m_socket,
    buf,
    MAXRECV,
    0,
    (sockaddr *)&remote_addr,
    (socklen_t*)&addr_len);

  if( status > 0 ) {
    RCLCPP_DEBUG(node->get_logger(), "%4i bytes received from %s:%i", status, inet_ntoa( remote_addr.sin_addr ), ntohs( remote_addr.sin_port ) );
  }
  else if( status == 0 ) {
    RCLCPP_DEBUG(node->get_logger(), "Connection closed by peer" );
  }

  HostAddr.sin_addr =remote_addr.sin_addr;
  remote_ip_exist = true;

  return status;
}

int UdpMulticastSocket::send(const char* buf, unsigned int sz, int port)
{
  HostAddr.sin_family = AF_INET;
  HostAddr.sin_port = htons(port);
  return sendto(m_socket, buf, sz, 0, (sockaddr*)&HostAddr, sizeof(HostAddr));
}
