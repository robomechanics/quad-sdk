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
#ifndef __MOCAP_OPTITRACK_SOCKET_H__
#define __MOCAP_OPTITRACK_SOCKET_H__

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include <stdexcept>

/// \brief Exception class thrown by socket classes in this file.
class SocketException : public std::runtime_error
{
  public:
   
    /// \brief Constructor
    /// \param description Error message
    SocketException ( std::string description ) : std::runtime_error( description ) {}
    
    ~SocketException () throw() {}
};

/// \brief Allows to retrieve data from a UDP multicast group
class UdpMulticastSocket
{
  public:
    
    /// \brief Maximum number of bytse that can be read at a time
    static const int MAXRECV = 3000;

    /// Creates a socket and joins the multicast group with the given address
    UdpMulticastSocket( const int local_port, const std::string multicast_ip = "224.0.0.1" );
    
    ///
    ~UdpMulticastSocket();
    
    /// \brief Retrieve data from multicast group.
    /// \return The number of bytes received or -1 if no data is available
    ///
    /// This call is non-blocking.
    int recv();

    /// \brief Returns a pointer to the internal buffer, holding the received data.
    ///
    /// The buffer size may be obtained from MAXRECV.
    const char* getBuffer() { return &buf[0]; }

    int send(const char* buf, unsigned int sz, int port);

  private:

    int m_socket;
    sockaddr_in m_local_addr;
    sockaddr_in HostAddr;
    bool remote_ip_exist;
    char buf [ MAXRECV + 1 ];
};

#endif /*__MOCAP_OPTITRACK_SOCKET_H__*/
