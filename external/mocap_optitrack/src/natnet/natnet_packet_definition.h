/* 
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
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
#ifndef __MOCAP_OPTITRACK_NATNET_PACKET_DEFINITION_H__
#define __MOCAP_OPTITRACK_NATNET_PACKET_DEFINITION_H__

#define NATNET_MAX_NAMELENGTH   256
#define NATNET_MAX_PACKETSIZE   100000  // max size of packet (actual packet size is dynamic)

namespace natnet
{
  struct MessageType
  {
    static const int Connect;
    static const int ServerInfo;
    static const int Request;
    static const int Response;
    static const int RequestModelDef;
    static const int ModelDef;
    static const int RequestFrameOfData;
    static const int FrameOfData;
    static const int MessageString;
    static const int UnrecognizedRequest;
    static const int Undefined;
  };

  // Sender definition
  typedef struct 
  {
    char name[NATNET_MAX_NAMELENGTH];            // sending app's name
    unsigned char version[4];             // sending app's version [major.minor.build.revision]
    unsigned char natNetVersion[4];       // sending app's NatNet version [major.minor.build.revision]
  } __attribute__ ((__packed__)) Sender;
  
  // Packet definition
  typedef struct
  {
    unsigned short messageId;               // message ID (e.g. NAT_FRAMEOFDATA)
    unsigned short numDataBytes;            // Num bytes in payload
    union
    {
      unsigned char  cData[NATNET_MAX_PACKETSIZE];
      char           szData[NATNET_MAX_PACKETSIZE];
      unsigned long  lData[NATNET_MAX_PACKETSIZE/4];
      float          fData[NATNET_MAX_PACKETSIZE/4];
      Sender         sender;
    } data;                                 // Payload incoming from NatNet Server
  } __attribute__ ((__packed__)) Packet;
}

#endif