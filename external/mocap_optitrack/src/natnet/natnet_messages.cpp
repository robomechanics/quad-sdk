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
#include "natnet_messages.h"

#include <cstring>
#include <cinttypes>

#include <rclcpp/logging.hpp>
#include "natnet_packet_definition.h"

namespace natnet
{

namespace utilities
{
  void seek(MessageBuffer::const_iterator& iter, size_t offset)
  {
    iter += offset;
  }

  template <typename T> 
  void read_and_seek(MessageBuffer::const_iterator& iter, T& target)
  {
    // Breaking up the steps for clarity.
    //    *iter <- points to current value (const char) in message buffer
    //    &(*iter) <- return address of this const char (char const*)
    //    (T const*) pData <- casts char const* into T const*
    //    *((T const*) pData) <- dereference typed pointer and copy it into target
    char const* pData = &(*iter);
    target = *((T const*) pData);
    // ROS_DEBUG("\t sizeof(%s) = %d", TypeParseTraits<T>::name, (int)sizeof(T));
    seek(iter, sizeof(T));
  }

  void decode_marker_id(int sourceID, int& pOutEntityID, int& pOutMemberID)
  {
      if (pOutEntityID)
          pOutEntityID = sourceID >> 16;

      if (pOutMemberID)
          pOutMemberID = sourceID & 0x0000ffff;
  }

  // Funtion that assigns a time code values to 5 variables passed as arguments
  // Requires an integer from the packet as the timecode and timecodeSubframe
  void decode_timecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int& hour, int& minute, int& second, int& frame, int& subframe)
  {
    hour = (inTimecode>>24)&255;
    minute = (inTimecode>>16)&255;
    second = (inTimecode>>8)&255;
    frame = inTimecode&255;
    subframe = inTimecodeSubframe;
  }

  // Takes timecode and assigns it to a string
  void stringify_timecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize)
  {
    bool bValid;
    int hour, minute, second, frame, subframe;
    decode_timecode(inTimecode, inTimecodeSubframe, hour, minute, second, frame, subframe);

    snprintf(Buffer,BufferSize,"%2d:%2d:%2d:%2d.%d",hour, minute, second, frame, subframe);
    for(unsigned int i=0; i < strlen(Buffer); i++)
    {
      if(Buffer[i]==' ')
      {
        Buffer[i]='0';
      }
    }
  }

} // namespace utilities


void ConnectionRequestMessage::serialize(
  MessageBuffer& msgBuffer, 
  mocap_optitrack::DataModel const*)
{
  natnet::Packet pkt;
  pkt.messageId = natnet::MessageType::Connect;
  pkt.numDataBytes = 0;

  // Resize the message buffer and copy contents
  msgBuffer.resize(4); // 2 bytes for messageId and 2 for numDataBtyes
  char *pBuffer = &msgBuffer[0];
  memcpy(pBuffer, &pkt, msgBuffer.size());
}


void ServerInfoMessage::deserialize(
  MessageBuffer const& msgBuffer, 
  mocap_optitrack::DataModel* dataModel)
{
  char const* pBuffer = &msgBuffer[0];
  natnet::Packet const* packet = (natnet::Packet const*)pBuffer;

  int nver[4] = {0, 0, 0, 0};
  int sver[4] = {0, 0, 0, 0};
  for(int i=0;i<4;++i) {
    nver[i] = (int)(packet->data.sender.natNetVersion[i]);
    sver[i] = (int)(packet->data.sender.version[i]);
  }

  dataModel->setVersions(nver, sver);
}


void DataFrameMessage::RigidBodyMessagePart::deserialize(
  rclcpp::Logger logger,
  MessageBuffer::const_iterator& msgBufferIter, 
  mocap_optitrack::RigidBody& rigidBody,
  mocap_optitrack::Version const& natNetVersion)
{
  // Read id, position and orientation of each rigid body
  utilities::read_and_seek(msgBufferIter, rigidBody.bodyId);
  utilities::read_and_seek(msgBufferIter, rigidBody.pose);

  RCLCPP_DEBUG(logger, "  Rigid body ID: %d", rigidBody.bodyId);
  RCLCPP_DEBUG(logger, "    Pos: [%3.2f,%3.2f,%3.2f], Ori: [%3.2f,%3.2f,%3.2f,%3.2f]",
           rigidBody.pose.position.x,
           rigidBody.pose.position.y,
           rigidBody.pose.position.z,
           rigidBody.pose.orientation.x,
           rigidBody.pose.orientation.y,
           rigidBody.pose.orientation.z,
           rigidBody.pose.orientation.w);

  // NatNet version 2.0 and later
  if (natNetVersion >= mocap_optitrack::Version("2.0"))
  {
    // Mean marker error
    utilities::read_and_seek(msgBufferIter, rigidBody.meanMarkerError);
    RCLCPP_DEBUG(logger, "    Mean marker error: %3.2f", rigidBody.meanMarkerError);
  }

  // NatNet version 2.6 and later
  if (natNetVersion >= mocap_optitrack::Version("2.6"))
  {
    // params
    short params = 0; 
    utilities::read_and_seek(msgBufferIter, params);
    rigidBody.isTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
    RCLCPP_DEBUG(logger, "    Successfully tracked in this frame: %s", 
      (rigidBody.isTrackingValid ? "YES" : "NO"));
  }
}


void DataFrameMessage::deserialize(
  rclcpp::Logger logger,
  MessageBuffer const& msgBuffer, 
  mocap_optitrack::DataModel* dataModel)
{
  // Get iterator to beginning of buffer and skip the header
  MessageBuffer::const_iterator msgBufferIter = msgBuffer.begin();
  utilities::seek(msgBufferIter, 4); // Skip the header (4 bytes)

  // Next 4 bytes is the frame number
  utilities::read_and_seek(msgBufferIter, dataModel->frameNumber);
  RCLCPP_DEBUG(logger, "=== BEGIN DATA FRAME ===");
  RCLCPP_DEBUG(logger, "Frame number: %d", dataModel->frameNumber);

  // Here on out its conveinent to get a pointer directly
  // to the ModelFrame object as well as the NatNetVersion
  mocap_optitrack::ModelFrame* dataFrame = &(dataModel->dataFrame);
  mocap_optitrack::Version const& NatNetVersion = dataModel->getNatNetVersion();

  // Next 4 bytes is the number of data sets (markersets, rigidbodies, etc)
  int numMarkerSets = 0;
  utilities::read_and_seek(msgBufferIter, numMarkerSets);
  RCLCPP_DEBUG(logger, "*** MARKER SETS ***");
  RCLCPP_DEBUG(logger, "Marker set count: %d", numMarkerSets);
  dataFrame->markerSets.resize(numMarkerSets);

  // Loop through number of marker sets and get name and data
  // TODO: Whether this correctly parses marker sets has not been tested
  int icnt = 0;
  for (auto& markerSet : dataFrame->markerSets)
  {
    // Markerset name
    strcpy(markerSet.name, &(*msgBufferIter));
    utilities::seek(msgBufferIter, strlen(markerSet.name) + 1);
    RCLCPP_DEBUG(logger, "  Marker set %d: %s", icnt++, markerSet.name);

    // Read number of markers that belong to the model
    int numMarkers = 0;
    utilities::read_and_seek(msgBufferIter, numMarkers);
    markerSet.markers.resize(numMarkers);
    RCLCPP_DEBUG(logger, "  Number of markers: %d", numMarkers);

    int jcnt = 0;
    for (auto& marker : markerSet.markers)
    {
      // read marker positions
      utilities::read_and_seek(msgBufferIter, marker);
      RCLCPP_DEBUG(logger, "    Marker %d: [x=%3.2f,y=%3.2f,z=%3.2f]", 
        jcnt++, marker.x, marker.y, marker.z);
    }
  }

  // Loop through unlabeled markers
  RCLCPP_DEBUG(logger, "*** UNLABELED MARKERS (Deprecated) ***");
  int numUnlabeledMarkers = 0;
  utilities::read_and_seek(msgBufferIter, numUnlabeledMarkers);
  dataFrame->otherMarkers.resize(numUnlabeledMarkers);
  RCLCPP_DEBUG(logger, "Unlabled marker count: %d", numUnlabeledMarkers);

  // Loop over unlabled markers
  icnt = 0;
  for (auto& marker : dataFrame->otherMarkers)
  {
    // read positions of 'other' markers
    utilities::read_and_seek(msgBufferIter, marker);
    RCLCPP_DEBUG(logger, "  Marker %d: [x=%3.2f,y=%3.2f,z=%3.2f]", 
        icnt++, marker.x, marker.y, marker.z);
    // Deprecated
  }

  // Loop through rigidbodies
  RCLCPP_DEBUG(logger, "*** RIGID BODIES ***");
  int numRigidBodies = 0;
  utilities::read_and_seek(msgBufferIter, numRigidBodies);
  dataFrame->rigidBodies.resize(numRigidBodies);
  RCLCPP_DEBUG(logger, "Rigid count: %d", numRigidBodies);

  // Loop over rigid bodies
  for (auto& rigidBody : dataFrame->rigidBodies)
  {
    DataFrameMessage::RigidBodyMessagePart rigidBodyMessagePart;
    rigidBodyMessagePart.deserialize(logger, msgBufferIter, rigidBody, dataModel->getNatNetVersion());
  }

  // Skeletons (NatNet version 2.1 and later)
  // TODO: skeletons not currently included in data model. Parsing
  //       is happening.. but need to copy into a model.
  if (NatNetVersion >= mocap_optitrack::Version("2.1"))
  {
    RCLCPP_DEBUG(logger, "*** SKELETONS ***");
    int numSkeletons = 0;
    utilities::read_and_seek(msgBufferIter, numSkeletons);
    RCLCPP_DEBUG(logger, "Skeleton count: %d", numSkeletons);

    // Loop through skeletons
    for (int j=0; j < numSkeletons; j++)
    {
      // skeleton id
      int skeletonId = 0;
      utilities::read_and_seek(msgBufferIter, skeletonId);
      RCLCPP_DEBUG(logger, "Skeleton ID: %d", skeletonId);

      // Number of rigid bodies (bones) in skeleton
      int numRigidBodies = 0;
      utilities::read_and_seek(msgBufferIter, numRigidBodies);
      RCLCPP_DEBUG(logger, "Rigid body count: %d", numRigidBodies);

      // Loop through rigid bodies (bones) in skeleton
      for (int j=0; j < numRigidBodies; j++)
      {
        mocap_optitrack::RigidBody rigidBody;
        DataFrameMessage::RigidBodyMessagePart rigidBodyMessagePart;
        rigidBodyMessagePart.deserialize(logger, msgBufferIter, rigidBody, NatNetVersion);
      } // next rigid body
    } // next skeleton
  }

  // Labeled markers (NatNet version 2.3 and later)
  // TODO: like skeletons, labeled markers are not accounted for
  //       in the data model. They are being parsed but not recorded.
  if (NatNetVersion >= mocap_optitrack::Version("2.3"))
  {
    RCLCPP_DEBUG(logger, "*** LABELED MARKERS ***");
    int numLabeledMarkers = 0;
    utilities::read_and_seek(msgBufferIter, numLabeledMarkers);
    RCLCPP_DEBUG(logger, "Labeled marker count: %d", numLabeledMarkers);

    // Loop through labeled markers
    for (int j=0; j < numLabeledMarkers; j++)
    {
      int id = 0; 
      utilities::read_and_seek(msgBufferIter, id);
      int modelId, markerId;
      utilities::decode_marker_id(id, modelId, markerId);

      mocap_optitrack::Marker marker;
      utilities::read_and_seek(msgBufferIter, marker);
      
      float size;
      utilities::read_and_seek(msgBufferIter, size);

      if (NatNetVersion >= mocap_optitrack::Version("2.6"))
      {
        // marker params
        short params = 0;
        utilities::read_and_seek(msgBufferIter, params);
        // marker was not visible (occluded) in this frame
        bool bOccluded = (params & 0x01) != 0;
        // position provided by point cloud solve     
        bool bPCSolved = (params & 0x02) != 0;
        // position provided by model solve
        bool bModelSolved = (params & 0x04) != 0;  
        if (NatNetVersion >= mocap_optitrack::Version("3.0"))
        {
          // marker has an associated model
          bool bHasModel = (params & 0x08) != 0;
          // marker is an unlabeled marker
          bool bUnlabeled = (params & 0x10) != 0;   
          // marker is an active marker 
          bool bActiveMarker = (params & 0x20) != 0;
        }
      }

      RCLCPP_DEBUG(logger, "  MarkerID: %d, ModelID: %d", markerId, modelId);
      RCLCPP_DEBUG(logger, "    Pos: [%3.2f,%3.2f,%3.2f]", 
        marker.x, marker.y, marker.z);
      RCLCPP_DEBUG(logger, "    Size: %3.2f", size);

      // NatNet version 3.0 and later
      if (NatNetVersion >= mocap_optitrack::Version("3.0"))
      {
        // Marker residual
        float residual = 0.0f;
        utilities::read_and_seek(msgBufferIter, residual);
        RCLCPP_DEBUG(logger, "    Residual:  %3.2f", residual);
      }
    }
  }

  // Force Plate data (NatNet version 2.9 and later)
  // TODO: This is definitely not in the data model..
  if (NatNetVersion >= mocap_optitrack::Version("2.9"))
  {
    RCLCPP_DEBUG(logger, "*** FORCE PLATES ***");
    int numForcePlates;
    utilities::read_and_seek(msgBufferIter, numForcePlates);
    RCLCPP_DEBUG(logger, "Force plate count: %d", numForcePlates);
    for (int iForcePlate = 0; iForcePlate < numForcePlates; iForcePlate++)
    {
        // ID
        int forcePlateId = 0;
        utilities::read_and_seek(msgBufferIter, forcePlateId);
        RCLCPP_DEBUG(logger, "Force plate ID: %d", forcePlateId);

        // Channel Count
        int numChannels = 0; 
        utilities::read_and_seek(msgBufferIter, numChannels);
        RCLCPP_DEBUG(logger, "  Number of channels: %d", numChannels);

        // Channel Data
        for (int i = 0; i < numChannels; i++)
        {
            RCLCPP_DEBUG(logger, "    Channel %d: ", i);
            int numFrames = 0;
            utilities::read_and_seek(msgBufferIter, numFrames);
            for (int j = 0; j < numFrames; j++)
            {
                float val = 0.0f;  
                utilities::read_and_seek(msgBufferIter, val);
                RCLCPP_DEBUG(logger, "      Frame %d: %3.2f", j, val);
            }
        }
    }
  }

  // Device data (NatNet version 3.0 and later)
  // TODO: Also not in the data model..
  if (NatNetVersion >= mocap_optitrack::Version("3.0"))
  {
    RCLCPP_DEBUG(logger, "*** DEVICE DATA ***");
    int numDevices;
    utilities::read_and_seek(msgBufferIter, numDevices);
    RCLCPP_DEBUG(logger, "Device count: %d", numDevices);

    for (int iDevice = 0; iDevice < numDevices; iDevice++)
    {
      // ID
      int deviceId = 0;
      utilities::read_and_seek(msgBufferIter, deviceId);
      RCLCPP_DEBUG(logger, "  Device ID: %d", deviceId);

      // Channel Count
      int numChannels = 0;
      utilities::read_and_seek(msgBufferIter, numChannels);

      // Channel Data
      for (int i = 0; i < numChannels; i++)
      {
        RCLCPP_DEBUG(logger, "    Channel %d: ", i);
        int nFrames = 0; 
        utilities::read_and_seek(msgBufferIter, nFrames);
        for (int j = 0; j < nFrames; j++)
        {
            float val = 0.0f;
            utilities::read_and_seek(msgBufferIter, val);
            RCLCPP_DEBUG(logger, "      Frame %d: %3.2f", j, val);
        }
      }
    }
  }

  // software latency (removed in version 3.0)
  RCLCPP_DEBUG(logger, "*** DIAGNOSTICS ***");
  if (NatNetVersion < mocap_optitrack::Version("3.0"))
  {
    utilities::read_and_seek(msgBufferIter, dataFrame->latency);
    RCLCPP_DEBUG(logger, "Software latency : %3.3f", dataFrame->latency);
  }

  // timecode
  unsigned int timecode = 0;
  utilities::read_and_seek(msgBufferIter, timecode);
  unsigned int timecodeSub = 0;
  utilities::read_and_seek(msgBufferIter, timecodeSub);
  char szTimecode[128] = "";
  utilities::stringify_timecode(timecode, timecodeSub, szTimecode, 128);

  // timestamp
  double timestamp = 0.0f;

  // NatNet version 2.7 and later - increased from single to double precision
  if (NatNetVersion >= mocap_optitrack::Version("2.7"))
  {
    utilities::read_and_seek(msgBufferIter, timestamp);
  }
  else
  {
    float fTimestamp = 0.0f;
    utilities::read_and_seek(msgBufferIter, fTimestamp);
    timestamp = (double)fTimestamp;
  }
  RCLCPP_DEBUG(logger, "Timestamp: %3.3f", timestamp);

  // Loop over rigid bodies to add optitrack timestamp
  for (auto& rigidBody : dataFrame->rigidBodies)
  {
    rigidBody.trackTimestamp = timestamp;
  }

  // high res timestamps (version 3.0 and later)
  if (NatNetVersion >= mocap_optitrack::Version("3.0"))
  {
    uint64_t cameraMidExposureTimestamp = 0;
    utilities::read_and_seek(msgBufferIter, cameraMidExposureTimestamp);
    RCLCPP_DEBUG(logger, "Mid-exposure timestamp: %" PRIu64 "", cameraMidExposureTimestamp);

    uint64_t cameraDataReceivedTimestamp = 0;
    utilities::read_and_seek(msgBufferIter, cameraDataReceivedTimestamp);
    RCLCPP_DEBUG(logger, "Camera data received timestamp: %" PRIu64 "", cameraDataReceivedTimestamp);

    uint64_t transmitTimestamp = 0;
    utilities::read_and_seek(msgBufferIter, transmitTimestamp);
    RCLCPP_DEBUG(logger, "Transmit timestamp: %" PRIu64 "", transmitTimestamp);
  }

  // frame params
  short params = 0;  
  utilities::read_and_seek(msgBufferIter, params);
  // 0x01 Motive is recording
  bool bIsRecording = (params & 0x01) != 0;
  // 0x02 Actively tracked model list has changed
  bool bTrackedModelsChanged = (params & 0x02) != 0;

  // end of data tag
  int eod = 0; 
  utilities::read_and_seek(msgBufferIter, eod);
  RCLCPP_DEBUG(logger, "=== END DATA FRAME ===");
}


void MessageDispatcher::dispatch(
  rclcpp::Logger logger,
  MessageBuffer const& msgBuffer, 
  mocap_optitrack::DataModel* dataModel)
{
  // Grab message ID by casting to a natnet packet type
  char const* pMsgBuffer = &msgBuffer[0];
  natnet::Packet const* packet = (natnet::Packet const*)(pMsgBuffer);
  // ROS_DEBUG("Message ID: %d", packet->messageId);
  // ROS_DEBUG("Byte count : %d", (int)msgBuffer.size());

  if (packet->messageId == natnet::MessageType::ModelDef ||
      packet->messageId == natnet::MessageType::FrameOfData)
  {
    if (dataModel->hasServerInfo())
    {
      DataFrameMessage msg;
      msg.deserialize(logger, msgBuffer, dataModel);
    }
    else
    {
      RCLCPP_WARN(logger, "Client has not received server info request. Parsing data message aborted.");
    }
    return;
  }

  if (packet->messageId == natnet::MessageType::ServerInfo)
  {
    natnet::ServerInfoMessage msg;
    msg.deserialize(msgBuffer, dataModel);
    RCLCPP_INFO_ONCE(logger, "NATNet Version : %s",
      dataModel->getNatNetVersion().getVersionString().c_str());
    RCLCPP_INFO_ONCE(logger, "Server Version : %s",
      dataModel->getServerVersion().getVersionString().c_str());
    return;
  }

  if (packet->messageId == natnet::MessageType::UnrecognizedRequest)
  {
    RCLCPP_WARN(logger, "Received unrecognized request");
  }
}


} // namespace
