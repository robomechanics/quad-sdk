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
#include "mocap_optitrack/data_model.h"

namespace mocap_optitrack
{

RigidBody::RigidBody() : 
  isTrackingValid(false)
{
}

bool RigidBody::hasValidData() const
{
    return isTrackingValid;
}


void ModelDescription::clear()
{
  markerNames.clear();
}

void MarkerSet::clear()
{
  markers.clear();
}


ModelFrame::ModelFrame() : 
    latency(0.0)
{
}

void ModelFrame::clear()
{
  markerSets.clear();
  otherMarkers.clear();
  rigidBodies.clear();
}


ServerInfo::ServerInfo() :
    natNetVersion(0,0,0,0),
    serverVersion(0,0,0,0)
{

}


DataModel::DataModel() :
  hasValidServerInfo(false)
{

}

void DataModel::clear()
{
  dataFrame.clear();
}

void DataModel::setVersions(int* nver, int* sver)
{
  serverInfo.natNetVersion.setVersion(nver[0], nver[1], nver[2], nver[3]);
  serverInfo.serverVersion.setVersion(sver[0], sver[1], sver[2], sver[3]);
  hasValidServerInfo = true;
}

void DataModel::setVersions(int64_t* nver, int64_t* sver)
{
  serverInfo.natNetVersion.setVersion(nver[0], nver[1], nver[2], nver[3]);
  serverInfo.serverVersion.setVersion(sver[0], sver[1], sver[2], sver[3]);
  hasValidServerInfo = true;
}

Version const& DataModel::getNatNetVersion() const
{
  return serverInfo.natNetVersion;
}

Version const& DataModel::getServerVersion() const
{
  return serverInfo.serverVersion;
}

}