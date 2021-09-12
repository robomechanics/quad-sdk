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
#ifndef __MOCAP_OPTITRACK_DATA_MODEL_H__
#define __MOCAP_OPTITRACK_DATA_MODEL_H__

#include <string>
#include <vector>

#include <mocap_optitrack/version.h>

namespace mocap_optitrack
{


/// \brief Data object holding the position of a single mocap marker in 3d space
struct Marker
{
  float x;
  float y;
  float z;
};

struct __attribute__ ((__packed__)) Pose
{
  struct __attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
  } position;
  struct __attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
    float w;
  } orientation;
};

/// \brief Data object holding information about a single rigid body within a mocap skeleton
struct RigidBody
{
    RigidBody();
    int bodyId;
    Pose pose;
    float meanMarkerError;
    bool isTrackingValid;

    bool hasValidData() const;
};

/// \brief Data object describing a single tracked model
struct ModelDescription
{
    ModelDescription();
    void clear();

    std::string name;
    std::vector<std::string> markerNames;
};

struct MarkerSet
{
    void clear();

    char name[256];
    std::vector<Marker> markers;
};

/// \brief Data object holding poses of a tracked model's components
struct ModelFrame
{
    ModelFrame();
    void clear();

    std::vector<MarkerSet> markerSets;
    std::vector<Marker> otherMarkers;
    std::vector<RigidBody> rigidBodies;

    float latency;
};

/// \brief Data object holding server info
struct ServerInfo
{
    ServerInfo();
    Version natNetVersion;
    Version serverVersion;
};

/// \brief The data model for this node
class DataModel
{
public:
    DataModel();

    int frameNumber;
    ModelFrame dataFrame;

    void clear();

    void setVersions(int* nver, int* sver);
    Version const& getNatNetVersion() const;
    Version const& getServerVersion() const;
    bool hasServerInfo() const {return hasValidServerInfo;};

private:
    ServerInfo serverInfo;
    bool hasValidServerInfo;
};

}

#endif