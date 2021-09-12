/* 
 * Copyright (c) 2016, Tony Baltovski
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
#include <mocap_optitrack/version.h>

#include <sstream>

namespace mocap_optitrack
{

Version::Version()
{
  setVersion(0, 0, 0, 0);
}

Version::Version(int major, int minor, int revision, int build)
{
  setVersion(major, minor, revision, build);
}

Version::Version(const std::string& version)
  : v_string(version)
{
  int major=0, minor=0, revision=0, build=0;
  std::sscanf(version.c_str(), "%d.%d.%d.%d", &major, &minor, &revision, &build);
  setVersion(major, minor, revision, build);
}

Version::~Version()
{
}
void Version::setVersion(int major, int minor, int revision, int build)
{
  v_major = major;
  v_minor = minor;
  v_revision = revision;
  v_build = build;

  std::stringstream sstr;
  sstr << v_major << "." << v_minor << "." << v_revision << "." << v_build;
  v_string  = sstr.str();
}

std::string const& Version::getVersionString() const
{
  return v_string;
}

bool Version::operator > (const Version& comparison) const
{
  if (v_major > comparison.v_major)
    return true;
  if (v_minor > comparison.v_minor)
    return true;
  if (v_revision > comparison.v_revision)
    return true;
  if (v_build > comparison.v_build)
    return true;
  return false;
}

bool Version::operator >= (const Version& comparison) const
{
  return ((*this > comparison) || (*this == comparison));
}

bool Version::operator < (const Version& comparison) const
{
  if (v_major < comparison.v_major)
    return true;
  if (v_minor < comparison.v_minor)
    return true;
  if (v_revision < comparison.v_revision)
    return true;
  if (v_build < comparison.v_build)
    return true;
  return false;
}

bool Version::operator <= (const Version& comparison) const
{
  return ((*this < comparison) || (*this == comparison));
}

bool Version::operator == (const Version& comparison) const
{
  return v_major == comparison.v_major
      && v_minor == comparison.v_minor
      && v_revision == comparison.v_revision
      && v_build == comparison.v_build;
}

}