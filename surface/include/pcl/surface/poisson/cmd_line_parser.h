/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho,
 *                      Johns Hopkins University
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef POISSON_CMD_LINE_PARSER_H_
#define POISSON_CMD_LINE_PARSER_H_
#include <stdarg.h>
#include <string.h>

#include "pcl/surface/poisson/Geometry.h"

namespace pcl
{
  namespace surface
  {
#ifdef WIN32
    int strcasecmp(char* c1,char* c2);
#endif

    class cmdLineReadable
    {
    public:
      int set;
      cmdLineReadable (void);
      virtual
      ~cmdLineReadable (void);
      virtual int
      read (char** argv, int argc);
    };

    class cmdLineInt : public cmdLineReadable
    {
    public:
      int value;
      cmdLineInt ();
      cmdLineInt (const int& v);
      int
      read (char** argv, int argc);
    };
    class cmdLineFloat : public cmdLineReadable
    {
    public:
      float value;
      cmdLineFloat ();
      cmdLineFloat (const float& f);
      int
      read (char** argv, int argc);
    };
    class cmdLineString : public cmdLineReadable
    {
    public:
      char* value;
      cmdLineString ();
      ~cmdLineString ();
      int
      read (char** argv, int argc);
    };
    class cmdLinePoint3D : public cmdLineReadable
    {
    public:
      Point3D<float> value;
      cmdLinePoint3D ();
      cmdLinePoint3D (const Point3D<float>& v);
      cmdLinePoint3D (const float& v0, const float& v1, const float& v2);
      int
      read (char** argv, int argc);
    };

    // This reads the arguments in argc, matches them against "names" and sets
    // the values of "r" appropriately. Parameters start with "--"
    void
    cmdLineParse (int argc, char **argv, char** names, int num, cmdLineReadable** r, int dumpError = 1);

    char*
    GetFileExtension (char* fileName);
  }
}
#endif // CMD_LINE_PARSER_INCLUDED
