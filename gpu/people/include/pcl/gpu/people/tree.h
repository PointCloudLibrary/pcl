/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: $
 * @authors: Cedric Cagniart, Koen Buys
 */

#ifndef PCL_GPU_PEOPLE_TREE_H_
#define PCL_GPU_PEOPLE_TREE_H_

#include<boost/cstdint.hpp>  //#include <cstdint>
#include <pcl/gpu/containers/device_array.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        // this has nothing to do here...
        static const double focal = 1000.;

        // ###############################################
        // compile type values
        enum { NUMATTRIBS  = 2000 };
        enum { NUMLABELS = 32 };

        // ###############################################
        // base data types used in the structures
        typedef boost::int16_t Attrib;
        typedef boost::uint8_t Label;
        typedef boost::uint32_t Label32;
        typedef boost::uint16_t Depth;

        struct AttribLocation {
          inline AttribLocation () {du1=dv1=du2=dv2=0;}
          inline AttribLocation (int u1, int v1, int u2, int v2): du1 (static_cast<boost::int16_t>(u1)),
                                                                  dv1 (static_cast<boost::int16_t>(v1)),
                                                                  du2 (static_cast<boost::int16_t>(u2)),
                                                                  dv2 (static_cast<boost::int16_t>(v2)){}

          boost::int16_t du1,dv1,du2,dv2;
        };
        static const Label NOLABEL = 31;

        // ###############################################
        // Tree basic Structure
        struct Node {
          inline Node () {}
          inline Node (const AttribLocation& l, const Attrib& t):loc(l),thresh(t){}
          AttribLocation loc;
          Attrib         thresh;
        };

        // GPU Typedefs
        typedef DeviceArray<Label>      D_Label_8;
        typedef DeviceArray<Label32>    D_Label_32;
        typedef DeviceArray<Depth>      D_Depth;

      } // end namespace Trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl
#endif  // PCL_GPU_PEOPLE_TREES_TREE_H_
