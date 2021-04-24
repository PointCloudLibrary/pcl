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

#pragma once

#include "label_common.h"
#include <cstdint>
#include <iostream>
#include <vector>

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
        enum { NUM_ATTRIBS  = 2000 };
        enum { NUM_LABELS = 32 };

        // ###############################################
        // base data types used in the structures

        using std::uint8_t;
        using std::int16_t;
        using std::uint16_t;
        using std::int32_t;
        using std::uint32_t;

        using Attrib = std::int16_t;
        using Label = std::uint8_t;
        using Label32 = std::uint32_t;
        using Depth = std::uint16_t;

        struct AttribLocation
        {
          inline AttribLocation () {du1=dv1=du2=dv2=0;}
          inline AttribLocation (int u1, int v1, int u2, int v2): du1 (static_cast<std::int16_t>(u1)),
                                                                  dv1 (static_cast<std::int16_t>(v1)),
                                                                  du2 (static_cast<std::int16_t>(u2)),
                                                                  dv2 (static_cast<std::int16_t>(v2))
          {}

          std::int16_t du1,dv1,du2,dv2;
        };

        ////////////////////////////////////////////////
        // Tree basic Structure
        struct Node 
        {
          Node () {}
          Node (const AttribLocation& l, const Attrib& t) : loc(l), thresh(t) {}
          AttribLocation loc;
          Attrib         thresh;
        };

        struct Histogram
        {
          float label_prob[NUM_PARTS];
        };

        ////////////////////////////////////////////////
        // tree_io - Reading and writing AttributeLocations
        inline std::ostream& operator << (std::ostream& os, const AttribLocation& aloc ) { return os<<aloc.du1<<" "<<aloc.dv1<<" "<<aloc.du2<<" "<<aloc.dv2<<"\n"; }
        inline std::istream& operator >> (std::istream& is, AttribLocation& aloc ) { return is >> aloc.du1 >> aloc.dv1 >> aloc.du2 >> aloc.dv2; }
        inline std::istream& operator >> (std::istream& is, Node& n) { return is >> n.loc >> n.thresh; }

        void writeAttribLocs( const std::string& filename, const std::vector<AttribLocation>& alocs );
        void readAttribLocs( const std::string& filename, std::vector<AttribLocation>& alocs );
        void readThreshs( const std::string& filename, std::vector<Attrib>& threshs );
        void writeThreshs( const std::string& filename, const std::vector<Attrib>& threshs );

        ////////////////////////////////////////////////
        // tree_run

        /** The stream points to ascii data that goes:
          * ##################
          * TreeHeight
          * du1 dv1 du2 dv2 thresh
          * du1 dv1 du2 dv2 thresh
          * ............
          * label
          * label
          * ##################
          *
          * there are 2^threeheight -1 nodes ( [du1 dv1 du2 dv2 thresh] lines )
          * there are 2^threeheight   leaves ( [label] lines )
          */
        int loadTree( std::istream& is, std::vector<Node>&  tree, std::vector<Label>& leaves );
        int loadTree( const std::string&  filename, std::vector<Node>&  tree, std::vector<Label>& leaves );
        void runThroughTree( int maxDepth, const std::vector<Node>& tree, const std::vector<Label>& leaves, int W, int H, const std::uint16_t* dmap, Label* lmap );

      } // end namespace Trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl
