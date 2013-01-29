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
 * $Id:  $
 * @authors: Koen Buys, Anatoly Baksheev
 *
 */

#include <pcl/gpu/people/tree.h>
#include <pcl/gpu/people/label_common.h>
#include <pcl/console/print.h>

#include <fstream>
#include <string>
#include <cassert>
#include <stdexcept>
#include <limits>
#include <iostream>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        /**
         * \brief Simple helper structure to fetch the texture without bothering about limits
         * This will be done by CUDA directly for the run time part of the stuff
         */
        struct Tex2Dfetcher
        {
	  	  Tex2Dfetcher( const boost::uint16_t* dmap, int W, int H ) : m_dmap(dmap), m_W(W), m_H(H) {}

          inline boost::uint16_t operator () ( float uf, float vf ) 
          {
            int u = static_cast<int>(uf);
            int v = static_cast<int>(vf);
            if( u < 0 ) u = 0;
            if( v < 0 ) v = 0;
            if( u >= m_W ) u = m_W-1;
            if( v >= m_H ) v = m_H-1;
            
            return m_dmap[u+v*m_W]; // this is going to be SLOOOWWW
          }
          const boost::uint16_t*  m_dmap;
          const int               m_W;
          const int               m_H;
        };
      } // end namespace trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl

using pcl::gpu::people::trees::NUM_ATTRIBS;

//////////////////////////////////////////////////////////////////////////////////
// tree_run

int
pcl::gpu::people::trees::loadTree ( std::istream&       is,
                                    std::vector<Node>&  tree,
                                    std::vector<Label>& leaves )
{
  // load the depth
  int maxDepth;
  is >> maxDepth;
  int numNodes  = (1 << maxDepth) - 1; //pow(2.0,maxDepth)-1;
  int numLeaves = (1 << maxDepth);    //pow(2.0,maxDepth);

  // alloc
  tree.resize(numNodes);
  leaves.resize(numLeaves);

  // read
  for(int ni = 0; ni < numNodes; ++ni)
    is >> tree[ni];

  // the int is necessary.. otherwise it will format it as unsigned char
  for(int li = 0; li < numLeaves; ++li) 
  {
      int l; is >> l; leaves[li] = l; 
  }

  // Check loading of the tree in terminal
  PCL_DEBUG("[pcl::gpu::people::trees::loadTree] : (D) : loaded %d nodes, %d leaves and depth %d\n", numNodes, numLeaves, maxDepth);

  if( is.fail() )
     throw std::runtime_error(std::string("(E) malformed *.tree stream") );
  return maxDepth;
}

int 
pcl::gpu::people::trees::loadTree ( const std::string&        filename,
                                          std::vector<Node>&  tree,
                                          std::vector<Label>& leaves )
{
  std::ifstream fin(filename.c_str() );
  if( !fin.is_open() ) 
    throw std::runtime_error(std::string("[pcl::gpu::people::trees::loadTree] : (E) could not open ") + filename );
  return loadTree(fin, tree, leaves);
}

void 
pcl::gpu::people::trees::runThroughTree( int maxDepth,
                                    const std::vector<Node>&  tree,
                                    const std::vector<Label>& leaves,
                                    int                       W, 
                                    int                       H,
                                    const uint16_t*           dmap,
                                    Label*                    lmap )
{
  Tex2Dfetcher tfetch( dmap, W, H ); // the tex fetcher

  int numNodes = (1 << maxDepth) - 1; //pow(2.0,maxDepth) - 1;
  for(int y = 0; y < H; ++y) 
  {
    for(int x = 0; x < W; ++x) 
    {
      uint16_t depth = tfetch((float)x,(float)y);
      if(depth == std::numeric_limits<uint16_t>::max() ) 
      {
        lmap[x+W*y] = pcl::gpu::people::NOLABEL;
        continue;
      }

      double scale = focal / depth;

      // and now for the tree processing 
      int nid = 0;
      for(int d = 0; d < maxDepth; ++d) 
      {
        const Node& node = tree[nid];
        const AttribLocation& loc = node.loc;
        uint16_t d1       = tfetch((float)(x+loc.du1*scale), (float)(y+loc.dv1*scale));
        uint16_t d2       = tfetch((float)(x+loc.du2*scale), (float)(y+loc.dv2*scale));
        int32_t delta     = int32_t(d1) - int32_t(d2);
        bool test = delta > int32_t(node.thresh);

        nid = test ? (nid*2+2) : (nid*2+1);
      }
      lmap[x+W*y] = leaves[nid-numNodes];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////
// tree_io

void 
pcl::gpu::people::trees::writeAttribLocs( const std::string& filename,
                                          const std::vector<AttribLocation>& alocs )
{
  std::ofstream fout(filename.c_str());
  if(!fout.is_open()) 
    throw std::runtime_error(std::string("(E) could not open ") + filename);

  // first we write the number of attribs we intend to write so that we ll avoid mismatches
  assert( alocs.size() == NUM_ATTRIBS );
  fout << NUM_ATTRIBS << "\n";
  for(int ai=0;ai<NUM_ATTRIBS;++ai)
    fout << alocs[ai];
}

void 
pcl::gpu::people::trees::readAttribLocs( const std::string& filename,
                                          std::vector<AttribLocation>& alocs )
{
  std::ifstream fin(filename.c_str() );
  if(!fin.is_open()) 
    throw std::runtime_error(std::string("(E) could not open ") + filename );

  int numAttribs;
  fin >> numAttribs;
  if( numAttribs != NUM_ATTRIBS ) 
    throw std::runtime_error(std::string("(E) the attribloc file has a wrong number of attribs ") + filename );

  alocs.resize(NUM_ATTRIBS);
  for(int ai = 0; ai < NUM_ATTRIBS; ++ai) 
    fin >> alocs[ai];
}

void 
pcl::gpu::people::trees::readThreshs( const std::string& filename, 
                                      std::vector<Attrib>& threshs )
{
  std::ifstream fin(filename.c_str() );
  if(!fin.is_open()) 
    throw std::runtime_error(std::string("(E) could not open " + filename) );

  int numThreshs;
  fin >> numThreshs;
  threshs.resize(numThreshs);
  for(int ti =0;ti<numThreshs;++ti)
    fin >> threshs[ti];

  if( fin.fail() ) 
    throw std::runtime_error(std::string("(E) malformed thresh file: " + filename) );
}

void 
pcl::gpu::people::trees::writeThreshs( const std::string& filename, 
                                       const std::vector<Attrib>& threshs )
{
  std::ofstream fout(filename.c_str() );
  if(!fout.is_open()) 
    throw std::runtime_error(std::string("(E) could not open " + filename) );

  int numThreshs = threshs.size();

  fout << numThreshs << "\n";
  for(int ti =0;ti<numThreshs;++ti)
    fout << threshs[ti] << "\n";  
}

