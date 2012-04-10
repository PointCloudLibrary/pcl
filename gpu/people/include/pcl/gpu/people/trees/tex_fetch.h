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


#ifndef PCL_GPU_PEOPLE_TREES_TEX_FETCH_H_
#define PCL_GPU_PEOPLE_TREES_TEX_FETCH_H_

#include <boost/cstdint.hpp> //#include <cstdint>
#include <cmath> // for round
#include <opencv2/core/core.hpp>

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
	  	    inline Tex2Dfetcher( const boost::uint16_t* dmap, int W, int H ):m_dmap(dmap),m_W(W), m_H(H){}

          inline boost::uint16_t operator () ( float uf, float vf ) {
            int u = cvRound(uf);
            int v = cvRound(vf);
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

#endif
