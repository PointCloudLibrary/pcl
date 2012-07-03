/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright  (c) 2009-2012,  Willow Garage,  Inc.
 *  Copyright  (c) 2006,  Michael Kazhdan and Matthew Bolitho,
 *                      Johns Hopkins University
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms,  with or without
 *  modification,  are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice,  this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice,  this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage,  Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,  INCLUDING,  BUT NOT
 *  LIMITED TO,  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,  INDIRECT,
 *  INCIDENTAL,  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO,  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE,  DATA,  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY,  WHETHER IN CONTRACT,  STRICT
 *  LIABILITY,  OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE,  EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: binary_node.h 5922 2012-06-13 19:46:55Z nburrus $
 *
 */

#ifndef PCL_POISSON_BINARY_NODE_H_
#define PCL_POISSON_BINARY_NODE_H_

namespace pcl {
  namespace poisson {

    template<class Real>
    class BinaryNode
    {
    public:
      static inline int CenterCount (int depth){return 1<<depth;}
      static inline int CumulativeCenterCount (int maxDepth){return  (1<< (maxDepth+1))-1;}
      static inline int Index (int depth,  int offSet){return  (1<<depth)+offSet-1;}
      static inline int CornerIndex (int maxDepth, int depth, int offSet, int forwardCorner)
      {return  (offSet+forwardCorner)<< (maxDepth-depth);}
      static inline Real CornerIndexPosition (int index, int maxDepth)
      {return Real (index)/ (1<<maxDepth);}
      static inline Real Width (int depth)
      {return Real (1.0/ (1<<depth));}      

      // Fix for Bug #717 with Visual Studio that generates wrong code for this function
      // when global optimization is enabled (release mode).
#ifdef _MSC_VER
      static __declspec(noinline) void CenterAndWidth (int depth, int offset, Real& center, Real& width)
#else
      static inline void CenterAndWidth (int depth, int offset, Real& center, Real& width)
#endif
      {
        width=Real (1.0/ (1<<depth));
        center=Real ( (0.5+offset)*width);
      }

#ifdef _MSC_VER
      static __declspec(noinline) void CenterAndWidth (int idx, Real& center, Real& width)
#else
      static inline void CenterAndWidth (int idx, Real& center, Real& width)
#endif
      {
        int depth, offset;
        DepthAndOffset (idx, depth, offset);
        CenterAndWidth (depth, offset, center, width);
      }
      static inline void DepthAndOffset (int idx,  int& depth, int& offset)
      {
        int i = idx+1;
        depth = -1;
        while (i)
        {
          i>>=1;
          depth++;
        }
        offset= (idx+1)- (1<<depth);
      }
    };


  }
}

#endif /* PCL_POISSON_BINARY_NODE_H_ */
