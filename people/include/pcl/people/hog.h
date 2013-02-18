// Adaptation of "gradientMex.cpp" provided with Piotr Dollar's MATLAB toolbox:
/*******************************************************************************
* Piotr's Image&Video Toolbox      Version 3.00
* Copyright 2012 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License
*
* Copyright (c) 2012, Piotr Dollar
* Copyright (c) 2013-, Open Perception, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met: 
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution. 
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
* either expressed or implied, of the FreeBSD Project.
*
*******************************************************************************/

#ifndef PCL_PEOPLE_HOG_H_
#define PCL_PEOPLE_HOG_H_

#include <cmath>
#include <algorithm>

#define eps 0.0001
#define PI 3.1415926535897931

namespace pcl
{
  namespace people
  {
    class HOG
    {
	public:

      /** \brief Constructor. */
      HOG ();

      /** \brief Destructor. */
      virtual ~HOG ();
      
      /**
       * \brief snap to one of oBin orientations using binary search
       **/
      int
      compOrient (double dx, double dy, double *ux, double *uy, int oBin);

      /**
       * \brief compute gradient magnitude (*2) and orientation
       **/
      void
      compGradImg (double *I, double *G, int *O, int h, int w, int nCh, int oBin);

      /**
       * \brief compute HOG features
       */
      void
      compute (double *I, int h, int w, int nCh, int sBin, int oBin, int oGran, double* H);
    };
  } /* namespace people */
} /* namespace pcl */
#include <pcl/people/impl/hog.hpp>
#endif /* PCL_PEOPLE_HOG_H_ */
