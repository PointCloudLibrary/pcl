/*
 * Software License Agreement (Simplified BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 * Copyright (c) 2012, Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
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
 * hog.h
 * Created on: Nov 30, 2012
 * Derived from Piotr Dollar's MATLAB Image&Video Toolbox      Version 3.00. 
 * Non-SSE version of the code provided by Matteo Munaro, Stefano Ghidoni and Stefano Michieletto
 */

#pragma once

#include <pcl/pcl_macros.h> // export macro

namespace pcl
{ 
  namespace people
  {
    /** \brief @b HOG represents a class for computing the HOG descriptor described in 
      * Dalal, N. and Triggs, B., "Histograms of oriented gradients for human detection", CVPR 2005.
      * \author Matteo Munaro, Stefano Ghidoni, Stefano Michieletto
      * \ingroup people
      */
    class PCL_EXPORTS HOG
    {
    public:

      /** \brief Constructor. */
      HOG ();

      /** \brief Destructor. */
      virtual ~HOG ();
      
      /** 
       * \brief Compute gradient magnitude and orientation at each location (uses sse). 
       * 
       * \param[in] I Image as array of float.
       * \param[in] h Image height.
       * \param[in] w Image width.
       * \param[in] d Image number of channels.
       * \param[out] M Gradient magnitude for each image point.
       * \param[out] O Gradient orientation for each image point.
       */
      void 
      gradMag ( float *I, int h, int w, int d, float *M, float *O ) const;

      /** 
       * \brief Compute n_orients gradient histograms per bin_size x bin_size block of pixels.  
       * 
       * \param[in] M Gradient magnitude for each image point.
       * \param[in] O Gradient orientation for each image point.
       * \param[in] h Image height.
       * \param[in] w Image width.
       * \param[in] bin_size Spatial bin size.
       * \param[in] n_orients Number of orientation bins.
       * \param[in] soft_bin If true, each pixel can contribute to multiple spatial bins (using bilinear interpolation).
       * \param[out] H Gradient histograms.
       */
      void 
      gradHist ( float *M, float *O, int h, int w, int bin_size, int n_orients, bool soft_bin, float *H) const;
      
      /** 
       * \brief Normalize histogram of gradients. 
       * 
       * \param[in] H Gradient histograms.
       * \param[in] h Image height.
       * \param[in] w Image width.
       * \param[in] bin_size Spatial bin size.
       * \param[in] n_orients Number of orientation bins.  
       * \param[in] clip Value at which to clip histogram bins.      
       * \param[out] G Normalized gradient histograms.
       */
      void 
      normalization ( float *H, int h, int w, int bin_size, int n_orients, float clip, float *G ) const;
      
      /**
       * \brief Compute HOG descriptor.
       * 
       * \param[in] I Image as array of float between 0 and 1.
       * \param[in] h Image height.
       * \param[in] w Image width.
       * \param[in] n_channels Image number of channels.
       * \param[in] bin_size Spatial bin size.  
       * \param[in] n_orients Number of orientation bins.     
       * \param[in] soft_bin If true, each pixel can contribute to multiple spatial bins (using bilinear interpolation).
       * \param[out] descriptor HOG descriptor.
       */
      void
      compute (float *I, int h, int w, int n_channels, int bin_size, int n_orients, bool soft_bin, float *descriptor);
      
      /**
       * \brief Compute HOG descriptor with default parameters.
       * 
       * \param[in] I Image as array of float between 0 and 1.
       * \param[out] descriptor HOG descriptor.
       */
      void
      compute (float *I, float *descriptor) const;
      
        private:
    
      /** 
       * \brief Compute x and y gradients for just one column (uses sse). 
       */
      void 
      grad1 ( float *I, float *Gx, float *Gy, int h, int w, int x ) const; 
      
      /** 
       * \brief Build lookup table a[] s.t. a[dx/2.02*n]~=acos(dx). 
       */
      float* 
      acosTable () const;
      
      /** 
       * \brief Helper for gradHist, quantize O and M into O0, O1 and M0, M1 (uses sse). 
       */
      void 
      gradQuantize ( float *O, float *M, int *O0, int *O1, float *M0, float *M1, int n_orients, int nb, int n, float norm ) const;
      
      /** 
       * \brief Platform independent aligned memory allocation (see also alFree).
       */ 
      void* 
      alMalloc ( std::size_t size, int alignment ) const;
      
      /** 
       * \brief Platform independent aligned memory de-allocation (see also alMalloc).
       */ 
      void 
      alFree (void* aligned) const;
      
    protected:
      
      /** \brief image height (default = 128) */
      int h_;
      
      /** \brief image width (default = 64) */
      int w_;
      
      /** \brief image number of channels (default = 3) */
      int n_channels_;
      
      /** \brief spatial bin size (default = 8) */
      int bin_size_; 
      
      /** \brief number of orientation bins (default = 9) */
      int n_orients_;
      
      /** \brief if true, each pixel can contribute to multiple spatial bins (using bilinear interpolation) (default = true) */
      bool soft_bin_;   
      
      /** \brief value at which to clip histogram bins (default = 0.2) */
      float clip_; 
      
    };
  } /* namespace people */
} /* namespace pcl */
