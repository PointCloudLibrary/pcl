/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 * Copyright (c) 2004, Sylvain Paris and Francois Sillion

 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */


#ifndef FAST_BILATERAL_H_
#define FAST_BILATERAL_H_


#include <pcl/filters/filter.h>

namespace pcl
{
  /** \brief Implementation of a fast bilateral filter for smoothing depth information in organized point clouds
   *  Based on the following paper:
   *    * Sylvain Paris and Frédo Durand
   *      "A Fast Approximation of the Bilateral Filter using a Signal Processing Approach"
   *       European Conference on Computer Vision (ECCV'06)
   *
   *  More details on the webpage: http://people.csail.mit.edu/sparis/bf/
   */
  template<typename PointT>
  class FastBilateralFilter : public Filter<PointT>
  {
    using Filter<PointT>::input_;
    typedef typename Filter<PointT>::PointCloud PointCloud;

    public:
    
      typedef boost::shared_ptr< FastBilateralFilter<PointT> > Ptr;
      typedef boost::shared_ptr< const FastBilateralFilter<PointT> > ConstPtr;

      FastBilateralFilter ()
        :  sigma_s_ (15.0f)
         , sigma_r_ (0.05f)
         , early_division_ (false)
      {
      }

      inline void
      setSigmaS (float sigma_s)
      { sigma_s_ = sigma_s; }

      inline float
      getSigmaS ()
      { return sigma_s_; }


      inline void
      setSigmaR (float sigma_r)
      { sigma_r_ = sigma_r; }

      inline float
      getSigmaR ()
      { return sigma_r_; }


      inline void
      setEarlyDivision (bool early_division)
      { early_division_ = early_division; }

      inline bool
      getEarlyDivision ()
      { return early_division_; }


      void
      applyFilter (PointCloud &output);

    private:
      float sigma_s_;
      float sigma_r_;
      bool early_division_;

      class Array3D
      {
        public:
          Array3D (const size_t width, const size_t height, const size_t depth)
          {
            x_dim_ = width;
            y_dim_ = height;
            z_dim_ = depth;
            v_ = std::vector<Eigen::Vector2f> (width*height*depth, Eigen::Vector2f (0.0f, 0.0f));
          }

          Eigen::Vector2f&
          operator () (const size_t x, const size_t y, const size_t z)
          { return v_[(x * y_dim_ + y) * z_dim_ + z]; }

          const Eigen::Vector2f&
          operator () (const size_t x, const size_t y, const size_t z) const
          { return v_[(x * y_dim_ + y) * z_dim_ + z]; }

          void
          resize (const size_t width, const size_t height, const size_t depth)
          {
            x_dim_ = width;
            y_dim_ = height;
            z_dim_ = depth;
            v_.resize (x_dim_ * y_dim_ * z_dim_);
          }

          Eigen::Vector2f
          trilinear_interpolation (const float x,
                                   const float y,
                                   const float z);

          static inline size_t
          clamp (const size_t min_value,
                 const size_t max_value,
                 const size_t x);

          size_t x_size () const { return x_dim_; }
          size_t y_size () const { return y_dim_; }
          size_t z_size () const { return z_dim_; }

          std::vector<Eigen::Vector2f >::iterator begin () { return v_.begin (); }
          std::vector<Eigen::Vector2f >::iterator end () { return v_.end (); }

          std::vector<Eigen::Vector2f >::const_iterator begin () const { return v_.begin (); }
          std::vector<Eigen::Vector2f >::const_iterator end () const { return v_.end (); }

        private:
          std::vector<Eigen::Vector2f > v_;
          size_t x_dim_, y_dim_, z_dim_;
      };


  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/fast_bilateral.hpp>
#else
#define PCL_INSTANTIATE_FastBilateralFilter(T) template class PCL_EXPORTS pcl::FastBilateralFilter<T>;
#endif


#endif /* FAST_BILATERAL_H_ */
