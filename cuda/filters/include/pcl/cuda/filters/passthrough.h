/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 */

#pragma once

#include <pcl_cuda/filters/filter.h>
#include <thrust/count.h>
#include <thrust/remove.h>
#include <vector_types.h>

namespace pcl_cuda
{

  /** \brief Check if a specific point is valid or not. Applicable to AOS structures. */
  struct isFiniteAOS
  {
    __inline__ __device__ bool
    operator () (const PointXYZRGB &pt)
    //operator () (const float3 &pt)
    {
      return (isfinite (pt.x) && isfinite (pt.y) && isfinite (pt.z));
    }
  };

  /** \brief Check if a specific point is valid or not. Applicable to SOA structures. */
  struct isFiniteSOA
  {
    __inline__ __device__ bool
    operator () (const float &pt)
    {
      return (isfinite (pt));
    }
  };

  /** \brief Check if a specific point is valid or not. Applicable to SOA structures. */
  struct isFiniteZIPSOA
  {
    __inline__ __device__ bool
    operator () (const PointCloudSOA<Device>::tuple_type& tuple)
    {
      using thrust::get;
      return (!isfinite (get<0> (tuple)) || 
              !isfinite (get<1> (tuple)) || 
              !isfinite (get<2> (tuple)));
    }
  };

  ///////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b PassThrough uses the base Filter class methods to pass through
    * all data that satisfies the user given constraints.
    */
  template <typename CloudT>
  class PassThrough: public Filter<CloudT>
  {
    public:
      using Filter<CloudT>::filter_name_;

      using PointCloud = typename PCLCUDABase<CloudT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      /** \brief Empty constructor. */
      PassThrough ()
      {
        filter_name_ = "PassThrough";
      };

    protected:
      /** \brief Filter a Point Cloud.
        * \param output the resultant point cloud message
        */
      void 
      applyFilter (PointCloud &output)
      {
        std::cerr << "applyFilter" << std::endl;
      }
  };
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class PassThrough<PointCloudAOS<Device> >: public Filter<PointCloudAOS<Device> >
  {
    public:
      /** \brief Empty constructor. */
      PassThrough ()
      {
        filter_name_ = "PassThroughAOS";
      };

    protected:
      /** \brief Filter a Point Cloud.
        * \param output the resultant point cloud message
        */
      void 
      applyFilter (PointCloud &output)
      {
        // Allocate enough space
        output.resize (input_->points.size ());
        // Copy data
        Device<PointXYZRGB>::type::iterator nr_points = thrust::copy_if (input_->points.begin (), input_->points.end (), output.begin (), isFiniteAOS ());
        //Device<float3>::type::iterator nr_points = thrust::copy_if (input_->points.begin (), input_->points.end (), output.begin (), isFiniteAOS ());
        output.resize (nr_points - output.begin ());

        //std::cerr << "[applyFilterAOS]: ";
        //std::cerr << input_->points.size () << " " << output.size () << std::endl;
      }
  };
 
  //////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class PassThrough<PointCloudSOA<Device> >: public Filter<PointCloudSOA<Device> >
  {
    public:
      /** \brief Empty constructor. */
      PassThrough () : zip_(false)
      {
        filter_name_ = "PassThroughSOA";
      };

      inline void
      setZip (bool zip)
      {
        zip_ = zip;
      }


    protected:
      /** \brief Filter a Point Cloud.
        * \param output the resultant point cloud message
        */
      void 
      applyFilter (PointCloud &output)
      {
        if (!zip_)
        {
          // Allocate enough space
          output.resize (input_->size ());
          // Copy data
          Device<float>::type::iterator nr_points = thrust::copy_if (input_->points_x.begin (), input_->points_x.end (), output.points_x.begin (), isFiniteSOA ());
          nr_points = thrust::copy_if (input_->points_y.begin (), input_->points_y.end (), output.points_y.begin (), isFiniteSOA ());
          nr_points = thrust::copy_if (input_->points_z.begin (), input_->points_z.end (), output.points_z.begin (), isFiniteSOA ());
          output.resize (nr_points - output.points_z.begin ());
        
          //std::cerr << "[applyFilterSOA]: ";
          //std::cerr << input_->size () << " " << output.size () << std::endl;
        }

        else
        {
          output = *input_;
          PointCloud::zip_iterator result = thrust::remove_if (output.zip_begin (), output.zip_end (), isFiniteZIPSOA ());
          PointCloud::iterator_tuple result_tuple = result.get_iterator_tuple ();
          PointCloud::float_iterator xiter = thrust::get<0> (result_tuple),
                                     yiter = thrust::get<1> (result_tuple),
                                     ziter = thrust::get<2> (result_tuple);

          unsigned badpoints = distance (xiter, output.points_x.end ());
          unsigned goodpoints = distance (output.points_x.begin (), xiter);

          output.resize (goodpoints);

          //std::cerr << "[applyFilterSOA-ZIP]: ";
          //std::cerr << input_->size () << " " << output.size () << std::endl;
        }
      }

    private:
      bool zip_;
  };
}
