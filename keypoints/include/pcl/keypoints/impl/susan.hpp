/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#ifndef PCL_SUSAN_IMPL_HPP_
#define PCL_SUSAN_IMPL_HPP_

#include <pcl/common/io.h> // for getFieldIndex
#include <pcl/keypoints/susan.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::setNonMaxSupression (bool nonmax)
{
  nonmax_ = nonmax;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::setGeometricValidation (bool validate)
{
  geometric_validation_ = validate;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void 
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::setRadius (float radius)
{ 
  search_radius_ = radius; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void 
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::setDistanceThreshold (float distance_threshold) 
{
  distance_threshold_ = distance_threshold; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void 
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::setAngularThreshold (float angular_threshold) 
{ 
  angular_threshold_ = angular_threshold; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void 
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::setIntensityThreshold (float intensity_threshold) 
{ 
  intensity_threshold_ = intensity_threshold; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void 
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::setNormals (const PointCloudNConstPtr &normals)
{ 
  normals_ = normals;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::setSearchSurface (const PointCloudInConstPtr &cloud) 
{ 
  surface_ = cloud; 
  normals_.reset (new pcl::PointCloud<NormalT>);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::setNumberOfThreads (unsigned int nr_threads)
{
  threads_ = nr_threads;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void
// pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::USAN (const PointInT& nucleus,
//                                                                     const NormalT& nucleus_normal,
//                                                                     const pcl::Indices& neighbors, 
//                                                                     const float& t,
//                                                                     float& response,
//                                                                     Eigen::Vector3f& centroid) const
// {
//   float area = 0;
//   response = 0;
//   float x0 = nucleus.x;
//   float y0 = nucleus.y;
//   float z0 = nucleus.z;
//   //xx xy xz yy yz zz
//   std::vector<float> coefficients(6);
//   memset (&coefficients[0], 0, sizeof (float) * 6);
//   for (const auto& index : neighbors)
//   {
//     if (std::isfinite ((*normals_)[index].normal_x))
//     {
//       Eigen::Vector3f diff = (*normals_)[index].getNormal3fMap () - nucleus_normal.getNormal3fMap ();
//       float c = diff.norm () / t;
//       c = -1 * pow (c, 6.0);
//       c = std::exp (c);
//       Eigen::Vector3f xyz = (*surface_)[index].getVector3fMap ();
//       centroid += c * xyz;
//       area += c;
//       coefficients[0] += c * (x0 - xyz.x) * (x0 - xyz.x);
//       coefficients[1] += c * (x0 - xyz.x) * (y0 - xyz.y);
//       coefficients[2] += c * (x0 - xyz.x) * (z0 - xyz.z);
//       coefficients[3] += c * (y0 - xyz.y) * (y0 - xyz.y);
//       coefficients[4] += c * (y0 - xyz.y) * (z0 - xyz.z);
//       coefficients[5] += c * (z0 - xyz.z) * (z0 - xyz.z);
//     }
//   }

//   if (area > 0)
//   {
//     centroid /= area;
//     if (area < geometric_threshold)
//       response = geometric_threshold - area;
//     // Look for edge direction
//     // X direction
//     if ((coefficients[3]/coefficients[0]) < 1 && (coefficients[5]/coefficients[0]) < 1)
//       direction = Eigen::Vector3f (1, 0, 0);
//     else
//     {
//       // Y direction
//       if ((coefficients[0]/coefficients[3]) < 1 && (coefficients[5]/coefficients[3]) < 1)
//         direction = Eigen::Vector3f (0, 1, 0);
//       else
//       {
//         // Z direction
//         if ((coefficients[0]/coefficients[5]) < 1 && (coefficients[3]/coefficients[5]) < 1)
//           direction = Eigen::Vector3f (0, 1, 0);
//         // Diagonal edge
//         else 
//         {
//           //XY direction
//           if ((coefficients[2]/coeffcients[1]) < 1 && (coeffcients[4]/coeffcients[1]) < 1)
//           {
//             if (coeffcients[1] > 0)
//               direction = Eigen::Vector3f (1,1,0);
//             else
//               direction = Eigen::Vector3f (-1,1,0);
//           }
//           else
//           {
//             //XZ direction
//             if ((coefficients[1]/coeffcients[2]) > 1 && (coeffcients[4]/coeffcients[2]) < 1)
//             {
//               if (coeffcients[2] > 0)
//                 direction = Eigen::Vector3f (1,0,1);
//               else
//                 direction = Eigen::Vector3f (-1,0,1);
//             }
//             //YZ direction
//             else
//             {
//               if (coeffcients[4] > 0)
//                 direction = Eigen::Vector3f (0,1,1);
//               else
//                 direction = Eigen::Vector3f (0,-1,1);
//             }
//           }
//         }
//       }
//     }
    
//     // std::size_t max_index = std::distance (coefficients.begin (), max);
//     // switch (max_index)
//     // {
//     //   case 0 : direction = Eigen::Vector3f (1, 0, 0); break;
//     //   case 1 : direction = Eigen::Vector3f (1, 1, 0); break;
//     //   case 2 : direction = Eigen::Vector3f (1, 0, 1); break;
//     //   case 3 : direction = Eigen::Vector3f (0, 1, 0); break;
//     //   case 4 : direction = Eigen::Vector3f (0, 1, 1); break;
//     //   case 5 : direction = Eigen::Vector3f (0, 0, 1); break;
//     // }
//   }
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> bool
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::initCompute ()
{
  if (!Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed!\n", name_.c_str ());
    return (false);
  }
  
  if (normals_->empty ())
  {
    PointCloudNPtr normals (new PointCloudN ());
    normals->reserve (normals->size ());
    if (!surface_->isOrganized ())
    {
      pcl::NormalEstimation<PointInT, NormalT> normal_estimation;
      normal_estimation.setInputCloud (surface_);
      normal_estimation.setRadiusSearch (search_radius_);
      normal_estimation.compute (*normals);
    }
    else
    {
      IntegralImageNormalEstimation<PointInT, NormalT> normal_estimation;
      normal_estimation.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointInT, NormalT>::SIMPLE_3D_GRADIENT);
      normal_estimation.setInputCloud (surface_);
      normal_estimation.setNormalSmoothingSize (5.0);
      normal_estimation.compute (*normals);
    }
    normals_ = normals;
  }
  if (normals_->size () != surface_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] normals given, but the number of normals does not match the number of input points!\n", name_.c_str ());
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> bool
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::isWithinNucleusCentroid (const Eigen::Vector3f& nucleus,
                                                                                       const Eigen::Vector3f& centroid,
                                                                                       const Eigen::Vector3f& nc,
                                                                                       const PointInT& point) const
{
  Eigen::Vector3f pc = centroid - point.getVector3fMap ();
  Eigen::Vector3f pn = nucleus - point.getVector3fMap ();
  Eigen::Vector3f pc_cross_nc = pc.cross (nc);
  return ((pc_cross_nc.norm () <= tolerance_) && (pc.dot (nc) >= 0) && (pn.dot (nc) <= 0));
}

// template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> bool
// pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::isValidQueryPoint3D (int point_index) const
// {
//   return (isFinite (surface_->points [point_index]) && 
//           isFinite (normals_->points [point_index]));
// }

// template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> bool
// pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::isValidQueryPoint2D (int point_index) const
// {
//   return (isFinite (surface_->points [point_index]));
// }

// template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> bool
// pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::isWithinSusan2D (int nucleus, int neighbor) const
// {
//   return (std::abs (intensity_ ((*surface_)[nucleus]) - 
//                 intensity_ ((*surface_)[neighbor])) <= intensity_threshold_);
// }

// template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> bool
// pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::isWithinSusan3D (int nucleus, int neighbor) const
// {
//   Eigen::Vector3f nucleus_normal = normals_->point[nucleus].getVector3fMap ();
//   return (1 - nucleus_normal.dot ((*normals_)[*index].getNormalVector3fMap ()) <= angular_threshold_);
// }

// template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> bool
// pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::isWithinSusanH (int nucleus, int neighbor) const
// {
//   Eigen::Vector3f nucleus_normal = normals_->point[nucleus].getVector3fMap ();
//   return ((1 - nucleus_normal.dot ((*normals_)[*index].getNormalVector3fMap ()) <= angular_threshold_) || 
//           (std::abs (intensity_ ((*surface_)[nucleus]) - intensity_ ((*surface_)[neighbor])) <= intensity_threshold_));
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT, typename IntensityT> void
pcl::SUSANKeypoint<PointInT, PointOutT, NormalT, IntensityT>::detectKeypoints (PointCloudOut &output)
{
  typename pcl::PointCloud<PointOutT>::Ptr response (new pcl::PointCloud<PointOutT>);
  response->reserve (surface_->size ());

  // Check if the output has a "label" field
  label_idx_ = pcl::getFieldIndex<PointOutT> ("label", out_fields_);

  const auto input_size = static_cast<pcl::index_t> (input_->size ());
  for (pcl::index_t point_index = 0; point_index < input_size; ++point_index)
  {
    const PointInT& point_in = input_->points [point_index];
    const NormalT& normal_in = normals_->points [point_index];
    if (!isFinite (point_in) || !isFinite (normal_in))
      continue;

    Eigen::Vector3f nucleus = point_in.getVector3fMap ();
    Eigen::Vector3f nucleus_normal = normals_->points [point_index].getNormalVector3fMap ();
    float nucleus_intensity = intensity_ (point_in);
    pcl::Indices nn_indices;
    std::vector<float> nn_dists;
    tree_->radiusSearch (point_in, search_radius_, nn_indices, nn_dists);
    float area = 0;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero ();
    // Exclude nucleus from the usan
    std::vector<int> usan; usan.reserve (nn_indices.size () - 1);
    for (const auto& index : nn_indices)
    {
      if ((index != point_index) && std::isfinite ((*normals_)[index].normal_x))
      {
        // if the point fulfill condition
        if ((std::abs (nucleus_intensity - intensity_ ((*input_)[index])) <= intensity_threshold_) ||
            (1 - nucleus_normal.dot ((*normals_)[index].getNormalVector3fMap ()) <= angular_threshold_))
        {
          ++area;
          centroid += (*input_)[index].getVector3fMap ();
          usan.push_back (index);
        }
      }
    }

    float geometric_threshold = 0.5f * (static_cast<float> (nn_indices.size () - 1));
    if ((area > 0) && (area < geometric_threshold))
    {
      // if no geometric validation required add the point to the response
      if (!geometric_validation_)
      {
        PointOutT point_out;
        point_out.getVector3fMap () = point_in.getVector3fMap ();
        //point_out.intensity = geometric_threshold - area; 
        intensity_out_.set (point_out, geometric_threshold - area);
        // if a label field is found use it to save the index
        if (label_idx_ != -1)
        {
          // save the index in the cloud
          std::uint32_t label = static_cast<std::uint32_t> (point_index);
          memcpy (reinterpret_cast<char*> (&point_out) + out_fields_[label_idx_].offset,
                  &label, sizeof (std::uint32_t));
        }
        response->push_back (point_out);
      }
      else
      {
        centroid /= area;
        Eigen::Vector3f dist = nucleus - centroid;
        // Check the distance <= distance_threshold_
        if (dist.norm () >= distance_threshold_)
        {
          // point is valid from distance point of view 
          Eigen::Vector3f nc = centroid - nucleus;
          // Check the contiguity
          auto usan_it = usan.cbegin ();
          for (; usan_it != usan.cend (); ++usan_it)
          {
            if (!isWithinNucleusCentroid (nucleus, centroid, nc, (*input_)[*usan_it]))
              break;
          }
          // All points within usan lies on the segment [nucleus centroid]
          if (usan_it == usan.end ())
          {
            PointOutT point_out;
            point_out.getVector3fMap () = point_in.getVector3fMap ();
            // point_out.intensity = geometric_threshold - area; 
            intensity_out_.set (point_out, geometric_threshold - area);
            // if a label field is found use it to save the index
            if (label_idx_ != -1)
            {
              // save the index in the cloud
              std::uint32_t label = static_cast<std::uint32_t> (point_index);
              memcpy (reinterpret_cast<char*> (&point_out) + out_fields_[label_idx_].offset,
                      &label, sizeof (std::uint32_t));
            }
            response->push_back (point_out);
          }
        }
      }
    }
  }
  
  response->height = 1;
  response->width = response->size ();
  
  if (!nonmax_)
  {
    output = *response;
    for (std::size_t i = 0; i < response->size (); ++i)
      keypoints_indices_->indices.push_back (i);
    // we don not change the denseness
    output.is_dense = input_->is_dense;
  }
  else
  {
    output.clear ();
    output.reserve (response->size());
    
    for (pcl::index_t idx = 0; idx < static_cast<pcl::index_t> (response->size ()); ++idx)
    {
      const PointOutT& point_in = response->points [idx];
      const NormalT& normal_in = normals_->points [idx];
      //const float intensity = (*response)[idx].intensity;
      const float intensity = intensity_out_ ((*response)[idx]);
      if (!isFinite (point_in) || !isFinite (normal_in) || (intensity == 0))
        continue;
      pcl::Indices nn_indices;
      std::vector<float> nn_dists;
      tree_->radiusSearch (idx, search_radius_, nn_indices, nn_dists);
      bool is_minima = true;
      for (const auto& nn_index : nn_indices)
      {
//        if (intensity > (*response)[nn_index].intensity)
        if (intensity > intensity_out_ ((*response)[nn_index]))
        {
          is_minima = false;
          break;
        }
      }
      if (is_minima)
      {
        output.push_back ((*response)[idx]);
        keypoints_indices_->indices.push_back (idx);
      }
    }
    
    output.height = 1;
    output.width = output.size();
    output.is_dense = true;
  }
}

#define PCL_INSTANTIATE_SUSAN(T,U,N) template class PCL_EXPORTS pcl::SUSANKeypoint<T,U,N>;
#endif // #ifndef PCL_HARRIS_KEYPOINT_3D_IMPL_H_

