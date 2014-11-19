/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#ifndef PCL_KMEANS_HPP_
#define PCL_KMEANS_HPP_

#include <pcl/ml/kmeans.h>

//#include <pcl/common/io.h>

//#include <stdio.h>
//#include <stdlib.h>
//#include <time.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::Kmeans<PointT>::Kmeans () 
  : cluster_field_name_ ("")
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::Kmeans<PointT>::~Kmeans ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Kmeans<PointT>::k_means ()
{
}

template <typename PointT> void
pcl::Kmeans<PointT>::cluster (std::vector<PointIndices> &clusters)
{
  if (!initCompute () ||
      (input_ != 0   && input_->points.empty ()) ||
      (indices_ != 0 && indices_->empty ()))
  {
    clusters.clear ();
    return;
  }

  pcl::PointCloud <PointT> point;
  std::vector<pcl::PCLPointField> fields;

  int user_index = -1;
  // if no cluster field name is set, check for X Y Z
  if (strcmp (cluster_field_name_.c_str (), "") == 0)
  {
    int x_index = -1;
    int y_index = -1;
    int z_index = -1;
    x_index = pcl::getFieldIndex (point, "x", fields);
    if (y_index != -1)
      y_index = pcl::getFieldIndex (point, "y", fields);
    if (z_index != -1)
      z_index = pcl::getFieldIndex (point, "z", fields);

    if (x_index == -1 && y_index == -1 && z_index == -1)
    {
      PCL_ERROR ("Failed to find match for field 'x y z'\n" );
      return;
    }

    PCL_INFO ("Use X Y Z as input data\n");
    // create input data
/*
    for (size_t i = 0; i < input_->points.size (); i++)
    {
      DataPoint data (3);
      data[0] = input_->points[i].data[0];
      


    }
*/

    std::cout << "x index: " << x_index << std::endl;
    
    float x = 0.0;
    memcpy (&x, &input_->points[0] + fields[x_index].offset, sizeof(float));
    
    std::cout << "xxx: " << x << std::endl;
    

    //memcpy (&x, reinterpret_cast<float*> (&input_->points[0]) + x_index, sizeof (float));
    

    //int rgba_index = 1;

    //pcl::RGB rgb;
    //memcpy (&rgb, reinterpret_cast<const char*> (&input_->points[index_vector[i].cloud_point_index]) + rgba_index, sizeof (RGB));

    
    
  }
  // if cluster field name is set, check if field name is valied
  else
  {
    user_index = pcl::getFieldIndex (point, cluster_field_name_.c_str (), fields);

    if (user_index == -1)
    {
      PCL_ERROR ("Failed to find match for field '%s'\n", cluster_field_name_.c_str ());
      return;
    }
  }

  
  
  
/*
  int xyz_index = -1;
  pcl::PointCloud <PointT> point;
  xyz_index = pcl::getFieldIndex (point, "r", fields);


  if (xyz_index == -1 && strcmp (cluster_field_name_.c_str (), "") == 0)
  {
    PCL_ERROR ("Failed to find match for field '%s'\n", cluster_field_name_.c_str ());
  }


  std::cout << "index: " << xyz_index << std::endl;
  
  std::string t = pcl::getFieldsList (point);
  std::cout << "t: " << t << std::endl;
*/
  
  //std::vector <pcl::PCLPointField> fields;
  //pcl::getFieldIndex (*input_, "xyz", fields);
  
  
  //std::cout << "field: " << fields[xyz_index].count << std::endl;
  

/*
  for (size_t i = 0; i < fields[vfh_idx].count; ++i)
  {
    
    //vfh.second[i] = point.points[0].histogram[i];
    
  }
*/



  deinitCompute ();
}




#define PCL_INSTANTIATE_Kmeans(T) template class PCL_EXPORTS pcl::Kmeans<T>;

#endif    // PCL_KMEANS_HPP_
