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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef PCL_CRF_SEGMENTATION_HPP_
#define PCL_CRF_SEGMENTATION_HPP_

#include <pcl/segmentation/crf_segmentation.h>

#include <pcl/common/io.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::CrfSegmentation<PointT>::CrfSegmentation () :
  voxel_grid_ (),
  input_cloud_ (new pcl::PointCloud<PointT>),
  normal_cloud_ (new pcl::PointCloud<pcl::PointNormal>),
  filtered_cloud_ (new pcl::PointCloud<PointT>),
  filtered_anno_ (new pcl::PointCloud<pcl::PointXYZRGBL>),
  filtered_normal_ (new pcl::PointCloud<pcl::PointNormal>),
  voxel_grid_leaf_size_ (Eigen::Vector4f (0.001f, 0.001f, 0.001f, 0.0f))
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::CrfSegmentation<PointT>::~CrfSegmentation ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::setInputCloud (typename pcl::PointCloud<PointT>::Ptr input_cloud)
{
  if (input_cloud_ != NULL)
    input_cloud_.reset ();

  input_cloud_ = input_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::setAnnotatedCloud (typename pcl::PointCloud<pcl::PointXYZRGBL>::Ptr anno_cloud)
{
  if (anno_cloud_ != NULL)
    anno_cloud_.reset ();

  anno_cloud_ = anno_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::setNormalCloud (typename pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud)
{
  if (normal_cloud_ != NULL)
    normal_cloud_.reset ();

  normal_cloud_ = normal_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::setVoxelGridLeafSize (const float x, const float y, const float z)
{
  voxel_grid_leaf_size_.x () = x;
  voxel_grid_leaf_size_.y () = y;
  voxel_grid_leaf_size_.z () = z;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::setSmoothnessKernelParameters (const float sx, const float sy, const float sz, 
                                                             const float w)
{
  smoothness_kernel_param_[0] = sx;
  smoothness_kernel_param_[1] = sy;
  smoothness_kernel_param_[2] = sz;
  smoothness_kernel_param_[3] = w;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::setAppearanceKernelParameters (float sx, float sy, float sz, 
                                                             float sr, float sg, float sb,
                                                             float w)
{
  appearance_kernel_param_[0] = sx;
  appearance_kernel_param_[1] = sy;
  appearance_kernel_param_[2] = sz;
  appearance_kernel_param_[3] = sr;
  appearance_kernel_param_[4] = sg;
  appearance_kernel_param_[5] = sb;
  appearance_kernel_param_[6] = w;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::setSurfaceKernelParameters (float sx, float sy, float sz, 
                                                          float snx, float sny, float snz,
                                                          float w)
{
  surface_kernel_param_[0] = sx;
  surface_kernel_param_[1] = sy;
  surface_kernel_param_[2] = sz;
  surface_kernel_param_[3] = snx;
  surface_kernel_param_[4] = sny;
  surface_kernel_param_[5] = snz;
  surface_kernel_param_[6] = w;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::createVoxelGrid ()
{
  // Filter the input cloud
  // Set the voxel grid input cloud
  voxel_grid_.setInputCloud (input_cloud_);
  // Set the voxel grid leaf size
  voxel_grid_.setLeafSize (voxel_grid_leaf_size_.x (), voxel_grid_leaf_size_.y (), voxel_grid_leaf_size_.z () );
  // Only downsample XYZ (if this is set to false, color values set to 0)
  voxel_grid_.setDownsampleAllData (true);
  // Save leaf information
  //voxel_grid_.setSaveLeafLayout (true);
  // apply the filter
  voxel_grid_.filter (*filtered_cloud_);

  // Filter the annotated cloud
  if (anno_cloud_->points.size () > 0)
  {
    pcl::VoxelGridLabel vg;

    vg.setInputCloud (anno_cloud_);
    // Set the voxel grid leaf size
    vg.setLeafSize (voxel_grid_leaf_size_.x (), voxel_grid_leaf_size_.y (), voxel_grid_leaf_size_.z () );
    // Only downsample XYZ
    vg.setDownsampleAllData (true);
    // Save leaf information
    //vg.setSaveLeafLayout (false);
    // apply the filter
    vg.filter (*filtered_anno_);
  }

  // Filter the annotated cloud
  if (normal_cloud_->points.size () > 0)
  {
    pcl::VoxelGrid<pcl::PointNormal> vg;
    vg.setInputCloud (normal_cloud_);
    // Set the voxel grid leaf size
    vg.setLeafSize (voxel_grid_leaf_size_.x (), voxel_grid_leaf_size_.y (), voxel_grid_leaf_size_.z () );
    // Only downsample XYZ
    vg.setDownsampleAllData (true);
    // Save leaf information
    //vg.setSaveLeafLayout (false);
    // apply the filter
    vg.filter (*filtered_normal_);    
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::createDataVectorFromVoxelGrid ()
{
  // get the dimension of the voxel grid
  //Eigen::Vector3i min_b, max_b;
  //min_b = voxel_grid_.getMinBoxCoordinates ();
  //max_b = voxel_grid_.getMaxBoxCoordinates ();
  
  //std::cout << "min_b: " << min_b.x () << " " << min_b.y () << " " << min_b.z () << std::endl;
  //std::cout << "max_b: " << max_b.x () << " " << max_b.y () << " " << max_b.z () << std::endl;

  // compute the voxel grid dimensions
  //dim_.x () = abs (max_b.x () - min_b.x ());
  //dim_.y () = abs (max_b.y () - min_b.y ());
  //dim_.z () = abs (max_b.z () - min_b.z ());
  
  //std::cout << dim_.x () * dim_.y () * dim_.z () << std::endl;

  // reserve the space for the data vector
  //data_.reserve (dim_.x () * dim_.y () * dim_.z ());

/*
  std::vector<Eigen::Vector3i> data;
  std::vector<Eigen::Vector3i> color;
  // fill the data vector
  for (int kk = min_b.z (), k = 0; kk <= max_b.z (); kk++, k++)
  {
    for (int jj = min_b.y (), j = 0; jj <= max_b.y (); jj++, j++)
    {
      for (int ii = min_b.x (), i = 0; ii <= max_b.x (); ii++, i++)
      {
        Eigen::Vector3i ijk (ii, jj, kk);
        int index = voxel_grid_.getCentroidIndexAt (ijk);
        if (index != -1)
        {
          data_.push_back (Eigen::Vector3i (i, j, k));
          color_.push_back (input_cloud_->points[index].getRGBVector3i ());
        }
      }
    }
  }
*/

  
/*
  // get the size of the input fields
  std::vector< pcl::PCLPointField > fields;
  pcl::getFields (*input_cloud_, fields);
  
  for (int i = 0; i < fields.size (); i++)
    std::cout << fields[i] << std::endl;
*/


  // reserve space for the data vector
  data_.resize (filtered_cloud_->points.size ());

  std::vector< pcl::PCLPointField > fields;
  // check if we have color data
  bool color_data = false;
  int rgba_index = -1;  
  rgba_index = pcl::getFieldIndex (*input_cloud_, "rgb", fields);
  if (rgba_index == -1)
    rgba_index = pcl::getFieldIndex (*input_cloud_, "rgba", fields);
  if (rgba_index >= 0)
  {
    color_data = true;
    color_.resize (filtered_cloud_->points.size ());    
  }


/*
  // check if we have normal data
  bool normal_data = false;
  int normal_index = -1;  
  rgba_index = pcl::getFieldIndex (*input_cloud_, "normal_x", fields);
  if (rgba_index >= 0)
  {
    normal_data = true;
    normal_.resize (filtered_cloud_->points.size ());    
  }
*/

  // fill the data vector
  for (size_t i = 0; i < filtered_cloud_->points.size (); i++)
  {
    Eigen::Vector3f p (filtered_anno_->points[i].x,
                       filtered_anno_->points[i].y,
                       filtered_anno_->points[i].z);
    Eigen::Vector3i c = voxel_grid_.getGridCoordinates (p.x (), p.y (), p.y ());
    data_[i] = c;

    if (color_data)
    {    
      uint32_t rgb = *reinterpret_cast<int*>(&filtered_cloud_->points[i].rgba);
      uint8_t r = (rgb >> 16) & 0x0000ff;
      uint8_t g = (rgb >> 8)  & 0x0000ff;
      uint8_t b = (rgb)       & 0x0000ff;
      color_[i] = Eigen::Vector3i (r, g, b);
    }

/*
    if (normal_data)
    {
      float n_x = filtered_cloud_->points[i].normal_x;
      float n_y = filtered_cloud_->points[i].normal_y;
      float n_z = filtered_cloud_->points[i].normal_z;
      normal_[i] = Eigen::Vector3f (n_x, n_y, n_z);
    }
*/
  }

  normal_.resize (filtered_normal_->points.size ());
  for (size_t i = 0; i < filtered_normal_->points.size (); i++)
  {
    float n_x = filtered_normal_->points[i].normal_x;
    float n_y = filtered_normal_->points[i].normal_y;
    float n_z = filtered_normal_->points[i].normal_z;
    normal_[i] = Eigen::Vector3f (n_x, n_y, n_z);
  }
  

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::createUnaryPotentials (std::vector<float> &unary,
                                                     std::vector<int> &labels,
                                                     unsigned int n_labels)
{
  /* initialize random seed: */
  srand ( static_cast<unsigned int> (time (NULL)) );
  //srand ( time (NULL) );

  // Certainty that the groundtruth is correct
  const float GT_PROB = 0.9f;
  const float u_energy = -logf ( 1.0f / static_cast<float> (n_labels) );
  const float n_energy = -logf ( (1.0f - GT_PROB) / static_cast<float>(n_labels - 1) );
  const float p_energy = -logf ( GT_PROB );

  for (size_t k = 0; k < filtered_anno_->points.size (); k++)
  {
    int label = filtered_anno_->points[k].label;

    if (labels.size () == 0 && label > 0)
      labels.push_back (label);

    // add color to the color vector if not added yet
    int c_idx;
    for (c_idx = 0; c_idx < static_cast<int> (labels.size ()) ; c_idx++)
    {
      if (labels[c_idx] == label)
        break;

      if (c_idx == static_cast<int>(labels.size () -1) && label > 0)
      {
        if (labels.size () < n_labels)
          labels.push_back (label);
        else
          label = 0;
      }
    }

   /* generate secret number: */
    //double iSecret = static_cast<double> (rand ())  / static_cast<double> (RAND_MAX);
   
    /* 
    if (k < 100)
      std::cout << iSecret << std::endl;
    */

/*
    int gg = 5; //static_cast<int> (labels.size ());
    if (iSecret < 0.5)
    {
      int r = 0;
      if (gg != 0)
        r = rand () % (gg - 1 + 1) + 1;
      else
        r = 0;
      c_idx = r;      
    }
*/
  
    // set the engeries for the labels
    size_t u_idx = k * n_labels;
    if (label > 0)
    {
      for (size_t i = 0; i < n_labels; i++)
        unary[u_idx + i] = n_energy;
      unary[u_idx + c_idx] = p_energy;

      if (label == 1)
      {
        const float PROB = 0.2f;
        const float n_energy2 = -logf ( (1.0f - PROB) / static_cast<float>(n_labels - 1) );
        const float p_energy2 = -logf ( PROB );

        for (size_t i = 0; i < n_labels; i++)
          unary[u_idx + i] = n_energy2;
        unary[u_idx + c_idx] = p_energy2;
      }
    
    }
    else
    {
      for (size_t i = 0; i < n_labels; i++)
        unary[u_idx + i] = u_energy;
    } 
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::CrfSegmentation<PointT>::segmentPoints (pcl::PointCloud<pcl::PointXYZRGBL> &output)
{
  // create the voxel grid
  createVoxelGrid ();
  std::cout << "create Voxel Grid - DONE" << std::endl;
  
  // create the data Vector
  createDataVectorFromVoxelGrid ();
  std::cout << "create Data Vector from Voxel Grid - DONE" << std::endl;

  // number of labels
  const int n_labels = 10;
  // number of data points
  int N = static_cast<int> (data_.size ());

  // create unary potentials
  std::vector<int> labels;
  std::vector<float> unary;
  if (anno_cloud_->points.size () > 0)
  {
    unary.resize (N * n_labels);
    createUnaryPotentials (unary, labels, n_labels);


    std::cout << "labels size: " << labels.size () << std::endl;
    for (size_t i = 0; i < labels.size (); i++)
    {
      std::cout << labels[i] << std::endl;
    }

  }
  std::cout << "create unary potentials - DONE" << std::endl;


/*
  pcl::PointCloud<pcl::PointXYZRGBL> tmp_cloud_OLD;
  tmp_cloud_OLD = *filtered_anno_;

  // Setup the CRF model
	DenseCRF2D crfOLD(N, 1, n_labels);

  float * unaryORI = new float[N*n_labels];
  for (int i = 0; i < N*n_labels; i++)
    unaryORI[i] = unary[i];
  crfOLD.setUnaryEnergy( unaryORI );

  float * pos = new float[N*3];
  for (int i = 0; i < N; i++)
  {
    pos[i * 3] = data_[i].x ();
    pos[i * 3 +1] = data_[i].y ();
    pos[i * 3 +2] = data_[i].z ();
  }  
	crfOLD.addPairwiseGaussian( pos, 3, 3, 3, 2.0 );

  float * col = new float[N*3];
  for (int i = 0; i < N; i++)
  {
    col[i * 3] = color_[i].x ();
    col[i * 3 +1] = color_[i].y ();
    col[i * 3 +2] = color_[i].z ();
  }  
	crfOLD.addPairwiseBilateral(pos, col,  20, 20, 20, 10, 10, 10, 1.3 );

	short * map = new short[N];
	crfOLD.map(10, map);

  for (size_t i = 0; i < N; i++)
  {
    tmp_cloud_OLD.points[i].label = map[i];
  }


*/

  //float * resultOLD = new float[N*n_labels];
  //crfOLD.inference (10, resultOLD);
  
  //std::vector<float> baryOLD;
  //crfOLD.getBarycentric (0, baryOLD);
  //std::vector<float> featuresOLD;
  //crfOLD.getFeature (1, featuresOLD);
  
/*
  for(int i = 0; i < 25; i++)
  {
    std::cout << baryOLD[i] << std::endl;
  }
*/


  // create the output cloud
  //output = *filtered_anno_;



  // ----------------------------------//
  // --------      -------------------//

  pcl::PointCloud<pcl::PointXYZRGBL> tmp_cloud;
  tmp_cloud = *filtered_anno_;

  // create dense CRF
  DenseCrf crf (N, n_labels);

  // set the unary potentials
  crf.setUnaryEnergy (unary);

  // set the data vector
  crf.setDataVector (data_);

  // set the color vector
  crf.setColorVector (color_);

  std::cout << "create dense CRF - DONE" << std::endl;


  // add the smoothness kernel
  crf.addPairwiseGaussian (smoothness_kernel_param_[0],
                           smoothness_kernel_param_[1],
                           smoothness_kernel_param_[2],
                           smoothness_kernel_param_[3]);
  std::cout << "add smoothness kernel - DONE" << std::endl;

  // add the appearance kernel
  crf.addPairwiseBilateral (appearance_kernel_param_[0],
                            appearance_kernel_param_[1],
                            appearance_kernel_param_[2],
                            appearance_kernel_param_[3],
                            appearance_kernel_param_[4],
                            appearance_kernel_param_[5],
                            appearance_kernel_param_[6]);
  std::cout << "add appearance kernel - DONE" << std::endl;

  crf.addPairwiseNormals (data_, normal_,
                          surface_kernel_param_[0],
                          surface_kernel_param_[1],
                          surface_kernel_param_[2],
                          surface_kernel_param_[3],
                          surface_kernel_param_[4],
                          surface_kernel_param_[5],
                          surface_kernel_param_[6]);
  std::cout << "add surface kernel - DONE" << std::endl;

  // map inference
  std::vector<int> r (N);
  crf.mapInference (n_iterations_, r);

  //std::vector<float> result (N*n_labels);
  //crf.inference (n_iterations_, result);

  //std::vector<float> bary;
  //crf.getBarycentric (0, bary);
  //std::vector<float> features;
  //crf.getFeatures (1, features);

  std::cout << "Map inference - DONE" << std::endl;

  // create the output cloud
  output = *filtered_anno_;

  for (int i = 0; i < N; i++)
  {
    output.points[i].label = labels[r[i]];
  }


/*
  bool c = true;
  for (size_t i = 0; i < tmp_cloud.points.size (); i++)
  {
    if (tmp_cloud.points[i].label != tmp_cloud_OLD.points[i].label)
    {
      
      std::cout << "idx: " << i << " =  " <<tmp_cloud.points[i].label << " |  " << tmp_cloud_OLD.points[i].label << std::endl;
      c = false;
      break;
    }
  }
  
  if (c)
    std::cout << "DEBUG - OUTPUT - OK" << std::endl;
  else
    std::cout << "DEBUG - OUTPUT - ERROR" << std::endl;
*/



/*
  for (size_t i = 0; i < 25; i++)
  {
    std::cout << result[i] << " |  " << resultOLD[i] << std::endl;
  }
  

  c = true;
  for (size_t i = 0; i < result.size (); i++)
  {
    if (result[i] != resultOLD[i])
    {
      std::cout << result[i] << " |  " << resultOLD[i] << std::endl;
      
      c = false;
      break;
    }
  }
  
  if (c)
    std::cout << "DEBUG - OUTPUT - OK" << std::endl;
  else
    std::cout << "DEBUG - OUTPUT - ERROR" << std::endl;
*/


}

#define PCL_INSTANTIATE_CrfSegmentation(T) template class pcl::CrfSegmentation<T>;

#endif    // PCL_CRF_SEGMENTATION_HPP_
