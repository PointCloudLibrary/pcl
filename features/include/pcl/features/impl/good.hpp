/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2017-, Open Perception, Inc.
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
 */

#ifndef PCL_FEATURES_IMPL_GOOD_H_
#define PCL_FEATURES_IMPL_GOOD_H_
#include <pcl/features/good.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_types.h>


#include <numeric> ///std::accumulate

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, int BinN> void
pcl::GOODEstimation<PointInT, BinN>::projectPointCloudToPlane (const PointCloudInConstPtr &pc_in, const pcl::ModelCoefficients::Ptr &coefficients, PointCloudIn &pc_out)
{
  ///Create the projection object
  pcl::ProjectInliers<PointInT> projection;
  projection.setModelType (pcl::SACMODEL_NORMAL_PLANE); 
  projection.setInputCloud (pc_in);
  projection.setModelCoefficients (coefficients);
  projection.filter (pc_out);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, int BinN> std::vector<unsigned>
pcl::GOODEstimation<PointInT, BinN>::convert2DHistogramTo1DHistogram (const std::vector<std::vector<unsigned> > &histogram_2D)
{
  std::vector<unsigned> histogram (BinN * BinN);
  for (size_t i = 0, j = 0; i < BinN; ++i, j += BinN)
    std::copy (histogram_2D[i].begin (), histogram_2D[i].end (), &histogram[j]);

  return histogram;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, int BinN> int8_t 
pcl::GOODEstimation<PointInT, BinN>::signDisambiguation ( const PointCloudIn &projected_view,  const int8_t axis) const
{
  int8_t sign = 1;
  unsigned int positive = 0; 
  unsigned int negative = 0; 
  const unsigned int threshold_overall = std::max (1u, (unsigned int) trunc (projected_view.size ()/10));

  for (size_t i = 0; i < projected_view.size(); ++i)
  { 
    if (projected_view.at(i).data[axis] > threshold_) 
      ++positive;
    else if (projected_view.at(i).data[axis] < -threshold_)
      ++negative;
  }
  sign = ((int) (negative - positive) >= (int) threshold_overall)? -1 : 1;
  return (sign);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, int BinN> void 
pcl::GOODEstimation<PointInT, BinN>::create2DHistogramFromProjection (const PointCloudInPtr &projected_view, const float largest_side, 
                                                                      const int8_t axis_a, const int8_t axis_b, std::vector<std::vector<unsigned int> > &histogram) const
{  
  const float half = .5f * largest_side; 
  const float interval = largest_side / (float) BinN; 
  
  for (size_t i = 0; i < projected_view->size (); ++i)
  {
    const float a = sign_ *  projected_view->at (i).data[axis_a] + half;
    float b ;
    (axis_b == Y)? b = sign_ * projected_view->at (i).data[axis_b] + half : b = projected_view->at (i).data[axis_b] + half ;
    
    const int idx_o = (int) trunc (a / interval); //outer index
    const int idx_i = (int) trunc (b / interval); //inner index
    if ((idx_o < BinN)
          and (idx_i < BinN)
          and (idx_o >= 0)
          and (idx_i >= 0))
      ++histogram[idx_o][idx_i];
  }  
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, int BinN> void
pcl::GOODEstimation<PointInT, BinN>::normalizeHistogram (const std::vector<unsigned int> histogram, std::vector<float> &normalized_histogram)
{
  ///compute sumation of all histogram's bins. 
  float sum_all_bins = std::accumulate (histogram.begin (), histogram.end (), 0);
  
  if (sum_all_bins != 0)
  { 
    size_t idx = 0; 
    for(std::vector<unsigned int>::const_iterator it = histogram.begin(); it != histogram.end(); ++it)
    {
      normalized_histogram.at (idx) = (*it / sum_all_bins);
      idx++;
    }  
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, int BinN> float
pcl::GOODEstimation<PointInT, BinN>::viewpointEntropy (const std::vector<float> &normalized_histogram)
{
  ///NOTE: http://stats.stackexchange.com/questions/66108/why-is-entropy-maximised-when-the-probability-distribution-is-uniform
  float entropy =0;  
  for(std::vector<float>::const_iterator it = normalized_histogram.begin(); it != normalized_histogram.end(); ++it)
  {
    if (*it != 0)
    {
      float entropy_tmp = *it * log2 (*it);
      entropy += entropy_tmp;
    }
  }  
  return (-entropy); 
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, int BinN> int 
pcl::GOODEstimation<PointInT, BinN>::findMaxViewPointEntropy (const std::vector<float> &view_point_entropy)
{
  int index = 0;
  std::vector<float>::const_iterator it;
  it = std::max_element (view_point_entropy.begin(), view_point_entropy.end());
  index = it - view_point_entropy.begin ();
  return (index);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, int BinN> float
pcl::GOODEstimation<PointInT, BinN>::varianceOfHistogram (const std::vector<float> &histogram)
{
  ///NOTE: https://people.richland.edu/james/lecture/m170/ch06-prb.html
  ///NOTE: http://www.stat.yale.edu/Courses/1997-98/101/rvmnvar.htm  
  
  float mean = 0;
  float variance = 0;
  for (size_t i = 0; i < histogram.size(); i++)
  {
    mean += (i+1) * histogram.at(i);
  }
  
  for (size_t i = 0; i < histogram.size(); i++)
  {
    variance += std::pow ((i+1)-mean , 2) * histogram.at(i);
  }  
  return (variance);  
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, int BinN> void
pcl::GOODEstimation<PointInT, BinN>::objectViewHistogram (int maximum_entropy_index, 
                                                          const std::vector<std::vector<float> > &normalized_projected_views,
                                                          std::vector<float> &sorted_normalized_projected_views, 
                                                          std::string &name_of_sorted_projected_plane)
{
  float variance1 = 0;
  float variance2 = 0;

  sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(), normalized_projected_views.at(maximum_entropy_index).begin(), 
                                            normalized_projected_views.at(maximum_entropy_index).end());    
   
  switch (maximum_entropy_index)
  {
    case 0 :
      
      name_of_sorted_projected_plane += "YoZ - ";      
      variance1 = varianceOfHistogram (normalized_projected_views.at(1));      
      variance2 = varianceOfHistogram (normalized_projected_views.at(2));
      
      if (variance1 <= variance2)
      {
        name_of_sorted_projected_plane += "XoZ - XoY ";
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(), normalized_projected_views.at(1).begin(),normalized_projected_views.at(1).end());	
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(), normalized_projected_views.at(2).begin(),normalized_projected_views.at(2).end());
      }
      else
      {
        name_of_sorted_projected_plane += "XoY - XoZ ";
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(), normalized_projected_views.at(2).begin(),normalized_projected_views.at(2).end());
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(), normalized_projected_views.at(1).begin(),normalized_projected_views.at(1).end());
      }
      break;
      
    case 1 :
      name_of_sorted_projected_plane += "XoZ - ";            
      variance1 = varianceOfHistogram (normalized_projected_views.at(0));      
      variance2 = varianceOfHistogram (normalized_projected_views.at(2));
            
      if (variance1 <= variance2)
      {
        name_of_sorted_projected_plane += "YoZ - XoY ";
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(),normalized_projected_views.at(0).begin(),normalized_projected_views.at(0).end());
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(),normalized_projected_views.at(2).begin(),normalized_projected_views.at(2).end());
      }
      else
      {
        name_of_sorted_projected_plane += "XoY - YoZ ";
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(), normalized_projected_views.at(2).begin(),normalized_projected_views.at(2).end());
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(), normalized_projected_views.at(0).begin(),normalized_projected_views.at(0).end());
      }
      break;

    case 2 :
      name_of_sorted_projected_plane += "XoY - ";		            
      variance1 = varianceOfHistogram (normalized_projected_views.at(0));
      variance2 = varianceOfHistogram (normalized_projected_views.at(1));

      if (variance1 <= variance2)
      {
        name_of_sorted_projected_plane += "YoZ - XoZ ";
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(),normalized_projected_views.at(0).begin(),normalized_projected_views.at(0).end());
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(),normalized_projected_views.at(1).begin(),normalized_projected_views.at(1).end());
      }
      else
      {
        name_of_sorted_projected_plane += "XoZ - YoZ ";
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(), normalized_projected_views.at(1).begin(), normalized_projected_views.at(1).end());
        sorted_normalized_projected_views.insert (sorted_normalized_projected_views.end(), normalized_projected_views.at(0).begin(), normalized_projected_views.at(0).end());
      }
      break;  
      
    default:
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, int BinN> void
pcl::GOODEstimation<PointInT, BinN>::computeFeature (PointCloudOut &output )	
{
  std::vector<float> object_description;
  float largest_side = 0;
  std::vector <float> view_point_entropy (3);
  PointCloudInPtr initial_cloud_projection_along_x_axis (new pcl::PointCloud<PointInT>);//Declare a boost share ptr to the pointCloud
  PointCloudInPtr initial_cloud_projection_along_y_axis (new pcl::PointCloud<PointInT>);//Declare a boost share ptr to the pointCloud
  PointCloudInPtr initial_cloud_projection_along_z_axis (new pcl::PointCloud<PointInT>);//Declare a boost share ptr to the pointCloud
  pcl::PointXYZ pt; 
  
  /* __________________________
  |                            |
  | construct ORF based on PCA |
  |____________________________| */
  
  /// compute principal directions	  
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*input_, centroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized (*input_, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver (covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
  eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));

  ///sorting eigen vectors based on eigen values
  eigen_vectors.col(0) = eigen_vectors.col(2);
  eigen_vectors.col(2) = eigen_vectors.col(0).cross (eigen_vectors.col(1));  

  /// move the points to the PCA based reference frame
  Eigen::Matrix4f p2w (Eigen::Matrix4f::Identity());
  p2w.block<3,3>(0,0) = eigen_vectors.transpose();
  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
  transformation_ = p2w;
  
  PointCloudInPtr transformed_point_cloud (new pcl::PointCloud<PointInT>);
  pcl::transformPointCloud (*input_, *transformed_point_cloud, p2w);
  transformed_point_cloud_ = transformed_point_cloud;
  ///compute the max, the min and the center of the diagonal (mean_diag)
  PointInT min_pt, max_pt;
  pcl::getMinMax3D (*transformed_point_cloud_, min_pt, max_pt);
  const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());
  
  /// centroid transform
  Eigen::Quaternionf qfinal (eigen_vectors); ///rotation matrix
  Eigen::Vector3f center_of_bbox = eigen_vectors*mean_diag + centroid.head<3>(); // Translation = Rotation * center_diag + (c0, c1, c2)
  center_of_bbox_.x = center_of_bbox(0,0);
  center_of_bbox_.y = center_of_bbox(1,0);
  center_of_bbox_.z = center_of_bbox(2,0);
  
  /* _______________________________
  |                                 |
  | construct three projection view |
  |_________________________________| */
  
  const static float OFFSET_TO_THE_ORIGIN = 0.3;
  
  /// ax+by+cz+d=0, where b=c=d=0, and a=1, or said differently, the YoZ plane.
  pcl::ModelCoefficients::Ptr coefficients_x (new pcl::ModelCoefficients ());
  coefficients_x->values.resize (4);
  coefficients_x->values[0] = 1.0; coefficients_x->values[1] = 0; coefficients_x->values[2] = 0; coefficients_x->values[3] = 0;
  projectPointCloudToPlane (transformed_point_cloud_, coefficients_x, *initial_cloud_projection_along_x_axis);
  for (size_t i = 0; i < initial_cloud_projection_along_x_axis->points.size(); i++)
  {
      initial_cloud_projection_along_x_axis->points.at(i).x = OFFSET_TO_THE_ORIGIN;
  }
  vector_of_projected_views_.push_back (initial_cloud_projection_along_x_axis);
 
  ///ax+by+cz+d=0, where a=c=d=0, and b=1, or said differently, the XoZ plane.
  pcl::ModelCoefficients::Ptr coefficients_y (new pcl::ModelCoefficients ());
  coefficients_y->values.resize (4);
  coefficients_y->values[0] = 0.0; coefficients_y->values[1] = 1.0; coefficients_y->values[2] = 0; coefficients_y->values[3] = 0;

  projectPointCloudToPlane (transformed_point_cloud_, coefficients_y, *initial_cloud_projection_along_y_axis);
  for (size_t i = 0; i < initial_cloud_projection_along_y_axis->points.size(); i++)
  {
      initial_cloud_projection_along_y_axis->points.at(i).y = OFFSET_TO_THE_ORIGIN;
  }
  vector_of_projected_views_.push_back (initial_cloud_projection_along_y_axis);
    
  /// ax+by+cz+d=0, where a=b=d=0, and c=1, or said differently, the XoY plane.
  pcl::ModelCoefficients::Ptr coefficients_z (new pcl::ModelCoefficients ());
  coefficients_z->values.resize (4); 
  coefficients_z->values[0] = 0; coefficients_z->values[1] = 0; coefficients_z->values[2] = 1.0;   coefficients_z->values[3] = 0;
  projectPointCloudToPlane (transformed_point_cloud_, coefficients_z, *initial_cloud_projection_along_z_axis);
  for (size_t i = 0; i < initial_cloud_projection_along_z_axis->points.size(); i++)
  {
      initial_cloud_projection_along_z_axis->points.at(i).z = OFFSET_TO_THE_ORIGIN;
  }		
  vector_of_projected_views_.push_back (initial_cloud_projection_along_z_axis);

  /// compute BoundingBox Dimensions
  Eigen::Vector4f bbox_dimensions = getAxisAlignedBoundingBox(*transformed_point_cloud_);
  bbox_dimensions_ = bbox_dimensions.head<3>();  
  largest_side = computeLargestSideOfBoundingBox ();		

  /* _________________________
  |                           |
  |  Axes sign disambiguation |
  |___________________________| */

  int8_t sx = 1, sy = 1;
  sx = signDisambiguation (*initial_cloud_projection_along_y_axis, X); ///XoZ Plane
  sy = signDisambiguation (*initial_cloud_projection_along_x_axis, Y); ///YoZ Plane
  sign_= sx * sy;    

  /* _______________________________________________________
  |                                                         |
  |  compute histograms of projection of the given object   |
  |_________________________________________________________| */
      
  ///initialize vectors
  std::vector<unsigned int> complete_object_histogram (3 * BinN * BinN);
  std::vector<float> complete_object_histogram_normalized (3 * BinN * BinN); ///each projection view is normalized sepreatly  
  std::vector<std::vector<unsigned int> > XOY_histogram (BinN, std::vector<unsigned int> (BinN));
  std::vector<std::vector<unsigned int> > XOZ_histogram (BinN, std::vector<unsigned int> (BinN));
  std::vector<std::vector<unsigned int> > YOZ_histogram (BinN, std::vector<unsigned int> (BinN));  
  std::vector<std::vector <float> > normalized_projected_views (3, std::vector <float> (BinN * BinN));

  ///YOZ Projection
  std::vector<unsigned int> histogramYOZ1D (BinN * BinN);
  std::vector<float> normalized_histogramYoZ (BinN * BinN);
  float YoZ_entropy = 0;  
  create2DHistogramFromProjection (initial_cloud_projection_along_x_axis, largest_side, Y, Z, YOZ_histogram);
  histogramYOZ1D = convert2DHistogramTo1DHistogram (YOZ_histogram);
  complete_object_histogram.insert (complete_object_histogram.end(), histogramYOZ1D.begin(), histogramYOZ1D.end());  
  normalizeHistogram (histogramYOZ1D, normalized_histogramYoZ);
  normalized_projected_views.at (0) = normalized_histogramYoZ;  
  complete_object_histogram_normalized.insert (complete_object_histogram_normalized.end(), normalized_histogramYoZ.begin(), normalized_histogramYoZ.end());
  YoZ_entropy = viewpointEntropy (normalized_histogramYoZ);
  view_point_entropy.at (0) = YoZ_entropy;
    
  ///XOZ Projection  
  std::vector<unsigned int> histogramXOZ1D (BinN * BinN);
  std::vector<float> normalized_histogramXoZ (BinN * BinN);
  float XoZ_entropy = 0;
  create2DHistogramFromProjection (initial_cloud_projection_along_y_axis, largest_side, X, Z, XOZ_histogram);
  histogramXOZ1D = convert2DHistogramTo1DHistogram (XOZ_histogram);
  complete_object_histogram.insert (complete_object_histogram.end(), histogramXOZ1D.begin(), histogramXOZ1D.end());
  normalizeHistogram (histogramXOZ1D, normalized_histogramXoZ);
  normalized_projected_views.at (1) = normalized_histogramXoZ;
  complete_object_histogram_normalized.insert (complete_object_histogram_normalized.end(), normalized_histogramXoZ.begin(), normalized_histogramXoZ.end());  
  XoZ_entropy = viewpointEntropy (normalized_histogramXoZ);
  view_point_entropy.at (1) = XoZ_entropy;

  ///XOY Projection
  std::vector<unsigned int> histogramXOY1D (BinN * BinN);
  std::vector<float> normalized_histogramXoY (BinN * BinN);
  float XoY_entropy = 0;  
  create2DHistogramFromProjection (initial_cloud_projection_along_z_axis, largest_side, X, Y, XOY_histogram);     
  histogramXOY1D = convert2DHistogramTo1DHistogram (XOY_histogram);
  complete_object_histogram.insert (complete_object_histogram.end(), histogramXOY1D.begin(), histogramXOY1D.end());  
  normalizeHistogram (histogramXOY1D, normalized_histogramXoY);
  normalized_projected_views.at (2) = normalized_histogramXoY;
  complete_object_histogram_normalized.insert (complete_object_histogram_normalized.end(), normalized_histogramXoY.begin(), normalized_histogramXoY.end());
  XoY_entropy = viewpointEntropy (normalized_histogramXoY);
  view_point_entropy.at (2) = XoY_entropy;
  
  /* ____________________________________
  |                                      |
  |  producing a GOOD shape description  |
  |______________________________________| */

  ///NOTE: The ordering of the three distribution vectors is first by decreasing values of entropy. 
  ///Afterwards the second and third vectors are sorted again by increasing values of variance.
  
  int maximum_entropy_index = findMaxViewPointEntropy (view_point_entropy);   

  std::string name_of_sorted_projected_plane;
  objectViewHistogram (maximum_entropy_index, normalized_projected_views, object_description, name_of_sorted_projected_plane);
  order_of_projected_plane_ =  name_of_sorted_projected_plane;    
  for(size_t i =0; i < object_description.size(); i++)
    output.points[0].histogram[i] = object_description.at(i);
    
}

#define PCL_INSTANTIATE_GOODEstimation(T, x) template class PCL_EXPORTS pcl::GOODEstimation<T, x >;

#endif /* PCL_FEATURES_IMPL_GOOD_H_ */
