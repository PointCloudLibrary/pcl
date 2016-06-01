/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Copyright (C) 2008 Ben Gurion University of the Negev, Beer Sheva, Israel.
 *
 *  All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met
 *
 *   * The use for research only (no for any commercial application).
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

#ifndef PCL_REGISTRATION_IMPL_IA_RELOC_H_
#define PCL_REGISTRATION_IMPL_IA_RELOC_H_

#include <pcl/registration/ia_ReLOC.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/random_sample.h>

#include <pcl/features/flare.h>
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/keypoints/flat_keypoint.h>

template <typename PointInT> void
  pcl::computeBBoxFromStddev(const pcl::PointCloud<PointInT> &cloud, std::vector<double> &bbox, const float sigmaFactor)
{
	size_t nPoints = cloud.size();

	std::vector<float> vXs(nPoints);
	std::vector<float> vYs(nPoints);
	std::vector<float> vZs(nPoints);
	for(int po=0; po<cloud.size(); po++)
	{
    vXs[po] = cloud[po].x;
    vYs[po] = cloud[po].y;
    vZs[po] = cloud[po].z;
	}

	//find sigma for each dimension
	double sigma_X;
	double sigma_Y;
  double sigma_Z;
  double mean_X;
  double mean_Y;
  double mean_Z;
  pcl::getMeanStd (vXs, mean_X, sigma_X);
  pcl::getMeanStd (vYs, mean_Y, sigma_Y);
  pcl::getMeanStd (vZs, mean_Z, sigma_Z);

  bbox.resize(6);
	bbox[0] = mean_X - sigma_X * sigmaFactor;
	bbox[1] = mean_X + sigma_X * sigmaFactor;
	bbox[2] = mean_Y - sigma_Y * sigmaFactor;
	bbox[3] = mean_Y + sigma_Y * sigmaFactor;
	bbox[4] = mean_Z - sigma_Z * sigmaFactor;
	bbox[5] = mean_Z + sigma_Z * sigmaFactor;
}


	template<typename T> void 
  pcl::extendBBox(T* bbox, const T factor_X, const T factor_Y, const T factor_Z)
	{
		T size_X = bbox[1] - bbox[0];
		T size_Y = bbox[3] - bbox[2];
		T size_Z = bbox[5] - bbox[4];

		T ext2_X = (size_X * factor_X - size_X)/2.0;
		T ext2_Y = (size_Y * factor_Y - size_Y)/2.0;
		T ext2_Z = (size_Z * factor_Z - size_Z)/2.0;

		bbox[0] = bbox[0] - ext2_X;
		bbox[1] = bbox[1] + ext2_X;
		bbox[2] = bbox[2] - ext2_Y;
		bbox[3] = bbox[3] + ext2_Y;
		bbox[4] = bbox[4] - ext2_Z;
		bbox[5] = bbox[5] + ext2_Z;
	}



///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar>
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::ReLOCInitialAlignment () :
  source_keypoints_ (new pcl::PointCloud<PointSource> ()),
  target_keypoints_ (new pcl::PointCloud<PointTarget> ()),
  source_lrf_cloud_ (new ReferenceFrames ()),
  target_lrf_cloud_ (new ReferenceFrames ()),
  source_flare_signed_distances_ (),
  target_flare_signed_distances_ (),
  source_normals_ (),
  target_normals_ (),
  seed_ (static_cast<unsigned int> (time (NULL))),
  use_random_detector_(false),
  flat_keypoint_min_neighbors_ (5),
  flat_keypoint_Rf_ (0.0),
  flat_keypoint_R_discard_ (0.0),
  flat_keypoint_R1_search_ (0.0),
  flat_keypoint_R2_search_ (0.0),
  flat_keypoint_T1_search_ (0.9),
  flat_keypoint_T2_search_ (0.9),
  n_random_keypoints_ (2000),
  flare_normal_radius_ (0.0f),
  flare_tangent_radius_ (0.0f),
  flare_margin_thresh_ (0.85f),
  flare_min_neighbors_for_normal_axis_ (6),
  flare_min_neighbors_for_tangent_axis_ (6),
  flare_x_support_sampling_perc_ (1.0f),
  matcher_TD_ (0.01f),
  hough_f_ (2.8), //1.4 in the paper
  hough_Sbin_ (0.0),
  ransac_T_ (0.0),
  ransac_N_ (1000)
{
  reg_name_ = "pcl::registration::ReLOCInitialAlignment";
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar>
template <typename PointT> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::detectKeypoints (
  typename pcl::PointCloud<PointT>::ConstPtr cloud,
  NormalsConstPtr normals,
  typename pcl::PointCloud<PointT>::Ptr &keypoints)
{

  if(use_random_detector_)
  {
    //Random detector
    pcl::RandomSample<PointT> random_sampling;
    random_sampling.setSeed(seed_);
    random_sampling.setSample(n_random_keypoints_);
    random_sampling.setInputCloud (cloud);
    random_sampling.filter (*keypoints);
  }
  else
  {
    //FlatKeypoint detector
    pcl::FlatKeypoint<PointT, PointT, NormalT> flat_keypoint;
    flat_keypoint.setSeed(seed_);
    flat_keypoint.setRadiusSearch (flat_keypoint_Rf_);
    flat_keypoint.setRdiscard (flat_keypoint_R_discard_);
    flat_keypoint.setR1search (flat_keypoint_R1_search_);
    flat_keypoint.setR2search (flat_keypoint_R2_search_);
    flat_keypoint.setT1search (flat_keypoint_T1_search_);
    flat_keypoint.setT2search (flat_keypoint_T2_search_);
    flat_keypoint.setMinNeighbors (flat_keypoint_min_neighbors_);
    flat_keypoint.setInputCloud (cloud);
    flat_keypoint.setNormals (normals);
    flat_keypoint.compute (*keypoints);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar>
template <typename PointT> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::computeFlares (
  typename pcl::PointCloud<PointT>::ConstPtr cloud,
  NormalsConstPtr normals,
  typename pcl::PointCloud<PointT>::ConstPtr keypoints,
  ReferenceFramesPtr &flares,
  std::vector<float> &flare_signed_distances
  )
{

  const float samplingIncr = 1.0f/flare_x_support_sampling_perc_;

  FLARELocalReferenceFrameEstimation<PointT, NormalT, ReferenceFrame> flare_descriptor;
  flare_descriptor.setRadiusSearch (flare_normal_radius_);
  flare_descriptor.setTangentRadius (flare_tangent_radius_);
  flare_descriptor.setMarginThresh (flare_margin_thresh_);
  flare_descriptor.setMinNeighboursForNormalAxis (flare_min_neighbors_for_normal_axis_);
  flare_descriptor.setMinNeighboursForTangentAxis (flare_min_neighbors_for_tangent_axis_);
  flare_descriptor.setInputCloud (keypoints);
  flare_descriptor.setSearchSurface (cloud);
  flare_descriptor.setInputNormals (normals);
  
  if(flare_x_support_sampling_perc_ != 1.0)
  {
    //create and set sampled point cloud for computation of X axis
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    PointCloudPtr sampled_cloud;
    typedef typename search::KdTree<PointT>::Ptr KdTreePtr;
    KdTreePtr sampled_tree;
    sampled_cloud.reset( new pcl::PointCloud<PointT>() );
    std::vector<int> indices_sampled;
    for(float sa = 0.0f; sa < (float)cloud->points.size (); sa += samplingIncr)
	    indices_sampled.push_back(static_cast<int> (sa) );
    copyPointCloud(*cloud, indices_sampled, *sampled_cloud);
    sampled_tree.reset (new search::KdTree<PointT> (false));
    sampled_tree->setInputCloud (sampled_cloud);
    flare_descriptor.setSearchMethodForSampledSurface (sampled_tree);
    flare_descriptor.setSearchSampledSurface (sampled_cloud);
  }

  flare_descriptor.compute (*flares);
  flare_signed_distances = flare_descriptor.getSignedDistancesFromHighestPoints();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::match (
  const std::vector<float> &target_flare_signed_distances,
  const std::vector<float> &source_flare_signed_distances,
  pcl::CorrespondencesPtr &correspondences)
{

  //sort source signed distances
  std::vector<float> sorted_source_flare_signed_distances = source_flare_signed_distances;
  size_t n_source_flare_signed_distances = sorted_source_flare_signed_distances.size();
  

	//find sorting ids w.r.t. descriptor first feature
	std::vector<int> sorted_source_flare_signed_distances_ids(sorted_source_flare_signed_distances.size());
	for(int id=0; id<sorted_source_flare_signed_distances_ids.size(); id++)
	{
		sorted_source_flare_signed_distances_ids[id] = id; 
	}

	//sort features w.r.t. first feature
	std::sort(sorted_source_flare_signed_distances_ids.begin(), sorted_source_flare_signed_distances_ids.end(), pcl::SortIdsWrtFloatDecr(sorted_source_flare_signed_distances) );	

	//sort w.r.t. indeces
  for(int di=0; di<n_source_flare_signed_distances; di++)
  {
    sorted_source_flare_signed_distances[di] = source_flare_signed_distances[ sorted_source_flare_signed_distances_ids[di] ];
  }



  //sort target signed distances
  std::vector<float> sorted_target_flare_signed_distances = target_flare_signed_distances;
  size_t n_target_flare_signed_distances = sorted_target_flare_signed_distances.size();
  

	//find sorting ids w.r.t. descriptor first feature
	std::vector<int> sorted_target_flare_signed_distances_ids(sorted_target_flare_signed_distances.size());
	for(int id=0; id<sorted_target_flare_signed_distances_ids.size(); id++)
	{
		sorted_target_flare_signed_distances_ids[id] = id; 
	}

	//sort features w.r.t. first feature
	std::sort(sorted_target_flare_signed_distances_ids.begin(), sorted_target_flare_signed_distances_ids.end(), pcl::SortIdsWrtFloatDecr(sorted_target_flare_signed_distances) );	

	//sort w.r.t. indeces
  for(int di=0; di<n_target_flare_signed_distances; di++)
  {
    sorted_target_flare_signed_distances[di] = target_flare_signed_distances[ sorted_target_flare_signed_distances_ids[di] ];
  }

	float* sorted_target_flare_signed_distances_ptr = &sorted_target_flare_signed_distances[0];
  float* sorted_source_flare_signed_distances_ptr = &sorted_source_flare_signed_distances[0];
	
  

	//find maxDiff_source_target
	float max_signed_dist_trg, max_signed_dist_src, min_signed_dist_trg, min_signed_dist_src;

	int i_desc = 0;
	while( (max_signed_dist_trg = *(sorted_target_flare_signed_distances_ptr++) ) == std::numeric_limits<float>::max() )
	{
		i_desc++;
	}
	if(i_desc == n_target_flare_signed_distances)
	{
		//no valid signed distances in trg
    correspondences->clear();
		return;
	}

	i_desc = 0;
	while( (max_signed_dist_src = *(sorted_source_flare_signed_distances_ptr++) ) == std::numeric_limits<float>::max() )
	{
		i_desc++;
	}
	if(i_desc == n_source_flare_signed_distances)
	{
		//no valid signed distances in src
    correspondences->clear();
		return;
	}

	sorted_target_flare_signed_distances_ptr = &sorted_target_flare_signed_distances[0] + n_target_flare_signed_distances - 1;
	while( (min_signed_dist_trg = *(sorted_target_flare_signed_distances_ptr--) ) == std::numeric_limits<float>::max() ) {}

	sorted_source_flare_signed_distances_ptr = &sorted_source_flare_signed_distances[0] + n_source_flare_signed_distances - 1;
	while( (min_signed_dist_src = *(sorted_source_flare_signed_distances_ptr--) ) == std::numeric_limits<float>::max() ) {}

	float max_diff_src_trg = std::max(fabs(max_signed_dist_trg - min_signed_dist_src), fabs(max_signed_dist_src - min_signed_dist_trg));


	float delta = max_diff_src_trg * matcher_TD_;

	


	
	//first scan to find num correct matches
  float min_delta, max_delta;
	int n_good_matches = 0;
	int fe1 = 0;
	bool end_scan = false;

	//skip max values
	sorted_target_flare_signed_distances_ptr = &sorted_target_flare_signed_distances[0];
	int fe0_start = 0;
	while( *(sorted_target_flare_signed_distances_ptr++)  == std::numeric_limits<float>::max() ) {fe0_start++;}
	sorted_target_flare_signed_distances_ptr--;
	

	//skip max values
	int i_min_delta = 0;
	while( sorted_source_flare_signed_distances[i_min_delta++] == std::numeric_limits<float>::max() ) {}
	i_min_delta--;


	for(int fe0 = fe0_start; fe0 < n_target_flare_signed_distances; fe0++)
	{
		min_delta = *sorted_target_flare_signed_distances_ptr + delta;
		max_delta = *sorted_target_flare_signed_distances_ptr - delta;

		while(sorted_source_flare_signed_distances[i_min_delta] > min_delta)
		{
			i_min_delta++;
			if(i_min_delta == n_source_flare_signed_distances)
			{
				end_scan = true;
				break;
			}
		}
		if(end_scan)
		{
			break;
		}
		fe1 = i_min_delta;
		while( (fe1<n_source_flare_signed_distances) && (sorted_source_flare_signed_distances[fe1] > max_delta) )
		{
			n_good_matches++;
			fe1++;
		}
		sorted_target_flare_signed_distances_ptr++;
	}

	if(n_good_matches == 0)
	{
    correspondences->clear();
		return;
	}

	//resize matches
	correspondences->resize(n_good_matches);


	//second scan to populate correspondences
	pcl::Correspondence* matches_ptr = &(*correspondences)[0];
	fe1 = 0;
	end_scan = false;

	//skip max values
	sorted_target_flare_signed_distances_ptr = &sorted_target_flare_signed_distances[0];
	fe0_start = 0;
	while( *(sorted_target_flare_signed_distances_ptr++)  == std::numeric_limits<float>::max() ) {fe0_start++;}
	sorted_target_flare_signed_distances_ptr--;
	

	//skip max values
	i_min_delta = 0;
	while( sorted_source_flare_signed_distances[i_min_delta++] == std::numeric_limits<float>::max() ) {}
	i_min_delta--;

	for(int fe0 = fe0_start; fe0 < n_target_flare_signed_distances; fe0++)
	{
		min_delta = *sorted_target_flare_signed_distances_ptr + delta;
		max_delta = *sorted_target_flare_signed_distances_ptr - delta;

		while(sorted_source_flare_signed_distances[i_min_delta] > min_delta)
		{
			i_min_delta++;
			if(i_min_delta == n_source_flare_signed_distances)
			{
				end_scan = true;
				break;
			}
		}
		if(end_scan)
		{
			break;
		}
		fe1 = i_min_delta;
		while( (fe1<n_source_flare_signed_distances) && (sorted_source_flare_signed_distances[fe1] > max_delta) )
		{
      //(*matches_ptr).weight = 1.0f - fabs(*sorted_target_flare_signed_distances_ptr - sorted_source_flare_signed_distances[fe1] )/max_diff_src_trg;
      (*matches_ptr).index_match = fe0;
      (*matches_ptr).index_query = fe1;

      matches_ptr++;
			fe1++;
		}
		sorted_target_flare_signed_distances_ptr++;
	}

  //find correspondences w.r.t. original unordered signed distances
	for(int ma=0; ma<correspondences->size(); ma++)
	{
    (*correspondences)[ma].index_query = sorted_source_flare_signed_distances_ids[ (*correspondences)[ma].index_query ];
    (*correspondences)[ma].index_match = sorted_target_flare_signed_distances_ids[ (*correspondences)[ma].index_match ];
	}

}

template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::houghVoting (
  pcl::CorrespondencesConstPtr matcher_correspondences,
  pcl::CorrespondencesPtr &hough_correspondences)
{

  PointCloudSourceConstPtr &source_ = input_;

  
  if(source_cloud_updated_)
  {
    // compute source centroid
    Eigen::Vector4f source_centroid_4f;
    pcl::compute3DCentroid<PointSource>(*source_, source_centroid_4f);
    Eigen::Vector3f source_centroid = source_centroid_4f.head<3>();

    // compute source votes
    source_votes_.resize (source_keypoints_->size ());
    for (size_t i = 0; i < source_keypoints_->size (); ++i)
    {
      Eigen::Vector3f x_ax ((*source_lrf_cloud_)[i].x_axis[0], (*source_lrf_cloud_)[i].x_axis[1], (*source_lrf_cloud_)[i].x_axis[2]);
      Eigen::Vector3f y_ax ((*source_lrf_cloud_)[i].y_axis[0], (*source_lrf_cloud_)[i].y_axis[1], (*source_lrf_cloud_)[i].y_axis[2]);
      Eigen::Vector3f z_ax ((*source_lrf_cloud_)[i].z_axis[0], (*source_lrf_cloud_)[i].z_axis[1], (*source_lrf_cloud_)[i].z_axis[2]);

      source_votes_[i].x () = x_ax.dot (source_centroid - source_keypoints_->at (i).getVector3fMap ());
      source_votes_[i].y () = y_ax.dot (source_centroid - source_keypoints_->at (i).getVector3fMap ());
      source_votes_[i].z () = z_ax.dot (source_centroid - source_keypoints_->at (i).getVector3fMap ());
    }
  }


  if(target_cloud_updated_)
  {
    //allocate hough space
    Eigen::Vector3d d_min, d_max, bin_size;

    d_min.setConstant (std::numeric_limits<double>::max ());
    d_max.setConstant (-std::numeric_limits<double>::max ());
    bin_size.setConstant (hough_Sbin_);


    //determine extension of hough space
    std::vector<double> bbox;
    pcl::computeBBoxFromStddev<PointTarget>(*target_, bbox);
    pcl::extendBBox(&bbox[0], hough_f_, hough_f_, hough_f_);
    d_min.x() = bbox[0];
    d_min.y() = bbox[2];
    d_min.z() = bbox[4];

    d_max.x() = bbox[1];
    d_max.y() = bbox[3];
    d_max.z() = bbox[5];

    hough_space_.reset (new pcl::HoughSpace3D<unsigned short int> (d_min, bin_size, d_max));
  }
  else
  {
    hough_space_->reset();
  }

  


  //cast votes
  for (size_t ma = 0; ma < matcher_correspondences->size(); ++ma)
  {
    Eigen::Vector3d target_vote;


    int target_index = matcher_correspondences->at (ma).index_match;
    int source_index = matcher_correspondences->at (ma).index_query;

    const Eigen::Vector3f& source_point_vote = source_votes_[source_index];

    const Eigen::Vector3f& target_point = target_keypoints_->at (target_index).getVector3fMap ();
    const ReferenceFrame&   target_point_rf = target_lrf_cloud_->at (target_index);
    
    Eigen::Vector3f target_point_rf_x (target_point_rf.x_axis[0], target_point_rf.x_axis[1], target_point_rf.x_axis[2]);
    Eigen::Vector3f target_point_rf_y (target_point_rf.y_axis[0], target_point_rf.y_axis[1], target_point_rf.y_axis[2]);
    Eigen::Vector3f target_point_rf_z (target_point_rf.z_axis[0], target_point_rf.z_axis[1], target_point_rf.z_axis[2]);


    target_vote.x () = target_point_rf_x[0] * source_point_vote.x () + target_point_rf_y[0] * source_point_vote.y () + target_point_rf_z[0] * source_point_vote.z () + target_point.x ();
    target_vote.y () = target_point_rf_x[1] * source_point_vote.x () + target_point_rf_y[1] * source_point_vote.y () + target_point_rf_z[1] * source_point_vote.z () + target_point.y ();
    target_vote.z () = target_point_rf_x[2] * source_point_vote.x () + target_point_rf_y[2] * source_point_vote.y () + target_point_rf_z[2] * source_point_vote.z () + target_point.z ();


    hough_space_->vote (target_vote, 1, (int)ma);
  } 

  // Finding max bins and voters
  std::vector<int> max_ids;
  hough_space_->findMaximum (max_ids);
  
  hough_correspondences.reset (new pcl::Correspondences (max_ids.size()));
  
    for (size_t co = 0; co < max_ids.size (); ++co)
    {
	    hough_correspondences->at (co) = matcher_correspondences->at (max_ids[co]);
    } 

}

template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::estimateRigidMotion (
  pcl::CorrespondencesConstPtr hough_correspondences,
  Eigen::Matrix4f &final_transformation,
  pcl::CorrespondencesPtr &ransac_correspondences)
{

  CorrespondenceRejectorSampleConsensus<PointSource> ransac;
  ransac.setInputSource (source_keypoints_);
  ransac.setInputTarget (target_keypoints_);
  ransac.setInlierThreshold (ransac_T_);
  ransac.setMaximumIterations (ransac_N_);
  ransac.setRefineModel (false);
  ransac.setInputCorrespondences (hough_correspondences);
  ransac.getCorrespondences (*ransac_correspondences);
  final_transformation_ = ransac.getBestTransformation();
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::computeTransformation (
  PointCloudSource &output,
  const Eigen::Matrix4f &guess)
{

  final_transformation_ = guess;

  PointCloudSourceConstPtr &source_ = input_;

  //Detection
  if(target_cloud_updated_)
  {
    std::cout << "Extract target keypoints..." << std::endl;
    detectKeypoints<PointTarget> (target_, target_normals_, target_keypoints_);
  }
  std::cout << "Target total points: " << target_->size () << "; Selected Keypoints: " << target_keypoints_->size () << std::endl;

  if(source_cloud_updated_)
  {
    std::cout << "Extract source keypoints..." << std::endl;
    detectKeypoints<PointSource> (source_, source_normals_, source_keypoints_);
  }
  std::cout << "Source total points: " << source_->size () << "; Selected Keypoints: " << source_keypoints_->size () << std::endl;

  //FLARE local reference frames computation
  if(target_cloud_updated_)
  {
    std::cout << "Compute target FLAREs...";
    computeFlares<PointTarget> (target_, target_normals_, target_keypoints_, target_lrf_cloud_, target_flare_signed_distances_);
  }
  std::cout << "target FLAREs computed" << std::endl;

  if(source_cloud_updated_)
  {
    std::cout << "Compute source FLAREs...";
    computeFlares<PointSource> (source_, source_normals_, source_keypoints_, source_lrf_cloud_, source_flare_signed_distances_); 
  }
  std::cout << "source FLAREs computed" << std::endl;


  //Matching
  std::cout << "Perform matching...";
  pcl::CorrespondencesPtr matcher_correspondences (new pcl::Correspondences ());
  match (target_flare_signed_distances_, source_flare_signed_distances_, matcher_correspondences); 
  std::cout << "Matcher correspondences: " << matcher_correspondences->size () << std::endl;
	if(matcher_correspondences->size () == 0)
	{
  	std::cout << "No matches" << std::endl;
		return;
	}	
  
  //Hough voting
  std::cout << "Perform hough voting...";
  pcl::CorrespondencesPtr hough_correspondences;
  houghVoting (matcher_correspondences, hough_correspondences);
  std::cout << "Hough correspondences: " << hough_correspondences->size () << std::endl;
  if( hough_correspondences->size () == 0 )
  {
    std::cout << "No matches after Hough voting" << std::endl;
    return;
  }


  //Ransac
  std::cout << "Estimate rigid motion through RANSAC+Horn...";
  pcl::CorrespondencesPtr ransac_correspondences (new pcl::Correspondences());
  estimateRigidMotion (hough_correspondences, final_transformation_, ransac_correspondences);
  std::cout << "Ransac correspondences: " << ransac_correspondences->size () << std::endl;
	if( ransac_correspondences->size () < 3 )
	{
		std::cout << "RANSAC failed: No consensus set" << std::endl;
		return ;
	}	


  // apply the final transformation
  pcl::transformPointCloud (*source_, output, final_transformation_);


  deinitCompute ();
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> bool
pcl::registration::ReLOCInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::initCompute ()
{
  // basic pcl initialization
  if (!pcl::PCLBase <PointSource>::initCompute ())
    return (false);

  std::srand (seed_);

  // check if source and target are given
  if (!input_ || !target_)
  {
    PCL_ERROR ("[%s::initCompute] Source or target dataset not given!\n", reg_name_.c_str ());
    return (false);
  }

  if (source_normals_->empty ())
  {
     PCL_ERROR ("[pcl::%s::initCompute] source_normals_ not set!\n", reg_name_.c_str ());
    return (false);   
  }

  if (source_normals_->size () != input_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] source_normals_ given, but the number of normals does not match the number of source points!\n", reg_name_.c_str ());
    return (false);
  }

  if (target_normals_->empty ())
  {
     PCL_ERROR ("[pcl::%s::initCompute] target_normals_ not set!\n", reg_name_.c_str ());
    return (false);   
  }

  if (target_normals_->size () != target_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] target_normals_ given, but the number of normals does not match the number of target points!\n", reg_name_.c_str ());
    return (false);
  }


  if(flat_keypoint_Rf_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flat_keypoint_Rf_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if(flat_keypoint_R_discard_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flat_keypoint_R_discard_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if(flat_keypoint_R1_search_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flat_keypoint_R1_search_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if(flat_keypoint_R2_search_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flat_keypoint_R2_search_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if(flare_normal_radius_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flare_normal_radius_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if(flare_tangent_radius_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] flare_tangent_radius_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if(hough_Sbin_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] hough_Sbin_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  if(ransac_T_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] ransac_T_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> bool
pcl::registration::ReLOCInitialAlignment<PointSource, PointTarget, NormalT, Scalar>::deinitCompute ()
{
  source_cloud_updated_ = false;
  target_cloud_updated_ = false;

  return (Registration<PointSource, PointTarget, Scalar>::deinitCompute ());
}

/////////////////////////////////////////////////////////////////////////////////////////

#define PCL_INSTANTIATE_ReLOCInitialAlignment(T,U,N,S) template class PCL_EXPORTS pcl::registration::ReLOCInitialAlignment<T,U,N,S>;

#endif // PCL_REGISTRATION_IMPL_IA_RELOC_H_