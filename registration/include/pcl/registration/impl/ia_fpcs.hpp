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

#ifndef PCL_REGISTRATION_IMPL_IA_FPCS_H_
#define PCL_REGISTRATION_IMPL_IA_FPCS_H_

#include <pcl/registration/ia_fpcs.h>
#include <pcl/common/time.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/registration/transformation_estimation_3point.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline float
pcl::getMeanPointDensity (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, float max_dist, int nr_threads)
{
  const float max_dist_sqr = max_dist * max_dist;
  const std::size_t s = cloud.size ();

  pcl::search::KdTree <PointT> tree;
  tree.setInputCloud (cloud);

  float mean_dist = 0.f;
  int num = 0;
  std::vector <int> ids (2);
  std::vector <float> dists_sqr (2);

#ifdef _OPENMP
#pragma omp parallel for \
  reduction (+:mean_dist, num) \
  private (ids, dists_sqr) shared (tree, cloud) \
  default (none)num_threads (nr_threads)
#endif

  for (int i = 0; i < 1000; i++)
  {
    tree.nearestKSearch (cloud->points[rand () % s], 2, ids, dists_sqr);
    if (dists_sqr[1] < max_dist_sqr)
    {
      mean_dist += std::sqrt (dists_sqr[1]);
      num++;
    }
  }

  return (mean_dist / num);
};


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline float
pcl::getMeanPointDensity (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector <int> &indices,
  float max_dist, int nr_threads)
{
  const float max_dist_sqr = max_dist * max_dist;
  const std::size_t s = indices.size ();

  pcl::search::KdTree <PointT> tree;
  tree.setInputCloud (cloud);

  float mean_dist = 0.f;
  int num = 0;
  std::vector <int> ids (2);
  std::vector <float> dists_sqr (2);

#ifdef _OPENMP
#pragma omp parallel for \
  reduction (+:mean_dist, num) \
  private (ids, dists_sqr) shared (tree, cloud, indices)    \
  default (none)num_threads (nr_threads)
#endif

  for (int i = 0; i < 1000; i++)
  {
    tree.nearestKSearch (cloud->points[indices[rand () % s]], 2, ids, dists_sqr);
    if (dists_sqr[1] < max_dist_sqr)
    {
      mean_dist += std::sqrt (dists_sqr[1]);
      num++;
    }
  }

  return (mean_dist / num);
};


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar>
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::FPCSInitialAlignment () :
  source_normals_ (),
  target_normals_ (),
  nr_threads_ (1),
  approx_overlap_ (0.5f),
  delta_ (1.f),
  score_threshold_ (FLT_MAX),
  nr_samples_ (0),
  max_norm_diff_ (90.f),
  max_runtime_ (0),
  fitness_score_ (FLT_MAX),
  diameter_ (),
  max_base_diameter_sqr_ (),
  use_normals_ (false),
  normalize_delta_ (true),
  max_pair_diff_ (),
  max_edge_diff_ (),
  coincidation_limit_ (),
  max_mse_ (),
  max_inlier_dist_sqr_ (),
  small_error_ (0.00001f)
{
  reg_name_ = "pcl::registration::FPCSInitialAlignment";
  max_iterations_ = 0;
  ransac_iterations_ = 1000;
  transformation_estimation_.reset (new pcl::registration::TransformationEstimation3Point <PointSource, PointTarget>);
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::computeTransformation (
  PointCloudSource &output,
  const Eigen::Matrix4f &guess)
{
  if (!initCompute ())
    return;

  final_transformation_ = guess;
  bool abort = false;
  std::vector <MatchingCandidates> all_candidates (max_iterations_);
  pcl::StopWatch timer;

  #ifdef _OPENMP
  #pragma omp parallel num_threads (nr_threads_)
  #endif
  {
    #ifdef _OPENMP
    std::srand (static_cast <unsigned int> (std::time (NULL)) ^ omp_get_thread_num ());    
    #pragma omp for schedule (dynamic)
    #endif
    for (int i = 0; i < max_iterations_; i++)
    {

      #ifdef _OPENMP
      #pragma omp flush (abort)
      #endif

      MatchingCandidates candidates (1);
      std::vector <int> base_indices (4);
      float ratio[2];
      all_candidates[i] = candidates;

      if (!abort)
      {
        // select four coplanar point base
        if (selectBase (base_indices, ratio) == 0)
        {
          // calculate candidate pair correspondences using diagonal lenghts of base
          pcl::Correspondences pairs_a, pairs_b;
          if (bruteForceCorrespondences (base_indices[0], base_indices[1], pairs_a) == 0 &&
            bruteForceCorrespondences (base_indices[2], base_indices[3], pairs_b) == 0)
          {
            // determine candidate matches by combining pair correspondences based on segment distances
            std::vector <std::vector <int> > matches;
            if (determineBaseMatches (base_indices, matches, pairs_a, pairs_b, ratio) == 0)
            {
              // check and evaluate candidate matches and store them
              handleMatches (base_indices, matches, candidates);
              if (candidates.size () != 0)
                all_candidates[i] = candidates;
            }
          }
        }

        // check terminate early (time or fitness_score threshold reached)
        abort = (candidates.size () > 0 ? candidates[0].fitness_score < score_threshold_ : abort);
        abort = (abort ? abort : timer.getTimeSeconds () > max_runtime_);


        #ifdef _OPENMP
        #pragma omp flush (abort)
        #endif
      }
    }
  }
  

  // determine best match over all trys
  finalCompute (all_candidates);

  // apply the final transformation
  pcl::transformPointCloud (*input_, output, final_transformation_);

  deinitCompute ();
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> bool
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::initCompute ()
{
  std::srand (static_cast <unsigned int> (std::time (NULL)));

  // basic pcl initialization
  if (!pcl::PCLBase <PointSource>::initCompute ())
    return (false);

  // check if source and target are given
  if (!input_ || !target_)
  {
    PCL_ERROR ("[%s::initCompute] Source or target dataset not given!\n", reg_name_.c_str ());
    return (false);
  }

  if (!target_indices_ || target_indices_->size () == 0)
  {
    target_indices_.reset (new std::vector <int> (static_cast <int> (target_->size ())));
    int index = 0;
    for (std::vector <int>::iterator it = target_indices_->begin (), it_e = target_indices_->end (); it != it_e; it++)
      *it = index++;
    target_cloud_updated_ = true;
  }

  // if a sample size for the point clouds is given; prefarably no sampling of target cloud
  if (nr_samples_ != 0)
  {
    const int ss = static_cast <int> (indices_->size ());
    const int sample_fraction_src = std::max (1, static_cast <int> (ss / nr_samples_));

    source_indices_ = pcl::IndicesPtr (new std::vector <int>);
    for (int i = 0; i < ss; i++)
    if (rand () % sample_fraction_src == 0)
      source_indices_->push_back ((*indices_) [i]);
  }
  else
    source_indices_ = indices_;

  // check usage of normals
  if (source_normals_ && target_normals_  && source_normals_->size () == input_->size () && target_normals_->size () == target_->size ())
    use_normals_ = true;

  // set up tree structures
  if (target_cloud_updated_)
  {
    tree_->setInputCloud (target_, target_indices_);
    target_cloud_updated_ = false;
  }

  // set predefined variables
  const int min_iterations = 4;
  const float diameter_fraction = 0.3f;

  // get diameter of input cloud (distance between farthest points)
  Eigen::Vector4f pt_min, pt_max;
  pcl::getMinMax3D (*target_, *target_indices_, pt_min, pt_max);
  diameter_ = (pt_max - pt_min).norm ();

  // derive the limits for the random base selection
  float max_base_diameter = diameter_* approx_overlap_ * 2.f;
  max_base_diameter_sqr_ = max_base_diameter * max_base_diameter;

  // normalize the delta
  if (normalize_delta_)
  {
    float mean_dist = getMeanPointDensity <PointTarget> (target_, *target_indices_, 0.05f * diameter_, nr_threads_);
    delta_ *= mean_dist;
  }

  // heuristic determination of number of trials to have high probabilty of finding a good solution
  if (max_iterations_ == 0)
  {
    float first_est = std::log (small_error_) / std::log (1.0 - std::pow ((double) approx_overlap_, (double) min_iterations));
    max_iterations_ = static_cast <int> (first_est / (diameter_fraction * approx_overlap_ * 2.f));
  }

  // set further parameter
  if (score_threshold_ == FLT_MAX)
    score_threshold_ = 1.f - approx_overlap_;

  if (max_iterations_ < 4)
    max_iterations_ = 4;

  if (max_runtime_ < 1)
    max_runtime_ = INT_MAX;

  // calculate internal parameters based on the the estimated point density
  max_pair_diff_ = delta_ * 2.f;
  max_edge_diff_ = delta_ * 4.f;
  coincidation_limit_ = delta_ * 2.f; // EDITED: originally std::sqrt (delta_ * 2.f)
  max_mse_ = powf (delta_* 2.f, 2.f);
  max_inlier_dist_sqr_ = powf (delta_ * 2.f, 2.f);

  // reset fitness_score
  fitness_score_ = FLT_MAX;

  return (true);
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> int
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::selectBase (
  std::vector <int> &base_indices,
  float (&ratio)[2])
{
  const float too_close_sqr = max_base_diameter_sqr_*0.01;

  Eigen::VectorXf coefficients (4);
  pcl::SampleConsensusModelPlane <PointTarget> plane (target_);
  plane.setIndices (target_indices_);
  Eigen::Vector4f centre_pt;
  float nearest_to_plane = FLT_MAX;

  // repeat base search until valid quadruple was found or ransac_iterations_ number of trys were unsuccessfull
  for (int i = 0; i < ransac_iterations_; i++)
  {
    // random select an appropriate point triple
    if (selectBaseTriangle (base_indices) < 0)
      continue;

    std::vector <int> base_triple (base_indices.begin (), base_indices.end () - 1);
    plane.computeModelCoefficients (base_triple, coefficients);
    pcl::compute3DCentroid (*target_, base_triple, centre_pt);

    // loop over all points in source cloud to find most suitable fourth point
    const PointTarget *pt1 = &(target_->points[base_indices[0]]);
    const PointTarget *pt2 = &(target_->points[base_indices[1]]);
    const PointTarget *pt3 = &(target_->points[base_indices[2]]);

    for (std::vector <int>::iterator it = target_indices_->begin (), it_e = target_indices_->end (); it != it_e; it++)
    {
      const PointTarget *pt4 = &(target_->points[*it]);

      float d1 = pcl::squaredEuclideanDistance (*pt4, *pt1);
      float d2 = pcl::squaredEuclideanDistance (*pt4, *pt2);
      float d3 = pcl::squaredEuclideanDistance (*pt4, *pt3);
      float d4 = (pt4->getVector3fMap ()  - centre_pt.head (3)).squaredNorm ();

      // check distance between points w.r.t minimum sampling distance; EDITED -> 4th point now also limited by max base line
      if (d1 < too_close_sqr || d2 < too_close_sqr || d3 < too_close_sqr || d4 < too_close_sqr ||
        d1 > max_base_diameter_sqr_ || d2 > max_base_diameter_sqr_ || d3 > max_base_diameter_sqr_)
        continue;

      // check distance to plane to get point closest to plane
      float dist_to_plane = pcl::pointToPlaneDistance (*pt4, coefficients);
      if (dist_to_plane < nearest_to_plane)
      {
        base_indices[3] = *it;
        nearest_to_plane = dist_to_plane;
      }
    }

    // check if at least one point fullfilled the conditions
    if (nearest_to_plane != FLT_MAX)
    {
      // order points to build largest quadrangle and calcuate intersection ratios of diagonals
      setupBase (base_indices, ratio);
      return (0);
    }
  }

  // return unsuccessfull if no quadruple was selected
  return (-1);
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> int
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::selectBaseTriangle (std::vector <int> &base_indices)
{
  int nr_points = static_cast <int> (target_indices_->size ());
  float best_t = 0.f;

  // choose random first point
  base_indices[0] = (*target_indices_)[rand () % nr_points];
  int *index1 = &base_indices[0];

  // random search for 2 other points (as far away as overlap allows)
  for (int i = 0; i < ransac_iterations_; i++)
  {
    int *index2 = &(*target_indices_)[rand () % nr_points];
    int *index3 = &(*target_indices_)[rand () % nr_points];

    Eigen::Vector3f u = target_->points[*index2].getVector3fMap () - target_->points[*index1].getVector3fMap ();
    Eigen::Vector3f v = target_->points[*index3].getVector3fMap () - target_->points[*index1].getVector3fMap ();
    float t = u.cross (v).squaredNorm (); // triangle area (0.5 * sqrt(t)) should be maximal

    // check for most suitable point triple
    if (t > best_t && u.squaredNorm () < max_base_diameter_sqr_ && v.squaredNorm () < max_base_diameter_sqr_)
    {
      best_t = t;
      base_indices[1] = *index2;
      base_indices[2] = *index3;
    }
  }

  // return if a triplet could be selected
  return (best_t == 0.f ? -1 : 0);
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::setupBase (
  std::vector <int> &base_indices,
  float (&ratio)[2])
{
  float best_t = FLT_MAX;
  const std::vector <int> copy (base_indices.begin (), base_indices.end ());
  std::vector <int> temp (base_indices.begin (), base_indices.end ());

  // loop over all combinations of base points
  for (std::vector <int>::const_iterator i = copy.begin (), i_e = copy.end (); i != i_e; i++)
  for (std::vector <int>::const_iterator j = copy.begin (), j_e = copy.end (); j != j_e; j++)
  {
    if (i == j)
      continue;

    for (std::vector <int>::const_iterator k = copy.begin (), k_e = copy.end (); k != k_e; k++)
    {
      if (k == j || k == i)
        continue;

      std::vector <int>::const_iterator l = copy.begin ();
      while (l == i || l == j || l == k)
        l++;

      temp[0] = *i;
      temp[1] = *j;
      temp[2] = *k;
      temp[3] = *l;

      // calculate diagonal intersection ratios and check for suitable segment to segment distances
      float ratio_temp[2];
      float t = segmentToSegmentDist (temp, ratio_temp);
      if (t < best_t)
      {
        best_t = t;
        ratio[0] = ratio_temp[0];
        ratio[1] = ratio_temp[1];
        base_indices = temp;
      }
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> float
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::segmentToSegmentDist (
  const std::vector <int> &base_indices,
  float (&ratio)[2])
{
  // get point vectors
  Eigen::Vector3f u = target_->points[base_indices[1]].getVector3fMap () - target_->points[base_indices[0]].getVector3fMap ();
  Eigen::Vector3f v = target_->points[base_indices[3]].getVector3fMap () - target_->points[base_indices[2]].getVector3fMap ();
  Eigen::Vector3f w = target_->points[base_indices[0]].getVector3fMap () - target_->points[base_indices[2]].getVector3fMap ();

  // calculate segment distances
  float a = u.dot (u);
  float b = u.dot (v);
  float c = v.dot (v);
  float d = u.dot (w);
  float e = v.dot (w);
  float D = a * c - b * b;
  float sN = 0.f, sD = D;
  float tN = 0.f, tD = D;

  // check segments
  if (D < small_error_)
  {
    sN = 0.f;
    sD = 1.f;
    tN = e;
    tD = c;
  }
  else
  {
    sN = (b * e - c * d);
    tN = (a * e - b * d);

    if (sN < 0.f)
    {
      sN = 0.f;
      tN = e;
      tD = c;
    }
    else if (sN > sD)
    {
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.f)
  {
    tN = 0.f;

    if (-d < 0.f)
      sN = 0.f;

    else if (-d > a)
      sN = sD;

    else
    {
      sN = -d;
      sD = a;
    }
  }

  else if (tN > tD)
  {
    tN = tD;

    if ((-d + b) < 0.f)
      sN = 0.f;

    else if ((-d + b) > a)
      sN = sD;

    else
    {
      sN = (-d + b);
      sD = a;
    }
  }

  // set intersection ratios
  ratio[0] = (std::abs (sN) < small_error_) ? 0.f : sN / sD;
  ratio[1] = (std::abs (tN) < small_error_) ? 0.f : tN / tD;

  Eigen::Vector3f x = w + (ratio[0] * u) - (ratio[1] * v);
  return (x.norm ());
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> int
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::bruteForceCorrespondences (
  int idx1,
  int idx2,
  pcl::Correspondences &pairs)
{
  const float max_norm_diff = 0.5f * max_norm_diff_ * M_PI / 180.f;

  // calculate reference segment distance and normal angle
  float ref_dist = pcl::euclideanDistance (target_->points[idx1], target_->points[idx2]);
  float ref_norm_angle = (use_normals_ ? (target_normals_->points[idx1].getNormalVector3fMap () -
                                          target_normals_->points[idx2].getNormalVector3fMap ()).norm () : 0.f);

  // loop over all pairs of points in source point cloud
  std::vector <int>::iterator it_out = source_indices_->begin (), it_out_e = source_indices_->end () - 1;
  std::vector <int>::iterator it_in, it_in_e = source_indices_->end ();
  for ( ; it_out != it_out_e; it_out++)
  {
    it_in = it_out + 1;
    const PointSource *pt1 = &(*input_)[*it_out];
    for ( ; it_in != it_in_e; it_in++)
    {
      const PointSource *pt2 = &(*input_)[*it_in];

      // check point distance compared to reference dist (from base)
      float dist = pcl::euclideanDistance (*pt1, *pt2);
      if (std::abs(dist - ref_dist) < max_pair_diff_)
      {
        // add here normal evaluation if normals are given
        if (use_normals_)
        {
          const NormalT *pt1_n = &(source_normals_->points[*it_out]);
          const NormalT *pt2_n = &(source_normals_->points[*it_in]);

          float norm_angle_1 = (pt1_n->getNormalVector3fMap () - pt2_n->getNormalVector3fMap ()).norm ();
          float norm_angle_2 = (pt1_n->getNormalVector3fMap () + pt2_n->getNormalVector3fMap ()).norm ();

          float norm_diff = std::min <float> (std::abs (norm_angle_1 - ref_norm_angle), std::abs (norm_angle_2 - ref_norm_angle));
          if (norm_diff > max_norm_diff)
            continue;
        }

        pairs.push_back (pcl::Correspondence (*it_in, *it_out, dist));
        pairs.push_back (pcl::Correspondence (*it_out, *it_in, dist));
      }
    }
  }

  // return success if at least one correspondence was found
  return (pairs.size () == 0 ? -1 : 0);
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> int
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::determineBaseMatches (
  const std::vector <int> &base_indices,
  std::vector <std::vector <int> > &matches,
  const pcl::Correspondences &pairs_a,
  const pcl::Correspondences &pairs_b,
  const float (&ratio)[2])
{
  // calculate edge lengths of base
  float dist_base[4];
  dist_base[0] = pcl::euclideanDistance (target_->points[base_indices[0]], target_->points[base_indices[2]]);
  dist_base[1] = pcl::euclideanDistance (target_->points[base_indices[0]], target_->points[base_indices[3]]);
  dist_base[2] = pcl::euclideanDistance (target_->points[base_indices[1]], target_->points[base_indices[2]]);
  dist_base[3] = pcl::euclideanDistance (target_->points[base_indices[1]], target_->points[base_indices[3]]);

  // loop over first point pair correspondences and store intermediate points 'e' in new point cloud
  PointCloudSourcePtr cloud_e (new PointCloudSource);
  cloud_e->resize (pairs_a.size () * 2);
  PointCloudSourceIterator it_pt = cloud_e->begin ();
  for (pcl::Correspondences::const_iterator it_pair = pairs_a.begin (), it_pair_e = pairs_a.end () ; it_pair != it_pair_e; it_pair++)
  {
    const PointSource *pt1 = &(input_->points[it_pair->index_match]);
    const PointSource *pt2 = &(input_->points[it_pair->index_query]);

    // calculate intermediate points using both ratios from base (r1,r2)
    for (int i = 0; i < 2; i++, it_pt++)
    {
      it_pt->x = pt1->x + ratio[i] * (pt2->x - pt1->x);
      it_pt->y = pt1->y + ratio[i] * (pt2->y - pt1->y);
      it_pt->z = pt1->z + ratio[i] * (pt2->z - pt1->z);
    }
  }

  // initialize new kd tree of intermediate points from first point pair correspondences
  KdTreeReciprocalPtr tree_e (new KdTreeReciprocal);
  tree_e->setInputCloud (cloud_e);

  std::vector <int> ids;
  std::vector <float> dists_sqr;

  // loop over second point pair correspondences
  for (pcl::Correspondences::const_iterator it_pair = pairs_b.begin (), it_pair_e = pairs_b.end () ; it_pair != it_pair_e; it_pair++)
  {
    const PointTarget *pt1 = &(input_->points[it_pair->index_match]);
    const PointTarget *pt2 = &(input_->points[it_pair->index_query]);

    // calculate intermediate points using both ratios from base (r1,r2)
    for (int i = 0; i < 2; i++)
    {
      PointTarget pt_e;
      pt_e.x = pt1->x + ratio[i] * (pt2->x - pt1->x);
      pt_e.y = pt1->y + ratio[i] * (pt2->y - pt1->y);
      pt_e.z = pt1->z + ratio[i] * (pt2->z - pt1->z);

      // search for corresponding intermediate points
      tree_e->radiusSearch (pt_e, coincidation_limit_, ids, dists_sqr);
      for (std::vector <int>::iterator it = ids.begin (), it_e = ids.end (); it != it_e; it++)
      {
        std::vector <int> match_indices (4);

        match_indices[0] = pairs_a[static_cast <int> (std::floor ((float)(*it/2.f)))].index_match;
        match_indices[1] = pairs_a[static_cast <int> (std::floor ((float)(*it/2.f)))].index_query;
        match_indices[2] = it_pair->index_match;
        match_indices[3] = it_pair->index_query;

        // EDITED: added coarse check of match based on edge length (due to rigid-body )
        if (checkBaseMatch (match_indices, dist_base) < 0)
          continue;

        matches.push_back (match_indices);
      }
    }
  }

  // return unsuccessfull if no match was found
  return (matches.size () > 0 ? 0 : -1);
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> int
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::checkBaseMatch (
  const std::vector <int> &match_indices,
  const float (&dist_ref)[4])
{
  float d0 = pcl::euclideanDistance (input_->points[match_indices[0]], input_->points[match_indices[2]]);
  float d1 = pcl::euclideanDistance (input_->points[match_indices[0]], input_->points[match_indices[3]]);
  float d2 = pcl::euclideanDistance (input_->points[match_indices[1]], input_->points[match_indices[2]]);
  float d3 = pcl::euclideanDistance (input_->points[match_indices[1]], input_->points[match_indices[3]]);

  // check edge distances of match w.r.t the base
  return (std::abs (d0 - dist_ref[0]) < max_edge_diff_ && std::abs (d1 - dist_ref[1]) < max_edge_diff_ &&
          std::abs (d2 - dist_ref[2]) < max_edge_diff_ && std::abs (d3 - dist_ref[3]) < max_edge_diff_) ? 0 : -1;
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::handleMatches (
  const std::vector <int> &base_indices,
  std::vector <std::vector <int> > &matches,
  MatchingCandidates &candidates)
{
  candidates.resize (1);
  float fitness_score = FLT_MAX;

  // loop over all Candidate matches
  for (std::vector <std::vector <int> >::iterator match_indices = matches.begin (), it_e = matches.end (); match_indices != it_e; match_indices++)
  {
    Eigen::Matrix4f transformation_temp;
    pcl::Correspondences correspondences_temp;

    // determine corresondences between base and match according to their distance to centroid
    linkMatchWithBase (base_indices, *match_indices, correspondences_temp);

    // check match based on residuals of the corresponding points after
    if (validateMatch (base_indices, *match_indices, correspondences_temp, transformation_temp) < 0)
      continue;

    // check resulting  using a sub sample of the source point cloud and compare to previous matches
    if (validateTransformation (transformation_temp, fitness_score) < 0)
      continue;

    // store best match as well as associated fitness_score and transformation
    candidates[0].fitness_score = fitness_score;
    candidates [0].transformation = transformation_temp;
    correspondences_temp.erase (correspondences_temp.end () - 1);
    candidates[0].correspondences = correspondences_temp;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::linkMatchWithBase (
  const std::vector <int> &base_indices,
  std::vector <int> &match_indices,
  pcl::Correspondences &correspondences)
{
  // calculate centroid of base and target
  Eigen::Vector4f centre_base, centre_match;
  pcl::compute3DCentroid (*target_, base_indices, centre_base);
  pcl::compute3DCentroid (*input_, match_indices, centre_match);

  PointTarget centre_pt_base;
  centre_pt_base.x = centre_base[0];
  centre_pt_base.y = centre_base[1];
  centre_pt_base.z = centre_base[2];

  PointSource centre_pt_match;
  centre_pt_match.x = centre_match[0];
  centre_pt_match.y = centre_match[1];
  centre_pt_match.z = centre_match[2];

  // find corresponding points according to their distance to the centroid
  std::vector <int> copy = match_indices;

  std::vector <int>::const_iterator it_base = base_indices.begin (), it_base_e = base_indices.end ();
  std::vector <int>::iterator it_match, it_match_e = copy.end ();
  std::vector <int>::iterator it_match_orig = match_indices.begin ();
  for (; it_base != it_base_e; it_base++, it_match_orig++)
  {
    float dist_sqr_1 = pcl::squaredEuclideanDistance (target_->points[*it_base], centre_pt_base);
    float best_diff_sqr = FLT_MAX;
    int best_index;

    for (it_match = copy.begin (); it_match != it_match_e; it_match++)
    {
      // calculate difference of distances to centre point
      float dist_sqr_2 = pcl::squaredEuclideanDistance (input_->points[*it_match], centre_pt_match);
      float diff_sqr = std::abs(dist_sqr_1 - dist_sqr_2);

      if (diff_sqr < best_diff_sqr)
      {
        best_diff_sqr = diff_sqr;
        best_index = *it_match;
      }
    }

    // assign new correspondence and update indices of matched targets
    correspondences.push_back (pcl::Correspondence (best_index, *it_base, best_diff_sqr));
    *it_match_orig = best_index;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> int
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::validateMatch (
  const std::vector <int> &base_indices,
  const std::vector <int> &match_indices,
  const pcl::Correspondences &correspondences,
  Eigen::Matrix4f &transformation)
{
  // only use triplet of points to simlify process (possible due to planar case)
  pcl::Correspondences correspondences_temp = correspondences;
  correspondences_temp.erase (correspondences_temp.end () - 1);

  // estimate transformation between correspondence set
  transformation_estimation_->estimateRigidTransformation (*input_, *target_, correspondences_temp, transformation);

  // transform base points
  PointCloudSource match_transformed;
  pcl::transformPointCloud (*input_, match_indices, match_transformed, transformation);

  // calculate residuals of transformation and check against maximum threshold
  std::size_t nr_points = correspondences_temp.size ();
  float mse = 0.f;
  for (std::size_t i = 0; i < nr_points; i++)
    mse += pcl::squaredEuclideanDistance (match_transformed.points [i], target_->points [base_indices[i]]);

  mse /= nr_points;
  return (mse < max_mse_ ? 0 : -1);
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> int
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::validateTransformation (
  Eigen::Matrix4f &transformation,
  float &fitness_score)
{
  // transform source point cloud
  PointCloudSource source_transformed;
  pcl::transformPointCloud (*input_, *source_indices_, source_transformed, transformation);

  std::size_t nr_points = source_transformed.size ();
  std::size_t terminate_value = fitness_score > 1 ? 0 : static_cast <std::size_t> ((1.f - fitness_score) * nr_points);

  float inlier_score_temp = 0;
  std::vector <int> ids;
  std::vector <float> dists_sqr;
  PointCloudSourceIterator it = source_transformed.begin ();

  for (std::size_t i = 0; i < nr_points; it++, i++)
  {
    // search for nearest point using kd tree search
    tree_->nearestKSearch (*it, 1, ids, dists_sqr);
    inlier_score_temp += (dists_sqr[0] < max_inlier_dist_sqr_ ? 1 : 0);

    // early terminating
    if (nr_points - i + inlier_score_temp < terminate_value)
      break;
  }

  // check current costs and return unsuccessfull if larger than previous ones
  inlier_score_temp /= static_cast <float> (nr_points);
  float fitness_score_temp = 1.f - inlier_score_temp;

  if (fitness_score_temp > fitness_score)
    return (-1);

  fitness_score = fitness_score_temp;
  return (0);
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT, typename Scalar> void
pcl::registration::FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::finalCompute (
  const std::vector <MatchingCandidates > &candidates)
{
  // get best fitness_score over all trys
  int nr_candidates = static_cast <int> (candidates.size ());
  int best_index = -1;
  float best_score = FLT_MAX;
  for (int i = 0; i < nr_candidates; i++)
  {
    const float &fitness_score = candidates [i][0].fitness_score;
    if (fitness_score < best_score)
    {
      best_score = fitness_score;
      best_index = i;
    }
  }

  // check if a valid candidate was available
  if (!(best_index < 0))
  {
    fitness_score_ = candidates [best_index][0].fitness_score;
    final_transformation_ = candidates [best_index][0].transformation;
    *correspondences_ = candidates [best_index][0].correspondences;

    // here we define convergence if resulting fitness_score is below 1-threshold
    converged_ = fitness_score_ < score_threshold_;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////

#endif // PCL_REGISTRATION_IMPL_IA_4PCS_H_
