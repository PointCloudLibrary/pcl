/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 */

#ifndef PCL_FEATURES_IMPL_SHOT_LRF_H_
#define PCL_FEATURES_IMPL_SHOT_LRF_H_

#include <Eigen/Eigenvalues> // for SelfAdjointEigenSolver
#include <utility>
#include <pcl/features/shot_lrf.h>

//////////////////////////////////////////////////////////////////////////////////////////////
// Compute a local Reference Frame for a 3D feature; the output is stored in the "rf" matrix
template<typename PointInT, typename PointOutT> float
pcl::SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getLocalRF (const int& current_point_idx, Eigen::Matrix3f &rf)
{
  const Eigen::Vector4f& central_point = (*input_)[current_point_idx].getVector4fMap ();
  pcl::Indices n_indices;
  std::vector<float> n_sqr_distances;

  this->searchForNeighbors (current_point_idx, search_parameter_, n_indices, n_sqr_distances);

  Eigen::Matrix<double, Eigen::Dynamic, 4> vij (n_indices.size (), 4);

  Eigen::Matrix3d cov_m = Eigen::Matrix3d::Zero ();

  double distance = 0.0;
  double sum = 0.0;

  int valid_nn_points = 0;

  for (std::size_t i_idx = 0; i_idx < n_indices.size (); ++i_idx)
  {
    Eigen::Vector4f pt = (*surface_)[n_indices[i_idx]].getVector4fMap ();
    if (pt.head<3> () == central_point.head<3> ())
		  continue;

    // Difference between current point and origin
    vij.row (valid_nn_points).matrix () = (pt - central_point).cast<double> ();
    vij (valid_nn_points, 3) = 0;

    distance = search_parameter_ - sqrt (n_sqr_distances[i_idx]);

    // Multiply vij * vij'
    cov_m += distance * (vij.row (valid_nn_points).head<3> ().transpose () * vij.row (valid_nn_points).head<3> ());

    sum += distance;
    valid_nn_points++;
  }

  if (valid_nn_points < 5)
  {
    //PCL_ERROR ("[pcl::%s::getLocalRF] Warning! Neighborhood has less than 5 vertexes. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOTLocalReferenceFrameEstimation", central_point[0], central_point[1], central_point[2]);
    rf.setConstant (std::numeric_limits<float>::quiet_NaN ());

    return (std::numeric_limits<float>::max ());
  }

  cov_m /= sum;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (cov_m);

  const double& e1c = solver.eigenvalues ()[0];
  const double& e2c = solver.eigenvalues ()[1];
  const double& e3c = solver.eigenvalues ()[2];

  if (!std::isfinite (e1c) || !std::isfinite (e2c) || !std::isfinite (e3c))
  {
    //PCL_ERROR ("[pcl::%s::getLocalRF] Warning! Eigenvectors are NaN. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOTLocalReferenceFrameEstimation", central_point[0], central_point[1], central_point[2]);
    rf.setConstant (std::numeric_limits<float>::quiet_NaN ());

    return (std::numeric_limits<float>::max ());
  }

  // Disambiguation
  Eigen::Vector4d v1 = Eigen::Vector4d::Zero ();
  Eigen::Vector4d v3 = Eigen::Vector4d::Zero ();
  v1.head<3> ().matrix () = solver.eigenvectors ().col (2);
  v3.head<3> ().matrix () = solver.eigenvectors ().col (0);

  int plusNormal = 0, plusTangentDirection1=0;
  for (int ne = 0; ne < valid_nn_points; ne++)
  {
    double dp = vij.row (ne).dot (v1);
    if (dp >= 0)
      plusTangentDirection1++;

    dp = vij.row (ne).dot (v3);
    if (dp >= 0)
      plusNormal++;
  }

  //TANGENT
  plusTangentDirection1 = 2*plusTangentDirection1 - valid_nn_points;
  if (plusTangentDirection1 == 0)
  {
		int points = 5; //std::min(valid_nn_points*2/2+1, 11);
		int medianIndex = valid_nn_points/2;

		for (int i = -points/2; i <= points/2; i++)
			if ( vij.row (medianIndex - i).dot (v1) > 0)
				plusTangentDirection1 ++;

		if (plusTangentDirection1 < points/2+1)
			v1 *= - 1;
	} 
  else if (plusTangentDirection1 < 0)
    v1 *= - 1;

  //Normal
  plusNormal = 2*plusNormal - valid_nn_points;
  if (plusNormal == 0)
  {
		int points = 5; //std::min(valid_nn_points*2/2+1, 11);
		int medianIndex = valid_nn_points/2;

		for (int i = -points/2; i <= points/2; i++)
			if ( vij.row (medianIndex - i).dot (v3) > 0)
				plusNormal ++;

		if (plusNormal < points/2+1)
			v3 *= - 1;
	} else if (plusNormal < 0)
    v3 *= - 1;

  rf.row (0).matrix () = v1.head<3> ().cast<float> ();
  rf.row (2).matrix () = v3.head<3> ().cast<float> ();
  rf.row (1).matrix () = rf.row (2).cross (rf.row (0));

  return (0.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  //check whether used with search radius or search k-neighbors
  if (this->getKSearch () != 0)
  {
    PCL_ERROR(
      "[pcl::%s::computeFeature] Error! Search method set to k-neighborhood. Call setKSearch(0) and setRadiusSearch( radius ) to use this class.\n",
      getClassName().c_str ());
    return;
  }
  tree_->setSortedResults (true);

  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // point result
    Eigen::Matrix3f rf;
    PointOutT& output_rf = output[i];

    //output_rf.confidence = getLocalRF ((*indices_)[i], rf);
    //if (output_rf.confidence == std::numeric_limits<float>::max ())
    if (getLocalRF ((*indices_)[i], rf) == std::numeric_limits<float>::max ())
    {
      output.is_dense = false;
    }

    for (int d = 0; d < 3; ++d)
    {
      output_rf.x_axis[d] = rf.row (0)[d];
      output_rf.y_axis[d] = rf.row (1)[d];
      output_rf.z_axis[d] = rf.row (2)[d];
    }
  }
}

#define PCL_INSTANTIATE_SHOTLocalReferenceFrameEstimation(T,OutT) template class PCL_EXPORTS pcl::SHOTLocalReferenceFrameEstimation<T,OutT>;

#endif    // PCL_FEATURES_IMPL_SHOT_LRF_H_

