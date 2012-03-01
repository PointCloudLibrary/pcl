/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 * $Id$
 */

#ifndef PCL_FEATURES_IMPL_SHOT_LRF_H_
#define PCL_FEATURES_IMPL_SHOT_LRF_H_

#include <utility>
#include <pcl/features/shot_lrf.h>

//////////////////////////////////////////////////////////////////////////////////////////////
// Compute a local Reference Frame for a 3D feature; the output is stored in the "rf" matrix
template<typename PointInT, typename PointOutT> float
pcl::SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::getLocalRF (const pcl::PointCloud<PointInT> &cloud, 
                                                                         const double search_radius, 
                                                                         const Eigen::Vector4f & central_point, 
                                                                         const std::vector<int> &indices, 
                                                                         const std::vector<float> &dists, 
                                                                         Eigen::Matrix3f &rf)
{
  Eigen::Matrix<double, Eigen::Dynamic, 4> vij (indices.size (), 4);

  Eigen::Matrix3d cov_m = Eigen::Matrix3d::Zero ();

  double distance = 0.0;
  double sum = 0.0;

  int valid_nn_points = 0;

  for (size_t i_idx = 0; i_idx < indices.size (); ++i_idx)
  {
    /*if (indices[i_idx] == index)
      continue;*/

    Eigen::Vector4f pt = cloud.points[indices[i_idx]].getVector4fMap (); 
    pt[3] = 0;

	  if (pt == central_point)
		  continue;

    // Difference between current point and origin
    vij.row (valid_nn_points) = (pt - central_point).cast<double> ();
    vij (valid_nn_points, 3) = 0;

    distance = search_radius - sqrt (dists[i_idx]);

    // Multiply vij * vij'
    cov_m += distance * (vij.row (valid_nn_points).head<3> ().transpose () * vij.row (valid_nn_points).head<3> ());

    sum += distance;
    valid_nn_points++;
  }

  if (valid_nn_points < 5)
  {
    PCL_ERROR ("[pcl::%s::getLocalRF] Warning! Neighborhood has less than 5 vertexes. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOTLocalReferenceFrameEstimation", central_point[0], central_point[1], central_point[2]);
    //rf.setIdentity ();
    rf.setConstant (std::numeric_limits<float>::quiet_NaN ());

    return (std::numeric_limits<float>::max ());
  }

  cov_m /= sum;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (cov_m);

  // Disambiguation
 
  Eigen::Vector3d v1c = solver.eigenvectors ().col (0);
  Eigen::Vector3d v2c = solver.eigenvectors ().col (1);
  Eigen::Vector3d v3c = solver.eigenvectors ().col (2);

  double e1c = solver.eigenvalues ()[0];
  double e2c = solver.eigenvalues ()[1];
  double e3c = solver.eigenvalues ()[2];

  if (!pcl_isfinite (e1c) || !pcl_isfinite (e2c) || !pcl_isfinite (e3c))
  {
    PCL_ERROR ("[pcl::%s::getLocalRF] Warning! Eigenvectors are NaN. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOTLocalReferenceFrameEstimation", central_point[0], central_point[1], central_point[2]);
    //rf.setIdentity ();
    rf.setConstant (std::numeric_limits<float>::quiet_NaN ());

    return (std::numeric_limits<float>::max ());
  }

  Eigen::Vector4d v1 = Eigen::Vector4d::Zero ();
  Eigen::Vector4d v3 = Eigen::Vector4d::Zero ();

  if (e1c > e2c)
  {
    if (e1c > e3c) // v1c > max(v2c,v3c)
    {
      v1.head<3> () = v1c;

      if (e2c > e3c)  // v1c > v2c > v3c
        v3.head<3> () = v3c;
      else // v1c > v3c > v2c
        v3.head<3> () = v2c;
    }
    else // v3c > v1c > v2c
    {
      v1.head<3> () = v3c;
      v3.head<3> () = v2c;
    }
  }
  else
  {
    if (e2c > e3c) // v2c > max(v1c,v3c)
    {
      v1.head<3> () = v2c;

      if (e1c > e3c)  // v2c > v1c > v3c
        v3.head<3> () = v3c;
      else // v2c > v3c > v1c
        v3.head<3> () = v1c;
    }
    else // v3c > v2c > v1c
    {
      v1.head<3> () = v3c;
      v3.head<3> () = v1c;
    }
  }

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
  if (abs ( plusTangentDirection1 - valid_nn_points + plusTangentDirection1 )  > 0 ) 
  {
	  if (plusTangentDirection1 < valid_nn_points - plusTangentDirection1)
		  v1 *= - 1;
  }
  else
  {
    plusTangentDirection1=0;
		int points = 5; ///std::min(valid_nn_points*2/2+1, 11);
		int medianIndex = valid_nn_points/2;

		for (int i = -points/2; i <= points/2; i++)
			if ( vij.row (medianIndex - i).dot (v1) > 0)
				plusTangentDirection1 ++;	
		
		if (plusTangentDirection1 < points/2+1)
			v1 *= - 1;
	}

  //Normal
	if( abs ( plusNormal - valid_nn_points + plusNormal )  > 0 ) 
  {
		if (plusNormal < valid_nn_points - plusNormal)
			v3 *= - 1;
	}
	else 
  {
		plusNormal = 0;
		int points = 5; //std::min(valid_nn_points*2/2+1, 11);
		//std::cout << points << std::endl;
		int medianIndex = valid_nn_points/2;

		for (int i = -points/2; i <= points/2; i++)
			if ( vij.row (medianIndex - i).dot (v3) > 0)
				plusNormal ++;	
	
		if (plusNormal < points/2+1)
			v3 *= - 1;
	}



  rf.row (0) = v1.cast<float> ().head<3> ();
  rf.row (2) = v3.cast<float> ().head<3> ();
  rf.row (1) = rf.row (2).cross (rf.row (0));

  return (0.0f);
}

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

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    int current_point_idx = indices_->at (i);
    Eigen::Vector4f current_point = input_->at (current_point_idx).getVector4fMap ();
    std::vector<int> neighbours_indices;
    std::vector<float> neighbours_sqr_distances;

    searchForNeighbors (current_point_idx, search_parameter_, neighbours_indices, neighbours_sqr_distances);

    // point result
    Eigen::Matrix3f rf;
    PointOutT& output_rf = output.at (i);

    output_rf.confidence = getLocalRF (*surface_, search_parameter_, current_point, neighbours_indices, neighbours_sqr_distances, rf);
    if (output_rf.confidence == std::numeric_limits<float>::max ())
    {
      output.is_dense = false;
    }
    
    output_rf.x_axis.getNormalVector3fMap () = rf.row (0);
    output_rf.y_axis.getNormalVector3fMap () = rf.row (1);
    output_rf.z_axis.getNormalVector3fMap () = rf.row (2);
  }

}

template <typename PointInT, typename PointOutT> void
pcl::SHOTLocalReferenceFrameEstimation<PointInT, PointOutT>::computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &output)
{
  //check whether used with search radius or search k-neighbors
  if (this->getKSearch () != 0)
  {
    PCL_ERROR(
      "[pcl::%s::computeFeature] Error! Search method set to k-neighborhood. Call setKSearch(0) and setRadiusSearch( radius ) to use this class.\n",
      getClassName().c_str ());
    return;
  }

  output.points.resize (indices_->size (), 10);
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    int current_point_idx = indices_->at (i);
    Eigen::Vector4f current_point = input_->at (current_point_idx).getVector4fMap ();
    std::vector<int> neighbours_indices;
    std::vector<float> neighbours_sqr_distances;

    searchForNeighbors (current_point_idx, search_parameter_, neighbours_indices, neighbours_sqr_distances);

    // point result
    Eigen::Matrix3f rf;

    output.points (i, 0) = getLocalRF (*surface_, search_parameter_, current_point, neighbours_indices, neighbours_sqr_distances, rf);
    if (output.points (i, 0) == std::numeric_limits<float>::max ())
    {
      output.is_dense = false;
    }
    

    output.points.block<1, 3> (i, 1) = rf.row (0);
    output.points.block<1, 3> (i, 4) = rf.row (1);
    output.points.block<1, 3> (i, 7) = rf.row (2);
  }

}

#define PCL_INSTANTIATE_SHOTLocalReferenceFrameEstimation(T,OutT) template class PCL_EXPORTS pcl::SHOTLocalReferenceFrameEstimation<T,OutT>;

#endif    // PCL_FEATURES_IMPL_SHOT_LRF_H_

