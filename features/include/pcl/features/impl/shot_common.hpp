/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef PCL_FEATURES_IMPL_SHOT_COMMON_H_
#define PCL_FEATURES_IMPL_SHOT_COMMON_H_

#include <utility>

// Useful constants.
#define PST_PI 3.1415926535897932384626433832795
#define PST_RAD_45 0.78539816339744830961566084581988
#define PST_RAD_90 1.5707963267948966192313216916398
#define PST_RAD_135 2.3561944901923449288469825374596
#define PST_RAD_180 PST_PI
#define PST_RAD_360 6.283185307179586476925286766558
#define PST_RAD_PI_7_8 2.7488935718910690836548129603691

const double zeroDoubleEps15 = 1E-15;
const float zeroFloatEps8 = 1E-8f;

//////////////////////////////////////////////////////////////////////////////////////////////
// Compute a local Reference Frame for a 3D feature; the output is stored in the "rf" vector
template <typename PointInT> float
pcl::getLocalRF (const pcl::PointCloud<PointInT> &cloud,
                 const double search_radius,
                 const Eigen::Vector4f & central_point,
                 const std::vector<int> &indices,
                 const std::vector<float> &dists,
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf)
{
  if (rf.size () != 3)
    rf.resize (3);

  // Allocate enough space
  Eigen::Vector4d *vij = new Eigen::Vector4d[indices.size ()];

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

    if (pt.head<3> () == central_point.head<3> ())
      continue;

    // Difference between current point and origin
    vij[valid_nn_points] = (pt - central_point).cast<double> ();
    vij[valid_nn_points][3] = 0;

    distance = search_radius - sqrt (dists[i_idx]);

    // Multiply vij * vij'
    cov_m += distance * (vij[valid_nn_points].head<3> () * vij[valid_nn_points].head<3> ().transpose ());

    sum += distance;
    valid_nn_points++;
  }

  if (valid_nn_points < 5)
  {
    PCL_ERROR ("[pcl::%s::getSHOTLocalRF] Warning! Neighborhood has less than 5 vertexes. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOT", central_point[0], central_point[1], central_point[2]);
    rf[0].setZero ();
    rf[1].setZero ();
    rf[2].setZero ();

    rf[0][0] = 1;
    rf[1][1] = 1;
    rf[2][2] = 1;

    delete [] vij;

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

  if(!pcl_isfinite(e1c) || !pcl_isfinite(e2c) || !pcl_isfinite(e3c)){
    PCL_ERROR ("[pcl::%s::getSHOTLocalRF] Warning! Eigenvectors are NaN. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOT", central_point[0], central_point[1], central_point[2]);
    rf[0].setZero ();
    rf[1].setZero ();
    rf[2].setZero ();

    rf[0][0] = 1;
    rf[1][1] = 1;
    rf[2][2] = 1;

    delete [] vij;

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
    double dp = vij[ne].dot (v1);
    if (dp >= 0)
      plusTangentDirection1++;

    dp = vij[ne].dot (v3);
    if (dp >= 0)
      plusNormal++;
  }

  //TANGENT
  if( abs ( plusTangentDirection1 - valid_nn_points + plusTangentDirection1 )  > 0 )
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
			if ( vij[medianIndex- i ].dot (v1) > 0)
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
			if ( vij[medianIndex- i ].dot (v3) > 0)
				plusNormal ++;

		if (plusNormal < points/2+1)
			v3 *= - 1;
	}



  rf[0] = v1.cast<float>();
  rf[2] = v3.cast<float>();
  rf[1] = rf[2].cross3 (rf[0]);
  rf[0][3] = 0; rf[1][3] = 0; rf[2][3] = 0;

  delete [] vij;

  return (0.0f);
}

#endif    // PCL_FEATURES_IMPL_SHOT_COMMON_H_

