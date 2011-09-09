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
  *
  */

#ifndef PCL_FEATURES_IMPL_SHOT_H_
#define PCL_FEATURES_IMPL_SHOT_H_

#include "pcl/features/shot.h"
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
/*!
  * \brief Check if val1 and val2 are equals.
  *
  * \param val1 first number to check.
  * \param val2 second number to check.
  * \return true if val1 is equal to val2, false otherwise.
  */
inline bool
areEquals (double val1, double val2, double zeroDoubleEps = zeroDoubleEps15)
{
  return (fabs (val1 - val2)<zeroDoubleEps);
};

//////////////////////////////////////////////////////////////////////////////////////////////
/*!
  * \brief Check if val1 and val2 are equals.
  *
  * \param val1 first number to check.
  * \param val2 second number to check.
  * \return true if val1 is equal to val2, false otherwise.
  */
inline bool
areEquals (float val1, float val2, float zeroFloatEps = zeroFloatEps8)
{
  return (fabs (val1 - val2)<zeroFloatEps);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT, typename PointOutT> float
pcl::SHOTEstimation<pcl::PointXYZRGBA, PointNT, PointOutT>::sRGB_LUT[256] = {- 1};

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT, typename PointOutT> float
pcl::SHOTEstimation<pcl::PointXYZRGBA, PointNT, PointOutT>::sXYZ_LUT[4000] = {- 1};

//////////////////////////////////////////////////////////////////////////////////////////////
/*!
  * \brief Converts RGB triplets to CIELab space.
  *
  * \param R red input component(8 bits)
  * \param G green input component(8 bits)
  * \param B blue input component(8 bits)
  * \param L "L" output component(float)
  * \param a "a" output component(float)
  * \param b "b" output component(float)
  */
template <typename PointNT, typename PointOutT> void
pcl::SHOTEstimation<pcl::PointXYZRGBA, PointNT, PointOutT>::RGB2CIELAB (unsigned char R, unsigned char G,
                                                                        unsigned char B, float &L, float &A,
                                                                        float &B2)
{
  if (sRGB_LUT[0] < 0)
  {
    for (int i = 0; i < 256; i++)
    {
      float f = i / 255.0f;
      if (f > 0.04045)
        sRGB_LUT[i] = (float)pow ((f + 0.055) / 1.055, 2.4);
      else
        sRGB_LUT[i] = f / 12.92;
    }

    for (int i = 0; i < 4000; i++)
    {
      float f = i / 4000.0f;
      if (f > 0.008856)
        sXYZ_LUT[i] = pow (f, (float)0.3333);
      else
        sXYZ_LUT[i] = (7.787 * f) + (16.0 / 116.0);
    }
  }

  float fr = sRGB_LUT[R];
  float fg = sRGB_LUT[G];
  float fb = sRGB_LUT[B];

  // Use white = D65
  const float x = fr * 0.412453 + fg * 0.357580 + fb * 0.180423;
  const float y = fr * 0.212671 + fg * 0.715160 + fb * 0.072169;
  const float z = fr * 0.019334 + fg * 0.119193 + fb * 0.950227;

  float vx = x / 0.95047;
  float vy = y;
  float vz = z / 1.08883;

  vx = sXYZ_LUT[int(vx*4000)];
  vy = sXYZ_LUT[int(vy*4000)];
  vz = sXYZ_LUT[int(vz*4000)];

  L = 116.0 * vy - 16.0;
  if (L>100)
    L=100;

  A = 500.0 * (vx - vy);
  if (A>120)
    A=120;
  else if (A<- 120)
    A=- 120;

  B2 = 200.0 * (vy - vz);
  if (B2>120)
    B2=120;
  else if (B2<- 120)
    B2=- 120;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Compute a local Reference Frame for a 3D feature; the output is stored in the "rf" vector
template <typename PointInT, typename PointNT, typename PointOutT> float
pcl::SHOTEstimationBase<PointInT, PointNT, PointOutT>::getSHOTLocalRF (
  const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
  const int index, const std::vector<int> &indices, const std::vector<float> &dists, 
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf)
{
  if (rf.size () != 3)
    rf.resize (3);

  Eigen::Vector4f central_point = cloud.points[index].getVector4fMap ();
  central_point[3] = 0;
  // Allocate enough space
  Eigen::Vector4d *vij = new Eigen::Vector4d[indices.size ()];

  Eigen::Matrix3d cov_m = Eigen::Matrix3d::Zero ();

  double distance = 0.0;
  double sum = 0.0;

  int valid_nn_points = 0;

  for (size_t i_idx = 0; i_idx < indices.size (); ++i_idx)
  {
    if (indices[i_idx] == index)
      continue;

    Eigen::Vector4f pt = cloud.points[indices[i_idx]].getVector4fMap (); 
	pt[3] = 0;
    // Difference between current point and origin
    vij[valid_nn_points] = (pt - central_point).cast<double> ();
	vij[valid_nn_points][3] = 0;

    distance = search_radius_ - sqrt (dists[i_idx]);

    // Multiply vij * vij'
    cov_m += distance * (vij[valid_nn_points].head<3> () * vij[valid_nn_points].head<3> ().transpose ());

    sum += distance;
    valid_nn_points++;
  }

  if (valid_nn_points < 5)
  {
    PCL_ERROR ("[pcl::%s::getSHOTLocalRF] Warning! Neighborhood has less than 5 vertexes. Aborting Local RF computation of feature point with index %d\n", getClassName ().c_str (), index);
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
  if( abs ( plusTangentDirection1 - valid_nn_points + plusTangentDirection1 )  > 0 ) {


	  if (plusTangentDirection1 < valid_nn_points - plusTangentDirection1)
		  v1 *= - 1;
	 
  }
   else{

	   plusTangentDirection1=0;
		int points = 5; ///std::min(valid_nn_points*2/2+1, 11);
		int index = valid_nn_points/2;

		for (int i = -points/2; i <= points/2; i++)
			if ( vij[index- i ].dot (v1) > 0)
				plusTangentDirection1 ++;	
		
		if (plusTangentDirection1 < points/2+1)
			v1 *= - 1;
	}

	if( abs ( plusNormal - valid_nn_points + plusNormal )  > 0 ) {
		if (plusNormal < valid_nn_points - plusNormal)
			v3 *= - 1;

	}
	else{

		plusNormal = 0;
		int points = 5; //std::min(valid_nn_points*2/2+1, 11);
		//std::cout << points << std::endl;
		int index = valid_nn_points/2;

		for (int i = -points/2; i <= points/2; i++)
			if ( vij[index- i ].dot (v3) > 0)
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

//////////////////////////////////////////////////////////////////////////////////////////////
// Quadrilinear interpolation; used when color and shape descriptions are NOT activated simultaneously
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::SHOTEstimationBase<PointInT, PointNT, PointOutT>::interpolateSingleChannel (
    const pcl::PointCloud<PointInT> &cloud,
    const std::vector<int> &indices,
    const std::vector<float> &dists,
    const Eigen::Vector4f &central_point,
    const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf,
    std::vector<double> &binDistance,
    const int nr_bins,
    Eigen::VectorXf &shot)
{
  if (rf.size () != 3)
  {
    PCL_ERROR ("[pcl::%s::interpolateSingleChannel] RF size different than 9! Aborting...\n");
    return;
  }

  for (size_t i_idx = 0; i_idx < indices.size (); ++i_idx)
  {
    Eigen::Vector4f delta = cloud.points[indices[i_idx]].getVector4fMap () - central_point;
	delta[3] = 0;

    // Compute the Euclidean norm
   double distance = sqrt (dists[i_idx]);

    if (areEquals (distance, 0.0))
      continue;

    double xInFeatRef = delta.dot (rf[0]); //(x * feat[i].rf[0] + y * feat[i].rf[1] + z * feat[i].rf[2]);
    double yInFeatRef = delta.dot (rf[1]); //(x * feat[i].rf[3] + y * feat[i].rf[4] + z * feat[i].rf[5]);
    double zInFeatRef = delta.dot (rf[2]); //(x * feat[i].rf[6] + y * feat[i].rf[7] + z * feat[i].rf[8]);

    // To avoid numerical problems afterwards
    if (fabs (yInFeatRef) < 1E-30)
      yInFeatRef  = 0;
    if (fabs (xInFeatRef) < 1E-30)
      xInFeatRef  = 0;
    if (fabs (zInFeatRef) < 1E-30)
      zInFeatRef  = 0;


    unsigned char bit4 = ((yInFeatRef > 0) || ((yInFeatRef == 0.0) && (xInFeatRef < 0))) ? 1 : 0;
    unsigned char bit3 = ((xInFeatRef > 0) || ((xInFeatRef == 0.0) && (yInFeatRef > 0))) ? !bit4 : bit4;

    assert (bit3 == 0 || bit3 == 1);

    int desc_index = (bit4<<3) + (bit3<<2);

    desc_index = desc_index << 1;

    if ((xInFeatRef * yInFeatRef > 0) || (xInFeatRef == 0.0))
      desc_index += (fabs (xInFeatRef) >= fabs (yInFeatRef)) ? 0 : 4;
    else
      desc_index += (fabs (xInFeatRef) > fabs (yInFeatRef)) ? 4 : 0;

    desc_index += zInFeatRef > 0 ? 1 : 0;

    // 2 RADII
    desc_index += (distance > radius1_2_) ? 2 : 0;

    int step_index = static_cast<int>(floor (binDistance[i_idx] +0.5));
    int volume_index = desc_index * (nr_bins+1);

    //Interpolation on the cosine (adjacent bins in the histogram)
    binDistance[i_idx] -= step_index;
    double intWeight = (1- fabs (binDistance[i_idx]));

    if (binDistance[i_idx] > 0)
      shot[volume_index + ((step_index+1) % nr_bins)] += binDistance[i_idx];
    else
      shot[volume_index + ((step_index - 1 + nr_bins) % nr_bins)] += - binDistance[i_idx];

    //Interpolation on the distance (adjacent husks)
   
    if (distance > radius1_2_)   //external sphere
    {
      double radiusDistance = (distance - radius3_4_) / radius1_2_;

      if (distance > radius3_4_) //most external sector, votes only for itself
        intWeight += 1 - radiusDistance;  //peso=1-d
      else  //3/4 of radius, votes also for the internal sphere
      {
        intWeight += 1 + radiusDistance;
        shot[(desc_index - 2) * (nr_bins+1) + step_index] -= radiusDistance;
	  }
    }
    else    //internal sphere
    {
      double radiusDistance = (distance - radius1_4_) / radius1_2_;

      if (distance < radius1_4_) //most internal sector, votes only for itself
        intWeight += 1 + radiusDistance;  //weight=1-d
      else  //3/4 of radius, votes also for the external sphere
      {
        intWeight += 1 - radiusDistance;
        shot[(desc_index + 2) * (nr_bins+1) + step_index] += radiusDistance;
	  }
    }

    //Interpolation on the inclination (adjacent vertical volumes)
    double inclinationCos = zInFeatRef / distance;
    if (inclinationCos < - 1.0)
      inclinationCos = - 1.0;
    if (inclinationCos > 1.0)
      inclinationCos = 1.0;

    double inclination = acos (inclinationCos);

    assert (inclination >= 0.0 && inclination <= PST_RAD_180);

    if (inclination > PST_RAD_90 || (fabs (inclination - PST_RAD_90) < 1e-30 && zInFeatRef <= 0))
    {
      double inclinationDistance = (inclination - PST_RAD_135) / PST_RAD_90;
      if (inclination > PST_RAD_135)
        intWeight += 1 - inclinationDistance;
      else
      {
        intWeight += 1 + inclinationDistance;
        assert ((desc_index + 1) * (nr_bins+1) + step_index >= 0 && (desc_index + 1) * (nr_bins+1) + step_index < descLength_);
        shot[(desc_index + 1) * (nr_bins+1) + step_index] -= inclinationDistance;
	  }
    }
    else
    {
      double inclinationDistance = (inclination - PST_RAD_45) / PST_RAD_90;
      if (inclination < PST_RAD_45)
        intWeight += 1 + inclinationDistance;
      else
      {
        intWeight += 1 - inclinationDistance;
        assert ((desc_index - 1) * (nr_bins+1) + step_index >= 0 && (desc_index - 1) * (nr_bins+1) + step_index < descLength_);
        shot[(desc_index - 1) * (nr_bins+1) + step_index] += inclinationDistance;
	  }
    }

    if (yInFeatRef != 0.0 || xInFeatRef != 0.0)
    {
      //Interpolation on the azimuth (adjacent horizontal volumes)
      double azimuth = atan2 (yInFeatRef, xInFeatRef);

      int sel = desc_index >> 2;
      double angularSectorSpan = PST_RAD_45;
      double angularSectorStart = - PST_RAD_PI_7_8;

      double azimuthDistance = (azimuth - (angularSectorStart + angularSectorSpan*sel)) / angularSectorSpan;

      assert ((azimuthDistance < 0.5 || areEquals (azimuthDistance, 0.5)) && (azimuthDistance > - 0.5 || areEquals (azimuthDistance, - 0.5)));

      azimuthDistance = (std::max)(- 0.5, std::min (azimuthDistance, 0.5));

      if (azimuthDistance > 0)
      {
        intWeight += 1 - azimuthDistance;
        int interp_index = (desc_index + 4) % maxAngularSectors_;
        assert (interp_index * (nr_bins+1) + step_index >= 0 && interp_index * (nr_bins+1) + step_index < descLength_);
        shot[interp_index * (nr_bins+1) + step_index] += azimuthDistance;
      }
      else
      {
        int interp_index = (desc_index - 4 + maxAngularSectors_) % maxAngularSectors_;
        assert (interp_index * (nr_bins+1) + step_index >= 0 && interp_index * (nr_bins+1) + step_index < descLength_);
        intWeight += 1 + azimuthDistance;
        shot[interp_index * (nr_bins+1) + step_index] -= azimuthDistance;
      }

    }

    assert (volume_index + step_index >= 0 &&  volume_index + step_index < descLength_);
    shot[volume_index + step_index] += intWeight;

  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Quadrilinear interpolation; used when color and shape descriptions are both activated
template <typename PointNT, typename PointOutT> void
pcl::SHOTEstimation<pcl::PointXYZRGBA, PointNT, PointOutT>::interpolateDoubleChannel (
  const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, 
  const std::vector<int> &indices, 
  const std::vector<float> &dists,
  const Eigen::Vector4f &central_point, 
  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf,
  std::vector<double> &binDistanceShape,
  std::vector<double> &binDistanceColor, 
  const int nr_bins_shape, 
  const int nr_bins_color, 
  Eigen::VectorXf &shot)
{
  if (rf.size () != 3)
  {
    PCL_ERROR ("[pcl::%s::interpolateDoubleChannel] RF size different than 9! Aborting...\n");
    return;
  }

  int shapeToColorStride = nr_grid_sector_*(nr_bins_shape+1);

  for (size_t i_idx = 0; i_idx < indices.size (); ++i_idx)
  {
    Eigen::Vector4f delta = cloud.points[indices[i_idx]].getVector4fMap () - central_point;
	delta[3] = 0;

    // Compute the Euclidean norm
    double distance = sqrt (dists[i_idx]);

    if (areEquals (distance, 0.0))
      continue;

    double xInFeatRef = delta.dot (rf[0]); //(x * feat[i].rf[0] + y * feat[i].rf[1] + z * feat[i].rf[2]);
    double yInFeatRef = delta.dot (rf[1]); //(x * feat[i].rf[3] + y * feat[i].rf[4] + z * feat[i].rf[5]);
    double zInFeatRef = delta.dot (rf[2]); //(x * feat[i].rf[6] + y * feat[i].rf[7] + z * feat[i].rf[8]);

    // To avoid numerical problems afterwards
    if (fabs (yInFeatRef) < 1E-30)
      yInFeatRef  = 0;
    if (fabs (xInFeatRef) < 1E-30)
      xInFeatRef  = 0;
    if (fabs (zInFeatRef) < 1E-30)
      zInFeatRef  = 0;

    unsigned char bit4 = ((yInFeatRef > 0) || ((yInFeatRef == 0.0) && (xInFeatRef < 0))) ? 1 : 0;
    unsigned char bit3 = ((xInFeatRef > 0) || ((xInFeatRef == 0.0) && (yInFeatRef > 0))) ? !bit4 : bit4;

    assert (bit3 == 0 || bit3 == 1);

    int desc_index = (bit4<<3) + (bit3<<2);

    desc_index = desc_index << 1;

    if ((xInFeatRef * yInFeatRef > 0) || (xInFeatRef == 0.0))
      desc_index += (fabs (xInFeatRef) >= fabs (yInFeatRef)) ? 0 : 4;
    else
      desc_index += (fabs (xInFeatRef) > fabs (yInFeatRef)) ? 4 : 0;

    desc_index += zInFeatRef > 0 ? 1 : 0;

    // 2 RADII
    desc_index += (distance > radius1_2_) ? 2 : 0;

    int step_index_shape = static_cast<int>(floor (binDistanceShape[i_idx] +0.5));
    int step_index_color = static_cast<int>(floor (binDistanceColor[i_idx] +0.5));

    int volume_index_shape = desc_index * (nr_bins_shape+1);
    int volume_index_color = shapeToColorStride + desc_index * (nr_bins_color+1);

    //Interpolation on the cosine (adjacent bins in the histrogram)
    binDistanceShape[i_idx] -= step_index_shape;
    binDistanceColor[i_idx] -= step_index_color;

    double intWeightShape = (1- fabs (binDistanceShape[i_idx]));
    double intWeightColor = (1- fabs (binDistanceColor[i_idx]));

    if (binDistanceShape[i_idx] > 0)
      shot[volume_index_shape + ((step_index_shape + 1) % nr_bins_shape)] += binDistanceShape[i_idx];
    else
      shot[volume_index_shape + ((step_index_shape - 1 + nr_bins_shape) % nr_bins_shape)] -= binDistanceShape[i_idx];

    if (binDistanceColor[i_idx] > 0)
      shot[volume_index_color + ((step_index_color+1) % nr_bins_color)] += binDistanceColor[i_idx];
    else
      shot[volume_index_color + ((step_index_color - 1 + nr_bins_color) % nr_bins_color)] -= binDistanceColor[i_idx];

    //Interpolation on the distance (adjacent husks)
   
    if (distance > radius1_2_)   //external sphere
    {
      double radiusDistance = (distance - radius3_4_) / radius1_2_;

      if (distance > radius3_4_) //most external sector, votes only for itself
      {
        intWeightShape += 1 - radiusDistance; //weight=1-d
        intWeightColor += 1 - radiusDistance; //weight=1-d
      }
      else  //3/4 of radius, votes also for the internal sphere
      {
        intWeightShape += 1 + radiusDistance;
        intWeightColor += 1 + radiusDistance;
        shot[(desc_index - 2) * (nr_bins_shape+1) + step_index_shape] -= radiusDistance;
        shot[shapeToColorStride + (desc_index - 2) * (nr_bins_color+1) + step_index_color] -= radiusDistance;
      }
    }
    else    //internal sphere
    {
      double radiusDistance = (distance - radius1_4_) / radius1_2_;

      if (distance < radius1_4_) //most internal sector, votes only for itself
      {
        intWeightShape += 1 + radiusDistance;
        intWeightColor += 1 + radiusDistance; //weight=1-d
      }
      else  //3/4 of radius, votes also for the external sphere
      {
        intWeightShape += 1 - radiusDistance; //weight=1-d
        intWeightColor += 1 - radiusDistance; //weight=1-d
        shot[(desc_index + 2) * (nr_bins_shape+1) + step_index_shape] += radiusDistance;
        shot[shapeToColorStride + (desc_index + 2) * (nr_bins_color+1) + step_index_color] += radiusDistance;
      }
    }

    //Interpolation on the inclination (adjacent vertical volumes)
    double inclinationCos = zInFeatRef / distance;
    if (inclinationCos < - 1.0)
      inclinationCos = - 1.0;
    if (inclinationCos > 1.0)
      inclinationCos = 1.0;

    double inclination = acos (inclinationCos);

    assert (inclination >= 0.0 && inclination <= PST_RAD_180);

    if (inclination > PST_RAD_90 || (fabs (inclination - PST_RAD_90) < 1e-30 && zInFeatRef <= 0))
    {
      double inclinationDistance = (inclination - PST_RAD_135) / PST_RAD_90;
      if (inclination > PST_RAD_135)
      {
        intWeightShape += 1 - inclinationDistance;
        intWeightColor += 1 - inclinationDistance;
      }
      else
      {
        intWeightShape += 1 + inclinationDistance;
        intWeightColor += 1 + inclinationDistance;
        assert ((desc_index + 1) * (nr_bins_shape+1) + step_index_shape >= 0 && (desc_index + 1) * (nr_bins_shape+1) + step_index_shape < descLength_);
        assert (shapeToColorStride + (desc_index + 1) * (nr_bins_color+ 1) + step_index_color >= 0 && shapeToColorStride + (desc_index + 1) * (nr_bins_color+1) + step_index_color < descLength_);
        shot[(desc_index + 1) * (nr_bins_shape+1) + step_index_shape] -= inclinationDistance;
        shot[shapeToColorStride + (desc_index + 1) * (nr_bins_color+1) + step_index_color] -= inclinationDistance;
      }
    }
    else
    {
      double inclinationDistance = (inclination - PST_RAD_45) / PST_RAD_90;
      if (inclination < PST_RAD_45)
      {
        intWeightShape += 1 + inclinationDistance;
        intWeightColor += 1 + inclinationDistance;
      }
      else
      {
        intWeightShape += 1 - inclinationDistance;
        intWeightColor += 1 - inclinationDistance;
        assert ((desc_index - 1) * (nr_bins_shape+1) + step_index_shape >= 0 && (desc_index - 1) * (nr_bins_shape+1) + step_index_shape < descLength_);
        assert (shapeToColorStride + (desc_index - 1) * (nr_bins_color+ 1) + step_index_color >= 0 && shapeToColorStride + (desc_index - 1) * (nr_bins_color+1) + step_index_color < descLength_);
        shot[(desc_index - 1) * (nr_bins_shape+1) + step_index_shape] += inclinationDistance;
        shot[shapeToColorStride + (desc_index - 1) * (nr_bins_color+1) + step_index_color] += inclinationDistance;
      }
    }

    if (yInFeatRef != 0.0 || xInFeatRef != 0.0)
    {
      //Interpolation on the azimuth (adjacent horizontal volumes)
      double azimuth = atan2 (yInFeatRef, xInFeatRef);

      int sel = desc_index >> 2;
      double angularSectorSpan = PST_RAD_45;
      double angularSectorStart = - PST_RAD_PI_7_8;

      double azimuthDistance = (azimuth - (angularSectorStart + angularSectorSpan*sel)) / angularSectorSpan;
      assert ((azimuthDistance < 0.5 || areEquals (azimuthDistance, 0.5)) && (azimuthDistance > - 0.5 || areEquals (azimuthDistance, - 0.5)));
      azimuthDistance = (std::max)(- 0.5, std::min (azimuthDistance, 0.5));

      if (azimuthDistance > 0)
      {
        intWeightShape += 1 - azimuthDistance;
        intWeightColor += 1 - azimuthDistance;
        int interp_index = (desc_index + 4) % maxAngularSectors_;
        assert (interp_index * (nr_bins_shape+1) + step_index_shape >= 0 && interp_index * (nr_bins_shape+1) + step_index_shape < descLength_);
        assert (shapeToColorStride + interp_index * (nr_bins_color+1) + step_index_color >= 0 && shapeToColorStride + interp_index * (nr_bins_color+1) + step_index_color < descLength_);
        shot[interp_index * (nr_bins_shape+1) + step_index_shape] += azimuthDistance;
        shot[shapeToColorStride + interp_index * (nr_bins_color+1) + step_index_color] += azimuthDistance;
      }
      else
      {
        int interp_index = (desc_index - 4 + maxAngularSectors_) % maxAngularSectors_;
        intWeightShape += 1 + azimuthDistance;
        intWeightColor += 1 + azimuthDistance;
        assert (interp_index * (nr_bins_shape+1) + step_index_shape >= 0 && interp_index * (nr_bins_shape+1) + step_index_shape < descLength_);
        assert (shapeToColorStride + interp_index * (nr_bins_color+1) + step_index_color >= 0 && shapeToColorStride + interp_index * (nr_bins_color+1) + step_index_color < descLength_);
        shot[interp_index * (nr_bins_shape+1) + step_index_shape] -= azimuthDistance;
        shot[shapeToColorStride + interp_index * (nr_bins_color+1) + step_index_color] -= azimuthDistance;
      }
    }

    assert (volume_index_shape + step_index_shape >= 0 &&  volume_index_shape + step_index_shape < descLength_);
    assert (volume_index_color + step_index_color >= 0 &&  volume_index_color + step_index_color < descLength_);
    shot[volume_index_shape + step_index_shape] += intWeightShape;
    shot[volume_index_color + step_index_color] += intWeightColor;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT, typename PointOutT> void
pcl::SHOTEstimation<pcl::PointXYZRGBA, PointNT, PointOutT>::computePointSHOT (
  const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, const pcl::PointCloud<PointNT> &normals,
  const int index, const std::vector<int> &indices, const std::vector<float> &dists, Eigen::VectorXf &shot,
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf)
{
  if (rf.size () != 3)
    rf.resize (3);

  // Clear the resultant shot
  shot.setZero ();

  std::vector<double> binDistanceShape;
  std::vector<double> binDistanceColor;

  int nNeighbors = indices.size ();

  //Skip the current feature if the number of its neighbors is not sufficient for its description
  if (nNeighbors < 5)
  {
    PCL_WARN ("[pcl::%s::computePointSHOT] Warning! Neighborhood has less than 5 vertexes. Aborting description of point with index %d\n", getClassName ().c_str (), index);
    return;
  }

  //Compute the local Reference Frame for the current 3D point
  if (getSHOTLocalRF (cloud, normals, index, indices, dists, rf))
	  return;

  //If shape description is enabled, compute the bins activated by each neighbor of the current feature in the shape histogram
  if (b_describe_shape_)
  {
    binDistanceShape.resize (nNeighbors);

    for (size_t i_idx = 0; i_idx < indices.size (); ++i_idx)
    {
      double cosineDesc = normals.points[indices[i_idx]].getNormalVector4fMap ().dot (rf[2]); //feat[i].rf[6]*normal[0] + feat[i].rf[7]*normal[1] + feat[i].rf[8]*normal[2];

      if (cosineDesc > 1.0)
        cosineDesc = 1.0;
      if (cosineDesc < - 1.0)
        cosineDesc = - 1.0;

      binDistanceShape[i_idx] = ((1.0 + cosineDesc) * nr_shape_bins_) / 2;
    }
  }

  //If color description is enabled, compute the bins activated by each neighbor of the current feature in the color histogram
  if (b_describe_color_)
  {
    binDistanceColor.resize (nNeighbors);

    unsigned char redRef = cloud.points[index].rgba >> 16 & 0xFF;
    unsigned char greenRef = cloud.points[index].rgba >> 8& 0xFF;
    unsigned char blueRef = cloud.points[index].rgba & 0xFF;

    float LRef, aRef, bRef;

    RGB2CIELAB (redRef, greenRef, blueRef, LRef, aRef, bRef);
    LRef /= 100.0;
    aRef /= 120.0;
    bRef /= 120.0;    //normalized LAB components (0<L<1, -1<a<1, -1<b<1)

    for (size_t i_idx = 0; i_idx < indices.size (); ++i_idx)
    {
      unsigned char red = cloud.points[indices[i_idx]].rgba >> 16 & 0xFF;
      unsigned char green = cloud.points[indices[i_idx]].rgba >> 8 & 0xFF;
      unsigned char blue = cloud.points[indices[i_idx]].rgba & 0xFF;

      float L, a, b;

      RGB2CIELAB (red, green, blue, L, a, b);
      L /= 100.0;
      a /= 120.0;
      b /= 120.0;   //normalized LAB components (0<L<1, -1<a<1, -1<b<1)

      double colorDistance = (fabs (LRef - L) + ((fabs (aRef - a) + fabs (bRef - b)) / 2)) /3;

      if (colorDistance > 1.0)
        colorDistance = 1.0;
      if (colorDistance < 0.0)
        colorDistance = 0.0;

      binDistanceColor[i_idx] = colorDistance * nr_color_bins_;
    }
  }

  //Apply quadrilinear interpolation on the activated bins in the shape and/or color histogram(s)

  if (b_describe_shape_ && b_describe_color_)
    interpolateDoubleChannel (cloud, indices, dists, cloud.points[index].getVector4fMap (), rf, binDistanceShape, binDistanceColor,
                              nr_shape_bins_, nr_color_bins_,
                              shot);
  else if (b_describe_color_)
    interpolateSingleChannel (cloud, indices, dists, cloud.points[index].getVector4fMap (), rf, binDistanceColor, nr_color_bins_, shot);
  else
    interpolateSingleChannel (cloud, indices, dists, cloud.points[index].getVector4fMap (), rf, binDistanceShape, nr_shape_bins_, shot);

  //Normalize the final histogram
  double accNorm = 0;
  for (int j=0; j< descLength_; j++)
    accNorm += shot[j]*shot[j];

  accNorm = sqrt (accNorm);

  for (int j=0; j< descLength_; j++)
    shot[j] /= accNorm;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::SHOTEstimation<PointInT, PointNT, PointOutT>::computePointSHOT (
  const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals,
  const int index, const std::vector<int> &indices, const std::vector<float> &dists, Eigen::VectorXf &shot,
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf)
{
  if (rf.size () != 3)
    rf.resize (3);

   // Clear the resultant shot
  shot.setZero ();

  std::vector<double> binDistanceShape;

  int nNeighbors = indices.size ();

  if (nNeighbors < 5)
  {
    PCL_WARN ("[pcl::%s::computePointSHOT] Warning! Neighborhood has less than 5 vertexes. Aborting description of point with index %d\n", getClassName ().c_str (), index);
    return;
  }

  if (getSHOTLocalRF (cloud, normals, index, indices, dists, rf))
	  return;


  binDistanceShape.resize (nNeighbors);

  for (size_t i_idx = 0; i_idx < indices.size (); ++i_idx)
  {
    double cosineDesc = normals.points[indices[i_idx]].getNormalVector4fMap ().dot (rf[2]); //feat[i].rf[6]*normal[0] + feat[i].rf[7]*normal[1] + feat[i].rf[8]*normal[2];

    if (cosineDesc > 1.0)
      cosineDesc = 1.0;
    if (cosineDesc < - 1.0)
      cosineDesc = - 1.0;

    binDistanceShape[i_idx] = ((1.0 + cosineDesc) * nr_shape_bins_) / 2;

  }

  interpolateSingleChannel (cloud, indices, dists, cloud.points[index].getVector4fMap (), rf, binDistanceShape, nr_shape_bins_, shot);

  double accNorm = 0;
  for (int j=0; j< descLength_; j++)
    accNorm += shot[j]*shot[j];

  accNorm = sqrt (accNorm);

  for (int j=0; j< descLength_; j++)
    shot[j] /= accNorm;

}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT, typename PointOutT> void
pcl::SHOTEstimation<pcl::PointXYZRGBA, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  if (rf_.size () != 3)
    rf_.resize (3);

  // Compute the current length of the descriptor
  descLength_ = (b_describe_shape_) ? nr_grid_sector_*(nr_shape_bins_+1) : 0;
  descLength_ +=   (b_describe_color_) ? nr_grid_sector_*(nr_color_bins_+1) : 0;

  // Useful values
  sqradius_ = search_radius_*search_radius_;
  radius3_4_ = (search_radius_*3) / 4;
  radius1_4_ = search_radius_ / 4;
  radius1_2_ = search_radius_ / 2;

  shot_.setZero (descLength_);
  rf_[0].setZero ();
  rf_[1].setZero ();
  rf_[2].setZero ();

  if (output.points[0].descriptor.size () != (size_t)descLength_)
    for (size_t idx = 0; idx < indices_->size (); ++idx)
      output.points[idx].descriptor.resize (descLength_);
//  if (output.points[0].size != (size_t)descLength_)
//  {
//    PCL_ERROR ("[pcl::%s::computeFeature] The desired descriptor size (%lu) does not match the output point type feature (%lu)! Aborting...\n", getClassName ().c_str (), descLength_, (unsigned long) output.points[0].size);
//    return;
//  }

  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
    this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    // Compute the SHOT descriptor for the current 3D feature
    computePointSHOT (*surface_, *normals_, idx, nn_indices, nn_dists, shot_, rf_);

    // Copy into the resultant cloud
    for (int d = 0; d < shot_.size (); ++d)
      output.points[idx].descriptor[d] = shot_[d];
    for (int d = 0; d < 9; ++d)
      output.points[idx].rf[d] = rf_[d/3][d%3];
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::SHOTEstimationBase<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  if (rf_.size () != 3)
    rf_.resize (3);

  descLength_ = nr_grid_sector_ * (nr_shape_bins_+1);

  sqradius_ = search_radius_ * search_radius_;
  radius3_4_ = (search_radius_*3) / 4;
  radius1_4_ = search_radius_ / 4;
  radius1_2_ = search_radius_ / 2;

  shot_.setZero (descLength_);
  rf_[0].setZero ();
  rf_[1].setZero ();
  rf_[2].setZero ();

  if (output.points[0].descriptor.size () != (size_t)descLength_)
    for (size_t idx = 0; idx < indices_->size (); ++idx)
      output.points[idx].descriptor.resize (descLength_);
//  if (output.points[0].size != (size_t)descLength_)
//  {
//    PCL_ERROR ("[pcl::%s::computeFeature] The desired descriptor size (%lu) does not match the output point type feature (%lu)! Aborting...\n", getClassName ().c_str (), descLength_, (unsigned long) output.points[0].size);
//    return;
//  }

  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
    this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists);

    // Estimate the SHOT at each patch
    computePointSHOT (*surface_, *normals_, idx, nn_indices, nn_dists, shot_, rf_);

    // Copy into the resultant cloud
    for (int d = 0; d < shot_.size (); ++d)
      output.points[idx].descriptor[d] = shot_[d];
    for (int d = 0; d < 9; ++d)
      output.points[idx].rf[d] = rf_[d/3][d%3];
  }
}

#define PCL_INSTANTIATE_SHOTEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::SHOTEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_SHOT_H_

