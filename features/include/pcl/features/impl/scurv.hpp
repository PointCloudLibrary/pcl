/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2018, University of Innsbruck (Antonio J Rodríguez-Sánchez, Tomas Turecek, Alex Melniciuc)
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
#ifndef PCL_FEATURES_IMPL_SCURV_H_
#define PCL_FEATURES_IMPL_SCURV_H_

#include <pcl/features/scurv.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/principal_curvatures.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/exceptions.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <unsupported/Eigen/Splines>
#include <Eigen/SVD>

#include <iostream>
#include <string.h>
#include <math.h>
#include <numeric>
#include <algorithm>
#include <functional>
#include <typeinfo>
#include <stdio.h>
#include <stdlib.h>
#include <utility>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT> double
pcl::SCurVEstimation<PointInT, PointNT>::getNormalizedValue (double x, double low, double high)
{
  return (x - low) / (high - low);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT> double
pcl::SCurVEstimation<PointInT, PointNT>::getHermiteDerivativeInterpolation(HermitePoint point0, HermitePoint point1, double xi)
{
  double h, delta, del0, del1, c2, c3, x;
  h = point1.x - point0.x;
  delta = ( point1.f - point0.f ) / h;
  del0 = ( point0.d - delta ) / h;
  del1 = ( point1.d - delta ) / h;
  c2 = -( del0 + del0 + del1 );
  c3 = ( del0 + del1 ) / h;
  x = xi - point0.x;
  return point0.f + x * ( point0.d + x * ( c2 + x * c3 ) );
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT> void
pcl::SCurVEstimation<PointInT, PointNT>::normalizeScale (pcl::PointCloud<pcl::PointNormal> &cloud, int min_range, int max_range)
{
  Eigen::Vector4f min_pt, max_pt;
  Eigen::Vector3f min_coeff, max_coeff;
  double minimum, maximum;

  pcl::getMinMax3D(cloud, min_pt, max_pt);
  min_coeff << min_pt(0), min_pt(1), min_pt(2);
  max_coeff << max_pt(0), max_pt(1), max_pt(2);
  minimum = min_coeff.minCoeff();
  maximum = max_coeff.maxCoeff();

  double factor = std::abs(minimum - maximum) / (max_range - min_range);
  for (size_t k = 0; k < cloud.points.size(); k++)
  {
    cloud.points[k].x = (double)cloud.points[k].x / factor;
    cloud.points[k].y = (double)cloud.points[k].y / factor;
    cloud.points[k].z = (double)cloud.points[k].z / factor;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT> double
pcl::SCurVEstimation<PointInT, PointNT>::signMultiplied (double arg1, double arg2)
{
  double value = NAN;

  if ( std::fabs(arg1 - 0.0) < std::numeric_limits<double>::epsilon()  )
  {
    value = 0.0;
  }
  else if ( arg1 < 0.0 )
  {
    if ( std::fabs(arg2 - 0.0) < std::numeric_limits<double>::epsilon() )
    {
      value = 0.0;
    }
    else if ( arg2 < 0.0 )
    {
      value = 1.0;
    }
    else if ( 0.0 < arg2 )
    {
      value = -1.0;
    }
  }
  else if ( 0.0 < arg1 )
  {
    if ( std::fabs(arg2 - 0.0) < std::numeric_limits<double>::epsilon() )
    {
      value = 0.0;
    }
    else if ( arg2 < 0.0 )
    {
      value = -1.0;
    }
    else if ( 0.0 < arg2 )
    {
      value = 1.0;
    }
  }

  return value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT> void
pcl::SCurVEstimation<PointInT, PointNT>::setSplinePchip (int n, std::vector<double> x, std::vector<double> f, std::vector<double> &d)
{
  double del1, del2, dmax, dmin, drat1, drat2, dsave, h1, h2, hsum, hsumt3, temp, w1, w2;
  int i, ierr, nless1;
//
//  Check the arguments.
//
  if ( n < 2 )
  {
    ierr = -1;
    throw pcl::PCLException ("Number of data points less than 2!",  __FILE__, BOOST_CURRENT_FUNCTION, __LINE__);
  }

  for ( i = 1; i < n; i++ )
  {
    if ( x[i] <= x[i-1] )
    {
      ierr = -3;
      throw pcl::PCLException ("X array not strictly increasing!",  __FILE__, BOOST_CURRENT_FUNCTION, __LINE__);
    }
  }

  ierr = 0;
  nless1 = n - 1;
  h1 = x[1] - x[0];
  del1 = ( f[1] - f[0] ) / h1;
  dsave = del1;
//
//  Special case N=2, use linear interpolation.
//
  if ( n == 2 )
  {
    d[0] = del1;
    d[n-1] = del1;
    return;
  }
//
//  Normal case, 3 <= N.
//
  h2 = x[2] - x[1];
  del2 = ( f[2] - f[1] ) / h2;
//
//  Set D(1) via non-centered three point formula, adjusted to be
//  shape preserving.
//
  hsum = h1 + h2;
  w1 = ( h1 + hsum ) / hsum;
  w2 = -h1 / hsum;
  d[0] = w1 * del1 + w2 * del2;

  if ( this->signMultiplied ( d[0], del1 ) <= 0.0 )
  {
    d[0] = 0.0;
  }
//
//  Need do this check only if monotonicity switches.
//
  else if ( this->signMultiplied ( del1, del2 ) < 0.0 )
  {
     dmax = 3.0 * del1;

     if ( std::abs ( dmax ) < std::abs ( d[0] ) )
     {
       d[0] = dmax;
     }

  }
//
//  Loop through interior points.
//
  for ( i = 2; i <= nless1; i++ )
  {
    if ( 2 < i )
    {
      h1 = h2;
      h2 = x[i] - x[i-1];
      hsum = h1 + h2;
      del1 = del2;
      del2 = ( f[i] - f[i-1] ) / h2;
    }
//
//  Set D(I)=0 unless data are strictly monotonic.
//
    d[i-1] = 0.0;

    temp = this->signMultiplied ( del1, del2 );


//
//  Count number of changes in direction of monotonicity.
//
    if ( std::fabs(temp - 0.0) < std::numeric_limits<double>::epsilon() )
    {
      if ( del2 != 0.0 )
      {
        if ( this->signMultiplied ( dsave, del2 ) < 0.0 )
        {
          ierr = ierr + 1;
        }
        dsave = del2;
      }
    }
    else if ( temp < 0.0 )
    {
      ierr = ierr + 1;
      dsave = del2;
    }
//
//  Use Brodlie modification of Butland formula.
//
    else
    {
      hsumt3 = 3.0 * hsum;
      w1 = ( hsum + h1 ) / hsumt3;
      w2 = ( hsum + h2 ) / hsumt3;
      dmax = std::max ( std::abs ( del1 ), std::abs ( del2 ) );
      dmin = std::min ( std::abs ( del1 ), std::abs ( del2 ) );
      drat1 = del1 / dmax;
      drat2 = del2 / dmax;
      d[i-1] = dmin / ( w1 * drat1 + w2 * drat2 );
    }
  }
//
//  Set D(N) via non-centered three point formula, adjusted to be
//  shape preserving.
//
  w1 = -h2 / hsum;
  w2 = ( h2 + hsum ) / hsum;
  d[n-1] = w1 * del1 + w2 * del2;

  if ( this->signMultiplied ( d[n-1], del2 ) <= 0.0 )
  {
    d[n-1] = 0.0;
  }
  else if ( this->signMultiplied ( del1, del2 ) < 0.0 )
  {
//
//  Need do this check only if monotonicity switches.
//
    dmax = 3.0 * del2;

    if ( std::abs ( dmax ) < std::abs ( d[n-1] ) )
    {
      d[n-1] = dmax;
    }

  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT> void
pcl::SCurVEstimation<PointInT, PointNT>::compute (pcl::PointCloud<pcl::SCurVSignature210> &output)
{
  if (! this->initCompute())
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  // Copy the header
  output.header = input_->header;

  // Resize the output dataset
  // Important! We should only allocate precisely how many elements we will need, otherwise
  // we risk at pre-allocating too much memory which could lead to bad_alloc 
  // (see http://dev.pointclouds.org/issues/657)
  output.width = output.height = 1;
  output.is_dense = true;
  output.points.resize (1);

  // Perform the actual feature computation
  this->computeFeature (output);

  this->deinitCompute ();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT> void
pcl::SCurVEstimation<PointInT, PointNT>::computeFeature (pcl::PointCloud<pcl::SCurVSignature210> &output)
{
  if (k_ > input_->points.size() - 1) {
    k_ = input_->points.size() - 1;
  }
  // need to use this vector for storing the nearest k_ points indices
  std::vector<int> pointIndices(k_);
  // need to use this vector for storing the squared distances between each of those
  // k_ nearest neighbors
  std::vector<float> euclideanDistances(k_);


  int min_rad = 0;
  int max_rad = 300;
  int nrays = 20;
  int R = (int) ((max_rad - min_rad) / nrays);
  Eigen::Matrix<int, 21, 1> R_bins, bin_rad;
  Eigen::Matrix<double, 10, 1> bins;
  for (int r = 0; r < 21; r++)
  {
    R_bins[r] = r * 15;
    if (r < 10)
    {
      bins[9 - r] = 1 - log10(r + 1);
    }
  }
  
  pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud(new pcl::PointCloud<pcl::PointNormal>);
  copyPointCloud(*input_, *input_cloud);

  int cnt = 0;
  Eigen::MatrixXd k1(input_cloud->points.size(),R_bins.size());
  Eigen::MatrixXd k2(input_cloud->points.size(),R_bins.size());
  
  // input cloud normalization
  this->normalizeScale(*input_cloud, min_rad, max_rad);

  // input cloud positioning at center
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*input_cloud, centroid);
  pcl::demeanPointCloud (*input_cloud, centroid, *input_cloud);

  // k-d tree object
  pcl::search::KdTree<pcl::PointNormal> kdtree;
  kdtree.setInputCloud(input_cloud);

  // compute all cloud, point by point
  for (size_t i = 0; i < input_cloud->points.size(); i++)
  {
    // make sure that the current point is not a NaN
    if ( pcl::isFinite(input_cloud->points[i]) && pcl_isfinite(normals_->points[i].normal_x) && pcl_isfinite(normals_->points[i].normal_y) && pcl_isfinite(normals_->points[i].normal_z) )
    {
      pcl::PointNormal currentPoint;
      currentPoint.getVector3fMap () = input_cloud->points[i].getVector3fMap ();
      currentPoint.getNormalVector3fMap () = normals_->points[i].getNormalVector3fMap ();
      
      // find the neighborhood of current point using KD tree nearest k search
      if (kdtree.nearestKSearch(currentPoint, k_, pointIndices, euclideanDistances) > 0)
      {
        // Here will be extracted indices and squared distances of
        // nearest k_ neighbors of the current point.
        
        // Here will be computed the principal curvature of
        // the surface described by those points.

        // This point cloud structure will store all neighbors of the current point.
        pcl::PointCloud<pcl::PointNormal>::Ptr neighb_of_curr_point(new pcl::PointCloud<pcl::PointNormal>);
        neighb_of_curr_point->width = pointIndices.size();
        neighb_of_curr_point->height = 1;
        neighb_of_curr_point->points.resize(neighb_of_curr_point->width * neighb_of_curr_point->height);
        
        //The neighborhood of current point        
        for (size_t k = 1; k < pointIndices.size(); ++k)
        {
          neighb_of_curr_point->points[k] = input_cloud->points[pointIndices[k]];
        }
        neighb_of_curr_point->points[0] = currentPoint;
        
        float rad = sqrt(pow(currentPoint.x, 2) + pow(currentPoint.y, 2) + pow(currentPoint.z, 2));

        for (size_t k = 0; k < bin_rad.size(); k++)
        {
          if (rad - R < R_bins[k] && rad >= R_bins[k]) {
            bin_rad[k] = 1;
          } else {
            bin_rad[k] = 0;
          }
        }

        for (size_t k = 0; k < R_bins.size(); k++)
        {
          k1(cnt, k) = 0;
          k2(cnt, k) = 1;
        }
  
        //Sordted Euclidean Distance to every point from the neighborhood        
        for (size_t j = 0; j < euclideanDistances.size(); j++)
        {
          euclideanDistances[j] = sqrt(pow((neighb_of_curr_point->points[j].x - currentPoint.x), 2) +
                pow((neighb_of_curr_point->points[j].y - currentPoint.y), 2) +
                pow((neighb_of_curr_point->points[j].z - currentPoint.z), 2));
        }
        
        // vector for keeping the curvatures        
        std::vector<double> k_aux(neighb_of_curr_point->points.size() - 1);
        // this is for normalized positions
        Eigen::MatrixXd pos_aux(neighb_of_curr_point->points.size() - 1, 3);
        
        //Value of computeted curvature of every point from the neighborhood        
        for (size_t j = 0; j < neighb_of_curr_point->points.size() - 1; j++)
        {
          k_aux[j] = sqrt(pow(currentPoint.normal_x - normals_->points[pointIndices[j + 1]].normal_x, 2) + 
              pow(currentPoint.normal_y - normals_->points[pointIndices[j + 1]].normal_y, 2) + 
              pow(currentPoint.normal_z - normals_->points[pointIndices[j + 1]].normal_z, 2)) / euclideanDistances[j + 1];
          
          // normalizing the position
          double denominator = sqrt(pow((neighb_of_curr_point->points[j + 1].x - currentPoint.x), 2) +
              pow((neighb_of_curr_point->points[j + 1].y - currentPoint.y), 2) +
              pow((neighb_of_curr_point->points[j + 1].z - currentPoint.z), 2));

          pos_aux(j, 0) = (neighb_of_curr_point->points[j + 1].x - currentPoint.x) / denominator;
          pos_aux(j, 1) = (neighb_of_curr_point->points[j + 1].y - currentPoint.y) / denominator;
          pos_aux(j, 2) = (neighb_of_curr_point->points[j + 1].z - currentPoint.z) / denominator;
        }

        // projection: matrix for projection of the nighb points onto tangent plane
        Eigen::MatrixXd projection(neighb_of_curr_point->points.size() - 1, 2);

        // project points onto tangent plane        

	Eigen::Matrix<double, 1, 3> normals;
	normals(0,0) = currentPoint.normal_x;
	normals(0,1) = currentPoint.normal_y;
	normals(0,2) = currentPoint.normal_z;

        //svd and orth. basis for the null space
        Eigen::Matrix<double, 3, 2> orthNullSp;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(normals, Eigen::ComputeFullV);
        orthNullSp << svd.matrixV().block<3, 2>(0, 1);

        // Project all the points from the neighborhood of the
        // current point onto its plane
        projection = pos_aux * orthNullSp;
        
        // and transform into polar coordinate system : https://ch.mathworks.com/help/matlab/ref/cart2pol.html
        // cart2pol(projection(:,1),projection(:,2))
        std::vector<double> theta(neighb_of_curr_point->points.size() - 1);
        std::vector<double> rho(neighb_of_curr_point->points.size() - 1);
        
        for (size_t k = 0; k < neighb_of_curr_point->points.size() - 1; ++k)
        {
          theta[k] = atan2(projection(k, 1), projection(k, 0));
          
          if (theta[k] < 0)
            theta[k] += 2 * M_PI;
        
          rho[k] = sqrt(pow(projection(k, 0), 2) + pow(projection(k, 1), 2));
        }
      
        std::vector<int> index(neighb_of_curr_point->points.size() - 1);
        std::iota(std::begin(index), std::end(index), 0);//fill index with {0,1,2,..}
        std::stable_sort(std::begin(index), std::end(index), [&](size_t i, size_t j){ return theta[i] < theta[j]; });
        std::stable_sort(std::begin(theta), std::end(theta));

        // curvatures vector reordered by theta 
        std::vector<double> k_aux1(k_aux.size());
        //curvatures vector reordered by theta
        for (size_t k = 0; k < k_aux.size(); k++)
        {
          k_aux1[k] = k_aux[index[k]];
        }

        std::vector<double>::iterator iter;
        for (iter = theta.begin(); iter != theta.end(); iter++)
        {
          std::vector<double>::iterator it;
          it = std::find(iter+1, theta.end(), *iter);
          if(it != theta.end())
          {
            *iter-= 0.0001;
          }
        }
        
        bool duplicate = false;        
        for (size_t k = 0; k < theta.size() - 1; k++)
        {
          if ( std::fabs(theta[k] - theta[k + 1]) < std::numeric_limits<double>::epsilon() )
            duplicate = true;
        }        
        
        if(!duplicate)
        {
          // prepares the knots for the spline interpolation
          std::vector<double> pos_ord2(k_aux1.size() + 2);
          float interval_begin = 1;
          std::iota(std::begin(pos_ord2), std::end(pos_ord2), interval_begin);
          float iteration = 0.1000;
          auto interval_stop = std::max_element(std::begin(pos_ord2), std::end(pos_ord2));
          float interval_end = *interval_stop;
        
          //Prepares the knots for the spline interpolation
          std::vector<double> k_aux2(k_aux1.size() + 2);
          //Curvature values used for the spline interpolation
          k_aux2[0] = k_aux1[k_aux1.size() - 1];
          for (int k = 0; k < k_aux1.size(); k++)
          {
            k_aux2[k + 1] = k_aux1[k];
          }
          k_aux2[k_aux1.size()+1] = k_aux1[0];
        
          Eigen::RowVectorXd theKnotVect(pos_ord2.size());
          Eigen::RowVectorXd theCurvatureValues(k_aux2.size());
          Eigen::DenseIndex degree = 3; 
          
          for (size_t k = 0; k < pos_ord2.size(); k++)
          {  
            theKnotVect(k) = this->getNormalizedValue(pos_ord2[k],interval_begin,interval_end);
            theCurvatureValues(k) = k_aux2[k];
          }
        
          // computing the spline interpolation
          Eigen::Spline<double, 1> theSpline = Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(theCurvatureValues, degree, theKnotVect);

          // reordering pos_aux by indices obtained from sorting theta values
          Eigen::MatrixXd pos_aux1(index.size(), 3);
          for (size_t k = 0; k < index.size(); k++)
          {
            pos_aux1(k, 0) = pos_aux(index[k], 0);
            pos_aux1(k, 1) = pos_aux(index[k], 1);
            pos_aux1(k, 2) = pos_aux(index[k], 2);
          }

          Eigen::MatrixXd pos_aux2(index.size() + 2, 3);
          pos_aux2(0, 0) = pos_aux1(index.size() - 1, 0);
          pos_aux2(0, 1) = pos_aux1(index.size() - 1, 1);
          pos_aux2(0, 2) = pos_aux1(index.size() - 1, 2);
          for (size_t k = 0; k < index.size(); k++)
          {
            pos_aux2(k + 1, 0) = pos_aux1(k, 0);
            pos_aux2(k + 1, 1) = pos_aux1(k, 1);
            pos_aux2(k + 1, 2) = pos_aux1(k, 2);
          }  
          pos_aux2(index.size() + 1, 0) = pos_aux1(0, 0);
          pos_aux2(index.size() + 1, 1) = pos_aux1(0, 1);
          pos_aux2(index.size() + 1, 2) = pos_aux1(0, 2);

          // getting curvatures values from spline interpolation
          std::vector<double> curv_sp;
          for (float k = 1; k <= pos_ord2[pos_ord2.size()-1]+iteration; k += iteration)
          {
            curv_sp.push_back(*theSpline(this->getNormalizedValue(k,interval_begin,interval_end)).data());
          }

          // finding the max curvature
          auto biggest = std::max_element(std::begin(curv_sp), std::end(curv_sp));
          float kmax = *biggest;
          int ind_k1 = std::distance(std::begin(curv_sp), biggest);

          // bins assignament        
          for (size_t k = 0; k < R_bins.size(); k++)
          {
            k1(cnt, k) = kmax * bin_rad[k];
          }

          // spline from Fig.1c
          std::vector<double> pos_aux2_1(index.size()+2);
          std::vector<double> pos_aux2_2(index.size()+2);
          std::vector<double> pos_aux2_3(index.size()+2);

          Eigen::VectorXi indices(index.size()+2);
          for (size_t k = 0; k < index.size() + 2; k++)
          {
            pos_aux2_1[k] = pos_aux2(k,0);
            pos_aux2_2[k] = pos_aux2(k,1);
            pos_aux2_3[k] = pos_aux2(k,2);
          }

          std::vector<double> derives_1(index.size() + 2);
          std::vector<double> derives_2(index.size() + 2);
          std::vector<double> derives_3(index.size() + 2);
          this->setSplinePchip(index.size()+2, pos_ord2, pos_aux2_1, derives_1);
          this->setSplinePchip(index.size()+2, pos_ord2, pos_aux2_2, derives_2);
          this->setSplinePchip(index.size()+2, pos_ord2, pos_aux2_3, derives_3);
          int c_numSamples = (float)(pos_ord2[pos_ord2.size()-1]-1+iteration) / iteration;
          std::vector<double> samples(c_numSamples), interpolations(c_numSamples);
          for (float k = 0; k < c_numSamples; k++) samples[k] = k*iteration + 1;
          std::vector<double> pos_spA, pos_spB, pos_spC;        
          for (float k = 0; k < c_numSamples; k++)
          {
            float percent = float(k) / float(c_numSamples - 1);
            float x = (pos_aux2_1.size() - 1) * percent;
                    int ind = int(floor(x));
            float xi = k*iteration + 1;
                    if (k == c_numSamples - 1) {
              ind--;
            }
            pos_spA.push_back(this->getHermiteDerivativeInterpolation({pos_ord2[ind],pos_aux2_1[ind],derives_1[ind]},{pos_ord2[ind+1],pos_aux2_1[ind+1],derives_1[ind+1]},xi));
            pos_spB.push_back(this->getHermiteDerivativeInterpolation({pos_ord2[ind],pos_aux2_2[ind],derives_2[ind]},{pos_ord2[ind+1],pos_aux2_2[ind+1],derives_2[ind+1]},xi));
            pos_spC.push_back(this->getHermiteDerivativeInterpolation({pos_ord2[ind],pos_aux2_3[ind],derives_3[ind]},{pos_ord2[ind+1],pos_aux2_3[ind+1],derives_3[ind+1]},xi));
          }
          
          // get coordinates of max curv value
          Eigen::Vector3d coord_k1;
          coord_k1(0, 0) = pos_spA[ind_k1];
          coord_k1(1, 0) = pos_spB[ind_k1];
          coord_k1(2, 0) = pos_spC[ind_k1];

          // minimum curvature(orthogonal coordinates)
          Eigen::MatrixXd matk1(3, pos_spA.size());
          Eigen::MatrixXd m_ck1(1, pos_spA.size());

          for (size_t k = 0; k < pos_spA.size(); k++)
          {
            matk1(0, k) = coord_k1(0, 0);
            matk1(1, k) = coord_k1(1, 0);
            matk1(2, k) = coord_k1(2, 0);

            m_ck1(0, k) = sqrt(pow(coord_k1(0, 0), 2) + pow(coord_k1(1, 0), 2) + pow(coord_k1(2, 0), 2));
          }
          
          Eigen::Vector3d v(3), w(3);
          std::vector<double> p_k2_vec;
          for (size_t k = 0; k < pos_spA.size(); k++)
          {
            v(0) = matk1(0, k); w(0) = pos_spA[k];
            v(1) = matk1(1, k); w(1) = pos_spB[k];
            v(2) = matk1(2, k); w(2) = pos_spC[k];
            p_k2_vec.push_back(abs(v.dot(w) / (m_ck1(0, k) * sqrt(pow(pos_spA[k], 2) + pow(pos_spB[k], 2) + pow(pos_spC[k], 2)))));
          }

          // finding the min curvature
          auto lowest = std::min_element(std::begin(p_k2_vec), std::end(p_k2_vec));
          int ind_k2 = std::distance(std::begin(p_k2_vec), lowest);
        
          // assigning to the radius bin
          for (size_t k = 0; k < R_bins.size(); k++)
          {
            k2(cnt, k) = curv_sp[ind_k2] * bin_rad[k];
          }
          cnt++;
        }
      }
    }
    
  }
  
  Eigen::MatrixXd desc_aux2(cnt,R_bins.size());
  if (cnt > 1)
  {
    // normalize curvatures to be between 0 and 1
    for (size_t u = 0; u < desc_aux2.rows(); u++)
    {
      for (size_t v = 0; v < desc_aux2.cols(); v++)
      {
        if (k1(u, v) > 1)
          k1(u, v) = 1;
        if (k2(u, v) > 1)
          k2(u, v) = 1;
        if (k1(u, v) < 0)
          k1(u, v) = 0;
        if (k2(u, v) < 0)
          k2(u, v) = 0;
        desc_aux2(u, v) = (k1(u, v) + k2(u, v)) / 2;
        if ( std::fabs(desc_aux2(u, v) - 0) < std::numeric_limits<double>::epsilon() )
          desc_aux2(u, v) = NAN;
      }
    }

    Eigen::MatrixXi aux2 = Eigen::MatrixXi::Constant(bins.size(),R_bins.size(),0);
    for (size_t u = 0; u < desc_aux2.rows(); u++)
    {
      for (size_t v = 0; v < desc_aux2.cols(); v++)
      {
        for (size_t b = 0; b < bins.size() - 1; b++)
        {
          if (!isnan(desc_aux2(u,v)) && desc_aux2(u,v) > bins[b] && (desc_aux2(u,v) < bins[b+1] || std::fabs(desc_aux2(u,v) - bins[b+1]) < std::numeric_limits<double>::epsilon()))
            aux2(b,v)++;
        }
      }
    }

    Eigen::Map<Eigen::RowVectorXi> scurv(aux2.data(), aux2.size());

    output.points.resize(1);
    output.width = 1;
    output.height = 1;
    for (int i=0; i<scurv.size(); i++)
      output.points[0].histogram[i] = scurv[i];
  }
}

#define PCL_INSTANTIATE_SCurVEstimation(T,NT) template class PCL_EXPORTS pcl::SCurVEstimation<T,NT>;

#endif    // PCL_FEATURES_IMPL_SCURV_H_

