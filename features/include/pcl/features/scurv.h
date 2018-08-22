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
 * $Id: pfh.hpp 5027 2012-03-12 03:10:45Z rusu $
 *
 */
#ifndef PCL_SCURV_H_
#define PCL_SCURV_H_

#include <pcl/features/feature.h>
#define GRIDSIZE 64
#define GRIDSIZE_H GRIDSIZE/2
#include <vector>

namespace pcl
{
  /** \brief @b SCurVEstimation compute and incorporate surface curvatures and distributions of local
    * surface point projections that represent flatness, concavity and convexity in a 3D object-centered
    * and view-dependent descriptor for a given point cloud dataset containing points.
    * For more information about the SCurV descriptor, see:
    * Antonio J Rodríguez-Sánchez, Sandor Szedmak and Justus Piater, "SCurV: A 3D descriptor for object classification",
    * IEEE/RSJ International Conference on Intelligento Robot Systems (IROS) 2015
    * \author Antonio J Rodríguez-Sánchez
    * \ingroup features
    */

  template <typename PointInT, typename PointNT,  typename PointOutT = pcl::SCurVSignature210>
  class SCurVEstimation: public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
	typedef boost::shared_ptr<SCurVEstimation<PointInT, PointNT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const SCurVEstimation<PointInT, PointNT, PointOutT> > ConstPtr;

      /** \brief Empty constructor. */
      SCurVEstimation () : lut_ (), local_cloud_ ()
      {
        feature_name_ = "SCurVEstimation";
        lut_.resize (GRIDSIZE);
        for (int i = 0; i < GRIDSIZE; ++i)
        {
          lut_[i].resize (GRIDSIZE);
          for (int j = 0; j < GRIDSIZE; ++j)
            lut_[i][j].resize (GRIDSIZE);
        }
        //lut_.resize (boost::extents[GRIDSIZE][GRIDSIZE][GRIDSIZE]);
        search_radius_ = 0;
        k_ = 5;
      }

      /** \brief ... */
      void
      compute (PointCloudOut &output);

    protected:
      /** \brief ... */
      void 
      computeFeature (PointCloudOut &output);

      /** \brief ... */
      void
      computeSCurV (PointCloudIn &pc, std::vector<float> &hist);
      
      /** \brief ... */
      void
      normalize2D (pcl::PointCloud<pcl::PointNormal> &cloudToBeNormalized, int min_range, int max_range);
      
      /** \brief ... */
      void
      positioningAtCenter (pcl::PointCloud<pcl::PointNormal> &cloudToBeNormalized);

      /** \brief ... */
      bool
      compare (int index1, int index2, std::vector<double> &data);
	
      /** \brief ... */
	double
      uvalue (double x, double low, double high);

      /** \brief ... */
      double
      HermiteDerivativeInterpolate (double x1, double x2, double f1, double f2, double d1, double d2, double xi);

      /** \brief ... */
      Eigen::MatrixXd
      svd_orthoBasisForTheNullSpace (const Eigen::MatrixXd &theMatrix);

      /** \brief ... */
      double
      pchst (double arg1, double arg2);

      /** \brief ... */
      void
      spline_pchip_set (int n, std::vector<double> x, std::vector<double> f, std::vector<double> &d);

    private:

      /** \brief ... */
      std::vector<std::vector<std::vector<int> > > lut_;
      
      /** \brief ... */
      PointCloudIn local_cloud_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/scurv.hpp>
#endif

#endif // #
