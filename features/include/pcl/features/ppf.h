/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
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
 */

#ifndef PCL_PPF_H_
#define PCL_PPF_H_

#include <pcl/features/feature.h>
#include <pcl/features/boost.h>

namespace pcl
{
  /** \brief
    * \param[in] p1 
    * \param[in] n1
    * \param[in] p2 
    * \param[in] n2
    * \param[out] f1
    * \param[out] f2
    * \param[out] f3
    * \param[out] f4
    */
  PCL_EXPORTS bool
  computePPFPairFeature (const Eigen::Vector4f &p1, const Eigen::Vector4f &n1,
                         const Eigen::Vector4f &p2, const Eigen::Vector4f &n2,
                         float &f1, float &f2, float &f3, float &f4);


  /** \brief Class that calculates the "surflet" features for each pair in the given
    * pointcloud. Please refer to the following publication for more details:
    *    B. Drost, M. Ulrich, N. Navab, S. Ilic
    *    Model Globally, Match Locally: Efficient and Robust 3D Object Recognition
    *    2010 IEEE Conference on Computer Vision and Pattern Recognition (CVPR)
    *    13-18 June 2010, San Francisco, CA
    *
    * PointOutT is meant to be pcl::PPFSignature - contains the 4 values of the Surflet
    * feature and in addition, alpha_m for the respective pair - optimization proposed by
    * the authors (see above)
    *
    * \author Alexandru-Eugen Ichim
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class PPFEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<PPFEstimation<PointInT, PointNT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const PPFEstimation<PointInT, PointNT, PointOutT> > ConstPtr;
      using PCLBase<PointInT>::indices_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef pcl::PointCloud<PointOutT> PointCloudOut;

      /** \brief Empty Constructor. */
      PPFEstimation ();


    private:
      /** \brief The method called for actually doing the computations
        * \param[out] output the resulting point cloud (which should be of type pcl::PPFSignature);
        * its size is the size of the input cloud, squared (i.e., one point for each pair in
        * the input cloud);
        */
      void
      computeFeature (PointCloudOut &output);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/ppf.hpp>
#endif

#endif // PCL_PPF_H_
