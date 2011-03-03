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
 * $Id: pfh.h 36113 2011-02-22 00:00:50Z rusu $
 *
 */

#ifndef PCL_PFH_H_
#define PCL_PFH_H_

#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief Compute the 4-tuple representation containing the three angles and one distance between two points
    * represented by Cartesian coordinates and normals.
    * \note For explanations about the features, please see the literature mentioned above (the order of the
    * features might be different).
    * \param p1 the first XYZ point
    * \param n1 the first surface normal
    * \param p2 the second XYZ point
    * \param n2 the second surface normal
    * \param f1 the first angular feature (angle between the projection of nq_idx and u)
    * \param f2 the second angular feature (angle between nq_idx and v)
    * \param f3 the third angular feature (angle between np_idx and |p_idx - q_idx|)
    * \param f4 the distance feature (p_idx - q_idx)
    */
  bool 
  computePairFeatures (const Eigen::Vector4f &p1, const Eigen::Vector4f &n1, 
                       const Eigen::Vector4f &p2, const Eigen::Vector4f &n2, 
                       float &f1, float &f2, float &f3, float &f4);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b PFHEstimation estimates the Point Feature Histogram (PFH) descriptor for a given point cloud dataset
    * containing points and normals.
    *
    * @note If you use this code in any academic work, please cite:
    *
    * <ul>
    * <li> R.B. Rusu, N. Blodow, Z.C. Marton, M. Beetz.
    *      Aligning Point Cloud Views using Persistent Feature Histograms.
    *      In Proceedings of the 21st IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS),
    *      Nice, France, September 22-26 2008.
    * </li>
    * <li> R.B. Rusu, Z.C. Marton, N. Blodow, M. Beetz.
    *      Learning Informative Point Classes for the Acquisition of Object Model Maps.
    *      In Proceedings of the 10th International Conference on Control, Automation, Robotics and Vision (ICARCV),
    *      Hanoi, Vietnam, December 17-20 2008.
    * </li>
    * </ul>
    *
    * @note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \ref FPFHEstimationOMP for examples on parallel implementations of the FPFH (Fast Point Feature Histogram).
    * \author Radu Bogdan Rusu
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class PFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn  PointCloudIn;

      /** \brief Empty constructor. */
      PFHEstimation () : nr_subdiv_ (5), d_pi_ (1.0 / (2.0 * M_PI))
      {
        feature_name_ = "PFHEstimation";
      };

      /** \brief Compute the 4-tuple representation containing the three angles and one distance between two points
        * represented by Cartesian coordinates and normals.
        * \note For explanations about the features, please see the literature mentioned above (the order of the
        * features might be different).
        * \param cloud the dataset containing the XYZ Cartesian coordinates of the two points
        * \param normals the dataset containing the surface normals (assuming normalized vectors) at each point in cloud
        * \param p_idx the index of the first point (source)
        * \param q_idx the index of the second point (target)
        * \param f1 the first angular feature (angle between the projection of nq_idx and u)
        * \param f2 the second angular feature (angle between nq_idx and v)
        * \param f3 the third angular feature (angle between np_idx and |p_idx - q_idx|)
        * \param f4 the distance feature (p_idx - q_idx)
        */
      bool 
      computePairFeatures (const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals, 
                           int p_idx, int q_idx, float &f1, float &f2, float &f3, float &f4);

      /** \brief Estimate the PFH (Point Feature Histograms) individual signatures of the three angular (f1, f2, f3)
        * features for a given point based on its spatial neighborhood of 3D points with normals
        * \param cloud the dataset containing the XYZ Cartesian coordinates of the two points
        * \param normals the dataset containing the surface normals at each point in \a cloud
        * \param indices the k-neighborhood point indices in the dataset
        * \param nr_split the number of subdivisions for each angular feature interval
        * \param pfh_histogram the resultant (combinatorial) PFH histogram representing the feature at the query point
        */
      void 
      computePointPFHSignature (const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals, 
                                const std::vector<int> &indices, int nr_split, Eigen::VectorXf &pfh_histogram);

      /** \brief Set the number of subdivisions for each angular feature interval.
        * \param nr_subdiv the number of subdivisions
        */
      inline void 
      setNrSubdivisions (int nr_subdiv) { nr_subdiv_ = nr_subdiv; }

      /** \brief Get the number of subdivisions for the feature interval.
        * \param nr_subdiv the resultant number of subdivisions as set by the user
        */
      inline void 
      getNrSubdivisions (int &nr_subdiv) { nr_subdiv = nr_subdiv_; }

    protected:

      /** \brief Estimate the Point Feature Histograms (PFH) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the PFH feature estimates
        */
      void 
      computeFeature (PointCloudOut &output);

    private:
      /** \brief The number of subdivisions for each angular feature interval. */
      int nr_subdiv_;

      /** \brief Placeholder for a point's PFH signature. */
      Eigen::VectorXf pfh_histogram_;

      /** \brief Placeholder for a PFH 4-tuple. */
      Eigen::Vector4f pfh_tuple_;

      /** \brief Placeholder for a histogram index. */
      int f_index_[3];

      /** \brief Float constant = 1.0 / (2.0 * M_PI) */
      float d_pi_; 
  };
}

#endif  //#ifndef PCL_PFH_H_


