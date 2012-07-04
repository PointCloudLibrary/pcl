/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *
 */

#ifndef PCL_REGISTRATION_H_
#define PCL_REGISTRATION_H_

#include <boost/function.hpp>
#include <boost/bind.hpp>

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation.h>

namespace pcl
{
  /** \brief @b Registration represents the base registration class. 
    * All 3D registration methods should inherit from this class.
    * \author Radu Bogdan Rusu, Michael Dixon
    * \ingroup registration
    */
  template <typename PointSource, typename PointTarget>
  class Registration : public PCLBase<PointSource>
  {
    public:
      using PCLBase<PointSource>::initCompute;
      using PCLBase<PointSource>::deinitCompute;
      using PCLBase<PointSource>::input_;
      using PCLBase<PointSource>::indices_;

      typedef boost::shared_ptr< Registration<PointSource, PointTarget> > Ptr;
      typedef boost::shared_ptr< const Registration<PointSource, PointTarget> > ConstPtr;

      typedef typename pcl::KdTree<PointTarget> KdTree;
      typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;
     
      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

      typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;
      
      typedef typename pcl::registration::TransformationEstimation<PointSource, PointTarget> TransformationEstimation;
      typedef typename TransformationEstimation::Ptr TransformationEstimationPtr;
      typedef typename TransformationEstimation::ConstPtr TransformationEstimationConstPtr;

      /** \brief Empty constructor. */
      Registration () : reg_name_ (),
                        tree_ (new pcl::KdTreeFLANN<PointTarget>),
                        nr_iterations_(0),
                        max_iterations_(10),
                        ransac_iterations_ (0),
                        target_ (),
                        final_transformation_ (Eigen::Matrix4f::Identity ()),
                        transformation_ (Eigen::Matrix4f::Identity ()),
                        previous_transformation_ (Eigen::Matrix4f::Identity ()),
                        transformation_epsilon_ (0.0), 
                        euclidean_fitness_epsilon_ (-std::numeric_limits<double>::max ()),
                        corr_dist_threshold_ (std::sqrt (std::numeric_limits<double>::max ())),
                        inlier_threshold_ (0.05),
                        converged_ (false), 
                        min_number_correspondences_ (3), 
                        correspondence_distances_ (),
                        transformation_estimation_ (),
                        update_visualizer_ (NULL),
                        point_representation_ ()
      {
      }

      /** \brief destructor. */
      virtual ~Registration () {}

      /** \brief Provide a pointer to the transformation estimation object. (e.g., SVD, point to plane etc.) 
       *  \param te is the pointer to the corresponding transformation estimation object
       */
      void
      setTransformationEstimation (const TransformationEstimationPtr &te) { transformation_estimation_ = te; }

      /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to)
        * \param cloud the input point cloud target
        */
      virtual inline void 
      setInputTarget (const PointCloudTargetConstPtr &cloud);

      /** \brief Get a pointer to the input point cloud dataset target. */
      inline PointCloudTargetConstPtr const 
      getInputTarget () { return (target_ ); }

      /** \brief Get the final transformation matrix estimated by the registration method. */
      inline Eigen::Matrix4f 
      getFinalTransformation () { return (final_transformation_); }

      /** \brief Get the last incremental transformation matrix estimated by the registration method. */
      inline Eigen::Matrix4f 
      getLastIncrementalTransformation () { return (transformation_); }

      /** \brief Set the maximum number of iterations the internal optimization should run for.
        * \param nr_iterations the maximum number of iterations the internal optimization should run for
        */
      inline void 
      setMaximumIterations (int nr_iterations) { max_iterations_ = nr_iterations; }

      /** \brief Get the maximum number of iterations the internal optimization should run for, as set by the user. */
      inline int 
      getMaximumIterations () { return (max_iterations_); }

      /** \brief Set the number of iterations RANSAC should run for.
        * \param ransac_iterations is the number of iterations RANSAC should run for
        */
      inline void 
      setRANSACIterations (int ransac_iterations) { ransac_iterations_ = ransac_iterations; }

      /** \brief Get the number of iterations RANSAC should run for, as set by the user. */
      inline double 
      getRANSACIterations () { return (ransac_iterations_); }

      /** \brief Set the inlier distance threshold for the internal RANSAC outlier rejection loop.
        * 
        * The method considers a point to be an inlier, if the distance between the target data index and the transformed 
        * source index is smaller than the given inlier distance threshold. 
        * The value is set by default to 0.05m.
        * \param inlier_threshold the inlier distance threshold for the internal RANSAC outlier rejection loop
        */
      inline void 
      setRANSACOutlierRejectionThreshold (double inlier_threshold) { inlier_threshold_ = inlier_threshold; }

      /** \brief Get the inlier distance threshold for the internal outlier rejection loop as set by the user. */
      inline double 
      getRANSACOutlierRejectionThreshold () { return (inlier_threshold_); }

      /** \brief Set the maximum distance threshold between two correspondent points in source <-> target. If the 
        * distance is larger than this threshold, the points will be ignored in the alignment process.
        * \param distance_threshold the maximum distance threshold between a point and its nearest neighbor 
        * correspondent in order to be considered in the alignment process
        */
      inline void 
      setMaxCorrespondenceDistance (double distance_threshold) { corr_dist_threshold_ = distance_threshold; }

      /** \brief Get the maximum distance threshold between two correspondent points in source <-> target. If the 
        * distance is larger than this threshold, the points will be ignored in the alignment process.
        */
      inline double 
      getMaxCorrespondenceDistance () { return (corr_dist_threshold_); }

      /** \brief Set the transformation epsilon (maximum allowable difference between two consecutive 
        * transformations) in order for an optimization to be considered as having converged to the final 
        * solution.
        * \param epsilon the transformation epsilon in order for an optimization to be considered as having 
        * converged to the final solution.
        */
      inline void 
      setTransformationEpsilon (double epsilon) { transformation_epsilon_ = epsilon; }

      /** \brief Get the transformation epsilon (maximum allowable difference between two consecutive 
        * transformations) as set by the user.
        */
      inline double 
      getTransformationEpsilon () { return (transformation_epsilon_); }

      /** \brief Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before 
        * the algorithm is considered to have converged. 
        * The error is estimated as the sum of the differences between correspondences in an Euclidean sense, 
        * divided by the number of correspondences.
        * \param epsilon the maximum allowed distance error before the algorithm will be considered to have
        * converged
        */

      inline void 
      setEuclideanFitnessEpsilon (double epsilon) { euclidean_fitness_epsilon_ = epsilon; }

      /** \brief Get the maximum allowed distance error before the algorithm will be considered to have converged,
        * as set by the user. See \ref setEuclideanFitnessEpsilon
        */
      inline double 
      getEuclideanFitnessEpsilon () { return (euclidean_fitness_epsilon_); }

      /** \brief Provide a boost shared pointer to the PointRepresentation to be used when comparing points
        * \param point_representation the PointRepresentation to be used by the k-D tree
        */
      inline void
      setPointRepresentation (const PointRepresentationConstPtr &point_representation)
      {
        point_representation_ = point_representation;
      }

      /** \brief Register the user callback function which will be called from registration thread
       * in order to update point cloud obtained after each iteration
       * \param[in] visualizerCallback reference of the user callback function
       */
      template<typename FunctionSignature> inline bool
      registerVisualizationCallback (boost::function<FunctionSignature> &visualizerCallback)
      {
        if (visualizerCallback != NULL)
        {
          update_visualizer_ = visualizerCallback;
          return (true);
        }
        else
          return (false);
      }

      /** \brief Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
        * \param max_range maximum allowable distance between a point and its correspondence in the target 
        * (default: double::max)
        */
      inline double 
      getFitnessScore (double max_range = std::numeric_limits<double>::max ());

      /** \brief Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
        * from two sets of correspondence distances (distances between source and target points)
        * \param[in] distances_a the first set of distances between correspondences
        * \param[in] distances_b the second set of distances between correspondences
        */
      inline double 
      getFitnessScore (const std::vector<float> &distances_a, const std::vector<float> &distances_b);

      /** \brief Return the state of convergence after the last align run */
      inline bool 
      hasConverged () { return (converged_); }

      /** \brief Call the registration algorithm which estimates the transformation and returns the transformed source 
        * (input) as \a output.
        * \param output the resultant input transfomed point cloud dataset
        */
      inline void 
      align (PointCloudSource &output);

      /** \brief Call the registration algorithm which estimates the transformation and returns the transformed source 
        * (input) as \a output.
        * \param output the resultant input transfomed point cloud dataset
        * \param guess the initial gross estimation of the transformation
        */
      inline void 
      align (PointCloudSource &output, const Eigen::Matrix4f& guess);

      /** \brief Abstract class get name method. */
      inline const std::string&
      getClassName () const { return (reg_name_); }

    protected:
      /** \brief The registration method name. */
      std::string reg_name_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief The number of iterations the internal optimization ran for (used internally). */
      int nr_iterations_;

      /** \brief The maximum number of iterations the internal optimization should run for.
        * The default value is 10.
        */
      int max_iterations_;

      /** \brief The number of iterations RANSAC should run for. */
      int ransac_iterations_;

      /** \brief The input point cloud dataset target. */
      PointCloudTargetConstPtr target_;

      /** \brief The final transformation matrix estimated by the registration method after N iterations. */
      Eigen::Matrix4f final_transformation_;

      /** \brief The transformation matrix estimated by the registration method. */
      Eigen::Matrix4f transformation_;

      /** \brief The previous transformation matrix estimated by the registration method (used internally). */
      Eigen::Matrix4f previous_transformation_;

      /** \brief The maximum difference between two consecutive transformations in order to consider convergence 
        * (user defined). 
        */
      double transformation_epsilon_;

      /** \brief The maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the 
        * algorithm is considered to have converged. The error is estimated as the sum of the differences between 
        * correspondences in an Euclidean sense, divided by the number of correspondences.
        */
      double euclidean_fitness_epsilon_;

      /** \brief The maximum distance threshold between two correspondent points in source <-> target. If the 
        * distance is larger than this threshold, the points will be ignored in the alignement process.
        */
      double corr_dist_threshold_;

      /** \brief The inlier distance threshold for the internal RANSAC outlier rejection loop.
        * The method considers a point to be an inlier, if the distance between the target data index and the transformed 
        * source index is smaller than the given inlier distance threshold. The default value is 0.05. 
        */
      double inlier_threshold_;

      /** \brief Holds internal convergence state, given user parameters. */
      bool converged_;

      /** \brief The minimum number of correspondences that the algorithm needs before attempting to estimate the 
        * transformation. The default value is 3.
        */
      int min_number_correspondences_;

      /** \brief A set of distances between the points in the source cloud and their correspondences in the 
        * target.                                                                                           
        */                                                                                                  
      std::vector<float> correspondence_distances_;                                                              

      /** \brief A TransformationEstimation object, used to calculate the 4x4 rigid transformation. */
      TransformationEstimationPtr transformation_estimation_;

      /** \brief Callback function to update intermediate source point cloud position during it's registration
        * to the target point cloud.
        */
      boost::function<void(const pcl::PointCloud<PointSource> &cloud_src,
                           const std::vector<int> &indices_src,
                           const pcl::PointCloud<PointTarget> &cloud_tgt,
                           const std::vector<int> &indices_tgt)> update_visualizer_;

      /** \brief Search for the closest nearest neighbor of a given point.
        * \param cloud the point cloud dataset to use for nearest neighbor search
        * \param index the index of the query point
        * \param indices the resultant vector of indices representing the k-nearest neighbors
        * \param distances the resultant distances from the query point to the k-nearest neighbors
        */
      inline bool
      searchForNeighbors (const PointCloudSource &cloud, int index, 
                          std::vector<int> &indices, std::vector<float> &distances)
      {
        int k = tree_->nearestKSearch (cloud, index, 1, indices, distances);
        if (k == 0)
          return (false);
        return (true);
      }

    private:
 
      /** \brief Abstract transformation computation method with initial guess */
      virtual void 
      computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess) = 0;

      /** \brief The point representation used (internal). */
      PointRepresentationConstPtr point_representation_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
}

#include <pcl/registration/impl/registration.hpp>

#endif  //#ifndef PCL_REGISTRATION_H_
