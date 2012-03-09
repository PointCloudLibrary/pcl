/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: automated_tree_segmentation.h $
 *
 */

#ifndef PCL_SEGMENTATION_AUTOMATED_TREE_SEGMENTATION_H_
#define PCL_SEGMENTATION_AUTOMATED_TREE_SEGMENTATION_H_

#include <pcl/segmentation/automated_segmentation.h>

namespace pcl
{
  /** \brief @b AutomatedTreeSegmentation attempts to segment trees from point clouds.
    * \details The algorithm within uses point location, curvature and intensity.
    * The curvature of points will be calculated if it is not present in the input cloud's PointType.
    * The intensity of points can be omitted but doing so could yield in slower and less accurate computations.
    * Initially developed for LIDAR scans.
    * <br><br>
    * Usage example:
    * \code
    * // Initialization (cloud_in, cloud_out and indices_of_trees are all pointers)
    * pcl::AutomatedTreeSegmentation<PointType> yggdrasil;
    * yggdrasil.setInputCloud (cloud_in);
    *
    * // We happen to know there are 3 trees in the cloud. This line is not required, but does aid the segmentation process
    * yggdrasil.setTargetNumberOfTrees (3);
    *
    * // Perform segmentation and return the point indices that make up the trees
    * yggdrasil.segment (*indices_of_trees);
    *
    * // Filter the detected trees from the input point cloud
    * pcl::ExtractIndices<PointType> filter;
    * filter.setInputCloud (cloud_in);
    * filter.setIndices (indices_of_trees);
    * filter.setNegative (true);
    * filter.filter (*cloud_out);
    * \endcode
    * \author Frits Florentinus
    * \ingroup segmentation
    */
  template<typename PointT>
  class AutomatedTreeSegmentation : public AutomatedSegmentation<PointT>
  {
    public:
      /** \brief Empty constructor. */
      AutomatedTreeSegmentation () :
          target_number_of_trees_ (-1)
      {
      }

      /** \brief Set the target number of trees that need be found by the segmentation algorithm.
        * \details This acts as a cost function for the algorithm, allowing an iterative process.
        * A value of -1 (default value) indicates that the algorithm should figure it out for himself.
        * \param[in] target_number_of_trees the new target number of trees.
        */
      inline void
      setTargetNumberOfTrees (int target_number_of_trees)
      {
        target_number_of_trees_ = target_number_of_trees;
      }

      /** \brief Get the target number of trees that need be found by the segmentation algorithm.
        * \return The current target number of trees.
        */
      inline int
      getTargetNumberOfTrees ()
      {
        return (target_number_of_trees_);
      }

    protected:
      typedef typename pcl::traits::fieldList<PointT>::type FieldList;

      /** \brief Should modify the <tt>segmentation_probabilities_</tt> array, indicating the likelihood that points
        * are part of the desired segmentation (1 = part of segmentation, 0 = not part of segmentation).
        * \details The reason to use this type of weighting instead of boolean classification is to be able to
        * build more advanced systems that use \b AutomatedSegmentation classes as intermediate steps (e.g. SRAM).
        * For this reason, the <tt>applySegmentation ()</tt> method should ideally modify the <tt>segmentation_probabilities_</tt>
        * adaptively; change the output probabilities relative to the input probabilities instead of setting them to fixed values.
        * \note The initial/default values of <tt>segmentation_probabilities_</tt> are 0.5 if the corresponding points are
        * indexed by <tt>indices_</tt> and 0 if not indexed.
        */
      void
      applySegmentation ();

    private:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using AutomatedSegmentation<PointT>::segmentation_probabilities_;

      /** \brief The target number of trees that need be found by the segmentation algorithm.
        * \details This acts as a cost function for the algorithm, allowing an iterative process.
        * A value of -1 (default value) indicates that the algorithm should figure it out for himself.
        */
      int target_number_of_trees_;
  };
}

#endif  //#ifndef PCL_SEGMENTATION_AUTOMATED_TREE_SEGMENTATION_H_

