/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * @author: Koen Buys
 */

#ifndef PCL_GPU_PEOPLE_PERSON_H_
#define PCL_GPU_PEOPLE_PERSON_H_

#include <iostream>
#include <sstream>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/console/print.h>
#include <pcl/gpu/people/label_common.h>
#include <pcl/gpu/people/tree.h>

#include <opencv2/core/core.hpp>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        class MultiTreeLiveProc;
      };

      class PCL_EXPORTS Person
      {
        public:

          typedef boost::shared_ptr<Person> Ptr;
          typedef pcl::PointXYZRGB InputPointT;

          DeviceArray<InputPointT> cloud_device_;

          DeviceArray2D<unsigned short> depth_device_;
          DeviceArray2D<unsigned short> depth_device2_;

          DeviceArray2D<unsigned char> lmap_device_;
          DeviceArray2D<unsigned char> lmap_device2_;

          /** \brief Class constructor. */
          Person (std::string& tree_file);
          /** \brief Class destructor. */
          ~Person () {}

          //// PUBLIC METHODS /////
          /** \brief Loads an aditional tree to the RDF */
          int
          addTree (std::string& tree_file);

          /** \brief The actuall processing callback */
          void
          process (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

          /** \brief Read XML configuration file for a specific person */
          void 
          readPersonXMLConfig (std::istream& is);

          /** \brief Write XML configuration file for a specific person */
          void 
          writePersonXMLConfig (std::ostream& os);

          /** \brief Set the max number of pixels in a body part cluster, defaults to 25000 */
          inline void
          setMaxClusterSize (unsigned int max_cluster_size)
          {
            max_cluster_size_ = max_cluster_size;
          }

          /** \brief Get the max number of pixels in a body part cluster, defaults to 25000 */
          inline unsigned int
          getMaxClusterSize () const
          {
            return (max_cluster_size_);
          }

          /** \brief Set the number of body parts used in the RDF, defaults to 25 */
          inline void
          setNumberOfParts (unsigned int number_of_parts)
          {
            number_of_parts_ = number_of_parts;
          }
          /** \brief Get the number of body parts used in the RDF, defaults to 25 */
          inline unsigned int
          getNumberOfParts () const
          {
            return (number_of_parts_);
          }

          /** \brief Set the number of decision trees used in the RDF, defaults to one, max is 4 */
          inline void
          setNumberOfTrees (unsigned int number_of_trees)
          {
            number_of_trees_ = number_of_trees;
          }

          /** \brief Set the number of decision trees used in the RDF, defaults to one */
          inline unsigned int
          getNumberOfTrees () const
          {
            return (number_of_trees_);
          }

          /** \brief Set the minimal amount of pixels needed before accepting a body part cluster in Euclidean Labeled Clustering, defaults to 200 */
          inline void
          setClusterAreaThreshold (unsigned int cluster_area_threshold)
          {
            cluster_area_threshold_ = cluster_area_threshold;
          }

          /** \brief Get the minimal amount of pixels needed before accepting a body part cluster in Euclidean Labeled Clustering, defaults to 200 */
          inline unsigned int
          getClusterAreaThreshold () const
          {
            return (cluster_area_threshold_);
          }

          /** \brief Set the minimal amount of pixels needed before accepting a body part cluster in Seeded Hue Segmentation, defaults to 100 */
          inline void
          setClusterAreaThresholdSHS (unsigned int cluster_area_threshold_shs)
          {
            cluster_area_threshold_shs_ = cluster_area_threshold_shs;
          }

          /** \brief Set the minimal amount of pixels needed before accepting a body part cluster in Seeded Hue Segmentation, defaults to 100 */
          inline unsigned int
          getClusterAreaThresholdSHS () const
          {
            return (cluster_area_threshold_shs_);
          }
          /** \brief Set the sphere radius in which the cluster search and seeded hue segmentation will work, defaults to 0.05 */
          inline void
          setClusterTolerance (float cluster_tolerance)
          {
            cluster_tolerance_ = cluster_tolerance;
          }

          /** \brief Get the sphere radius in which the cluster search and seeded hue segmentation will work, defaults to 0.05 */
          inline float
          getClusterTolerance () const
          {
            return (cluster_tolerance_);
          }
          /** \brief Set the tolerance for the delta on the Hue in Seeded Hue Segmentation step */
          inline void
          setDeltaHueTolerance (unsigned int delta_hue_tolerance)
          {
            delta_hue_tolerance_ = delta_hue_tolerance;
          }

          /** \brief Get the tolerance for the delta on the Hue in Seeded Hue Segmentation step, defaults to 5 */
          inline unsigned int
          getDeltaHueTolerance () const
          {
            return (delta_hue_tolerance_);
          }

          /** \brief Set the scale of the radius for Euclidean Labeled Clustering */
          inline void
          setElecRadiusScale (float elec_radius_scale)
          {
            elec_radius_scale_ = elec_radius_scale;
          }

          /** \brief Get the scale of the radius for Euclidean Labeled Clustering */
          inline float
          getElecRadiusScale () const
          {
            return (elec_radius_scale_);
          }
          /** \brief Set if the Euclidean Labeled Cluster will do a brute force border search to refine it's result (=slow) */
          inline void
          setElecBruteForceBorder (bool elec_brute_force_border)
          {
            elec_brute_force_border_ = elec_brute_force_border;
          }

          /** \brief Get if the Euclidean Labeled Cluster will do a brute force border search to refine it's result (=slow) */
          inline bool
          getElecBruteForceBorder () const
          {
            return (elec_brute_force_border_);
          }

          /** \brief if set the proces step will do a second iteration with SHS */
          inline void
          setDoSHS (bool do_shs)
          {
            do_shs_ = do_shs;
          }
          /** \brief returns if the process step does or doesn't do a reiteration with SHS */
          inline bool
          getDoSHS () const
          {
            return (do_shs_);
          }
          /** \brief Sets how much the results from SHS are dilated before annotating them */
          inline void
          setDilationSize (unsigned int dilation_size)
          {
            dilation_size_ = dilation_size;
          }
          /** \brief Returns the dilation size in second iteration */
          inline unsigned int 
          getDilationSize () const
          {
            return (dilation_size_);
          }

          /** \brief Class getName method. */
          virtual std::string getClassName () const { return ("Person"); }

        public:
          unsigned int                                max_cluster_size_;
          unsigned int                                number_of_parts_;
          unsigned int                                number_of_trees_;
          unsigned int                                cluster_area_threshold_;
          unsigned int                                cluster_area_threshold_shs_;
          float                                       cluster_tolerance_;
          unsigned int                                delta_hue_tolerance_;
          float                                       elec_radius_scale_;
          bool                                        elec_brute_force_border_;
          bool                                        do_shs_;
          unsigned int                                dilation_size_;
          unsigned int                                counter_;

          boost::shared_ptr<trees::MultiTreeLiveProc> m_proc;
          cv::Mat                                     m_lmap;
          cv::Mat                                     m_cmap;
          cv::Mat                                     cmap;
          cv::Mat                                     m_bmap;
          std::string                                 name_;                  // Name of the person
          std::vector<float>                          max_part_size_;         // Max primary eigenvalue for each body part
          std::vector<std::vector<float> >            part_ideal_length_;     // Ideal length between two body parts
          std::vector<std::vector<float> >            max_length_offset_;     // Max allowed length offset between two body parts
      };
    }
  }
}
#endif // PCL_GPU_PEOPLE_PERSON_H_
