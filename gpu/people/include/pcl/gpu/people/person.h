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
#include <pcl/gpu/people/tree_live.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      class Person
      {
        public:
          /** \brief Class constructor. */
          Person (std::string& tree_file) : 
                      max_cluster_size_(25000),
                      number_of_parts_(25),
                      number_of_trees_(1),
                      cluster_area_threshold_(200),
                      cluster_area_threshold_shs_(100),
                      cluster_tolerance_(0.05),
                      delta_hue_tolerance_(5),
                      elec_radius_scale_(1.0f),
                      elec_brute_force_border_(false),
                      do_shs_(true),
                      dilation_size_(2), 
                      name_("Generic")
          {
            /// Load the first tree
            std::ifstream fin (tree_file.c_str ());
            assert (fin.is_open ());
            multi_tree_live_proc_ = new pcl::gpu::people::trees::MultiTreeLiveProc (fin);
            fin.close ();
          };

          /** \brief Class destructor. */
          ~Person ()
          {};

          //// PUBLIC VARIABLES HERE ////
          pcl::gpu::DeviceArray<pcl::PointXYZRGB>   cloud_device_;
          pcl::gpu::DeviceArray<pcl::PointXYZRGBL>  labeled_device_;
          pcl::gpu::DeviceArray<unsigned short>     depth_device_;
          pcl::gpu::DeviceArray<char>               label_device_;
          pcl::gpu::DeviceArray<float>              float_depth_device_;

          //// PUBLIC METHODS /////
          /** \brief Loads an aditional tree to the RDF */
          int
          addTree (std::string& tree_file)
          {
            if(number_of_trees_ >= MAX_NR_TREES)
            {
              PCL_INFO ("Can't add another tree, we are already at max");
              return -1;
            }
            std::ifstream fin(tree_file.c_str() );
            if(!fin.is_open())
            {
              PCL_INFO ("Couldn't open this tree file");
              return -1;
            }
            multi_tree_live_proc_->addTree(fin);
            fin.close();
            number_of_trees_++;
            return 1;
          }

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

        protected:
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
          pcl::gpu::people::trees::MultiTreeLiveProc* multi_tree_live_proc_;
          std::string                                 name_;                  // Name of the person
          std::vector<float>                          max_part_size_;         // Max primary eigenvalue for each body part
          std::vector<std::vector<float> >            part_ideal_length_;     // Ideal length between two body parts
          std::vector<std::vector<float> >            max_length_offset_;     // Max allowed length offset between two body parts
      };
    }
  }
}
#endif // PCL_GPU_PEOPLE_PERSON_H_
