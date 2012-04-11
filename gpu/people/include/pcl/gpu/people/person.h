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

#include <pcl/gpu/people/label_common.h>
#include <pcl/gpu/people/tree.h>

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
                      dilation_size_(2)
          {
            /// Load the first tree
            std::ifstream fin(tree_file.c_str() );
            assert(fin.is_open() );
            multi_tree_live_proc_ = new pcl::gpu::people::trees::MultiTreeLiveProc(fin);
            fin.close();
          };

          /** \brief Class destructor. */
          ~Person ()
          {};

          /** \brief Loads an aditional tree to the RDF */
          int
          addTree (std::string& tree_file)
          {
            if(number_of_trees_ >= MAX_NR_TREES)
            {
              std::cout << "Can't add another tree, we are already at max" << std::endl;
              return -1;
            }
            std::ifstream fin(tree_file.c_str() );
            if(!fin.is_open())
            {
              std::cout << "Couldn't open this tree file" << std::endl;
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

          inline void
          setMaxClusterSize (unsigned int max_cluster_size)
          {
            max_cluster_size_ = max_cluster_size;
          }

          inline unsigned int
          getMaxClusterSize () const
          {
            return (max_cluster_size_);
          }

          inline void
          setNumberOfParts (unsigned int number_of_parts)
          {
            number_of_parts_ = number_of_parts;
          }

          inline unsigned int
          getNumberOfParts () const
          {
            return (number_of_parts_);
          }

          inline void
          setNumberOfTrees (unsigned int number_of_trees)
          {
            number_of_trees_ = number_of_trees;
          }

          inline unsigned int
          getNumberOfTrees () const
          {
            return (number_of_trees_);
          }

          inline void
          setClusterAreaThreshold (unsigned int cluster_area_threshold)
          {
            cluster_area_threshold_ = cluster_area_threshold;
          }

          inline unsigned int
          getClusterAreaThreshold () const
          {
            return (cluster_area_threshold_);
          }

          inline void
          setClusterAreaThresholdSHS (unsigned int cluster_area_threshold_shs)
          {
            cluster_area_threshold_shs_ = cluster_area_threshold_shs;
          }

          inline unsigned int
          getClusterAreaThresholdSHS () const
          {
            return (cluster_area_threshold_shs_);
          }

          inline void
          setClusterTolerance (float cluster_tolerance)
          {
            cluster_tolerance_ = cluster_tolerance;
          }

          inline float
          getClusterTolerance () const
          {
            return (cluster_tolerance_);
          }

          inline void
          setDeltaHueTolerance (unsigned int delta_hue_tolerance)
          {
            delta_hue_tolerance_ = delta_hue_tolerance;
          }

          inline unsigned int
          getDeltaHueTolerance () const
          {
            return (delta_hue_tolerance_);
          }

          inline void
          setElecRadiusScale (float elec_radius_scale)
          {
            elec_radius_scale_ = elec_radius_scale;
          }

          inline float
          getElecRadiusScale () const
          {
            return (elec_radius_scale_);
          }

          inline void
          setElecBruteForceBorder (bool elec_brute_force_border)
          {
            elec_brute_force_border_ = elec_brute_force_border;
          }

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
      };
    }
  }
}
#endif // PCL_GPU_PEOPLE_PERSON_H_
