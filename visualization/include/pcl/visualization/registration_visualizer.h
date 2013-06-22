/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_REGISTRATION_VISUALIZER_H_
#define PCL_REGISTRATION_VISUALIZER_H_

// PCL
#include <pcl/registration/registration.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl
{
  /** \brief @b RegistrationVisualizer represents the base class for rendering
   * the intermediate positions ocupied by the source point cloud during it's registration
   * to the target point cloud. A registration algorithm is considered as input and
   * it's covergence is rendered.
   * \author Gheorghe Lisca
   * \ingroup visualization
   */
  template<typename PointSource, typename PointTarget>
  class RegistrationVisualizer
  {

    public:
      /** \brief Empty constructor. */
      RegistrationVisualizer () : 
        viewer_ (),
        viewer_thread_ (),
        registration_method_name_ (),
        update_visualizer_ (),
        first_update_flag_ (),
        cloud_source_ (),
        cloud_target_ (),
        visualizer_updating_mutex_ (),
        cloud_intermediate_ (),
        cloud_intermediate_indices_ (),
        cloud_target_indices_ (),
        maximum_displayed_correspondences_ (0)
      {}

      /** \brief Set the registration algorithm whose intermediate steps will be rendered.
       * The method creates the local callback function pcl::RegistrationVisualizer::update_visualizer_ and
       * binds it to the local biffers update function pcl::RegistrationVisualizer::updateIntermediateCloud().
       * The local callback function pcl::RegistrationVisualizer::update_visualizer_ is then linked to
       * the pcl::Registration::update_visualizer_ callback function.
       * \param registration represents the registration method whose intermediate steps will be rendered.
       */
      bool
      setRegistration (pcl::Registration<PointSource, PointTarget> &registration)
      {
        // Update the name of the registration method to be desplayed
        registration_method_name_ = registration.getClassName();

        // Create the local callback function and bind it to the local function resposable for updating
        // the local buffers
        update_visualizer_ = boost::bind (&RegistrationVisualizer<PointSource, PointTarget>::updateIntermediateCloud,
                                          this, _1, _2, _3, _4);

        // Register the local callback function to the registration algorithm callback function
        registration.registerVisualizationCallback (this->update_visualizer_);

        // Flag that no visualizer update was done. It indicates to visualizer update function to copy
        // the registration input source and the target point clouds in the next call.
        visualizer_updating_mutex_.lock ();

        first_update_flag_ = false;

        visualizer_updating_mutex_.unlock ();

        return true;
      }

      /** \brief Start the viewer thread
       */
      void
      startDisplay ();

      /** \brief Stop the viewer thread
       */
      void
      stopDisplay ();

      /** \brief Updates visualizer local buffers cloud_intermediate, cloud_intermediate_indices, cloud_target_indices with
       * the newest registration intermediate results.
       * \param cloud_src represents the initial source point cloud
       * \param indices_src represents the incices of the intermediate source points used for the estimation of rigid transformation
       * \param cloud_tgt represents the target point cloud
       * \param indices_tgt represents the incices of the target points used for the estimation of rigid transformation
       */
      void
      updateIntermediateCloud (const pcl::PointCloud<PointSource> &cloud_src, const std::vector<int> &indices_src,
                               const pcl::PointCloud<PointTarget> &cloud_tgt, const std::vector<int> &indices_tgt);

      /** \brief Set maximum number of corresponcence lines whch will be rendered. */
      inline void
      setMaximumDisplayedCorrespondences (const int maximum_displayed_correspondences)
      {
        // This method is usualy called form other thread than visualizer thread
        // therefore same visualizer_updating_mutex_ will be used

        // Lock maximum_displayed_correspondences_
        visualizer_updating_mutex_.lock ();

        // Update maximum_displayed_correspondences_
        maximum_displayed_correspondences_ = maximum_displayed_correspondences;

        // Unlock maximum_displayed_correspondences_
        visualizer_updating_mutex_.unlock();
      }

      /** \brief Return maximum number of corresponcence lines which are rendered. */
      inline size_t
      getMaximumDisplayedCorrespondences()
      {
        return maximum_displayed_correspondences_;
      }

    private:
      /** \brief Initialize and run the visualization loop. This function will be runned in the internal thread viewer_thread_ */
      void
      runDisplay ();

      /** \brief Return the string obtained by concatenating a root_name and an id */
      inline std::string
      getIndexedName (std::string &root_name, size_t &id)
      {
        std::stringstream id_stream_;
        id_stream_ << id;
        std::string indexed_name_ = root_name + id_stream_.str ();
        return indexed_name_;
      }

      /** \brief The registration viewer. */
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

      /** \brief The thread running the runDisplay() function. */
      boost::thread viewer_thread_;

      /** \brief The name of the registration method whose intermediate results are rendered. */
      std::string registration_method_name_;

      /** \brief Callback function linked to pcl::Registration::update_visualizer_ */
      boost::function<void
      (const pcl::PointCloud<PointSource> &cloud_src, const std::vector<int> &indices_src, const pcl::PointCloud<
          PointTarget> &cloud_tgt, const std::vector<int> &indices_tgt)> update_visualizer_;

      /** \brief Updates source and target point clouds only for the first update call. */
      bool first_update_flag_;

      /** \brief The local buffer for source point cloud. */
      pcl::PointCloud<PointSource> cloud_source_;

      /** \brief The local buffer for target point cloud. */
      pcl::PointCloud<PointTarget> cloud_target_;

      /** \brief The mutex used for the sincronization of updating and rendering of the local buffers. */
      boost::mutex visualizer_updating_mutex_;

      /** \brief The local buffer for intermediate point cloud obtained during registration process. */
      pcl::PointCloud<PointSource> cloud_intermediate_;

      /** \brief The indices of intermediate points used for computation of rigid transformation. */
      std::vector<int> cloud_intermediate_indices_;

      /** \brief The indices of target points used for computation of rigid transformation. */
      std::vector<int> cloud_target_indices_;

      /** \brief The maximum number of displayed correspondences. */
      size_t maximum_displayed_correspondences_;

    };
}

#include <pcl/visualization/impl/registration_visualizer.hpp>

#endif  //#ifndef PCL_REGISTRATION_VISUALIZER_H_
