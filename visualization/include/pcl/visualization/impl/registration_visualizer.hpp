/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget> void
pcl::RegistrationVisualizer<PointSource, PointTarget>::startDisplay ()
{
  // Create and start the rendering thread. This will open the display window.
  viewer_thread_ = boost::thread (&pcl::RegistrationVisualizer<PointSource, PointTarget>::runDisplay, this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget> void
pcl::RegistrationVisualizer<PointSource, PointTarget>::stopDisplay ()
{
  // Stop the rendering thread. This will kill the display window.
  viewer_thread_.~thread ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget> void
pcl::RegistrationVisualizer<PointSource, PointTarget>::runDisplay ()
{
  // Open 3D viewer
  viewer_
      = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_->initCameraParameters ();

  // Create the handlers for the three point clouds buffers: cloud_source_, cloud_target_ and cloud_intermediate_
  pcl::visualization::PointCloudColorHandlerCustom<PointSource> cloud_source_handler_ (cloud_source_.makeShared (),
                                                                                       255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointTarget> cloud_target_handler_ (cloud_target_.makeShared (),
                                                                                       0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<PointSource> cloud_intermediate_handler_ (cloud_intermediate_.makeShared (),
                                                                                             255, 255, 0);

  // Create the view port for displaying initial source and target point clouds
  int v1 (0);
  viewer_->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer_->setBackgroundColor (0, 0, 0, v1);
  viewer_->addText ("Initial position of source and target point clouds", 10, 50, "title v1", v1);
  viewer_->addText ("Blue -> target", 10, 30, 0.0, 0.0, 1.0, "legend target v1", v1);
  viewer_->addText ("Red  -> source", 10, 10, 1.0, 0.0, 0.0, "legend source v1", v1);
  //
  viewer_->addPointCloud<PointSource> (cloud_source_.makeShared (), cloud_source_handler_, "cloud source v1", v1);
  viewer_->addPointCloud<PointTarget> (cloud_target_.makeShared (), cloud_target_handler_, "cloud target v1", v1);

  // Create the view port for displaying the registration process of source to target point cloud
  int v2 (0);
  viewer_->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  viewer_->setBackgroundColor (0.1, 0.1, 0.1, v2);
  std::string registration_port_title_ = "Registration using "+registration_method_name_;
  viewer_->addText (registration_port_title_, 10, 90, "title v2", v2);

  viewer_->addText ("Yellow -> intermediate", 10, 50, 1.0, 1.0, 0.0, "legend intermediate v2", v2);
  viewer_->addText ("Blue   -> target", 10, 30, 0.0, 0.0, 1.0, "legend target v2", v2);
  viewer_->addText ("Red    -> source", 10, 10, 1.0, 0.0, 0.0, "legend source v2", v1);

//    viewer_->addPointCloud<PointSource> (cloud_source_.makeShared (), cloud_source_handler_, "cloud source v2", v2);
  viewer_->addPointCloud<PointTarget> (cloud_target_.makeShared (), cloud_target_handler_, "cloud target v2", v2);
  viewer_->addPointCloud<PointSource> (cloud_intermediate_.makeShared (), cloud_intermediate_handler_,
                                       "cloud intermediate v2", v2);

  // Used to remove all old correspondences
  size_t  correspondeces_old_size = 0;

  // Add coordinate system to both ports
  viewer_->addCoordinateSystem (1.0, "global");

  // The root name of correspondence lines
  std::string line_root_ = "line";

  // Visualization loop
  while (!viewer_->wasStopped ())
  {
    // Lock access to visualizer buffers
    visualizer_updating_mutex_.lock ();

    // Updating intermediate point cloud
    // Remove old point cloud
    viewer_->removePointCloud ("cloud intermediate v2", v2);

    // Add the new point cloud
    viewer_->addPointCloud<PointSource> (cloud_intermediate_.makeShared (), cloud_intermediate_handler_,
                                           "cloud intermediate v2", v2);

    // Updating the correspondece lines

    std::string line_name_;
    // Remove the old correspondeces
    for (size_t correspondence_id = 0; correspondence_id < correspondeces_old_size; ++correspondence_id)
    {
      // Generate the line name
      line_name_ = getIndexedName (line_root_, correspondence_id);

      // Remove the current line according to it's name
      viewer_->removeShape (line_name_, v2);
    }

    // Display the new correspondences lines
    size_t correspondences_new_size = cloud_intermediate_indices_.size ();


    std::stringstream stream_;
    stream_ << "Random -> correspondences " << correspondences_new_size;
    viewer_->removeShape ("correspondences_size", 0);
    viewer_->addText (stream_.str(), 10, 70, 0.0, 1.0, 0.0, "correspondences_size", v2);

    // Display entire set of correspondece lines if no maximum displayed correspondences is set
    if( ( 0 < maximum_displayed_correspondences_ ) &&
        (maximum_displayed_correspondences_ < correspondences_new_size) )
      correspondences_new_size = maximum_displayed_correspondences_;

    // Actualize correspondeces_old_size
    correspondeces_old_size = correspondences_new_size;

    // Update new correspondence lines
    for (size_t correspondence_id = 0; correspondence_id < correspondences_new_size; ++correspondence_id)
    {
      // Generate random color for current correspondence line
      double random_red   = 255 * rand () / (RAND_MAX + 1.0);
      double random_green = 255 * rand () / (RAND_MAX + 1.0);
      double random_blue  = 255 * rand () / (RAND_MAX + 1.0);

      // Generate the name for current line
      line_name_ = getIndexedName (line_root_, correspondence_id);

      // Add the new correspondence line.
      viewer_->addLine (cloud_intermediate_[cloud_intermediate_indices_[correspondence_id]],
                        cloud_target_[cloud_target_indices_[correspondence_id]],
                        random_red, random_green, random_blue,
                        line_name_, v2);
    }

    // Unlock access to visualizer buffers
    visualizer_updating_mutex_.unlock ();

    // Render visualizer updated buffers
    viewer_->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));

  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget> void
pcl::RegistrationVisualizer<PointSource, PointTarget>::updateIntermediateCloud (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt)
{
  // Lock local buffers
  visualizer_updating_mutex_.lock ();

  // Update source and target point clouds if this is the first callback
  // Here we are sure that source and target point clouds are initialized
  if (!first_update_flag_)
  {
    first_update_flag_ = true;

    this->cloud_source_ = cloud_src;
    this->cloud_target_ = cloud_tgt;

    this->cloud_intermediate_ = cloud_src;
  }

  // Copy the intermediate point cloud and it's associates indices
  cloud_intermediate_ = cloud_src;
  cloud_intermediate_indices_ = indices_src;

  // Copy the intermediate indices associate to the target point cloud
  cloud_target_indices_ = indices_tgt;

  // Unlock local buffers
  visualizer_updating_mutex_.unlock ();
}
