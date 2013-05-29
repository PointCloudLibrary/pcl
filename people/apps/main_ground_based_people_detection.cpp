/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * main_ground_based_people_detection_app.cpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 *
 * Example file for performing people detection on a Kinect live stream.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */
  
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Mutex: //
boost::mutex cloud_mutex;

enum { COLS = 640, ROWS = 480 };

int print_help()
{
  cout << "*******************************************************" << std::endl;
  cout << "Ground based people detection app options:" << std::endl;
  cout << "   --help    <show_this_help>" << std::endl;
  cout << "   --svm     <path_to_svm_file>" << std::endl;
  cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>" << std::endl;
  cout << "   --min_h   <minimum_person_height (default = 1.3)>" << std::endl;
  cout << "   --max_h   <maximum_person_height (default = 2.3)>" << std::endl;
  cout << "*******************************************************" << std::endl;
  return 0;
}

void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud,
    bool* new_cloud_available_flag)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};
  
void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main (int argc, char** argv)
{
  if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
        return print_help();

  // Algorithm parameters:
  std::string svm_filename = "../../people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  float min_confidence = -1.5;
  float min_height = 1.3;
  float max_height = 2.3;
  float voxel_size = 0.06;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

  // Read if some parameters are passed from command line:
  pcl::console::parse_argument (argc, argv, "--svm", svm_filename);
  pcl::console::parse_argument (argc, argv, "--conf", min_confidence);
  pcl::console::parse_argument (argc, argv, "--min_h", min_height);
  pcl::console::parse_argument (argc, argv, "--max_h", max_height);

  // Read Kinect live stream:
  PointCloudT::Ptr cloud (new PointCloudT);
  bool new_cloud_available_flag = false;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
      boost::bind (&cloud_cb_, _1, cloud, &new_cloud_available_flag);
  interface->registerCallback (f);
  interface->start ();

  // Wait for the first frame:
  while(!new_cloud_available_flag) 
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  new_cloud_available_flag = false;

  cloud_mutex.lock ();    // for not overwriting the point cloud

  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Add point picking callback to viewer:
  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
  viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  // Spin until 'Q' is pressed:
  viewer.spin();
  std::cout << "done." << std::endl;
  
  cloud_mutex.unlock ();    

  // Ground plane estimation:
  Eigen::VectorXf ground_coeffs;
  ground_coeffs.resize(4);
  std::vector<int> clicked_points_indices;
  for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
    clicked_points_indices.push_back(i);
  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
  model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

  // Initialize new viewer:
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Create classifier for people detection:  
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setHeightLimits(min_height, max_height);         // set person classifier
//  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

  // For timing:
  static unsigned count = 0;
  static double last = pcl::getTime ();

  // Main loop:
  while (!viewer.wasStopped())
  {
    if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
    {
      new_cloud_available_flag = false;

      // Perform people detection on the new cloud:
      std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
      people_detector.setInputCloud(cloud);
      people_detector.setGround(ground_coeffs);                    // set floor coefficients
      people_detector.compute(clusters);                           // perform people detection

      ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

      // Draw cloud and people bounding boxes in the viewer:
      viewer.removeAllPointClouds();
      viewer.removeAllShapes();
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
      viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
      unsigned int k = 0;
      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
        {
          // draw theoretical person bounding box in the PCL viewer:
          it->drawTBoundingBox(viewer, k);
          k++;
        }
      }
      std::cout << k << " people found" << std::endl;
      viewer.spinOnce();

      // Display average framerate:
      if (++count == 30)
      {
        double now = pcl::getTime ();
        std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
        count = 0;
        last = now;
      }
      cloud_mutex.unlock ();
    }
  }

  return 0;
}

