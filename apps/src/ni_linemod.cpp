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
 * $Id: openni_viewer.cpp 5059 2012-03-14 02:12:17Z gedikli $
 *
 */

#include <boost/thread/thread.hpp>
#include <pcl/apps/timer.h>
#include <pcl/common/angles.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/search/organized.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

using namespace pcl;
using namespace std;

typedef PointXYZRGBA PointT;

#define SHOW_FPS 1

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class NILinemod
{
  public:
    typedef PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    bool added;

    NILinemod (Grabber& grabber)
      : cloud_viewer_ ("PointCloud")
      , grabber_ (grabber)
      , image_viewer_ ("Image")
      , first_frame_ (true)
    {
      added = false;
    }

    /////////////////////////////////////////////////////////////////////////
    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
      search_.setInputCloud (cloud);
    }

    CloudConstPtr
    getLatestCloud ()
    {
      // Lock while we swap our cloud and reset it.
      boost::mutex::scoped_lock lock (cloud_mutex_);
      CloudConstPtr temp_cloud;
      temp_cloud.swap (cloud_);
      return (temp_cloud);
    }

    /////////////////////////////////////////////////////////////////////////
    void 
    keyboard_callback (const visualization::KeyboardEvent& event, void*)
    {
      //if (event.getKeyCode())
      //  cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
      //else
      //  cout << "the special key \'" << event.getKeySym() << "\' was";
      //if (event.keyDown())
      //  cout << " pressed" << endl;
      //else
      //  cout << " released" << endl;
    }
    
    /////////////////////////////////////////////////////////////////////////
    void 
    mouse_callback (const visualization::MouseEvent& mouse_event, void*)
    {
      if (mouse_event.getType() == visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == visualization::MouseEvent::LeftButton)
      {
        cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    /** \brief Given a plane, and the set of inlier indices representing it,
      * segment out all the objects supported by it.
      */
    void
    segmentObject (int picked_idx, 
                   const CloudConstPtr &cloud, 
                   const PointIndices::Ptr &plane_indices, 
                   Cloud &object)
    {
      CloudPtr plane_hull (new Cloud);

      // Compute the convex hull
      ConvexHull<PointT> chull;
      chull.setDimension (2);
      chull.setInputCloud (cloud);
      chull.setIndices (plane_indices);
      chull.reconstruct (*plane_hull);

      // Remove the plane indices from the data
      PointIndices::Ptr everything_but_the_plane (new PointIndices);
      std::vector<int> indices_fullset (cloud->points.size ());
      for (int p_it = 0; p_it < static_cast<int> (indices_fullset.size ()); ++p_it)
        indices_fullset[p_it] = p_it;
      std::vector<int> indices_subset = plane_indices->indices;
      std::sort (indices_subset.begin (), indices_subset.end ());
      set_difference (indices_fullset.begin (), indices_fullset.end (), 
                      indices_subset.begin (), indices_subset.end (), 
                      inserter (everything_but_the_plane->indices, everything_but_the_plane->indices.begin ()));

      // Extract all clusters on the hull
      PointIndices::Ptr points_above_plane (new PointIndices);
      ExtractPolygonalPrismData<PointT> exppd;
      exppd.setInputCloud (cloud);
      exppd.setInputPlanarHull (plane_hull);
      exppd.setIndices (everything_but_the_plane);
      exppd.setHeightLimits (0.0, 0.5);
      exppd.segment (*points_above_plane);

      // Use an organized clustering segmentation to extract the individual clusters
      EuclideanClusterComparator<PointT, Normal, Label>::Ptr euclidean_cluster_comparator (new EuclideanClusterComparator<PointT, Normal, Label>);
      euclidean_cluster_comparator->setInputCloud (cloud);
      euclidean_cluster_comparator->setDistanceThreshold (0.03, false);
      // Set the entire scene to false, and the inliers of the objects located on top of the plane to true
      Label l; l.label = 0;
      PointCloud<Label>::Ptr scene (new PointCloud<Label> (cloud->width, cloud->height, l));
      // Mask the objects that we want to split into clusters
      for (int i = 0; i < static_cast<int> (points_above_plane->indices.size ()); ++i)
        scene->points[points_above_plane->indices[i]].label = 1;
      euclidean_cluster_comparator->setLabels (scene);

      vector<bool> exclude_labels (2);  exclude_labels[0] = true; exclude_labels[1] = false;
      euclidean_cluster_comparator->setExcludeLabels (exclude_labels);

      OrganizedConnectedComponentSegmentation<PointT, Label> euclidean_segmentation (euclidean_cluster_comparator);
      euclidean_segmentation.setInputCloud (cloud);

      PointCloud<Label> euclidean_labels;
      vector<PointIndices> euclidean_label_indices;
      euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

      // For each cluster found
      bool cluster_found = false;
      for (size_t i = 0; i < euclidean_label_indices.size (); i++)
      {
        if (cluster_found)
          break;
        // Check if the point that we picked belongs to it
        for (size_t j = 0; j < euclidean_label_indices[i].indices.size (); ++j)
        {
          if (picked_idx != euclidean_label_indices[i].indices[j])
            continue;
          //pcl::PointCloud<PointT> cluster;
          pcl::copyPointCloud (*cloud, euclidean_label_indices[i].indices, object);
          std::cerr << "inliers: " << euclidean_label_indices[i].indices.size () << std::endl;
          cluster_found = true;
          break;
          //object_indices = euclidean_label_indices[i].indices;
          //clusters.push_back (cluster);
        }
      }
    }


    /////////////////////////////////////////////////////////////////////////
    void
    segment (const PointT &picked_point, 
             int picked_idx,
             PlanarRegion<PointT> &region,
             PointIndices &indices)
    {
      // First frame is segmented using an organized multi plane segmentation approach from points and their normals
      if (first_frame_)
      {
        if (search_.getInputCloud ()->points.empty ())
        {
          PCL_ERROR ("[segment] Input cloud empty!\n");
          return;
        }
        // Estimate normals in the cloud
        IntegralImageNormalEstimation<PointT, Normal> ne;
        ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        ne.setMaxDepthChangeFactor (0.02f);
        ne.setNormalSmoothingSize (20.0f);
        PointCloud<Normal>::Ptr normal_cloud (new PointCloud<Normal>);
        ne.setInputCloud (search_.getInputCloud ());
        ne.compute (*normal_cloud);

        if (normal_cloud->points.empty ())
        {
          PCL_ERROR ("[segment] Estimated normals empty!\n");
          return;
        }

        // Segment out all planes
        OrganizedMultiPlaneSegmentation<PointT, Normal, Label> mps;
        mps.setMinInliers (5000);
        mps.setAngularThreshold (pcl::deg2rad (3.0)); // 3 degrees
        mps.setDistanceThreshold (0.02); // 2 cm
        mps.setMaximumCurvature (0.001); // a small curvature
        mps.setInputNormals (normal_cloud);
        mps.setInputCloud (search_.getInputCloud ());

        vector<PlanarRegion<PointT> > regions;
        vector<ModelCoefficients> model_coefficients;
        vector<PointIndices> inlier_indices;  
        PointCloud<Label>::Ptr labels (new PointCloud<Label>);
        vector<PointIndices> label_indices;
        vector<PointIndices> boundary_indices;
        PointIndices::Ptr plane_boundaries;
        mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
        //mps.segmentAndRefine (regions);//, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
        PCL_INFO ("Number of planar regions detected: %zu for a cloud of %zu points and %zu normals.\n", regions.size (), search_.getInputCloud ()->points.size (), normal_cloud->points.size ());

        double max_dist = numeric_limits<double>::max ();
        // Compute the distances from all the planar regions to the picked point, and select the closest region
        int idx = -1;
        for (size_t i = 0; i < regions.size (); ++i)
        {
          double dist = pointToPlaneDistance (picked_point, regions[i].getCoefficients ()); 
          if (dist < max_dist)
          {
            max_dist = dist;
            idx = i;
          }
        }

        // Get the plane that holds the object of interest
        if (idx != -1)
        {
          std::cerr << "found region closest with idx " << idx << std::endl;
          region = regions[idx]; 
          plane_boundaries.reset (new PointIndices (inlier_indices[idx]));
        }

        // Segment the object of interest
        if (plane_boundaries && !plane_boundaries->indices.empty ())
        {
          CloudPtr object (new Cloud);
          segmentObject (picked_idx, search_.getInputCloud (), plane_boundaries, *object);
          visualization::PointCloudColorHandlerCustom<PointT> red (object, 255, 0, 0);
          if (!cloud_viewer_.updatePointCloud (object, red, "object"))
            cloud_viewer_.addPointCloud (object, red, "object");
        }
        //first_frame_ = false;
      }
      // Subsequent frames are segmented by "tracking" the parameters of the previous frame
      // We do this by using the estimated inliers from previous frames in the current frame, 
      // and refining the coefficients
      else
      {
        SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (SACMODEL_PLANE);
        seg.setMethodType (SAC_RANSAC);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (search_.getInputCloud ());
        //seg.setIndices (indices);
        ModelCoefficients coefficients;
        PointIndices inliers;
        seg.segment (inliers, coefficients);
      }
    }

    /////////////////////////////////////////////////////////////////////////
    void 
    pp_callback (const visualization::PointPickingEvent& event, void*)
    {
      int idx = event.getPointIndex ();
      if (idx == -1)
        return;

      vector<int> indices (1);
      vector<float> distances (1);

      {
        // Use mutices to make sure we get the right cloud
        boost::mutex::scoped_lock lock1 (cloud_mutex_);

        PointT picked_pt;
        event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);

        stringstream ss;
        ss << "sphere_" << idx;
        cloud_viewer_.addSphere (picked_pt, 0.01, 1.0, 0.0, 0.0, ss.str ());

        if (!search_.isValid ())
          return;

        search_.nearestKSearch (picked_pt, 1, indices, distances);

        uint32_t width  = search_.getInputCloud ()->width,
                 height = search_.getInputCloud ()->height;
 
        int v = height - indices[0] / width,
            u = indices[0] % width;
        image_viewer_.addCircle (u, v, 5, 1.0, 0.0, 0.0, "circles", 1.0);
        image_viewer_.addBox (u-5, u+5, v-5, v+5, 0.0, 1.0, 0.0, "boxes", 0.5);
        image_viewer_.markPoint (u, v, visualization::red_color, visualization::blue_color, 10);
        added = !added;

        PlanarRegion<PointT> region;
        PointIndices region_indices;
        segment (picked_pt, indices[0], region, region_indices);

        if (region.getContour ().empty ())
        {
          PCL_ERROR ("No region detected. Please select another point and continue.\n");
          return;
        }
        else
        {
          cloud_viewer_.addPolygon (region, 0.0, 1.0, 0.0);
          cloud_viewer_.setShapeRenderingProperties (visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "polygon");
        }

/*        search_.nearestKSearch (region.getContour ()[0], 1, indices, distances);
        int v_p = height - indices[0] / width,
            u_p = indices[0] % width;
        for (int i = 1; i < region.getContour ().size (); ++i)
        {
          search_.nearestKSearch (region.getContour ()[i], 1, indices, distances);
          int v = height - indices[0] / width,
              u = indices[0] % width;

          image_viewer_.addLine (u_p, v_p, u, v, 0.0, 1.0, 0.0);
          v_p = v; u_p = u;
        }*/
      }
    }
    
    /////////////////////////////////////////////////////////////////////////
    void
    init ()
    {
      cloud_viewer_.registerMouseCallback (&NILinemod::mouse_callback, *this);
      cloud_viewer_.registerKeyboardCallback(&NILinemod::keyboard_callback, *this);
      cloud_viewer_.registerPointPickingCallback (&NILinemod::pp_callback, *this);
      boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&NILinemod::cloud_callback, this, _1);
      cloud_connection = grabber_.registerCallback (cloud_cb);
      
      image_viewer_.registerMouseCallback (&NILinemod::mouse_callback, *this);
      image_viewer_.registerKeyboardCallback(&NILinemod::keyboard_callback, *this);
    }

    /////////////////////////////////////////////////////////////////////////
    void
    run ()
    {
      grabber_.start ();
      
      bool image_init = false, cloud_init = false;
      unsigned char* rgb_data = 0;
      unsigned rgb_data_size = 0;

      while (!cloud_viewer_.wasStopped () && !image_viewer_.wasStopped ())
      {
        if (cloud_)
        {
          CloudConstPtr cloud = getLatestCloud ();
          if (!cloud_init)
          {
            cloud_viewer_.setPosition (0, 0);
            cloud_viewer_.setSize (cloud->width, cloud->height);
            cloud_init = !cloud_init;
          }

          if (!cloud_viewer_.updatePointCloud (cloud, "OpenNICloud"))
          {
            cloud_viewer_.addPointCloud (cloud, "OpenNICloud");
            cloud_viewer_.resetCameraViewpoint ("OpenNICloud");
          }

          if (!image_init)
          {
            image_viewer_.setPosition (cloud->width, 0);
            image_viewer_.setSize (cloud->width, cloud->height);
            image_init = !image_init;
          }

          image_viewer_.showRGBImage<PointT> (cloud);
        }

        cloud_viewer_.spinOnce ();
        image_viewer_.spinOnce ();
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber_.stop ();
      
      cloud_connection.disconnect ();
    }
    
    visualization::PCLVisualizer cloud_viewer_;
    Grabber& grabber_;
    boost::mutex cloud_mutex_;
    CloudConstPtr cloud_;
    
    visualization::ImageViewer image_viewer_;

    search::OrganizedNeighbor<PointT> search_;
  private:
    boost::signals2::connection cloud_connection, image_connection;
    bool first_frame_;
};

/* ---[ */
int
main (int, char**)
{
  string device_id ("#1");
  OpenNIGrabber grabber (device_id);
  NILinemod openni_viewer (grabber);

  openni_viewer.init ();
  openni_viewer.run ();
  
  return (0);
}
/* ]--- */
