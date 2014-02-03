/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * $Id: openni_viewer.cpp 5059 2012-03-14 02:12:17Z gedikli $
 *
 */

#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl/common/angles.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/geometry/polygon_operations.h>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

using namespace pcl;
using namespace pcl::console;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class ObjectSelection
{
  public:
    ObjectSelection ()
      : plane_comparator_ (new EdgeAwarePlaneComparator<PointT, Normal>)
      , rgb_data_ ()
    { 
      // Set the parameters for planar segmentation
      plane_comparator_->setDistanceThreshold (0.01f, false);
    }

    /////////////////////////////////////////////////////////////////////////
    virtual ~ObjectSelection ()
    {
      if (rgb_data_)
        delete[] rgb_data_;
    }

    /////////////////////////////////////////////////////////////////////////
    void
    estimateNormals (const typename PointCloud<PointT>::ConstPtr &input, PointCloud<Normal> &normals)
    {
      if (input->isOrganized ())
      {
        IntegralImageNormalEstimation<PointT, Normal> ne;
        // Set the parameters for normal estimation
        ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        ne.setMaxDepthChangeFactor (0.02f);
        ne.setNormalSmoothingSize (20.0f);
        // Estimate normals in the cloud
        ne.setInputCloud (input);
        ne.compute (normals);

        // Save the distance map for the plane comparator
        float *map=ne.getDistanceMap ();// This will be deallocated with the IntegralImageNormalEstimation object...
        distance_map_.assign(map, map+input->size() ); //...so we must copy the data out
        plane_comparator_->setDistanceMap(distance_map_.data());
      }
      else
      {
        NormalEstimation<PointT, Normal> ne;
        ne.setInputCloud (input);
        ne.setRadiusSearch (0.02f);
        ne.setSearchMethod (search_);
        ne.compute (normals);
      }
    }

    /////////////////////////////////////////////////////////////////////////
    void 
    keyboard_callback (const visualization::KeyboardEvent&, void*)
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
      * segment out the object of intererest supported by it. 
      *
      * \param[in] picked_idx the index of a point on the object
      * \param[in] cloud the full point cloud dataset
      * \param[in] plane_indices a set of indices representing the plane supporting the object of interest
      * \param[out] object the segmented resultant object 
      */
    void
    segmentObject (int picked_idx, 
                   const typename PointCloud<PointT>::ConstPtr &cloud, 
                   const PointIndices::Ptr &plane_indices, 
                   PointCloud<PointT> &object)
    {
      typename PointCloud<PointT>::Ptr plane_hull (new PointCloud<PointT>);

      // Compute the convex hull of the plane
      ConvexHull<PointT> chull;
      chull.setDimension (2);
      chull.setInputCloud (cloud);
      chull.setIndices (plane_indices);
      chull.reconstruct (*plane_hull);

      // Remove the plane indices from the data
      typename PointCloud<PointT>::Ptr plane (new PointCloud<PointT>);
      ExtractIndices<PointT> extract (true);
      extract.setInputCloud (cloud);
      extract.setIndices (plane_indices);
      extract.setNegative (false);
      extract.filter (*plane);
      PointIndices::Ptr indices_but_the_plane (new PointIndices);
      extract.getRemovedIndices (*indices_but_the_plane);

      // Extract all clusters above the hull
      PointIndices::Ptr points_above_plane (new PointIndices);
      ExtractPolygonalPrismData<PointT> exppd;
      exppd.setInputCloud (cloud);
      exppd.setIndices (indices_but_the_plane);
      exppd.setInputPlanarHull (plane_hull);
      exppd.setViewPoint (cloud->points[picked_idx].x, cloud->points[picked_idx].y, cloud->points[picked_idx].z);
      exppd.setHeightLimits (0.001, 0.5);           // up to half a meter
      exppd.segment (*points_above_plane);

      vector<PointIndices> euclidean_label_indices;
      // Prefer a faster method if the cloud is organized, over EuclidanClusterExtraction
      if (cloud_->isOrganized ())
      {
        // Use an organized clustering segmentation to extract the individual clusters
        typename EuclideanClusterComparator<PointT, Normal, Label>::Ptr euclidean_cluster_comparator (new EuclideanClusterComparator<PointT, Normal, Label>);
        euclidean_cluster_comparator->setInputCloud (cloud);
        euclidean_cluster_comparator->setDistanceThreshold (0.03f, false);
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
        euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
      }
      else
      {
        print_highlight (stderr, "Extracting individual clusters from the points above the reference plane ");
        TicToc tt; tt.tic ();

        EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setSearchMethod (search_);
        ec.setInputCloud (cloud);
        ec.setIndices (points_above_plane);
        ec.extract (euclidean_label_indices);
        
        print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%zu", euclidean_label_indices.size ()); print_info (" clusters]\n");
      }

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
          copyPointCloud (*cloud, euclidean_label_indices[i].indices, object);
          cluster_found = true;
          break;
        }
      }
    }

    /////////////////////////////////////////////////////////////////////////
    void
    segment (const PointT &picked_point, 
             int picked_idx,
             PlanarRegion<PointT> &region,
             typename PointCloud<PointT>::Ptr &object)
    {
      object.reset ();

      // Segment out all planes
      vector<ModelCoefficients> model_coefficients;
      vector<PointIndices> inlier_indices, boundary_indices;
      vector<PlanarRegion<PointT>, Eigen::aligned_allocator<PlanarRegion<PointT> > > regions;

      // Prefer a faster method if the cloud is organized, over RANSAC
      if (cloud_->isOrganized ())
      {
        print_highlight (stderr, "Estimating normals ");
        TicToc tt; tt.tic ();
        // Estimate normals
        PointCloud<Normal>::Ptr normal_cloud (new PointCloud<Normal>);
        estimateNormals (cloud_, *normal_cloud);
        print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%zu", normal_cloud->size ()); print_info (" points]\n");

        OrganizedMultiPlaneSegmentation<PointT, Normal, Label> mps;
        mps.setMinInliers (1000);
        mps.setAngularThreshold (deg2rad (3.0)); // 3 degrees
        mps.setDistanceThreshold (0.03); // 2 cm
        mps.setMaximumCurvature (0.001); // a small curvature
        mps.setProjectPoints (true);
        mps.setComparator (plane_comparator_);
        mps.setInputNormals (normal_cloud);
        mps.setInputCloud (cloud_);

        // Use one of the overloaded segmentAndRefine calls to get all the information that we want out
        PointCloud<Label>::Ptr labels (new PointCloud<Label>);
        vector<PointIndices> label_indices;
        mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
      }
      else
      {
        SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (SACMODEL_PLANE);
        seg.setMethodType (SAC_RANSAC);
        seg.setMaxIterations (10000);
        seg.setDistanceThreshold (0.005);

        // Copy XYZ and Normals to a new cloud
        typename PointCloud<PointT>::Ptr cloud_segmented (new PointCloud<PointT> (*cloud_));
        typename PointCloud<PointT>::Ptr cloud_remaining (new PointCloud<PointT>);

        ModelCoefficients coefficients;
        ExtractIndices<PointT> extract;
        PointIndices::Ptr inliers (new PointIndices ());

        // Up until 30% of the original cloud is left
        int i = 1;
        while (double (cloud_segmented->size ()) > 0.3 * double (cloud_->size ()))
        {
          seg.setInputCloud (cloud_segmented);

          print_highlight (stderr, "Searching for the largest plane (%2.0d) ", i++);
          TicToc tt; tt.tic ();
          seg.segment (*inliers, coefficients);
          print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%zu", inliers->indices.size ()); print_info (" points]\n");
 
          // No datasets could be found anymore
          if (inliers->indices.empty ())
            break;

          // Save this plane
          PlanarRegion<PointT> region;
          region.setCoefficients (coefficients);
          regions.push_back (region);

          inlier_indices.push_back (*inliers);
          model_coefficients.push_back (coefficients);

          // Extract the outliers
          extract.setInputCloud (cloud_segmented);
          extract.setIndices (inliers);
          extract.setNegative (true);
          extract.filter (*cloud_remaining);
          cloud_segmented.swap (cloud_remaining);
        }
      }
      print_highlight ("Number of planar regions detected: %zu for a cloud of %zu points\n", regions.size (), cloud_->size ());

      double max_dist = numeric_limits<double>::max ();
      // Compute the distances from all the planar regions to the picked point, and select the closest region
      int idx = -1;
      for (size_t i = 0; i < regions.size (); ++i)
      {
        double dist = pointToPlaneDistance (picked_point, regions[i].getCoefficients ()); 
        if (dist < max_dist)
        {
          max_dist = dist;
          idx = static_cast<int> (i);
        }
      }

      // Get the plane that holds the object of interest
      if (idx != -1)
      {
        plane_indices_.reset (new PointIndices (inlier_indices[idx]));

        if (cloud_->isOrganized ())
        {
          approximatePolygon (regions[idx], region, 0.01f, false, true);
          print_highlight ("Planar region: %zu points initial, %zu points after refinement.\n", regions[idx].getContour ().size (), region.getContour ().size ());
        }
        else
        {
          // Save the current region
          region = regions[idx]; 

          print_highlight (stderr, "Obtaining the boundary points for the region ");
          TicToc tt; tt.tic ();
          // Project the inliers to obtain a better hull
          typename PointCloud<PointT>::Ptr cloud_projected (new PointCloud<PointT>);
          ModelCoefficients::Ptr coefficients (new ModelCoefficients (model_coefficients[idx]));
          ProjectInliers<PointT> proj;
          proj.setModelType (SACMODEL_PLANE);
          proj.setInputCloud (cloud_);
          proj.setIndices (plane_indices_);
          proj.setModelCoefficients (coefficients);
          proj.filter (*cloud_projected);

          // Compute the boundary points as a ConvexHull
          ConvexHull<PointT> chull;
          chull.setDimension (2);
          chull.setInputCloud (cloud_projected);
          PointCloud<PointT> plane_hull;
          chull.reconstruct (plane_hull);
          region.setContour (plane_hull);
          print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%zu", plane_hull.size ()); print_info (" points]\n");
        }

      }

      // Segment the object of interest
      if (plane_indices_ && !plane_indices_->indices.empty ())
      {
        plane_.reset (new PointCloud<PointT>);
        copyPointCloud (*cloud_, plane_indices_->indices, *plane_);
        object.reset (new PointCloud<PointT>);
        segmentObject (picked_idx, cloud_, plane_indices_, *object);
      }
    }

    /////////////////////////////////////////////////////////////////////////
    /** \brief Point picking callback. This gets called when the user selects
      * a 3D point on screen (in the PCLVisualizer window) using Shift+click.
      *
      * \param[in] event the event that triggered the call
      */
    void 
    pp_callback (const visualization::PointPickingEvent& event, void*)
    {
      // Check to see if we got a valid point. Early exit.
      int idx = event.getPointIndex ();
      if (idx == -1)
        return;

      vector<int> indices (1);
      vector<float> distances (1);

      // Get the point that was picked
      PointT picked_pt;
      event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);

      print_info (stderr, "Picked point with index %d, and coordinates %f, %f, %f.\n", idx, picked_pt.x, picked_pt.y, picked_pt.z);

      // Add a sphere to it in the PCLVisualizer window
      stringstream ss;
      ss << "sphere_" << idx;
      cloud_viewer_->addSphere (picked_pt, 0.01, 1.0, 0.0, 0.0, ss.str ());

      // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
      search_->nearestKSearch (picked_pt, 1, indices, distances);

      // Add some marker to the image
      if (image_viewer_)
      {
        // Get the [u, v] in pixel coordinates for the ImageViewer. Remember that 0,0 is bottom left.
        uint32_t width  = search_->getInputCloud ()->width,
                 height = search_->getInputCloud ()->height;
        int v = height - indices[0] / width,
            u = indices[0] % width;

        image_viewer_->addCircle (u, v, 5, 1.0, 0.0, 0.0, "circles", 1.0);
        image_viewer_->addFilledRectangle (u-5, u+5, v-5, v+5, 0.0, 1.0, 0.0, "boxes", 0.5);
        image_viewer_->markPoint (u, v, visualization::red_color, visualization::blue_color, 10);
      }

      // Segment the region that we're interested in, by employing a two step process:
      //  * first, segment all the planes in the scene, and find the one closest to our picked point
      //  * then, use euclidean clustering to find the object that we clicked on and return it
      PlanarRegion<PointT> region;
      segment (picked_pt, indices[0], region, object_);

      // If no region could be determined, exit
      if (region.getContour ().empty ())
      {
        PCL_ERROR ("No planar region detected. Please select another point or relax the thresholds and continue.\n");
        return;
      }
      // Else, draw it on screen
      else
      {
        cloud_viewer_->addPolygon (region, 0.0, 0.0, 1.0, "region");
        cloud_viewer_->setShapeRenderingProperties (visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "region");

        // Draw in image space
        if (image_viewer_)
        {
          image_viewer_->addPlanarPolygon (search_->getInputCloud (), region, 0.0, 0.0, 1.0, "refined_region", 1.0);
        }
      }

      // If no object could be determined, exit
      if (!object_)
      {
        PCL_ERROR ("No object detected. Please select another point or relax the thresholds and continue.\n");
        return;
      }
      else
      {
        // Visualize the object in 3D...
        visualization::PointCloudColorHandlerCustom<PointT> red (object_, 255, 0, 0);
        if (!cloud_viewer_->updatePointCloud (object_, red, "object"))
          cloud_viewer_->addPointCloud (object_, red, "object");
        // ...and 2D
        if (image_viewer_)
        {
          image_viewer_->removeLayer ("object");
          image_viewer_->addMask (search_->getInputCloud (), *object_, "object");
        }

        // ...and 2D
        if (image_viewer_)
          image_viewer_->addRectangle (search_->getInputCloud (), *object_);
      }
    }
    
    /////////////////////////////////////////////////////////////////////////
    void
    compute ()
    {
      // Visualize the data
      while (!cloud_viewer_->wasStopped ())
      {
        /*// Add the plane that we're tracking to the cloud visualizer
        PointCloud<PointT>::Ptr plane (new Cloud);
        if (plane_)
          *plane = *plane_;
        visualization::PointCloudColorHandlerCustom<PointT> blue (plane, 0, 255, 0);
        if (!cloud_viewer_->updatePointCloud (plane, blue, "plane"))
          cloud_viewer_->addPointCloud (plane, "plane");
*/
        cloud_viewer_->spinOnce ();
        if (image_viewer_)
        {
          image_viewer_->spinOnce ();
          if (image_viewer_->wasStopped ())
            break;
        }
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }
    }
    
    /////////////////////////////////////////////////////////////////////////
    void
    initGUI ()
    {
      cloud_viewer_.reset (new visualization::PCLVisualizer ("PointCloud"));

      if (cloud_->isOrganized ())
      {
        // If the dataset is organized, and has RGB data, create an image viewer
        vector<pcl::PCLPointField> fields;
        int rgba_index = -1;
        rgba_index = getFieldIndex (*cloud_, "rgba", fields);
       
        if (rgba_index >= 0)
        {
          image_viewer_.reset (new visualization::ImageViewer ("RGB PCLImage"));

          image_viewer_->registerMouseCallback (&ObjectSelection::mouse_callback, *this);
          image_viewer_->registerKeyboardCallback(&ObjectSelection::keyboard_callback, *this);
          image_viewer_->setPosition (cloud_->width, 0);
          image_viewer_->setSize (cloud_->width, cloud_->height);

          int poff = fields[rgba_index].offset;
          // BGR to RGB
          rgb_data_ = new unsigned char [cloud_->width * cloud_->height * 3];
          for (uint32_t i = 0; i < cloud_->width * cloud_->height; ++i)
          {
            RGB rgb;
            memcpy (&rgb, reinterpret_cast<unsigned char*> (&cloud_->points[i]) + poff, sizeof (rgb));

            rgb_data_[i * 3 + 0] = rgb.r;
            rgb_data_[i * 3 + 1] = rgb.g;
            rgb_data_[i * 3 + 2] = rgb.b;
          }
          image_viewer_->showRGBImage (rgb_data_, cloud_->width, cloud_->height);
        }
        cloud_viewer_->setSize (cloud_->width, cloud_->height);
      }

      cloud_viewer_->registerMouseCallback (&ObjectSelection::mouse_callback, *this);
      cloud_viewer_->registerKeyboardCallback(&ObjectSelection::keyboard_callback, *this);
      cloud_viewer_->registerPointPickingCallback (&ObjectSelection::pp_callback, *this);
      cloud_viewer_->setPosition (0, 0);

      cloud_viewer_->addPointCloud (cloud_, "scene");
      cloud_viewer_->resetCameraViewpoint ("scene");
      cloud_viewer_->addCoordinateSystem (0.1, 0, 0, 0, "global");
    }

    /////////////////////////////////////////////////////////////////////////
    bool
    load (const std::string &file)
    {
      // Load the input file
      TicToc tt; tt.tic ();
      print_highlight (stderr, "Loading "); 
      print_value (stderr, "%s ", file.c_str ());
      cloud_.reset (new PointCloud<PointT>);
      if (io::loadPCDFile (file, *cloud_) < 0) 
      {
        print_error (stderr, "[error]\n");
        return (false);
      }
      print_info ("[done, "); print_value ("%g", tt.toc ()); 
      print_info (" ms : "); print_value ("%zu", cloud_->size ()); print_info (" points]\n");
      
      if (cloud_->isOrganized ())
        search_.reset (new search::OrganizedNeighbor<PointT>);
      else
        search_.reset (new search::KdTree<PointT>);

      search_->setInputCloud (cloud_);

      return (true);
    }
    
    /////////////////////////////////////////////////////////////////////////
    void
    save (const std::string &object_file, const std::string &plane_file)
    {
      PCDWriter w;
      if (object_ && !object_->empty ())
      {
        w.writeBinaryCompressed (object_file, *object_);
        w.writeBinaryCompressed (plane_file, *plane_);
        print_highlight ("Object successfully segmented. Saving results in: %s, and %s.\n", object_file.c_str (), plane_file.c_str ());
      }
    }

    boost::shared_ptr<visualization::PCLVisualizer> cloud_viewer_;
    boost::shared_ptr<visualization::ImageViewer> image_viewer_;
    
    typename PointCloud<PointT>::Ptr cloud_;
    typename search::Search<PointT>::Ptr search_;
  private:
    // Segmentation
    typename EdgeAwarePlaneComparator<PointT, Normal>::Ptr plane_comparator_;
    PointIndices::Ptr plane_indices_;
    unsigned char* rgb_data_;
    std::vector<float> distance_map_;

    // Results
    typename PointCloud<PointT>::Ptr plane_;
    typename PointCloud<PointT>::Ptr object_;
};

/* ---[ */
int
main (int argc, char** argv)
{
  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.empty ())
  {
    print_error ("  Need at least an input PCD file (e.g. scene.pcd) to continue!\n\n");
    print_info ("Ideally, need an input file, and three output PCD files, e.g., object.pcd, plane.pcd, rest.pcd\n");
    return (-1);
  }
  
  string object_file = "object.pcd", plane_file = "plane.pcd", rest_file = "rest.pcd";
  if (p_file_indices.size () >= 4)
    rest_file = argv[p_file_indices[3]];
  if (p_file_indices.size () >= 3)
    plane_file = argv[p_file_indices[2]];
  if (p_file_indices.size () >= 2)
    object_file = argv[p_file_indices[1]];


  PCDReader reader;
  // Test the header
  pcl::PCLPointCloud2 dummy;
  reader.readHeader (argv[p_file_indices[0]], dummy);
  if (dummy.height != 1 && getFieldIndex (dummy, "rgba") != -1)
  {
    print_highlight ("Enabling 2D image viewer mode.\n");
    ObjectSelection<PointXYZRGBA> s;
    if (!s.load (argv[p_file_indices[0]])) return (-1);
    s.initGUI ();
    s.compute ();
    s.save (object_file, plane_file);
  }
  else
  {
    ObjectSelection<PointXYZ> s;
    if (!s.load (argv[p_file_indices[0]])) return (-1);
    s.initGUI ();
    s.compute ();
    s.save (object_file, plane_file);
  }

  return (0);
}
/* ]--- */
