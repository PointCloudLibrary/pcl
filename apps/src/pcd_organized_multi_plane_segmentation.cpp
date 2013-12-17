/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 */
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/geometry/polygon_operations.h>

template<typename PointT>
class PCDOrganizedMultiPlaneSegmentation
{
  private:
    pcl::visualization::PCLVisualizer viewer;
    typename pcl::PointCloud<PointT>::ConstPtr cloud;
    bool refine_;
    float threshold_;
    bool depth_dependent_;
    bool polygon_refinement_;
  public:
    PCDOrganizedMultiPlaneSegmentation (typename pcl::PointCloud<PointT>::ConstPtr cloud_, bool refine)
    : viewer ("Viewer")
    , cloud (cloud_)
    , refine_ (refine)
    , threshold_ (0.02f)
    , depth_dependent_ (true)
    , polygon_refinement_ (false)
    {
      viewer.setBackgroundColor (0, 0, 0);
      //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
      //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
      viewer.addCoordinateSystem (1.0, "global");
      viewer.initCameraParameters ();
      viewer.registerKeyboardCallback(&PCDOrganizedMultiPlaneSegmentation::keyboard_callback, *this, 0);
    }

    void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      // do stuff and visualize here
      if (event.keyUp ())
      {
        switch (event.getKeyCode ())
        {
          case 'b':
          case 'B':
            if (threshold_ < 0.1f)
              threshold_ += 0.001f;
            process ();
            break;
          case 'v':
          case 'V':
            if (threshold_ > 0.001f)
              threshold_ -= 0.001f;
            process ();
            break;
            
          case 'n':
          case 'N':
            depth_dependent_ = !depth_dependent_;
            process ();
            break;
            
          case 'm':
          case 'M':
            polygon_refinement_ = !polygon_refinement_;
            process ();
            break;
        }
      }
    }
    
    void
    process ()
    {
      std::cout << "threshold: " << threshold_ << std::endl;
      std::cout << "depth dependent: " << (depth_dependent_ ? "true\n" : "false\n");
      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

      pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
      ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
      ne.setMaxDepthChangeFactor (0.02f);
      ne.setNormalSmoothingSize (20.0f);
      
      typename pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare (new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label> ());
      refinement_compare->setDistanceThreshold (threshold_, depth_dependent_);
      
      pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
      mps.setMinInliers (5000);
      mps.setAngularThreshold (0.017453 * 3.0); //3 degrees
      mps.setDistanceThreshold (0.03); //2cm
      mps.setRefinementComparator (refinement_compare);
      
      std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
      typename pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);
      typename pcl::PointCloud<PointT>::Ptr approx_contour (new pcl::PointCloud<PointT>);
      char name[1024];

      typename pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
      double normal_start = pcl::getTime ();
      ne.setInputCloud (cloud);
      ne.compute (*normal_cloud);
      double normal_end = pcl::getTime ();
      std::cout << "Normal Estimation took " << double (normal_end - normal_start) << std::endl;

      double plane_extract_start = pcl::getTime ();
      mps.setInputNormals (normal_cloud);
      mps.setInputCloud (cloud);
      if (refine_)
        mps.segmentAndRefine (regions);
      else
        mps.segment (regions);
      double plane_extract_end = pcl::getTime ();
      std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << " with planar regions found: " << regions.size () << std::endl;
      std::cout << "Frame took " << double (plane_extract_end - normal_start) << std::endl;

      typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);

      viewer.removeAllPointClouds (0);
      viewer.removeAllShapes (0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (cloud, 0, 255, 0);
      viewer.addPointCloud<PointT> (cloud, single_color, "cloud");
      
      pcl::PlanarPolygon<PointT> approx_polygon;
      //Draw Visualization
      for (size_t i = 0; i < regions.size (); i++)
      {
        Eigen::Vector3f centroid = regions[i].getCentroid ();
        Eigen::Vector4f model = regions[i].getCoefficients ();
        pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
        pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                           centroid[1] + (0.5f * model[1]),
                                           centroid[2] + (0.5f * model[2]));
        sprintf (name, "normal_%d", unsigned (i));
        viewer.addArrow (pt2, pt1, 1.0, 0, 0, std::string (name));

        contour->points = regions[i].getContour ();        
        sprintf (name, "plane_%02d", int (i));
        pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i], grn[i], blu[i]);
        viewer.addPointCloud (contour, color, name);

        pcl::approximatePolygon (regions[i], approx_polygon, threshold_, polygon_refinement_);
        approx_contour->points = approx_polygon.getContour ();
        std::cout << "polygon: " << contour->size () << " -> " << approx_contour->size () << std::endl;
        typename pcl::PointCloud<PointT>::ConstPtr approx_contour_const = approx_contour;
        
//        sprintf (name, "approx_plane_%02d", int (i));
//        viewer.addPolygon<PointT> (approx_contour_const, 0.5 * red[i], 0.5 * grn[i], 0.5 * blu[i], name);
        for (unsigned idx = 0; idx < approx_contour->points.size (); ++idx)
        {
          sprintf (name, "approx_plane_%02d_%03d", int (i), int(idx));
          viewer.addLine (approx_contour->points [idx], approx_contour->points [(idx+1)%approx_contour->points.size ()], 0.5 * red[i], 0.5 * grn[i], 0.5 * blu[i], name);
        }
      }
    }
    
    void
    run ()
    {
      // initial processing
      process ();
    
      while (!viewer.wasStopped ())
        viewer.spinOnce (100);
    }
};

int
main (int argc, char** argv)
{
  bool refine = pcl::console::find_switch (argc, argv, "-refine");
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud_xyz);
  PCDOrganizedMultiPlaneSegmentation<pcl::PointXYZ> multi_plane_app (cloud_xyz, refine);
  multi_plane_app.run ();
  return 0;
}
