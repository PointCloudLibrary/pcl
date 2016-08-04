/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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
 * $Id$
 *
 */

#include <iostream>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_fast_edge_aware.h>
#include <pcl/features/normal_3d.h>

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> test_data;

struct NormalEstimationStatistics
{
    double coverage_gt_normals_;   // how many normals are computed w.r.t. the number of ground truth normals
    double average_angular_error_;  // average normal estimation error
    double average_angular_error_deg_;  // average normal estimation error in [deg]
    double percentage_good_normals_;  // ratio of sufficiently accurate normals
    double number_images_;

    NormalEstimationStatistics ()
    {
      clear ();
    }

    void
    clear ()
    {
      coverage_gt_normals_ = 0.;
      average_angular_error_ = 0.;
      average_angular_error_deg_ = 0.;
      percentage_good_normals_ = 0.;
      number_images_ = 0.;
    }

    void
    addStatistics (double coverage_gt_normals_val,
                   double average_angular_error_val,
                   double average_angular_error_deg_val,
                   double percentage_good_normals_val)
    {
      coverage_gt_normals_ = (coverage_gt_normals_ * number_images_ + coverage_gt_normals_val) / (number_images_ + 1.);
      average_angular_error_ = (average_angular_error_ * number_images_ + average_angular_error_val) / (number_images_ + 1.);
      average_angular_error_deg_ = (average_angular_error_deg_ * number_images_ + average_angular_error_deg_val) / (number_images_ + 1.);
      percentage_good_normals_ = (percentage_good_normals_ * number_images_ + percentage_good_normals_val) / (number_images_ + 1.);
      number_images_ += 1.;
    }

    std::string
    getStatisticsString ()
    {
      std::stringstream text;
      text << "Coverage of estimated normals on gt_normals: " << 100. * coverage_gt_normals_ << "%" << std::endl;
      text << "Average normal estimation error: " << average_angular_error_ << std::endl;
      text << "Average normal estimation error [deg]: " << average_angular_error_deg_ << std::endl;
      text << "Percentage of good normals: " << percentage_good_normals_ << "%\n" << std::endl;
      return text.str ();
    }
};

class Evaluation
{
  public:
    Evaluation (int max_scan_line_width) :
        max_scan_line_width_ (max_scan_line_width)
    {
    }
    ;
    ~Evaluation ()
    {
    }
    ;

    void
    evaluateNormalEstimation (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud,
                              pcl::PointCloud<pcl::Normal>::Ptr& normals,
                              NormalEstimationStatistics* ne_statistics)
    {
      // 1. Estimate ground truth normals from gt_point_cloud
      pcl::PointCloud<pcl::Normal>::Ptr gt_normals (new pcl::PointCloud<pcl::Normal>);
      computeGroundTruthNormals (gt_point_cloud, gt_normals);

      // 2. Compute error of estimated normals
      int number_gt_normals = 0, number_normals = 0, number_good_normals = 0;
      double normal_error = 0., normal_error_deg = 0.;
      computeNormalEstimationError (gt_point_cloud, gt_normals, normals, max_scan_line_width_, number_gt_normals, number_normals, number_good_normals,
                                    normal_error, normal_error_deg);

      // 3. Visualize
      if (ne_statistics != 0)
        ne_statistics->addStatistics ((double) number_normals / (double) number_gt_normals, normal_error / (double) number_normals,
                                      normal_error_deg / (double) number_normals, 100. * (double) number_good_normals / (double) number_normals);
    }

  private:
    int max_scan_line_width_;

    void
    computeGroundTruthNormals (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud,
                               pcl::PointCloud<pcl::Normal>::Ptr gt_normals)
    {
      gt_normals->resize (gt_point_cloud->height * gt_point_cloud->width);
      gt_normals->header = gt_point_cloud->header;
      gt_normals->height = gt_point_cloud->height;
      gt_normals->width = gt_point_cloud->width;
      //gt_normals->is_dense = true;
      for (int v = 1; v < gt_point_cloud->height - 1; ++v)
      {
        for (int u = 1; u < gt_point_cloud->width - 1; ++u)
        {
          if (checkGroundTruthEdge (gt_point_cloud, u, v) == 0)
          {
            Eigen::Vector3f p, p1, p2;
            p = gt_point_cloud->at (u, v).getVector3fMap ();

            bool valid_neighborhood_points = true;
            if (checkGroundTruthEdge (gt_point_cloud, u + 1, v) == 0)
              p1 = gt_point_cloud->at (u + 1, v).getVector3fMap ();
            else if (checkGroundTruthEdge (gt_point_cloud, u - 1, v) == 0)
              p1 = gt_point_cloud->at (u - 1, v).getVector3fMap ();
            else
              valid_neighborhood_points = false;

            if (checkGroundTruthEdge (gt_point_cloud, u, v + 1) == 0)
              p2 = gt_point_cloud->at (u, v + 1).getVector3fMap ();
            else if (checkGroundTruthEdge (gt_point_cloud, u, v - 1) == 0)
              p2 = gt_point_cloud->at (u, v - 1).getVector3fMap ();
            else
              valid_neighborhood_points = false;

            if (valid_neighborhood_points == true)
            {
              Eigen::Vector3f n = (p1 - p).cross (p2 - p);
              n.normalize ();
              pcl::flipNormalTowardsViewpoint<pcl::PointXYZRGB> (gt_point_cloud->at (u, v), gt_point_cloud->sensor_origin_ (0),
                                                                 gt_point_cloud->sensor_origin_ (1), gt_point_cloud->sensor_origin_ (2), n (0), n (1), n (2));
              gt_normals->at (u, v).normal_x = n (0);
              gt_normals->at (u, v).normal_y = n (1);
              gt_normals->at (u, v).normal_z = n (2);
            }
            else
            {
              gt_normals->at (u, v).normal_x = std::numeric_limits<float>::quiet_NaN ();
              gt_normals->at (u, v).normal_y = std::numeric_limits<float>::quiet_NaN ();
              gt_normals->at (u, v).normal_z = std::numeric_limits<float>::quiet_NaN ();
            }
          }
        }
      }
    }

    int
    checkGroundTruthEdge (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud,
                          const int u,
                          const int v)
    {
      const double pi = 3.14159265359;
      if (u != 0 && v != 0 && u != gt_point_cloud->width - 1 && v != gt_point_cloud->height - 1)
      {
        // check for depth edges
        const double depth_factor = 0.01;
        const double depth = gt_point_cloud->at (u, v).z;
        const double edge_threshold = depth_factor * depth * depth;
        const double dzl = gt_point_cloud->at (u, v).z - gt_point_cloud->at (u - 1, v).z;
        const double dzr = gt_point_cloud->at (u + 1, v).z - gt_point_cloud->at (u, v).z;
        const double dzu = gt_point_cloud->at (u, v).z - gt_point_cloud->at (u, v - 1).z;
        const double dzb = gt_point_cloud->at (u, v + 1).z - gt_point_cloud->at (u, v).z;
        if ( ( (dzr < -edge_threshold || dzr > edge_threshold) && fabs (dzl - dzr) > 0.01)
            || ( (dzb < -edge_threshold || dzb > edge_threshold) && fabs (dzu - dzb) > 0.01))
          return (int) pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_OCCLUDING;
        // additionally check for surface edges
        const double min_detectable_edge_angle = 35.;
        const double alpha_left = atan2 (-dzl, (double) - (gt_point_cloud->at (u, v).x - gt_point_cloud->at (u - 1, v).x));
        const double alpha_right = atan2 (dzr, (double) (gt_point_cloud->at (u + 1, v).x - gt_point_cloud->at (u, v).x));
        double diff = fabs (alpha_left - alpha_right);
        if (diff != 0 && (diff < (180. - min_detectable_edge_angle) / 180. * pi || diff > (180. + min_detectable_edge_angle) / 180. * pi))
          return (int) pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_HIGH_CURVATURE;
        const double alpha_upper = atan2 (-dzu, (double) - (gt_point_cloud->at (u, v).y - gt_point_cloud->at (u, v - 1).y));
        const double alpha_below = atan2 (dzb, (double) (gt_point_cloud->at (u, v + 1).y - gt_point_cloud->at (u, v).y));
        diff = fabs (alpha_upper - alpha_below);
        if (diff != 0 && (diff < (180. - min_detectable_edge_angle) / 180. * pi || diff > (180. + min_detectable_edge_angle) / 180. * pi))
          return (int) pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_HIGH_CURVATURE;
      }
      return 0;
    }

    void
    computeNormalEstimationError (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud,
                                  const pcl::PointCloud<pcl::Normal>::Ptr& gt_normals,
                                  const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                                  const int padding,
                                  int& number_gt_normals,
                                  int& number_normals,
                                  int& number_good_normals,
                                  double& normal_error,
                                  double& normal_error_deg)
    {
      const double pi = 3.14159265359;
      number_gt_normals = 0;
      number_normals = 0;
      number_good_normals = 0;
      normal_error = 0.;
      normal_error_deg = 0.;
      //normals->is_dense = true;
      for (int v = padding; v < gt_point_cloud->height - padding; ++v)
      {
        for (int u = padding; u < gt_point_cloud->width - padding; ++u)
        {
          if (pcl_isnan(gt_normals->at(u,v).normal_z)==false)
          {
            // gt_normal exists
            ++number_gt_normals;
            if (pcl_isnan(normals->at(u,v).normal_z)==false)
            {
              // normal estimation has also found a normal
              ++number_normals;
              const Eigen::Vector3f n = normals->at(u,v).getNormalVector3fMap();
              const Eigen::Vector3f gt_n = gt_normals->at(u,v).getNormalVector3fMap();
              double d = std::max(-1., std::min(1.,(double)gt_n.dot(n)));
              normal_error += fabs(1 - d);
              normal_error_deg += 180./pi*acos(d);
              if (fabs(d) > 0.97)
              ++number_good_normals;
            }
          }
        }
      }
    }
  };

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, FastEdgeAwareNormalEstimation)
{
  // set up parameters for 3d edge detection
  const int max_scan_line_width = 30;
  pcl::EdgeDetectionConfig cfg (pcl::EdgeDetectionConfig::GAUSSIAN, 3, 0.01f, 40., true, 5, max_scan_line_width, 15);
  NormalEstimationStatistics statistics_total;

  for (size_t i = 0; i < test_data.size (); ++i)
  {
    // compute 3d surface normals (without taking support regions for computation that cross 3d edges)
    pcl::PointCloud<pcl::Normal>::Ptr normals_edge_aware = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    pcl::FastEdgeAwareNormalEstimation<pcl::PointXYZRGB, pcl::Normal> fast_edge_aware_normal_estimation;
    fast_edge_aware_normal_estimation.setEdgeDetectionConfig (cfg);
    fast_edge_aware_normal_estimation.setInputCloud (test_data[i]);
    fast_edge_aware_normal_estimation.compute (*normals_edge_aware);

    // compute evaluation measures and compare to ground truth
    NormalEstimationStatistics stats;
    Evaluation eval (max_scan_line_width);
    eval.evaluateNormalEstimation (test_data[i], normals_edge_aware, &stats);
    std::cout << i + 1 << ":\n" << stats.getStatisticsString () << std::endl;
    statistics_total.addStatistics (stats.coverage_gt_normals_, stats.average_angular_error_, stats.average_angular_error_deg_, stats.percentage_good_normals_);

    EXPECT_GT(stats.coverage_gt_normals_, 0.9) << "Low normal estimation coverage on image number " << i + 1;
    EXPECT_LT(stats.average_angular_error_deg_, 10.0) << "High normal estimation error on image number" << i + 1;
    EXPECT_GT(stats.percentage_good_normals_, 85.0) << "Low percentage of good normals on image number " << i + 1;
  }

  // evaluate overall performance
  std::cout << "Total:\n" << statistics_total.getStatisticsString () << std::endl;
  EXPECT_GT(statistics_total.coverage_gt_normals_, 0.925) << "Low total normal estimation coverage";
  EXPECT_LT(statistics_total.average_angular_error_deg_, 7.5) << "High total normal estimation error";
  EXPECT_GT(statistics_total.percentage_good_normals_, 90.0) << "Low total percentage of good normals";

}

/* ---[ */
int
main (int argc,
      char** argv)
{
  // open files
  if (argc < 4)
  {
    std::cerr
        << "No test file given. Please download `organized_edge_1.pcd`, `organized_edge_2.pcd`, and `organized_edge_3.pcd` and pass its path to the test.";
    return -1;
  }
  test_data.resize (3);
  for (int i = 0; i < 3; ++i)
  {
    std::string filename = argv[i + 1];
    std::cout << "Reading " << filename << std::endl;
    test_data[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, * (test_data[i])) != 0)  // load the file
    {
      PCL_ERROR("Couldn't read file.\n");
      return -1;
    }
    std::cout << "points: " << test_data[i]->points.size () << std::endl;
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
