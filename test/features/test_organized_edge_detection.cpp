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
#include <pcl/features/organized_edge_detection.h>


std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> test_data;

struct EdgeDetectionStatistics
{
  double recall;
  double precision;
  double number_images;

  EdgeDetectionStatistics()
  {
    clear();
  }

  void clear()
  {
    recall = 0.;
    precision = 0.;
    number_images = 0.;
  }

  void addStatistics(double recall_val, double precision_val)
  {
    recall = (recall*number_images + recall_val)/(number_images+1.);
    precision = (precision*number_images + precision_val)/(number_images+1.);
    number_images += 1.;
  }
};

struct NormalEstimationStatistics
{
  double coverage_gt_normals;   // how many normals are computed w.r.t. the number of ground truth normals
  double average_angular_error; // average normal estimation error
  double average_angular_error_deg; // average normal estimation error in [deg]
  double percentage_good_normals; // ratio of sufficiently accurate normals
  double number_images;

  NormalEstimationStatistics()
  {
    clear();
  }

  void clear()
  {
    coverage_gt_normals = 0.;
    average_angular_error = 0.;
    average_angular_error_deg = 0.;
    percentage_good_normals = 0.;
    number_images = 0.;
  }

  void addStatistics(double coverage_gt_normals_val, double average_angular_error_val, double average_angular_error_deg_val, double percentage_good_normals_val)
  {
    coverage_gt_normals = (coverage_gt_normals*number_images + coverage_gt_normals_val)/(number_images+1.);
    average_angular_error = (average_angular_error*number_images + average_angular_error_val)/(number_images+1.);
    average_angular_error_deg = (average_angular_error_deg*number_images + average_angular_error_deg_val)/(number_images+1.);
    percentage_good_normals = (percentage_good_normals*number_images + percentage_good_normals_val)/(number_images+1.);
    number_images += 1.;
  }
};

class Evaluation
{
public:
  Evaluation(int number_of_edgetypes, int max_scan_line_width)
  : number_of_edgetypes_(number_of_edgetypes), max_scan_line_width_(max_scan_line_width)
  { };
  ~Evaluation() {};

  void evaluateEdgeRecognition(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const pcl::PointCloud<pcl::Label>::Ptr& estimated_labels, EdgeDetectionStatistics* edge_detection_statistics)
  {
    // 1. Computation of the ground truth labels (= depth + surface edges)
    pcl::PointCloud<pcl::Label>::Ptr gt_labels = pcl::PointCloud<pcl::Label>::Ptr(new pcl::PointCloud<pcl::Label>);
    generateGroundTruthLabels(gt_point_cloud, gt_labels);

    // 2. Count the number of fits in the data
    const int search_radius = 1;
    std::vector<double> recall, precision;
    computePerformanceMeasures(*gt_labels, *estimated_labels, search_radius, recall, precision);

    // 3. Output statistics
    if (edge_detection_statistics != 0)
      edge_detection_statistics->addStatistics(recall[recall.size()-1], precision[precision.size()-1]);
  }

//  void evaluateNormalEstimation(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals, NormalEstimationStatistics* ne_statistics = 0);

private:

  int number_of_edgetypes_;

  int max_scan_line_width_;

  void generateGroundTruthLabels(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, pcl::PointCloud<pcl::Label>::Ptr& gt_labels)
  {
    // computation of the ground truth image (= color normalization of the scene image + depth edges)
    gt_labels->resize(gt_point_cloud->width * gt_point_cloud->height);
    gt_labels->width = gt_point_cloud->width;
    gt_labels->height = gt_point_cloud->height;

    // label surface and depth edges
    for (int v=0; v<gt_labels->height; ++v)
      for (int u=0; u<gt_labels->width; ++u)
        gt_labels->at(u,v).label = checkGroundTruthEdge(gt_point_cloud, u, v);

    // remove surface edges which are labeled directly next to a depth edge
    for (int v=1; v<gt_labels->height; ++v)
      for (int u=1; u<gt_labels->width; ++u)
        if ((gt_labels->at(u,v).label & pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_HIGH_CURVATURE) &&
            ((gt_labels->at(u-1,v).label & pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_OCCLUDING) ||
             ((gt_labels->at(u,v-1).label & pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_OCCLUDING))))
          gt_labels->at(u,v).label = 0;
  }

  int checkGroundTruthEdge(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const int u, const int v)
  {
    const double pi = 3.14159265359;
    if (u!=0 && v!=0 && u!=gt_point_cloud->width-1 && v!=gt_point_cloud->height-1)
    {
      // check for depth edges
      const double depth_factor = 0.01;
      const double depth = gt_point_cloud->at(u,v).z;
      const double edge_threshold = depth_factor*depth*depth;
      const double dzl = gt_point_cloud->at(u,v).z - gt_point_cloud->at(u-1,v).z;
      const double dzr = gt_point_cloud->at(u+1,v).z - gt_point_cloud->at(u,v).z;
      const double dzu = gt_point_cloud->at(u,v).z - gt_point_cloud->at(u,v-1).z;
      const double dzb = gt_point_cloud->at(u,v+1).z - gt_point_cloud->at(u,v).z;
      if ( ((dzr<-edge_threshold || dzr>edge_threshold) && fabs(dzl-dzr)>0.01) || ((dzb<-edge_threshold || dzb>edge_threshold) && fabs(dzu-dzb)>0.01) )
        return (int)pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_OCCLUDING;
      // additionally check for surface edges
      const double min_detectable_edge_angle = 35.;
      const double alpha_left = atan2(-dzl, (double)-(gt_point_cloud->at(u,v).x - gt_point_cloud->at(u-1,v).x));
      const double alpha_right = atan2(dzr, (double)(gt_point_cloud->at(u+1,v).x - gt_point_cloud->at(u,v).x));
      double diff = fabs(alpha_left - alpha_right);
      if (diff!=0 && (diff < (180.-min_detectable_edge_angle) / 180. * pi || diff > (180.+min_detectable_edge_angle) / 180. * pi))
        return (int)pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_HIGH_CURVATURE;
      const double alpha_upper = atan2(-dzu, (double)-(gt_point_cloud->at(u,v).y - gt_point_cloud->at(u,v-1).y));
      const double alpha_below = atan2(dzb, (double)(gt_point_cloud->at(u,v+1).y - gt_point_cloud->at(u,v).y));
      diff = fabs(alpha_upper - alpha_below);
      if (diff!=0 && (diff < (180.-min_detectable_edge_angle) / 180. * pi || diff > (180.+min_detectable_edge_angle) / 180. * pi))
        return (int)pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label>::EDGELABEL_HIGH_CURVATURE;
    }
    return 0;
  }

//  void computeGroundTruthNormals(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, pcl::PointCloud<pcl::Normal>::Ptr gt_normals);

//  void computeNormalEstimationError(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const pcl::PointCloud<pcl::Normal>::Ptr& gt_normals,
//      const pcl::PointCloud<pcl::Normal>::Ptr& normals, const int padding, int& number_gt_normals, int& number_normals, int& number_good_normals, double& normal_error, double& normal_error_deg);

  // compute the numbers for recall and precision
  // @param search_radius is half the side length of the search neighborhood for fitting estimates and gt pixels (e.g. an edge pixel is correctly labeled in the estimate if there is an edge pixel within the (2*search_radius+1) neighborhood in the gt_image)
  void computePerformanceMeasures(const pcl::PointCloud<pcl::Label>& gt_labels, const pcl::PointCloud<pcl::Label>& estimated_labels, const int search_radius, std::vector<double>& recall, std::vector<double>& precision)
  {
    // 1. recall
    recall.clear();
    recall.resize(number_of_edgetypes_+1, 0.);
    std::vector<int> surface_type_counter(number_of_edgetypes_+1, 0);
    for (int v=max_scan_line_width_; v<gt_labels.height-max_scan_line_width_; ++v)
    {
      for (int u=max_scan_line_width_; u<gt_labels.width-max_scan_line_width_; ++u)
      {
        for (int label_index=0; label_index<=number_of_edgetypes_; ++label_index)
        {
          const int gt_label = (label_index==0 ? 0 : 1<<(label_index-1));   // generate labels 0, 1, 2, 4, 8, 16, ...
          if ((gt_label==0 && (int)gt_labels.at(u,v).label!=0) || (gt_label & (int)gt_labels.at(u,v).label) == false)  // skip this gt_label if it is not part of the ground truth labeling in this point
            continue;
          bool correct = false;
          surface_type_counter[label_index]++;
          for (int dv=-search_radius; dv<=search_radius && correct==false; ++dv)
          {
            for (int du=-search_radius; du<=search_radius && correct==false; ++du)
            {
              const int x = u+du;
              const int y = v+dv;
              if (x<0 || x>=gt_labels.width || y<0 || y>=gt_labels.height)
                continue;
              if ((int)estimated_labels.at(x,y).label == gt_label || (((int)estimated_labels.at(x,y).label & gt_label) == true))
              {
                recall[label_index]+=1.;
                correct = true;
              }
            }
          }
        }
      }
    }
    // compute average recall over all label types
    double sum_recall = 0.;
    double sum_counter = 0.;
    for (int i=0; i<number_of_edgetypes_; ++i)
    {
      sum_recall += recall[i];
      sum_counter += surface_type_counter[i];
    }
    recall[recall.size()-1] = divide(sum_recall, sum_counter);
    // normalize recall
    for (int i=0; i<number_of_edgetypes_; ++i)
      recall[i] = divide(recall[i], surface_type_counter[i]);

    // 2. precision
    precision.clear();
    precision.resize(number_of_edgetypes_+1, 0.);
    surface_type_counter.clear();
    surface_type_counter.resize(number_of_edgetypes_+1, 0);
    for (int v=max_scan_line_width_; v<estimated_labels.height-max_scan_line_width_; ++v)
    {
      for (int u=max_scan_line_width_; u<estimated_labels.width-max_scan_line_width_; ++u)
      {
        for (int label_index=0; label_index<=number_of_edgetypes_; ++label_index)
        {
          const int estimated_label = (label_index==0 ? 0 : 1<<(label_index-1));   // generate labels 0, 1, 2, 4, 8, 16, ...
          if ((estimated_label==0 && (int)estimated_labels.at(u,v).label!=0) || (estimated_label & (int)estimated_labels.at(u,v).label) == false)  // skip this estimated_label if it is not part of the estimated labels in this point
            continue;
          bool correct = false;
          surface_type_counter[label_index]++;
          for (int dv=-search_radius; dv<=search_radius && correct==false; ++dv)
          {
            for (int du=-search_radius; du<=search_radius && correct==false; ++du)
            {
              const int x = u+du;
              const int y = v+dv;
              if (x<0 || x>=estimated_labels.width || y<0 || y>=estimated_labels.height)
                continue;
              if ((int)gt_labels.at(x,y).label == estimated_label || (((int)gt_labels.at(x,y).label & estimated_label) == true))
              {
                precision[label_index]+=1.;
                correct = true;
              }
            }
          }
        }
      }
    }
    // compute average precision over all label types
    double sum_precision = 0.;
    sum_counter = 0.;
    for (int i=0; i<number_of_edgetypes_; ++i)
    {
      sum_precision += precision[i];
      sum_counter += surface_type_counter[i];
    }
    precision[precision.size()-1] = divide(sum_precision, sum_counter);
    // normalize precision
    for (int i=0; i<number_of_edgetypes_; ++i)
      precision[i] = divide(precision[i], surface_type_counter[i]);
  }

  double divide(double a, double b)
  {
    //compute a/b
    if (b == 0)
    {
      if(a == 0)
        return 1;
      else
        return 0;
    }
    else
      return (a / b) * 1;
  }
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, OrganizedEdgeFromPoints)
{
  // set up parameters for 3d edge detection
  pcl::EdgeDetectionConfig cfg (pcl::EdgeDetectionConfig::GAUSSIAN, 3, 0.01f, 40., true, 5, 30, 15);
  EdgeDetectionStatistics statistics_total;

  for (size_t i = 0; i < test_data.size (); ++i)
  {
    // compute 3d surface and depth edges
    pcl::PointCloud<pcl::Label>::Ptr edge_labels = pcl::PointCloud<pcl::Label>::Ptr(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    // if this pointer is set to non-zero, the method also computes normals from support regions that do not extend over edges, the normal computation is very fast
    pcl::PointCloud<pcl::Normal>::Ptr normals_edge_aware;
    pcl::OrganizedEdgeFromPoints<pcl::PointXYZRGB, pcl::Normal, pcl::Label> edge_detection;
    edge_detection.setEdgeDetectionConfig (cfg);
    edge_detection.setInputCloud (test_data[i]);
    edge_detection.setReturnLabelIndices (false);  // if we do not need the label indices vector filled, computations are slightly faster
    edge_detection.setUseFastDepthDiscontinuityMode (true);  // use a specific fast implementation for estimating depth edges, otherwise we can use the standard method of OrganizedEdgeBase
    edge_detection.compute (*edge_labels, label_indices, normals_edge_aware);

    // compute evaluation measures and compare to ground truth
    EdgeDetectionStatistics stats;
    Evaluation eval(pcl::OrganizedEdgeFromPoints<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::num_of_edgetype_, 30);
    eval.evaluateEdgeRecognition(test_data[i], edge_labels, &stats);
    std::cout << i+1 << ":\tRecall: " << stats.recall << "\tPrecision: " << stats.precision << std::endl;
    statistics_total.addStatistics(stats.recall, stats.precision);

    EXPECT_GT(stats.recall, 0.7) << "Low recall on image number " << i+1;
    EXPECT_GT(stats.precision, 0.7) << "Low precision on image number " << i+1;
  }

  // evaluate overall performance
  double f = 2*statistics_total.recall*statistics_total.precision/(statistics_total.recall+statistics_total.precision);
  std::cout << "Total:\tRecall: " << statistics_total.recall << "\tPrecision: " << statistics_total.precision << "\tf=" << f << std::endl;
  EXPECT_GT(statistics_total.recall, 0.8) << "Low total recall";
  EXPECT_GT(statistics_total.precision, 0.8) << "Low total precision";
  EXPECT_GT(f, 0.85) << "Low total f1-measure";

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
