/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/apps/optronic_viewer/cloud_filter.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/random_sample.h>

#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/io/pcd_io.h>

extern char pcl::apps::optronic_viewer::g_voxel_grid_cf_name[] = "VoxelGrid Filter";
extern char pcl::apps::optronic_viewer::g_passthrough_cf_name[] = "PassThrough Filter";
extern char pcl::apps::optronic_viewer::g_radius_outlier_cf_name[] = "RadiusOutlier Filter";
extern char pcl::apps::optronic_viewer::g_fast_bilateral_cf_name[] = "Fast Bilateral Filter";
extern char pcl::apps::optronic_viewer::g_median_cf_name[] = "Median Filter";
extern char pcl::apps::optronic_viewer::g_random_sample_cf_name[] = "Random Sample Filter";
extern char pcl::apps::optronic_viewer::g_plane_cf_name[] = "Plane Filter";

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
VoxelGridCF::
VoxelGridCF ()
: filter_selection_page_ (new QWizardPage ())
, voxel_grid_size_x_ (0.01)
, voxel_grid_size_y_ (0.01)
, voxel_grid_size_z_ (0.01)
{
  voxel_grid_size_x_label_ = new QLabel (QObject::tr ("Voxel Grid Size in X-Direction:"));
  voxel_grid_size_x_line_edit_ = new QLineEdit ();
  voxel_grid_size_y_label_ = new QLabel (QObject::tr ("Voxel Grid Size in Y-Direction:"));
  voxel_grid_size_y_line_edit_ = new QLineEdit ();
  voxel_grid_size_z_label_ = new QLabel (QObject::tr ("Voxel Grid Size in Z-Direction:"));
  voxel_grid_size_z_line_edit_ = new QLineEdit ();

  QDoubleValidator * double_validator = new QDoubleValidator ();
  voxel_grid_size_x_line_edit_->setValidator (double_validator);
  voxel_grid_size_y_line_edit_->setValidator (double_validator);
  voxel_grid_size_z_line_edit_->setValidator (double_validator);

  std::stringstream ss_x;
  ss_x << voxel_grid_size_x_;
  voxel_grid_size_x_line_edit_->setText (QString (ss_x.str ().c_str ()));

  std::stringstream ss_y;
  ss_y << voxel_grid_size_y_;
  voxel_grid_size_y_line_edit_->setText (QString (ss_y.str ().c_str ()));

  std::stringstream ss_z;
  ss_z << voxel_grid_size_z_;
  voxel_grid_size_z_line_edit_->setText (QString (ss_z.str ().c_str ()));

  main_layout_ = new QVBoxLayout (filter_selection_page_);
  main_layout_->addWidget (voxel_grid_size_x_label_);
  main_layout_->addWidget (voxel_grid_size_x_line_edit_);
  main_layout_->addWidget (voxel_grid_size_y_label_);
  main_layout_->addWidget (voxel_grid_size_y_line_edit_);
  main_layout_->addWidget (voxel_grid_size_z_label_);
  main_layout_->addWidget (voxel_grid_size_z_line_edit_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
VoxelGridCF::
~VoxelGridCF ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
VoxelGridCF::
filter (
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
  voxel_grid_size_x_ = voxel_grid_size_x_line_edit_->text ().toFloat ();
  voxel_grid_size_y_ = voxel_grid_size_y_line_edit_->text ().toFloat ();
  voxel_grid_size_z_ = voxel_grid_size_z_line_edit_->text ().toFloat ();

  pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (cloud_in);
  sor.setLeafSize (voxel_grid_size_x_, voxel_grid_size_y_, voxel_grid_size_z_);
  sor.filter (*cloud_out);
}


//////////////////////////////////////////////////////////////////////////////////////////////
//############################################################################################
//############################################################################################
//////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
PassThroughCF::
PassThroughCF ()
: filter_selection_page_ (new QWizardPage ())
, filter_field_name_ ("z")
, filter_limits_min_ (0)
, filter_limits_max_ (5)
{
  filter_field_name_label_ = new QLabel (QObject::tr ("Filter Field Name:"));
  filter_field_name_line_edit_ = new QLineEdit ();
  filter_limits_min_label_ = new QLabel (QObject::tr ("Filter Limit - Minimum:"));
  filter_limits_min_line_edit_ = new QLineEdit ();
  filter_limits_max_label_ = new QLabel (QObject::tr ("Filter Limit - Maximum:"));
  filter_limits_max_line_edit_ = new QLineEdit ();

  QDoubleValidator * double_validator = new QDoubleValidator ();
  filter_limits_min_line_edit_->setValidator (double_validator);
  filter_limits_max_line_edit_->setValidator (double_validator);

  std::stringstream ss_x;
  ss_x << filter_field_name_;
  filter_field_name_line_edit_->setText (QString (ss_x.str ().c_str ()));

  std::stringstream ss_y;
  ss_y << filter_limits_min_;
  filter_limits_min_line_edit_->setText (QString (ss_y.str ().c_str ()));

  std::stringstream ss_z;
  ss_z << filter_limits_max_;
  filter_limits_max_line_edit_->setText (QString (ss_z.str ().c_str ()));

  main_layout_ = new QVBoxLayout (filter_selection_page_);
  main_layout_->addWidget (filter_field_name_label_);
  main_layout_->addWidget (filter_field_name_line_edit_);
  main_layout_->addWidget (filter_limits_min_label_);
  main_layout_->addWidget (filter_limits_min_line_edit_);
  main_layout_->addWidget (filter_limits_max_label_);
  main_layout_->addWidget (filter_limits_max_line_edit_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
PassThroughCF::
~PassThroughCF ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
PassThroughCF::
filter (
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
  filter_field_name_ = filter_field_name_line_edit_->text ().toStdString ();
  filter_limits_min_ = filter_limits_min_line_edit_->text ().toFloat ();
  filter_limits_max_ = filter_limits_max_line_edit_->text ().toFloat ();

  pcl::PassThrough<pcl::PointXYZRGBA> filter;
  filter.setInputCloud (cloud_in);
  filter.setFilterFieldName (filter_field_name_);
  filter.setFilterLimits (filter_limits_min_, filter_limits_max_);
  filter.filter (*cloud_out);
}


//////////////////////////////////////////////////////////////////////////////////////////////
//############################################################################################
//############################################################################################
//////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
RadiusOutlierCF::
RadiusOutlierCF ()
: filter_selection_page_ (new QWizardPage ())
, search_radius_ (0.05f)
, min_neighbors_in_radius_ (500)
{
  search_radius_label_ = new QLabel (QObject::tr ("Search Radius:"));
  search_radius_line_edit_ = new QLineEdit ();
  min_neighbors_in_radius_label_ = new QLabel (QObject::tr ("Minimum Number of Neighbors within Radius:"));
  min_neighbors_in_radius_line_edit_ = new QLineEdit ();

  QDoubleValidator * double_validator = new QDoubleValidator ();
  QIntValidator * int_validator = new QIntValidator ();
  search_radius_line_edit_->setValidator (double_validator);
  min_neighbors_in_radius_line_edit_->setValidator (int_validator);

  std::stringstream ss_x;
  ss_x << search_radius_;
  search_radius_line_edit_->setText (QString (ss_x.str ().c_str ()));

  std::stringstream ss_y;
  ss_y << min_neighbors_in_radius_;
  min_neighbors_in_radius_line_edit_->setText (QString (ss_y.str ().c_str ()));

  main_layout_ = new QVBoxLayout (filter_selection_page_);
  main_layout_->addWidget (search_radius_label_);
  main_layout_->addWidget (search_radius_line_edit_);
  main_layout_->addWidget (min_neighbors_in_radius_label_);
  main_layout_->addWidget (min_neighbors_in_radius_line_edit_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
RadiusOutlierCF::
~RadiusOutlierCF ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
RadiusOutlierCF::
filter (
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
  search_radius_ = search_radius_line_edit_->text ().toFloat ();
  min_neighbors_in_radius_ = min_neighbors_in_radius_line_edit_->text ().toInt ();

  pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> filter;
  filter.setInputCloud (cloud_in);
  filter.setRadiusSearch (search_radius_);
  filter.setMinNeighborsInRadius (min_neighbors_in_radius_);
  filter.filter (*cloud_out);
}


//////////////////////////////////////////////////////////////////////////////////////////////
//############################################################################################
//############################################################################################
//////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
FastBilateralCF::
FastBilateralCF ()
: filter_selection_page_ (new QWizardPage ())
, sigma_s_ (5.0f)
, sigma_r_ (0.03f)
{
  sigma_s_label_ = new QLabel (QObject::tr ("Half Size (sigma s) (Minimum: 1.0):"));
  sigma_s_line_edit_ = new QLineEdit ();
  sigma_r_label_ = new QLabel (QObject::tr ("Standard Deviation (sigma r) (Minimum: 0.0001):"));
  sigma_r_line_edit_ = new QLineEdit ();

  QDoubleValidator * double_validator = new QDoubleValidator ();
  sigma_s_line_edit_->setValidator (double_validator);
  sigma_r_line_edit_->setValidator (double_validator);

  std::stringstream ss_x;
  ss_x << sigma_s_;
  sigma_s_line_edit_->setText (QString (ss_x.str ().c_str ()));

  std::stringstream ss_y;
  ss_y << sigma_r_;
  sigma_r_line_edit_->setText (QString (ss_y.str ().c_str ()));

  main_layout_ = new QVBoxLayout (filter_selection_page_);
  main_layout_->addWidget (sigma_s_label_);
  main_layout_->addWidget (sigma_s_line_edit_);
  main_layout_->addWidget (sigma_r_label_);
  main_layout_->addWidget (sigma_r_line_edit_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
FastBilateralCF::
~FastBilateralCF ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
FastBilateralCF::
filter (
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
  sigma_s_ = sigma_s_line_edit_->text ().toFloat ();
  sigma_r_ = sigma_r_line_edit_->text ().toFloat ();

  //std::cerr << "sigma_s: " << sigma_s_ << std::endl;
  //std::cerr << "sigma_r: " << sigma_r_ << std::endl;

  if (sigma_s_ <= 1.0f)
    sigma_s_ = 1.0f;
  if (sigma_r_ <= 0.0001f)
    sigma_r_ = 0.0001f;

  pcl::FastBilateralFilter<pcl::PointXYZRGBA> fbf;
  fbf.setInputCloud (cloud_in);
  fbf.setSigmaS (sigma_s_);
  fbf.setSigmaR (sigma_r_);
  fbf.filter (*cloud_out);
}


//////////////////////////////////////////////////////////////////////////////////////////////
//############################################################################################
//############################################################################################
//////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
MedianCF::
MedianCF ()
: filter_selection_page_ (new QWizardPage ())
, max_allowed_movement_ (1.0f)
, window_size_ (1)
{
  max_allowed_movement_label_ = new QLabel (QObject::tr ("Maximum Allowed Movement (Minimum: 0.0):"));
  max_allowed_movement_line_edit_ = new QLineEdit ();
  window_size_label_ = new QLabel (QObject::tr ("Window Size (Minimum: 1):"));
  window_size_line_edit_ = new QLineEdit ();

  QDoubleValidator * double_validator = new QDoubleValidator ();
  QIntValidator * int_validator = new QIntValidator ();
  max_allowed_movement_line_edit_->setValidator (double_validator);
  window_size_line_edit_->setValidator (int_validator);

  std::stringstream ss_x;
  ss_x << max_allowed_movement_;
  max_allowed_movement_line_edit_->setText (QString (ss_x.str ().c_str ()));

  std::stringstream ss_y;
  ss_y << window_size_;
  window_size_line_edit_->setText (QString (ss_y.str ().c_str ()));

  main_layout_ = new QVBoxLayout (filter_selection_page_);
  main_layout_->addWidget (max_allowed_movement_label_);
  main_layout_->addWidget (max_allowed_movement_line_edit_);
  main_layout_->addWidget (window_size_label_);
  main_layout_->addWidget (window_size_line_edit_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
MedianCF::
~MedianCF ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
MedianCF::
filter (
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
  max_allowed_movement_ = max_allowed_movement_line_edit_->text ().toFloat ();
  window_size_ = window_size_line_edit_->text ().toInt ();

  if (max_allowed_movement_ <= 0.0f)
    max_allowed_movement_ = 0.0f;
  if (window_size_ < 1)
    window_size_ = 1;

  pcl::MedianFilter<pcl::PointXYZRGBA> fbf;
  fbf.setInputCloud (cloud_in);
  fbf.setMaxAllowedMovement (max_allowed_movement_);
  fbf.setWindowSize (window_size_);
  fbf.filter (*cloud_out);
}


//////////////////////////////////////////////////////////////////////////////////////////////
//############################################################################################
//############################################################################################
//////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
RandomSampleCF::
RandomSampleCF ()
: filter_selection_page_ (new QWizardPage ())
, seed_ (1)
, sample_ (1000)
{
  seed_label_ = new QLabel (QObject::tr ("Seed for Random Number Generator:"));
  seed_line_edit_ = new QLineEdit ();
  sample_label_ = new QLabel (QObject::tr ("Number of Samples drawn from the Input Cloud:"));
  sample_line_edit_ = new QLineEdit ();

  QIntValidator * int_validator = new QIntValidator ();
  seed_line_edit_->setValidator (int_validator);
  sample_line_edit_->setValidator (int_validator);

  std::stringstream ss_x;
  ss_x << seed_;
  seed_line_edit_->setText (QString (ss_x.str ().c_str ()));

  std::stringstream ss_y;
  ss_y << sample_;
  sample_line_edit_->setText (QString (ss_y.str ().c_str ()));

  main_layout_ = new QVBoxLayout (filter_selection_page_);
  main_layout_->addWidget (seed_label_);
  main_layout_->addWidget (seed_line_edit_);
  main_layout_->addWidget (sample_label_);
  main_layout_->addWidget (sample_line_edit_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
RandomSampleCF::
~RandomSampleCF ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
RandomSampleCF::
filter (
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
  seed_ = seed_line_edit_->text ().toFloat ();
  sample_ = sample_line_edit_->text ().toInt ();

  if (sample_ < 1)
    sample_ = 1;

  pcl::RandomSample<pcl::PointXYZRGBA> fbf;
  fbf.setInputCloud (cloud_in);
  fbf.setSeed (seed_);
  fbf.setSample (sample_);
  fbf.filter (*cloud_out);
}


//////////////////////////////////////////////////////////////////////////////////////////////
//############################################################################################
//############################################################################################
//////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
PlaneCF::
PlaneCF ()
: filter_selection_page_ (new QWizardPage ())
, threshold_ (0.05f)
, max_iterations_ (1000)
, refinement_sigma_ (2)
, max_refinement_iterations_ (50)
, return_negative_ (false)
, cluster_tolerance_ (0.02)
, min_cluster_size_ (100)
{
  threshold_label_ = new QLabel (QObject::tr ("Threshold for plane detection:"));
  threshold_line_edit_ = new QLineEdit ();
  max_iterations_label_ = new QLabel (QObject::tr ("Maximum number of iterations for plane estimation:"));
  max_iterations_line_edit_ = new QLineEdit ();
  refinement_sigma_label_ = new QLabel (QObject::tr ("Sigma for plane parameter refinement:"));
  refinement_sigma_line_edit_ = new QLineEdit ();
  max_refinement_iterations_label_ = new QLabel (QObject::tr ("Maximum number of iterations for plane parameter refinement:"));
  max_refinement_iterations_line_edit_ = new QLineEdit ();
  return_negative_label_ = new QLabel (QObject::tr ("Invert filter (return everything except plane)?"));
  return_negative_check_box_ = new QCheckBox ();
  cluster_tolerance_label_ = new QLabel (QObject::tr ("Cluster tolerance for plane segmentation:"));
  cluster_tolerance_line_edit_ = new QLineEdit ();
  min_cluster_size_label_ = new QLabel (QObject::tr ("Minimum cluster size:"));
  min_cluster_size_line_edit_ = new QLineEdit ();

  QDoubleValidator * double_validator = new QDoubleValidator ();
  QIntValidator * int_validator = new QIntValidator ();
  threshold_line_edit_->setValidator (double_validator);
  refinement_sigma_line_edit_->setValidator (double_validator);
  cluster_tolerance_line_edit_->setValidator (double_validator);
  max_iterations_line_edit_->setValidator (int_validator);
  max_refinement_iterations_line_edit_->setValidator (int_validator);
  min_cluster_size_line_edit_->setValidator (int_validator);

  {
    std::stringstream ss;
    ss << threshold_;
    threshold_line_edit_->setText (QString (ss.str ().c_str ()));
  }
  {
    std::stringstream ss;
    ss << max_iterations_;
    max_iterations_line_edit_->setText (QString (ss.str ().c_str ()));
  }
  {
    std::stringstream ss;
    ss << refinement_sigma_;
    refinement_sigma_line_edit_->setText (QString (ss.str ().c_str ()));
  }
  {
    std::stringstream ss;
    ss << max_refinement_iterations_;
    max_refinement_iterations_line_edit_->setText (QString (ss.str ().c_str ()));
  }
  {
    std::stringstream ss;
    ss << cluster_tolerance_;
    cluster_tolerance_line_edit_->setText (QString (ss.str ().c_str ()));
  }
  {
    std::stringstream ss;
    ss << min_cluster_size_;
    min_cluster_size_line_edit_->setText (QString (ss.str ().c_str ()));
  }

  if (return_negative_)
    return_negative_check_box_->setCheckState (Qt::Checked);
  else
    return_negative_check_box_->setCheckState (Qt::Unchecked);

  main_layout_ = new QVBoxLayout (filter_selection_page_);
  main_layout_->addWidget (threshold_label_);
  main_layout_->addWidget (threshold_line_edit_);
  main_layout_->addWidget (max_iterations_label_);
  main_layout_->addWidget (max_iterations_line_edit_);
  main_layout_->addWidget (refinement_sigma_label_);
  main_layout_->addWidget (refinement_sigma_line_edit_);
  main_layout_->addWidget (max_refinement_iterations_label_);
  main_layout_->addWidget (max_refinement_iterations_line_edit_);
  main_layout_->addWidget (return_negative_label_);
  main_layout_->addWidget (return_negative_check_box_);
  main_layout_->addWidget (cluster_tolerance_label_);
  main_layout_->addWidget (cluster_tolerance_line_edit_);
  main_layout_->addWidget (min_cluster_size_label_);
  main_layout_->addWidget (min_cluster_size_line_edit_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
PlaneCF::
~PlaneCF ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
PlaneCF::
filter (
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
  threshold_ = threshold_line_edit_->text ().toFloat ();
  max_iterations_ = max_iterations_line_edit_->text ().toInt ();
  refinement_sigma_ = refinement_sigma_line_edit_->text ().toFloat ();
  max_refinement_iterations_ = max_refinement_iterations_line_edit_->text ().toInt ();
  cluster_tolerance_ = cluster_tolerance_line_edit_->text ().toFloat ();
  min_cluster_size_ = min_cluster_size_line_edit_->text ().toInt ();

  if (max_iterations_ < 0)
    max_iterations_ = 0;
  if (max_refinement_iterations_ < 0)
    max_refinement_iterations_ = 0;

  if (cluster_tolerance_ < 0.0001)
    cluster_tolerance_ = 0.0001;

  return_negative_ = return_negative_check_box_->checkState () == Qt::Checked;

  pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (cloud_in));
  pcl::RandomSampleConsensus<pcl::PointXYZRGBA> sac (model, threshold_);
  sac.setMaxIterations (max_iterations_);
  bool res = sac.computeModel ();

  std::vector<int> inliers;
  sac.getInliers (inliers);

  if (!res || inliers.empty ())
  {
    std::cerr << "no planar model found!" << std::endl;
    return;
  }

  sac.refineModel (refinement_sigma_, max_refinement_iterations_);
  sac.getInliers (inliers);

  if (return_negative_)
  {
    pcl::PointIndices::Ptr everything_but_the_plane (new pcl::PointIndices ());
    std::vector<int> indices_fullset (cloud_in->size ());
    for (int p_it = 0; p_it < static_cast<int> (indices_fullset.size ()); ++p_it)
      indices_fullset[p_it] = p_it;
    
    std::sort (inliers.begin (), inliers.end ());
    std::set_difference (indices_fullset.begin (), indices_fullset.end (),
                         inliers.begin (), inliers.end (),
                         inserter (everything_but_the_plane->indices, everything_but_the_plane->indices.begin ()));

    // Extract largest cluster minus the plane
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointXYZRGBA> ec;
    ec.setClusterTolerance (cluster_tolerance_);
    ec.setMinClusterSize (min_cluster_size_);
    ec.setInputCloud (cloud_in);
    ec.setIndices (everything_but_the_plane);
    ec.extract (cluster_indices);

    if (!cluster_indices.empty ())
    {
      // Convert data back
      pcl::copyPointCloud (*cloud_in, cluster_indices[0].indices, *cloud_out);
    }
  }
  else
  {
    pcl::copyPointCloud (*cloud_in, inliers, *cloud_out);
  }
}






