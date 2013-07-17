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
 * $Id: normal_estimation.cpp 6746 2012-08-08 01:20:30Z rusu $
 *
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/transformation_estimation_lm.h>

#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;

Eigen::Vector4f    translation;
Eigen::Quaternionf orientation;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input_source.pcd input_target.pcd output.pcd [optional_arguments]\n", argv[0]);
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud, translation, orientation) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", getFieldsList (cloud).c_str ());

  return (true);
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &source,
         const pcl::PCLPointCloud2::ConstPtr &target,
         pcl::PCLPointCloud2 &transformed_source)
{
  // Convert data to PointCloud<T>
  PointCloud<PointNormal>::Ptr src (new PointCloud<PointNormal>);
  PointCloud<PointNormal>::Ptr tgt (new PointCloud<PointNormal>);
  fromPCLPointCloud2 (*source, *src);
  fromPCLPointCloud2 (*target, *tgt);

  // Estimate
  TicToc tt;
  tt.tic ();
  
  print_highlight (stderr, "Computing ");

#define Scalar double
//#define Scalar float

  TransformationEstimationLM<PointNormal, PointNormal, Scalar>::Ptr te (new TransformationEstimationLM<PointNormal, PointNormal, Scalar>);
  //TransformationEstimationSVD<PointNormal, PointNormal, Scalar>::Ptr te (new TransformationEstimationSVD<PointNormal, PointNormal, Scalar>);
  CorrespondenceEstimation<PointNormal, PointNormal, double>::Ptr cens (new CorrespondenceEstimation<PointNormal, PointNormal, double>);
  //CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>::Ptr cens (new CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>);
  //CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal, double>::Ptr cens (new CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal, double>);
  cens->setInputSource (src);
  cens->setInputTarget (tgt);
  //cens->setSourceNormals (src);

  CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new CorrespondenceRejectorOneToOne);
  
  CorrespondenceRejectorMedianDistance::Ptr cor_rej_med (new CorrespondenceRejectorMedianDistance);
  cor_rej_med->setInputSource<PointNormal> (src);
  cor_rej_med->setInputTarget<PointNormal> (tgt);

  CorrespondenceRejectorSampleConsensus<PointNormal>::Ptr cor_rej_sac (new CorrespondenceRejectorSampleConsensus<PointNormal>);
  cor_rej_sac->setInputSource (src);
  cor_rej_sac->setInputTarget (tgt);
  cor_rej_sac->setInlierThreshold (0.005);
  cor_rej_sac->setMaximumIterations (10000);

  CorrespondenceRejectorVarTrimmed::Ptr cor_rej_var (new CorrespondenceRejectorVarTrimmed);
  cor_rej_var->setInputSource<PointNormal> (src);
  cor_rej_var->setInputTarget<PointNormal> (tgt);
  
  CorrespondenceRejectorTrimmed::Ptr cor_rej_tri (new CorrespondenceRejectorTrimmed);

  IterativeClosestPoint<PointNormal, PointNormal, Scalar> icp;
  icp.setCorrespondenceEstimation (cens);
  icp.setTransformationEstimation (te);
  icp.addCorrespondenceRejector (cor_rej_o2o);
  //icp.addCorrespondenceRejector (cor_rej_var);
  //icp.addCorrespondenceRejector (cor_rej_med);
  //icp.addCorrespondenceRejector (cor_rej_tri);
  //icp.addCorrespondenceRejector (cor_rej_sac);
  icp.setInputSource (src);
  icp.setInputTarget (tgt);
  icp.setMaximumIterations (1000);
  icp.setTransformationEpsilon (1e-10);
  PointCloud<PointNormal> output;
  icp.align (output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points], has converged: ");
  print_value ("%d", icp.hasConverged ()); print_info (" with score: %f\n",  icp.getFitnessScore ());
  Eigen::Matrix4d transformation = icp.getFinalTransformation ();
  //Eigen::Matrix4f transformation = icp.getFinalTransformation ();
  PCL_DEBUG ("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n", 
      transformation (0, 0), transformation (0, 1), transformation (0, 2), transformation (0, 3),
      transformation (1, 0), transformation (1, 1), transformation (1, 2), transformation (1, 3),
      transformation (2, 0), transformation (2, 1), transformation (2, 2), transformation (2, 3),
      transformation (3, 0), transformation (3, 1), transformation (3, 2), transformation (3, 3));

  // Convert data back
  pcl::PCLPointCloud2 output_source;
  toPCLPointCloud2 (output, output_source);
  concatenateFields (*source, output_source, transformed_source);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  //w.writeBinaryCompressed (filename, output, translation, orientation);
  w.writeASCII (filename, output, translation, orientation);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

int
main (int argc, char** argv)
{
  print_info ("Estimate a rigid transformation using IterativeClosestPoint. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }
  
  bool debug = false;
  pcl::console::parse_argument (argc, argv, "-debug", debug);
  if (debug)
    pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 3)
  {
    print_error ("Need two input PCD files (source, target) and one output PCD file to continue.\n");
    return (-1);
  }

  // Load the input files
  pcl::PCLPointCloud2::Ptr src (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *src)) return (-1);
  pcl::PCLPointCloud2::Ptr tgt (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[1]], *tgt)) return (-1);

  // Perform the feature estimation
  pcl::PCLPointCloud2 output;
  compute (src, tgt, output);

  // Save into the output file
  saveCloud (argv[p_file_indices[2]], output);

  return (0);
}
