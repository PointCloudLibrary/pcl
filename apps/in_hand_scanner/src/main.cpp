/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#include <cstdlib> // EXIT_SUCCESS, EXIT_FAILURE
#include <pcl/apps/in_hand_scanner/in_hand_scanner.h>

int main (int argc, char** argv)
{
  pcl::ihs::InHandScanner scanner (argc, argv);
  scanner.run ();

  return (EXIT_SUCCESS);
}

////////////////////////////////////////////////////////////////////////////////

//// test

//#include <cstdlib> // EXIT_SUCCESS, EXIT_FAILURE

//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/apps/in_hand_scanner/custom_interactor_style.h>
//#include <pcl/apps/in_hand_scanner/eigen.h>

//void
//keyboardCallback (const pcl::visualization::KeyboardEvent& event, void* = NULL)
//{
//  if(event.getKeyCode ()==27 || event.getKeyCode ()=='q')
//  {
//    exit (EXIT_SUCCESS);
//  }
//}

//int main (int argc, char** argv)
//{
//  pcl::visualization::PCLVisualizer vis (argc, argv, "PCL in-hand scanner", pcl::ihs::CustomInteractorStyle::New ());
//  vis.getRenderWindow ()->SetSize (640, 480);
//  vis.registerKeyboardCallback (keyboardCallback);

//  vis.addCube (-15, 15, -15, 15, 48, 70);

//  pcl::ihs::CustomInteractorStyle* rwi = dynamic_cast <pcl::ihs::CustomInteractorStyle*> (vis.getInteractorStyle ().GetPointer ());
//  if (!rwi)
//  {
//    return (EXIT_FAILURE);
//  }
//  rwi->resetCamera ();
//  rwi->setPivot (Eigen::Vector3d (0,0,55));

//  vis.spin ();

//  return (EXIT_SUCCESS);
//}

//// end test

////////////////////////////////////////////////////////////////////////////////

//// test

//#include <cstdlib> // EXIT_SUCCESS, EXIT_FAILURE

//#include <pcl/io/pcd_io.h>
//#include <pcl/common/transforms.h>
//#include <pcl/registration/transformation_estimation_svd.h>
//#include <pcl/registration/eigen.h>
//#include <pcl/common/time.h>
//#include <Eigen/Eigenvalues>

//typedef pcl::PointXYZ           Point;
//typedef pcl::PointCloud <Point> Cloud;
//typedef pcl::PCDReader          Reader;

//typedef double                       Scalar;
//typedef Eigen::Matrix <Scalar, 3, 3> Mat3;
//typedef Eigen::Matrix <Scalar, 4, 4> Mat4;
//typedef Eigen::Matrix <Scalar, 3, 1> Vec3;
//typedef Eigen::Matrix <Scalar, 4, 1> Vec4;
//typedef Eigen::AngleAxis <Scalar>    AngleAxis;

//typedef pcl::registration::TransformationEstimationSVD <Point, Point, Scalar> TransformationEstimation;

//typedef Eigen::Matrix <Scalar, 3, Eigen::Dynamic> Pts;

//////////////////////////////////////////////////////////////////////////////////

//Mat4 horn (const Pts& src, const Pts& tgt)
//{
//  if (src.rows () != tgt.rows ())
//  {
//    exit (EXIT_FAILURE);
//  }

//  Vec3 c_s = src.rowwise ().mean ();
//  Vec3 c_t = tgt.rowwise ().mean ();

//  const Mat3 S = (src.colwise() - c_s) *
//                 (tgt.colwise() - c_t).transpose();

//  Mat4 Q;
//  Q << S(0,0) + S(1,1) + S(2,2), // first row
//      0,
//      0,
//      0,

//      S(1,2) - S(2,1),           // second row
//      S(0,0) - S(1,1) - S(2,2),
//      0,
//      0,

//      S(2,0)  - S(0,2),          // third row
//      S(0,1)  + S(1,0),
//      -S(0,0) + S(1,1) - S(2,2),
//      0,

//      S(0,1)  - S(1,0),          // fourth row
//      S(2,0)  + S(0,2),
//      S(1,2)  + S(2,1),
//      -S(0,0) - S(1,1) + S(2,2);

//  typedef Eigen::Transform<Scalar,3,Eigen::Isometry> Isometry;
//  typedef Eigen::Quaternion<Scalar> Quaternion;

//  const Eigen::SelfAdjointEigenSolver<Mat4> eigensolver(Q);

//  const Vec4 q = eigensolver.eigenvectors().rightCols<1>();

//  const Quaternion R(q(0),q(1),q(2),q(3));

//  // Calculate the translation and put the transformation together
//  Isometry T = Isometry(R).pretranslate(c_t-R.matrix()*c_s);

//  return T.matrix ();
//}

// ////////////////////////////////////////////////////////////////////////////////

//int main ()
//{
//  const int n = 100; // For the timing
//  pcl::StopWatch watch;

//  Cloud cloud_source;
//  Cloud cloud_target;

//  Reader reader;
//  reader.read ("/Users/martin/Downloads/bun000_UnStructured.pcd", cloud_source);

//  Mat3 R;
//  R = AngleAxis ( 35.f * 3.1415f/180.f, Vec3::UnitZ ()) *
//      AngleAxis (-12.f * 3.1415f/180.f, Vec3::UnitY ()) *
//      AngleAxis ( 58.f * 3.1415f/180.f, Vec3::UnitZ ());
//  Vec3 t = Vec3 (214.f, -178.f, 50.f);

//  Mat4 T_ref = Mat4::Identity ();
//  T_ref.topLeftCorner  <3,3> () = R;
//  T_ref.topRightCorner <3,1> () = t;

//  pcl::transformPointCloud (cloud_source, cloud_target, T_ref);

//  TransformationEstimation est;
//  Mat4 T_svd = Mat4::Identity ();
//  watch.reset ();
//  for (int i=0; i<n; ++i)
//  {
//    est.estimateRigidTransformation (cloud_source, cloud_target, T_svd);
//  }
//  std::cerr << "SVD: " << watch.getTime () << std::endl;

//  //////////////////////////////////////////////////////////////////////////////

//  typedef Eigen::Matrix <Scalar, 3, Eigen::Dynamic> Pts;

//  Mat4 T_umeyama;
//  Pts pts_source, pts_target;
//  watch.reset ();
//  for (int i=0; i<n; ++i)
//  {
//    pts_source.conservativeResize (Eigen::NoChange, cloud_source.size ());
//    pts_target.conservativeResize (Eigen::NoChange, cloud_target.size ());

//    for (int i=0; i<cloud_source.size (); ++i)
//    {
//      const Point& src = cloud_source[i];
//      const Point& tgt = cloud_target[i];
//      pts_source.col (i) = Vec3 (src.x, src.y, src.z);
//      pts_target.col (i) = Vec3 (tgt.x, tgt.y, tgt.z);
//    }

//    T_umeyama = Eigen::umeyama (pts_source, pts_target, false);
//  }
//  std::cerr << "Umeyama: " << watch.getTime () << std::endl;

//  Mat4 T_horn;
//  watch.reset ();
//  for (int i=0; i<n; ++i)
//  {
//    pts_source.conservativeResize (Eigen::NoChange, cloud_source.size ());
//    pts_target.conservativeResize (Eigen::NoChange, cloud_target.size ());

//    for (int i=0; i<cloud_source.size (); ++i)
//    {
//      const Point& src = cloud_source[i];
//      const Point& tgt = cloud_target[i];
//      pts_source.col (i) = Vec3 (src.x, src.y, src.z);
//      pts_target.col (i) = Vec3 (tgt.x, tgt.y, tgt.z);
//    }

//    T_horn = horn (pts_source, pts_target);
//  }
//  std::cerr << "Horn: " << watch.getTime () << std::endl;

//  std::cerr << "Reference:" << std::endl
//            << T_ref        << std::endl
//            << "SVD:"       << std::endl
//            << T_svd        << std::endl
//            << "Umeyama:"   << std::endl
//            << T_umeyama    << std::endl
//            << "Horn:"      << std::endl
//            << T_horn       << std::endl;

//  return (EXIT_SUCCESS);
//}

//// end test


////////////////////////////////////////////////////////////////////////////////

//// test

//#include <cstdlib> // EXIT_SUCCESS, EXIT_FAILURE
//#include <pcl/apps/in_hand_scanner/common_types.h>
//#include <pcl/apps/in_hand_scanner/icp.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/common/transforms.h>

//int main ()
//{
//  typedef pcl::ihs::ICP::PointProcessed PointProcessed;
//  typedef pcl::ihs::ICP::CloudProcessed CloudProcessed;
//  typedef pcl::ihs::ICP::PointModel     PointModel;
//  typedef pcl::ihs::ICP::CloudModel     CloudModel;
//  typedef pcl::ihs::Transformation      Transformation;

//  pcl::PLYReader reader;
//  pcl::visualization::PCLVisualizer visualizer;
//  CloudModel::Ptr     cloud_model (new CloudModel ());
//  CloudProcessed::Ptr cloud_data (new CloudProcessed ());

//  reader.read ("/Users/martin/Downloads/bun000.ply", *cloud_model);
//  reader.read ("/Users/martin/Downloads/bun045.ply", *cloud_data);

//  pcl::visualization::PointCloudColorHandlerCustom <PointModel> ch_model (cloud_model, 0, 128, 255);
//  visualizer.addPointCloud <PointModel> (cloud_model, ch_model, "cloud_model");

//  pcl::visualization::PointCloudColorHandlerCustom <pcl::ihs::ICP::PointProcessed> ch_data (cloud_data, 230, 25, 25);
//  visualizer.addPointCloud <PointProcessed> (cloud_data, ch_data, "cloud_data");

//  visualizer.spinOnce (5e3);

//  pcl::ihs::ICP icp;
//  Transformation T = Transformation::Identity ();
//  if (!icp.findTransformation (cloud_model, cloud_data, Transformation::Identity (), T))
//  {
//    std::cerr << "Please comment out 'if ((T_init_inv * it_in->getNormalVector4fMap ()).z () < 0.f)' in icp.cpp\n";
//    return (EXIT_FAILURE);
//  }
//  pcl::transformPointCloudWithNormals (*cloud_data, *cloud_data, T);
//  std::cerr << T << std::endl;
//  visualizer.updatePointCloud <PointProcessed> (cloud_data, ch_data, "cloud_data");

//  visualizer.spin ();

//  return (EXIT_SUCCESS);
//}

//// end test

////////////////////////////////////////////////////////////////////////////////
