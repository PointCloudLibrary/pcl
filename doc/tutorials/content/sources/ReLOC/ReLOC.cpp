#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ReLOC.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::registration;
using namespace std;

void main (int argc, char** argv)
{
  cout << "Load target" << endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("bun0.ply", *target) == -1) 
  {
    PCL_ERROR ("Couldn't read file bun0.ply \n");
    return ;
  }

  cout << "Load source" << endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("bun4.ply", *source) == -1) 
  {
    PCL_ERROR ("Couldn't read file bun4.ply \n");
    return ;
  }

  //apply a translation to the target for better disambiguation of normal signs
  Eigen::Vector3f translation_target;
  translation_target[0] = 0;
  translation_target[1] = -0.3;
  translation_target[2] = 0.5;
  Eigen::Quaternionf rotation_target(0.0f, 0.0f, 0.0f, 0.0f);

  pcl::transformPointCloud(*target, *target, translation_target, rotation_target);

  //apply a translation to the source for better disambiguation of normal signs
  Eigen::Vector3f translation_source;
  translation_source[0] = 0;
  translation_source[1] = 0.3;
  translation_source[2] = 0.5;
  Eigen::Quaternionf rotation_source(0.0f, 0.0f, 0.0f, 0.0f);

  pcl::transformPointCloud(*source, *source, translation_source, rotation_source);


  //Provide an estimation of the average distance between adjacent points of the two clouds (mesh resolution)
  float meshRes = 0.004f;


  // Estimate normals for source and target
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  ne.setRadiusSearch (3.0 * meshRes);

  cout << "Estimate target normals" << endl;
  pcl::PointCloud<pcl::Normal>::Ptr normals_trg (new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud (target);
  ne.compute (*normals_trg);

  cout << "Estimate source normals" << endl;
  pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(source);
  ne.compute(*normals_src);
  


  //Apply ReLOC algorithm
  pcl::registration::ReLOCInitialAlignment <PointXYZ, PointXYZ> ReLOC_ia;
  ReLOC_ia.setInputSource (source);
  ReLOC_ia.setInputTarget (target);
  ReLOC_ia.setSourceNormals (normals_src); 
  ReLOC_ia.setTargetNormals (normals_trg); 

  //set detector parameters
  ReLOC_ia.setSeed (0xFFFFFFFF);
  ReLOC_ia.setFlatKeypointRf (3.0*meshRes); //5.0 in the original paper
  ReLOC_ia.setFlatKeypointRdiscard (2.0*meshRes);
  ReLOC_ia.setFlatKeypointR1search (2.0*meshRes);
  ReLOC_ia.setFlatKeypointR2search (10.0*meshRes);
  //apply random detection instead of FlatKeypoint detector
  ReLOC_ia.useRandomDetector (true);
  ReLOC_ia.setNrandomKeypoints (300);

  //set FLARE parameters
  ReLOC_ia.setFlareNormalRadius (5.0*meshRes);
  ReLOC_ia.setFlareTangentRadius (20.0*meshRes);
  //ReLOC_ia.setFlareXsupportSamplingPerc (0.2); //used to speed up LRFs computation. Consider only a percentage of the points of the support used for the computation of x axis. 

  //set Hough voting and RANSAC parameters
  ReLOC_ia.setHoughSbin (2.0*meshRes);
  ReLOC_ia.setRansacT (4.0*meshRes); //8.0 in the original paper

  //estimate rigid motion and apply to source cloud
  PointCloud <PointXYZ> source_aligned;
  ReLOC_ia.align (source_aligned);
  Eigen::Matrix4f rigidMotion_4f = reLOC.getFinalTransformation();

}
