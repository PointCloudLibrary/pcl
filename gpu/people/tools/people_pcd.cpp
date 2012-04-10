/**
 * @brief This file is the execution node of the Human Tracking 
 * @copyright Copyright (2011) Willow Garage
 * @authors Koen Buys, Anatoly Baksheev
 **/
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_labeled_clusters.h>
#include <pcl/segmentation/seeded_hue_segmentation.h>

//#include <opencv2/opencv.hpp>

#include <pcl/gpu/people/conversions.h>
#include <pcl/gpu/people/optimized_elec.h>
#include <pcl/gpu/people/optimized_shs.h>
#include <pcl/gpu/people/label_conversion.h>
#include <pcl/gpu/people/label_segment.h>
#include <pcl/gpu/people/label_tree.h>
#include <pcl/gpu/people/tree_live.h>

//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <iostream>
#include <sstream>

#define WRITE
#define AREA_THRES      200
#define AREA_THRES2     100
#define CLUST_TOL       0.05
#define CLUST_TOL_SHS   0.05
#define DELTA_HUE_SHS   5
#define MAX_CLUST_SIZE  25000


class PeoplePCDApp
{
  public:
    //PeoplePCDApp () : viewer ("PCL People PCD App") {}
    PeoplePCDApp (){}

    void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, int counter)
    {
      //if (!viewer.wasStopped())
      //  viewer.showCloud (cloud);
      ////////////////////CALLBACK IMPL/////////////////////

      /// @todo rewrite this to a pointcloud::Ptr
      pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
      pcl::PointCloud<pcl::PointXYZRGB> cloud_in_filt;
      cloud_in = *cloud;

      cv::Mat dmat(cloud_in.height, cloud_in.width, CV_16U);

      // Project pointcloud back into the imageplane
      // TODO: do this directly in GPU?
      pcl::gpu::people::label_skeleton::makeDepthImage16FromPointCloud(dmat, cloud_in);

      // Process the depthimage (CUDA)
      m_proc->process(dmat, m_lmap);

      cv::Mat lmap(cloud_in.height, cloud_in.width, CV_8UC1);
      pcl::gpu::people::label_skeleton::smoothLabelImage(m_lmap, dmat, lmap);

#ifdef WRITE
      std::stringstream ss;
      ss << "d_"  << counter << ".png";
      cv::imwrite(ss.str(), dmat);
      ss.str("");
      ss << "l_"  << counter << ".png";
      cv::imwrite(ss.str(), m_lmap);
      ss.str("");
      ss << "s_"  << counter << ".png";
      cv::imwrite(ss.str(), lmap);

      cv::Mat input(cloud_in.height, cloud_in.width, CV_8UC3);
      pcl::gpu::people::label_skeleton::makeImageFromPointCloud(input, cloud_in);

      ss.str("");
      ss << "i_"  << counter << ".png";
      cv::imwrite(ss.str(), input);
#endif

      pcl::PointCloud<pcl::PointXYZRGBL> cloud_labels;

      pcl::gpu::people::conversion::colorLabelPointCloudFromArray(cloud_in, lmap.data, cloud_labels);

      // Creating the Search object for the search method of the extraction
      pcl::search::OrganizedNeighbor<pcl::PointXYZRGBL>::Ptr stree (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBL>);
      stree->setInputCloud(cloud_labels.makeShared());
      std::vector<std::vector<pcl::PointIndices> > cluster_indices;
      cluster_indices.resize(NUM_PARTS);

      // Make all the clusters
      optimized_elec(cloud_in, lmap, CLUST_TOL, cluster_indices, AREA_THRES, MAX_CLUST_SIZE, NUM_PARTS, false, 1.f);

      // Create a new struct to put the results in
      std::vector<std::vector<pcl::gpu::people::label_skeleton::Blob2, Eigen::aligned_allocator<pcl::gpu::people::label_skeleton::Blob2> > >       sorted;
      //clear out our matrix before starting again with it
      sorted.clear();
      //Set fixed size of outer vector length = number of parts
      sorted.resize(NUM_PARTS);

      //create the blob2 matrix
      pcl::gpu::people::label_skeleton::sortIndicesToBlob2 ( cloud_labels, AREA_THRES, sorted, cluster_indices );
      //Build relationships between the blobs
      pcl::gpu::people::label_skeleton::buildRelations ( sorted );

      // ////////////////////////////////////////////////////////////////////////////////////////// //
      // DISPLAY INTERMEDIATE RESULTS UPTILL FINDING OF THE TREES, NOT EVALUATING TREES YET
#if defined(WRITE)
      // color
      pcl::gpu::people::display::colorLMap( lmap, cmap );
      ss.str("");
      ss << "c_" << counter << ".png";
      cv::imwrite(ss.str(), cmap);
#endif

      // ////////////////////////////////////////////////////////////////////////////////////////////// //
      // if we found a neck display the tree, and continue with processing
      if(sorted[10].size() != 0)
      {
          unsigned int c = 0;
          pcl::gpu::people::label_skeleton::Tree2 t;
          pcl::gpu::people::label_skeleton::buildTree(sorted, cloud_in, Neck, c, t);

          cv::Mat mask(cloud_in.height, cloud_in.width, CV_8UC1, cv::Scalar(0));

          pcl::gpu::people::label_skeleton::makeFGMaskFromPointCloud(mask, t.indices, cloud_in);

          pcl::PointIndices seed;
#if defined(DISPL_BINMASK) || defined(WRITE)
          cv::Mat binmask(cloud_in.height, cloud_in.width, CV_8UC3, cv::Scalar(0));
#endif
          for(unsigned int v = 0; v < cloud_in.height; v++)
          {
            for(unsigned int u = 0; u < cloud_in.width; u++)
            {
              if(mask.at<char>(v,u) == cv::GC_PR_FGD)
              {
#if defined(DISPL_BINMASK) || defined(WRITE)
                binmask.at<cv::Vec3b>(v,u)[0] = cloud_in.points[v*cloud_in.width + u].b;
                binmask.at<cv::Vec3b>(v,u)[1] = cloud_in.points[v*cloud_in.width + u].g;
                binmask.at<cv::Vec3b>(v,u)[2] = cloud_in.points[v*cloud_in.width + u].r;
#endif
                seed.indices.push_back(v*cloud_in.width + u);
              }
            }
          }
#ifdef WRITE
          ss.str("");
          ss << "b_"<< counter << ".png";
          cv::imwrite(ss.str(), binmask);
#endif

      // //////////////////////////////////////////////////////////////////////////////////////////////// //
      // The second kdtree evaluation = seeded hue segmentation
      // Reuse the fist searchtree for this, in order to NOT build it again!
      pcl::PointIndices flower;
      //pcl::seededHueSegmentation(cloud_in, stree, CLUST_TOL_SHS, seed, flower, DELTA_HUE_SHS);
      optimized_shs2(cloud_in, CLUST_TOL_SHS, seed, flower, DELTA_HUE_SHS);

      cv::Mat flowermat(cloud_in.height, cloud_in.width, CV_8UC3, cv::Scalar(0));
      pcl::gpu::people::label_skeleton::makeImageFromPointCloud(flowermat, flower, cloud_in);

#ifdef WRITE
      ss.str("");
      ss << "f_" << counter << ".png";
      cv::imwrite(ss.str(), flowermat);
#endif
      cv::Mat flowergrownmat(cloud_in.height, cloud_in.width, CV_8UC3, cv::Scalar(0));

      int erosion_size = 2;
      cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT ,
                                                  cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                  cv::Point( erosion_size, erosion_size ) );

      cv::dilate(flowermat, flowergrownmat, element);

#ifdef WRITE
      ss.str("");
      ss << "g_" << counter << ".png";
      cv::imwrite(ss.str(), flowergrownmat);
#endif

      cv::Mat dmat2(cloud_in.height, cloud_in.width, CV_16U);
      for(unsigned int v = 0; v < cloud_in.height; v++)
      {
        for(unsigned int u = 0; u < cloud_in.width; u++)
        {
          if(flowergrownmat.at<cv::Vec3b>(v,u)[0] != 0 || flowergrownmat.at<cv::Vec3b>(v,u)[1] != 0 || flowergrownmat.at<cv::Vec3b>(v,u)[2] != 0)
          {
            dmat2.at<short>(v,u) = dmat.at<short>(v,u);
          }
          else
          {
            dmat2.at<short>(v,u) = std::numeric_limits<short>::max();
          }
        }
      }

      // //////////////////////////////////////////////////////////////////////////////////////////////// //
      // The second label evaluation

      // Process the depthimage
      m_proc->process(dmat2, m_lmap);
      cv::Mat lmap2(cloud_in.height, cloud_in.width, CV_8UC1);
      pcl::gpu::people::label_skeleton::smoothLabelImage(m_lmap, dmat2, lmap2);
      //cv::medianBlur(m_lmap, lmap2, 3);

#ifdef WRITE
      ss.str("");
      ss << "d2_" << counter << ".png";
      cv::imwrite(ss.str(), dmat2);
      ss.str("");
      ss << "l2_" << counter << ".png";
      cv::imwrite(ss.str(), m_lmap);
      ss.str("");
      ss << "s2_" << counter << ".png";
      cv::imwrite(ss.str(), lmap2);
      // Publish this on a image topic
      pcl::gpu::people::display::colorLMap( lmap2, cmap );
      ss.str("");
      ss << "c2_"  << counter << ".png";
      cv::imwrite(ss.str(), cmap);
#endif

      pcl::PointCloud<pcl::PointXYZRGBL> cloud_labels2;
      pcl::gpu::people::conversion::colorLabelPointCloudFromArray(cloud_in, lmap2.data, cloud_labels2);

#ifdef WRITE
      pcl::io::savePCDFileASCII ("2de_it_colorLabeledPointCloud.pcd", cloud_labels2);
      std::cout << "Saved " << cloud_labels2.points.size () << " data points to 2de_it_colorLabeledPointCloud.pcd." << std::endl; 
#endif

        std::vector<std::vector<pcl::PointIndices> > cluster_indices2;
        cluster_indices2.resize(NUM_PARTS);

        // avoid having the search tree to be build again
        //pcl::extractLabeledEuclideanClusters<pcl::PointXYZRGBL>(cloud_labels2, stree, CLUST_TOL, cluster_indices2, AREA_THRES2, MAX_CLUST_SIZE, NUM_PARTS);
        optimized_elec(cloud_in, lmap2, CLUST_TOL, cluster_indices2, AREA_THRES2, MAX_CLUST_SIZE, NUM_PARTS, false, 1.0f);

        std::vector<std::vector<pcl::gpu::people::label_skeleton::Blob2, Eigen::aligned_allocator<pcl::gpu::people::label_skeleton::Blob2> > >       sorted2;
        //clear out our matrix before starting again with it
        sorted2.clear();
        //Set fixed size of outer vector length = number of parts
        sorted2.resize(NUM_PARTS);
        //create the blob2 matrix
        pcl::gpu::people::label_skeleton::sortIndicesToBlob2 ( cloud_labels2, AREA_THRES2, sorted2, cluster_indices2 );
        pcl::gpu::people::label_skeleton::buildRelations ( sorted2 );

        // Test if the second tree is build up correctly
        if(sorted2[10].size() != 0)
        {
          pcl::gpu::people::label_skeleton::Tree2 t2;
          pcl::gpu::people::label_skeleton::buildTree(sorted, cloud_in, Neck, c, t2);
          int par = 0;
          for(int f=0;f<NUM_PARTS;f++)
          {
            if(t2.parts_lid[f] == NO_CHILD)
            {
              std::cerr << "1;";
              par++;
            }
            else
              std::cerr << "0;";
          }
          std::cerr<< t2.nr_parts << ";";
          std::cerr<< par << ";";
          std::cerr<< t2.total_dist_error << ";";
          std::cerr<< t2.norm_dist_error << ";";
          std::cerr<< counter << ";" << std::endl;
        }
        counter++;
      }
    }
    //pcl::visualization::CloudViewer         viewer;
    pcl::gpu::people::trees::MultiTreeLiveProc* m_proc;
    cv::Mat                                     m_lmap;
    cv::Mat                                     m_cmap;
    cv::Mat                                     cmap;
    cv::Mat                                     m_bmap;
};

int print_help()
{
  std::cout << "\nPeople tracking app options:" << std::endl;
  std::cout << "\t -numTrees \t<int> \tnumber of trees to load" << std::endl;
  std::cout << "\t -tree0 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -tree1 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -tree2 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -tree3 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -pcd   \t<path_to_pcd_file>" << std::endl;
  return 0;
}

int main(int argc, char** argv)
{
  if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
    return print_help();

  std::string treeFilenames[4] = 
  {
    "d:/TreeData/results/forest1/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt"
  };
  int         numTrees = 4;
  std::string pcdname = "d:/3/0015.pcd";
  pcl::console::parse_argument (argc, argv, "-numTrees", numTrees);
  pcl::console::parse_argument (argc, argv, "-tree0", treeFilenames[0]);
  pcl::console::parse_argument (argc, argv, "-tree1", treeFilenames[1]);
  pcl::console::parse_argument (argc, argv, "-tree2", treeFilenames[2]);
  pcl::console::parse_argument (argc, argv, "-tree3", treeFilenames[3]);
  pcl::console::parse_argument (argc, argv, "-pcd", pcdname);
  //Don't know if this assert is still needed with pcl::console?
  //AB: pcl::console does nothing if arg is not found
  assert(numTrees > 0 );
  assert(numTrees <= 4 );

  /// Create the app
  PeoplePCDApp app;

  /// Load the first tree
  std::ifstream fin0(treeFilenames[0].c_str() );
  assert(fin0.is_open() );
  app.m_proc = new pcl::gpu::people::trees::MultiTreeLiveProc(fin0);
  fin0.close();

  /// Load the other tree files
  for(int ti=1;ti<numTrees;++ti) {
    std::ifstream fin(treeFilenames[ti].c_str() );
    assert(fin.is_open() );
    app.m_proc->addTree(fin);
    fin.close();
  }


  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdname, *cloud) == -1) //* load the file
  {
    //PCL_ERROR ("Couldn't read file %s \n", pcdname);
    PCL_ERROR ("Couldn't read file\n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from " << pcdname
            << std::endl;

  for(size_t i = 0; i < cloud->points.size (); i++)
  {
    pcl::PointXYZRGB p;
    p.x = cloud->points[i].x;
    p.y = cloud->points[i].y;
    p.z = cloud->points[i].z;
    p.r = cloud->points[i].r;
    p.g = cloud->points[i].g;
    p.b = cloud->points[i].b;
    cloud2->points.push_back(p);
  }
  cloud2->width = cloud->width;
  cloud2->height = cloud->height;

  /// Run the app
  app.cloud_cb(cloud2, 1);
  return 0;
}
