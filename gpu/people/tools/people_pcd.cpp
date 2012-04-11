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

#include <pcl/common/time.h>
//#include <opencv2/opencv.hpp>

#include <pcl/io/png_io.h>

#include <pcl/gpu/people/conversions.h>
#include <pcl/gpu/people/optimized_elec.h>
#include <pcl/gpu/people/optimized_shs.h>
#include <pcl/gpu/people/label_conversion.h>
#include <pcl/gpu/people/label_segment.h>
#include <pcl/gpu/people/label_tree.h>
#include <pcl/gpu/people/tree_live.h>

#include <pcl/visualization/image_viewer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <boost/lexical_cast.hpp>

#include <iostream>

//#define WRITE


#define AREA_THRES      200
#define AREA_THRES2     100
#define CLUST_TOL       0.05
#define CLUST_TOL_SHS   0.05
#define DELTA_HUE_SHS   5
#define MAX_CLUST_SIZE  25000

using namespace pcl::gpu::people;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace std;
using namespace boost;
namespace pc = pcl::console;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void imwrite(const std::string& prefix, int counter, const cv::Mat& mat)
{   
   
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class PeoplePCDApp
{
  public:
    //PeoplePCDApp () : viewer ("PCL People PCD App") {}
    PeoplePCDApp () : final_view_("Final labeling")
    {
      allocate_buffers(480, 640);
    }

    void allocate_buffers(int rows, int cols)
    {
        depth.resize(rows * cols);
        depth.width = cols;
        depth.height = rows;
        depth.is_dense = true;
    };

    pcl::PointCloud<unsigned short> depth;

    pcl::gpu::DeviceArray<pcl::PointXYZRGB> cloud_device;

    void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, int counter)
    {
      //if (!viewer.wasStopped())
      //  viewer.showCloud (cloud);
      ////////////////////CALLBACK IMPL/////////////////////

      /// @todo rewrite this to a pointcloud::Ptr
      pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
      pcl::PointCloud<pcl::PointXYZRGB> cloud_in_filt;
      cloud_in = *cloud;

      cv::Mat dmat(depth.height, depth.width, CV_16U, &depth.points[0]);

      // Project pointcloud back into the imageplane
      // TODO: do this directly in GPU?
      pcl::gpu::people::label_skeleton::makeDepthImage16FromPointCloud(dmat, cloud_in);

      // Process the depthimage (CUDA)
      m_proc->process(dmat, m_lmap);

      cv::Mat lmap(cloud_in.height, cloud_in.width, CV_8UC1);
      pcl::gpu::people::label_skeleton::smoothLabelImage(m_lmap, dmat, lmap);

#ifdef WRITE      
      cv::imwrite("d_" + lexical_cast<string>(counter) + ".png", dmat);
      cv::imwrite("l_" + lexical_cast<string>(counter) + ".png", m_lmap);
      cv::imwrite("s_" + lexical_cast<string>(counter) + ".png", lmap);
                  
      cv::Mat input(cloud_in.height, cloud_in.width, CV_8UC3);
      pcl::gpu::people::label_skeleton::makeImageFromPointCloud(input, cloud_in);

      cv::imwrite("i_" + lexical_cast<string>(counter) + ".png", input);      
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
      cv::imwrite("c_" + lexical_cast<string>(counter) + ".png", cmap);      
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
     cv::imwrite("b_" + lexical_cast<string>(counter) + ".png", binmask);
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
      cv::imwrite("f_" + lexical_cast<string>(counter) + ".png", flowermat);
#endif
      cv::Mat flowergrownmat(cloud_in.height, cloud_in.width, CV_8UC3, cv::Scalar(0));

      int erosion_size = 2;
      cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT ,
                                                  cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                  cv::Point( erosion_size, erosion_size ) );

      cv::dilate(flowermat, flowergrownmat, element);

#ifdef WRITE
      cv::imwrite("g_" + lexical_cast<string>(counter) + ".png", flowergrownmat);
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
      cv::imwrite("d2_" + lexical_cast<string>(counter) + ".png", dmat2);
      cv::imwrite("l2_" + lexical_cast<string>(counter) + ".png", m_lmap);
      cv::imwrite("s2_" + lexical_cast<string>(counter) + ".png", lmap2);
                        
      
      cv::imwrite("c2_" + lexical_cast<string>(counter) + ".png", cmap);      
#endif
      pcl::gpu::people::display::colorLMap( lmap2, cmap );
      final_view_.showRGBImage(cmap.ptr<unsigned char>(), cmap.cols, cmap.rows);
      final_view_.spinOnce(1, true);

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
    trees::MultiTreeLiveProc::Ptr m_proc;
    cv::Mat                                     m_lmap;
    cv::Mat                                     m_cmap;
    cv::Mat                                     cmap;
    cv::Mat                                     m_bmap;


    ImageViewer final_view_;
};

int print_help()
{
  cout << "\nPeople tracking app options:" << endl;
  cout << "\t -numTrees \t<int> \tnumber of trees to load" << endl;
  cout << "\t -tree0 \t<path_to_tree_file>" << endl;
  cout << "\t -tree1 \t<path_to_tree_file>" << endl;
  cout << "\t -tree2 \t<path_to_tree_file>" << endl;
  cout << "\t -tree3 \t<path_to_tree_file>" << endl;
  cout << "\t -pcd   \t<path_to_pcd_file>" << endl;
  return 0;
}

int main(int argc, char** argv)
{
  if(pc::find_switch (argc, argv, "--help") || pc::find_switch (argc, argv, "-h"))
    return print_help();

  std::string treeFilenames[4] = 
  {
    "d:/TreeData/results/forest1/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt"
  };
  int         numTrees = 4;
  std::string pcdname = "d:/git/pcl/gpu/people/tools/test.pcd";
  pc::parse_argument (argc, argv, "-numTrees", numTrees);
  pc::parse_argument (argc, argv, "-tree0", treeFilenames[0]);
  pc::parse_argument (argc, argv, "-tree1", treeFilenames[1]);
  pc::parse_argument (argc, argv, "-tree2", treeFilenames[2]);
  pc::parse_argument (argc, argv, "-tree3", treeFilenames[3]);
  pc::parse_argument (argc, argv, "-pcd", pcdname);
  //Don't know if this assert is still needed with pcl::console?
  //AB: pcl::console does nothing if arg is not found
  assert(numTrees > 0 );
  assert(numTrees <= 4 );

  /// Create the app
  PeoplePCDApp app;

  /// Load the first tree
  std::ifstream fin0(treeFilenames[0].c_str() );
  assert(fin0.is_open() );
  app.m_proc.reset(new trees::MultiTreeLiveProc(fin0));
  fin0.close();

  /// Load the other tree files
  for(int ti=1;ti<numTrees;++ti) {
    std::ifstream fin(treeFilenames[ti].c_str() );
    assert(fin.is_open() );
    app.m_proc->addTree(fin);
    fin.close();
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  int res = pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdname, *cloud);
  if (res == -1) //* load the file
  {    
    PCL_ERROR ("Couldn't read file\n");
    return (-1);
  }
  cout << "Loaded " << cloud->width * cloud->height << " data points from " << pcdname << endl;

  /// Run the app
  
  {
    ScopeTime frame_time("frame_time");
    app.cloud_cb(cloud, 1);
  }

  app.final_view_.spin();
  return 0;
}
