#include <gtest/gtest.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/image_grabber.h>

#include <string>
#include <vector>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;

string tiff_dir_;
string pclzf_dir_;
string pcd_dir_;
vector<CloudT::ConstPtr> pcds_;


// Helper function for grabbing a cloud
void
cloud_callback (bool *signal_received, 
                CloudT::ConstPtr *ptr_to_fill, 
                const CloudT::ConstPtr &input_cloud)
{
  *signal_received = true;
  *ptr_to_fill = input_cloud;
}

TEST (PCL, ImageGrabberTIFF)
{
  // Get all clouds from the grabber
  pcl::ImageGrabber<PointT> grabber (tiff_dir_, 0, false, false);
  vector <CloudT::ConstPtr> tiff_clouds;
  CloudT::ConstPtr cloud_buffer;
  bool signal_received = false;
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> 
    fxn = boost::bind (cloud_callback, &signal_received, &cloud_buffer, _1);
  grabber.registerCallback (fxn);
  grabber.start ();
  for (size_t i = 0; i < grabber.numFrames (); i++)
  {
    grabber.trigger ();
    size_t niter = 0;
    while (!signal_received)
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (10000));
      if (++niter > 100)
      {
        ASSERT_TRUE (false);
        return;
      }
    }
    tiff_clouds.push_back (cloud_buffer);
    signal_received = false;
  }

  // Make sure they match
  EXPECT_EQ (pcds_.size (), tiff_clouds.size ());
  for (size_t i = 0; i < pcds_.size (); i++)
  {
    for (size_t j = 0; j < pcds_[i]->size (); j++)
    {
      const PointT &pcd_pt = pcds_[i]->at(j);
      const PointT &tiff_pt = tiff_clouds[i]->at(j);
      if (pcl_isnan (pcd_pt.x))
        EXPECT_TRUE (pcl_isnan (tiff_pt.x));
      else
        EXPECT_FLOAT_EQ (pcd_pt.x, tiff_pt.x);
      if (pcl_isnan (pcd_pt.y))
        EXPECT_TRUE (pcl_isnan (tiff_pt.y));
      else
        EXPECT_FLOAT_EQ (pcd_pt.y, tiff_pt.y);
      if (pcl_isnan (pcd_pt.z))
        EXPECT_TRUE (pcl_isnan (tiff_pt.z));
      else
        EXPECT_FLOAT_EQ (pcd_pt.z, tiff_pt.z);
      EXPECT_EQ (pcd_pt.r, tiff_pt.r);
      EXPECT_EQ (pcd_pt.g, tiff_pt.g);
      EXPECT_EQ (pcd_pt.b, tiff_pt.b);
      //EXPECT_EQ (pcd_pt.a, tiff_pt.a);    // the alpha channel in the tiff PCD set was saved as 255 but it should have been 0
    }
  }
}

TEST (PCL, ImageGrabberPCLZF)
{
  // Get all clouds from the grabber
  pcl::ImageGrabber<PointT> grabber (pclzf_dir_, 0, false, true);
  vector <CloudT::ConstPtr> pclzf_clouds;
  CloudT::ConstPtr cloud_buffer;
  bool signal_received = false;
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> 
    fxn = boost::bind (cloud_callback, &signal_received, &cloud_buffer, _1);
  grabber.registerCallback (fxn);
  grabber.start ();
  for (size_t i = 0; i < grabber.numFrames (); i++)
  {
    grabber.trigger ();
    size_t niter = 0;
    while (!signal_received)
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (10000));
      if (++niter > 100)
      {
        ASSERT_TRUE (false);
        return;
      }
    }
    pclzf_clouds.push_back (cloud_buffer);
    signal_received = false;
  }

  // Make sure they match
  EXPECT_EQ (pcds_.size (), pclzf_clouds.size ());
  for (size_t i = 0; i < pcds_.size (); i++)
  {
    for (size_t j = 0; j < pcds_[i]->size (); j++)
    {
      const PointT &pcd_pt = pcds_[i]->at(j);
      const PointT &pclzf_pt = pclzf_clouds[i]->at(j);
      if (pcl_isnan (pcd_pt.x))
        EXPECT_TRUE (pcl_isnan (pclzf_pt.x));
      else
        EXPECT_FLOAT_EQ (pcd_pt.x, pclzf_pt.x);
      if (pcl_isnan (pcd_pt.y))
        EXPECT_TRUE (pcl_isnan (pclzf_pt.y));
      else
        EXPECT_FLOAT_EQ (pcd_pt.y, pclzf_pt.y);
      if (pcl_isnan (pcd_pt.z))
        EXPECT_TRUE (pcl_isnan (pclzf_pt.z));
      else
        EXPECT_FLOAT_EQ (pcd_pt.z, pclzf_pt.z);
      EXPECT_EQ (pcd_pt.r, pclzf_pt.r);
      EXPECT_EQ (pcd_pt.g, pclzf_pt.g);
      EXPECT_EQ (pcd_pt.b, pclzf_pt.b);
      EXPECT_EQ (pcd_pt.a, pclzf_pt.a);
    }
  }
}

TEST (PCL, ImageGrabberSetIntrinsicsTIFF)
{
  pcl::ImageGrabber<PointT> grabber (tiff_dir_, 0, false, false);

  // Get all clouds from the grabber
  vector <CloudT::ConstPtr> tiff_clouds;
  CloudT::ConstPtr cloud_buffer;
  bool signal_received = false;
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> 
    fxn = boost::bind (cloud_callback, &signal_received, &cloud_buffer, _1);
  grabber.registerCallback (fxn);
  grabber.start ();
  // Change the camera parameters
  double fx_multiplier = 1.2;
  double fy_multiplier = 0.5;
  double cx_multiplier = 0.8;
  double cy_multiplier = 1.3;
  double fx_old, fy_old, cx_old, cy_old;
  grabber.getCameraIntrinsics (fx_old, fy_old, cx_old, cy_old);
  double fx_new = fx_multiplier * fx_old;
  double fy_new = fy_multiplier * fy_old;
  double cx_new = cx_multiplier * cx_old;
  double cy_new = cy_multiplier * cy_old;
  grabber.setCameraIntrinsics (fx_new, fy_new, cx_new, cy_new);
  // Collect the clouds
  for (size_t i = 0; i < grabber.numFrames (); i++)
  {
    grabber.trigger ();
    size_t niter = 0;
    while (!signal_received)
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (10000));
      if (++niter > 100)
      {
        ASSERT_TRUE (false);
        return;
      }
    }
    tiff_clouds.push_back (cloud_buffer);
    signal_received = false;
  }

  // Make sure they match
  EXPECT_EQ (pcds_.size (), tiff_clouds.size ());
  for (size_t i = 0; i < pcds_.size (); i++)
  {
    EXPECT_EQ (pcds_[i]->width, tiff_clouds[i]->width);
    EXPECT_EQ (pcds_[i]->height, tiff_clouds[i]->height);
    for (int x = 0; x < pcds_[i]->width; x++)
    {
      for (int y = 0; y < pcds_[i]->height; y++)
      {
        const PointT &pcd_pt = pcds_[i]->operator()(x,y);
        const PointT &tiff_pt = tiff_clouds[i]->operator()(x,y);
        if (pcl_isnan (pcd_pt.x))
          EXPECT_TRUE (pcl_isnan (tiff_pt.x));
        else
          EXPECT_NEAR ( pcd_pt.x * (x-cx_new), tiff_pt.x * fx_multiplier * (x-cx_old), 1E-4);
        if (pcl_isnan (pcd_pt.y))
          EXPECT_TRUE (pcl_isnan (tiff_pt.y));
        else
          EXPECT_NEAR ( pcd_pt.y * (y-cy_new), tiff_pt.y * fy_multiplier * (y-cy_old), 1E-4);
        if (pcl_isnan (pcd_pt.z))
          EXPECT_TRUE (pcl_isnan (tiff_pt.z));
        else
          EXPECT_FLOAT_EQ (pcd_pt.z, tiff_pt.z);
      }
    }
  }

}

TEST (PCL, ImageGrabberSetIntrinsicsPCLZF)
{
  pcl::ImageGrabber<PointT> grabber (pclzf_dir_, 0, false, true);

  // Get all clouds from the grabber
  vector <CloudT::ConstPtr> pclzf_clouds;
  CloudT::ConstPtr cloud_buffer;
  bool signal_received = false;
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> 
    fxn = boost::bind (cloud_callback, &signal_received, &cloud_buffer, _1);
  grabber.registerCallback (fxn);
  grabber.start ();
  // Change the camera parameters
  double fx_multiplier = 1.2;
  double fy_multiplier = 0.5;
  double cx_multiplier = 0.8;
  double cy_multiplier = 1.3;
  double fx_old, fy_old, cx_old, cy_old;
  grabber.getCameraIntrinsics (fx_old, fy_old, cx_old, cy_old);
  double fx_new = fx_multiplier * fx_old;
  double fy_new = fy_multiplier * fy_old;
  double cx_new = cx_multiplier * cx_old;
  double cy_new = cy_multiplier * cy_old;
  grabber.setCameraIntrinsics (fx_new, fy_new, cx_new, cy_new);
  // Collect the clouds
  for (size_t i = 0; i < grabber.numFrames (); i++)
  {
    grabber.trigger ();
    size_t niter = 0;
    while (!signal_received)
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (10000));
      if (++niter > 100)
      {
        ASSERT_TRUE (false);
        return;
      }
    }
    pclzf_clouds.push_back (cloud_buffer);
    signal_received = false;
  }

  // Make sure they match
  EXPECT_EQ (pcds_.size (), pclzf_clouds.size ());
  for (size_t i = 0; i < pcds_.size (); i++)
  {
    EXPECT_EQ (pcds_[i]->width, pclzf_clouds[i]->width);
    EXPECT_EQ (pcds_[i]->height, pclzf_clouds[i]->height);
    for (int x = 0; x < pcds_[i]->width; x++)
    {
      for (int y = 0; y < pcds_[i]->height; y++)
      {
        const PointT &pcd_pt = pcds_[i]->operator()(x,y);
        const PointT &pclzf_pt = pclzf_clouds[i]->operator()(x,y);
        if (pcl_isnan (pcd_pt.x))
          EXPECT_TRUE (pcl_isnan (pclzf_pt.x));
        else
          EXPECT_NEAR ( pcd_pt.x * (x-cx_new), pclzf_pt.x * fx_multiplier * (x-cx_old), 1E-4);
        if (pcl_isnan (pcd_pt.y))
          EXPECT_TRUE (pcl_isnan (pclzf_pt.y));
        else
          EXPECT_NEAR ( pcd_pt.y * (y-cy_new), pclzf_pt.y * fy_multiplier * (y-cy_old), 1E-4);
        if (pcl_isnan (pcd_pt.z))
          EXPECT_TRUE (pcl_isnan (pclzf_pt.z));
        else
          EXPECT_FLOAT_EQ (pcd_pt.z, pclzf_pt.z);
      }
    }
  }
}

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test files were given. Please add the path to grabber_sequences to this test." << std::endl;
    return (-1);
  }

  std::string grabber_sequences = argv[1];
  tiff_dir_ = grabber_sequences + "/tiff";
  pclzf_dir_ = grabber_sequences + "/pclzf";
  pcd_dir_ = grabber_sequences + "/pcd";
  // Get pcd files
  vector<string> pcd_files;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (pcd_dir_); itr != end_itr; ++itr)
  {
#if BOOST_FILESYSTEM_VERSION == 3
    if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ())) == ".PCD" )
#else
    if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->leaf ())) == ".PCD" )
#endif
    {
#if BOOST_FILESYSTEM_VERSION == 3
      pcd_files.push_back (itr->path ().string ());
      std::cout << "added: " << itr->path ().string () << std::endl;
#else
      pcd_files.push_back (itr->pcd_dir_ ().string ());
      std::cout << "added: " << itr->pcd_dir_ () << std::endl;
#endif
    }
  }
  sort (pcd_files.begin (), pcd_files.end ());
  // And load them
  for (size_t i = 0; i < pcd_files.size (); i++)
  {
    CloudT::Ptr cloud (new CloudT);
    pcl::io::loadPCDFile (pcd_files[i], *cloud); 
    pcds_.push_back (cloud);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
