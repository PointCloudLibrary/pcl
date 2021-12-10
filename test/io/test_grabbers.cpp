#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/image_grabber.h>

#include <string>
#include <thread>
#include <vector>
#include <boost/filesystem.hpp> // for directory_iterator, extension
#include <boost/algorithm/string/case_conv.hpp> // for to_upper_copy

using namespace std::chrono_literals;

using PointT = pcl::PointXYZRGBA;
using CloudT = pcl::PointCloud<PointT>;

std::string tiff_dir_;
std::string pclzf_dir_;
std::string pcd_dir_;
std::vector<CloudT::ConstPtr> pcds_;
std::vector<std::string> pcd_files_;


// Helper function for grabbing a cloud
void
cloud_callback (bool& signal_received,
                CloudT::ConstPtr& ptr_to_fill,
                const CloudT::ConstPtr& input_cloud)
{
  signal_received = true;
  ptr_to_fill = input_cloud;
}

TEST (PCL, PCDGrabber)
{
  pcl::PCDGrabber<PointT> grabber (pcd_files_, 10, false); // TODO add directory functionality
  EXPECT_EQ (grabber.size (), pcds_.size ());
  std::vector<CloudT::ConstPtr> grabbed_clouds;
  std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>
    fxn = [&] (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& input_cloud) { grabbed_clouds.push_back (input_cloud); };
  grabber.registerCallback (fxn);
  grabber.start ();
  // 1 second should be /plenty/ of time
  std::this_thread::sleep_for(1s);
  grabber.stop ();

  //// Make sure they match
  ASSERT_EQ (pcds_.size (), grabbed_clouds.size ());
  for (std::size_t i = 0; i < pcds_.size (); i++)
  {
    // Also compare against FileGrabber-style loaded cloud
    CloudT::ConstPtr cloud_from_file_grabber = grabber[i];
    EXPECT_EQ (grabbed_clouds[i]->size (), pcds_[i]->size ());
    EXPECT_EQ (cloud_from_file_grabber->size (), pcds_[i]->size ());
    for (std::size_t j = 0; j < pcds_[i]->size (); j++)
    {
      const PointT &pcd_pt = pcds_[i]->at(j);
      const PointT &grabbed_pt = grabbed_clouds[i]->at(j);
      const PointT &fg_pt = cloud_from_file_grabber->at(j);
      if (std::isnan (pcd_pt.x))
      {
        EXPECT_TRUE (std::isnan (grabbed_pt.x));
        EXPECT_TRUE (std::isnan (fg_pt.x));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.x, grabbed_pt.x);
        EXPECT_FLOAT_EQ (pcd_pt.x, fg_pt.x);
      }
      if (std::isnan (pcd_pt.y))
      {
        EXPECT_TRUE (std::isnan (grabbed_pt.y));
        EXPECT_TRUE (std::isnan (fg_pt.y));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.y, grabbed_pt.y);
        EXPECT_FLOAT_EQ (pcd_pt.y, fg_pt.y);
      }
      if (std::isnan (pcd_pt.z))
      {
        EXPECT_TRUE (std::isnan (grabbed_pt.z));
        EXPECT_TRUE (std::isnan (fg_pt.z));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.z, grabbed_pt.z);
        EXPECT_FLOAT_EQ (pcd_pt.z, fg_pt.z);
      }
      EXPECT_EQ (pcd_pt.r, grabbed_pt.r);
      EXPECT_EQ (pcd_pt.g, grabbed_pt.g);
      EXPECT_EQ (pcd_pt.b, grabbed_pt.b);
      EXPECT_EQ (pcd_pt.a, grabbed_pt.a);
      EXPECT_EQ (pcd_pt.r, fg_pt.r);
      EXPECT_EQ (pcd_pt.g, fg_pt.g);
      EXPECT_EQ (pcd_pt.b, fg_pt.b);
      EXPECT_EQ (pcd_pt.a, fg_pt.a);
    }
  }

}

TEST (PCL, ImageGrabberTIFF)
{
  // Get all clouds from the grabber
  pcl::ImageGrabber<PointT> grabber (tiff_dir_, 0, false, false);
  std::vector<CloudT::ConstPtr> tiff_clouds;
  CloudT::ConstPtr cloud_buffer;
  bool signal_received = false;
  std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>
    fxn = [&] (const CloudT::ConstPtr& input_cloud) { cloud_callback (signal_received, cloud_buffer, input_cloud); };
  grabber.registerCallback (fxn);
  grabber.setCameraIntrinsics (525., 525., 320., 240.); // Setting old intrinsics which were used to generate these tests
  grabber.start ();
  for (std::size_t i = 0; i < grabber.size (); i++)
  {
    grabber.trigger ();
    std::size_t niter = 0;
    while (!signal_received)
    {
      std::this_thread::sleep_for(10ms);
      if (++niter > 100)
      {
        #ifdef PCL_BUILT_WITH_VTK
          FAIL ();
        #endif
        return;
      }
    }
    tiff_clouds.push_back (cloud_buffer);
    signal_received = false;
  }

  // Make sure they match
  EXPECT_EQ (pcds_.size (), tiff_clouds.size ());
  for (std::size_t i = 0; i < pcds_.size (); i++)
  {
    // Also compare against dynamically loaded cloud
    CloudT::ConstPtr cloud_from_file_grabber = grabber[i];
    for (std::size_t j = 0; j < pcds_[i]->size (); j++)
    {
      const PointT &pcd_pt = pcds_[i]->at(j);
      const PointT &tiff_pt = tiff_clouds[i]->at(j);
      const PointT &tiff_fg_pt = cloud_from_file_grabber->at(j);
      if (std::isnan (pcd_pt.x))
      {
        EXPECT_TRUE (std::isnan (tiff_pt.x));
        EXPECT_TRUE (std::isnan (tiff_fg_pt.x));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.x, tiff_pt.x);
        EXPECT_FLOAT_EQ (pcd_pt.x, tiff_fg_pt.x);
      }
      if (std::isnan (pcd_pt.y))
      {
        EXPECT_TRUE (std::isnan (tiff_pt.y));
        EXPECT_TRUE (std::isnan (tiff_fg_pt.y));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.y, tiff_pt.y);
        EXPECT_FLOAT_EQ (pcd_pt.y, tiff_fg_pt.y);
      }
      if (std::isnan (pcd_pt.z))
      {
        EXPECT_TRUE (std::isnan (tiff_pt.z));
        EXPECT_TRUE (std::isnan (tiff_fg_pt.z));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.z, tiff_pt.z);
        EXPECT_FLOAT_EQ (pcd_pt.z, tiff_fg_pt.z);
      }
      EXPECT_EQ (pcd_pt.r, tiff_pt.r);
      EXPECT_EQ (pcd_pt.g, tiff_pt.g);
      EXPECT_EQ (pcd_pt.b, tiff_pt.b);
      EXPECT_EQ (pcd_pt.r, tiff_fg_pt.r);
      EXPECT_EQ (pcd_pt.g, tiff_fg_pt.g);
      EXPECT_EQ (pcd_pt.b, tiff_fg_pt.b);
      //EXPECT_EQ (pcd_pt.a, tiff_pt.a);    // the alpha channel in the PCD set was saved as 255 but it should have been 0
    }
  }
}

TEST (PCL, ImageGrabberPCLZF)
{
  // Get all clouds from the grabber
  pcl::ImageGrabber<PointT> grabber (pclzf_dir_, 0, false, true);
  std::vector <CloudT::ConstPtr> pclzf_clouds;
  CloudT::ConstPtr cloud_buffer;
  bool signal_received = false;
  std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>
    fxn = [&] (const CloudT::ConstPtr& input_cloud) { cloud_callback (signal_received, cloud_buffer, input_cloud); };
  grabber.registerCallback (fxn);
  grabber.start ();
  for (std::size_t i = 0; i < grabber.size (); i++)
  {
    grabber.trigger ();
    std::size_t niter = 0;
    while (!signal_received)
    {
      std::this_thread::sleep_for(10ms);
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
  EXPECT_EQ (pcds_.size (), grabber.size ());
  for (std::size_t i = 0; i < pcds_.size (); i++)
  {
    CloudT::ConstPtr cloud_from_file_grabber = grabber[i];
    for (std::size_t j = 0; j < pcds_[i]->size (); j++)
    {
      const PointT &pcd_pt = pcds_[i]->at(j);
      const PointT &pclzf_pt = pclzf_clouds[i]->at(j);
      const PointT &pclzf_fg_pt = cloud_from_file_grabber->at(j);
      if (std::isnan (pcd_pt.x))
      {
        EXPECT_TRUE (std::isnan (pclzf_pt.x));
        EXPECT_TRUE (std::isnan (pclzf_fg_pt.x));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.x, pclzf_pt.x);
        EXPECT_FLOAT_EQ (pcd_pt.x, pclzf_fg_pt.x);
      }
      if (std::isnan (pcd_pt.y))
      {
        EXPECT_TRUE (std::isnan (pclzf_pt.y));
        EXPECT_TRUE (std::isnan (pclzf_fg_pt.y));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.y, pclzf_pt.y);
        EXPECT_FLOAT_EQ (pcd_pt.y, pclzf_fg_pt.y);
      }
      if (std::isnan (pcd_pt.z))
      {
        EXPECT_TRUE (std::isnan (pclzf_pt.z));
        EXPECT_TRUE (std::isnan (pclzf_fg_pt.z));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.z, pclzf_pt.z);
        EXPECT_FLOAT_EQ (pcd_pt.z, pclzf_fg_pt.z);
      }
      EXPECT_EQ (pcd_pt.r, pclzf_pt.r);
      EXPECT_EQ (pcd_pt.g, pclzf_pt.g);
      EXPECT_EQ (pcd_pt.b, pclzf_pt.b);
      EXPECT_EQ (pcd_pt.r, pclzf_fg_pt.r);
      EXPECT_EQ (pcd_pt.g, pclzf_fg_pt.g);
      EXPECT_EQ (pcd_pt.b, pclzf_fg_pt.b);
      //EXPECT_EQ (pcd_pt.a, pclzf_pt.a);     // the alpha channel in the PCD set was saved as 255 but it should have been 0
    }
  }
}

TEST (PCL, ImageGrabberOMP)
{
  // Get all clouds from the grabber
  pcl::ImageGrabber<PointT> grabber (pclzf_dir_, 0, false, true);
  std::vector <CloudT::ConstPtr> pclzf_clouds;
  CloudT::ConstPtr cloud_buffer;
  bool signal_received = false;
  std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>
    fxn = [&] (const CloudT::ConstPtr& input_cloud) { cloud_callback (signal_received, cloud_buffer, input_cloud); };
  grabber.registerCallback (fxn);
  grabber.setNumberOfThreads (0); // Let OMP select
  grabber.start ();
  for (std::size_t i = 0; i < grabber.size (); i++)
  {
    grabber.trigger ();
    std::size_t niter = 0;
    while (!signal_received)
    {
      std::this_thread::sleep_for(10ms);
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
  EXPECT_EQ (pcds_.size (), grabber.size ());
  for (std::size_t i = 0; i < pcds_.size (); i++)
  {
    CloudT::ConstPtr cloud_from_file_grabber = grabber[i];
    for (std::size_t j = 0; j < pcds_[i]->size (); j++)
    {
      const PointT &pcd_pt = pcds_[i]->at(j);
      const PointT &pclzf_pt = pclzf_clouds[i]->at(j);
      const PointT &pclzf_fg_pt = cloud_from_file_grabber->at(j);
      if (std::isnan (pcd_pt.x))
      {
        EXPECT_TRUE (std::isnan (pclzf_pt.x));
        EXPECT_TRUE (std::isnan (pclzf_fg_pt.x));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.x, pclzf_pt.x);
        EXPECT_FLOAT_EQ (pcd_pt.x, pclzf_fg_pt.x);
      }
      if (std::isnan (pcd_pt.y))
      {
        EXPECT_TRUE (std::isnan (pclzf_pt.y));
        EXPECT_TRUE (std::isnan (pclzf_fg_pt.y));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.y, pclzf_pt.y);
        EXPECT_FLOAT_EQ (pcd_pt.y, pclzf_fg_pt.y);
      }
      if (std::isnan (pcd_pt.z))
      {
        EXPECT_TRUE (std::isnan (pclzf_pt.z));
        EXPECT_TRUE (std::isnan (pclzf_fg_pt.z));
      }
      else
      {
        EXPECT_FLOAT_EQ (pcd_pt.z, pclzf_pt.z);
        EXPECT_FLOAT_EQ (pcd_pt.z, pclzf_fg_pt.z);
      }
      EXPECT_EQ (pcd_pt.r, pclzf_pt.r);
      EXPECT_EQ (pcd_pt.g, pclzf_pt.g);
      EXPECT_EQ (pcd_pt.b, pclzf_pt.b);
      EXPECT_EQ (pcd_pt.r, pclzf_fg_pt.r);
      EXPECT_EQ (pcd_pt.g, pclzf_fg_pt.g);
      EXPECT_EQ (pcd_pt.b, pclzf_fg_pt.b);
      //EXPECT_EQ (pcd_pt.a, pclzf_pt.a);     // the alpha channel in the PCD set was saved as 255 but it should have been 0
    }
  }
}

TEST (PCL, ImageGrabberTimestamps)
{
  // Initialize the grabber but don't load
  pcl::ImageGrabber<PointT> grabber (pclzf_dir_, 0, false, true);
  std::uint64_t frame0_microsec, frame1_microsec; 
  ASSERT_EQ (grabber.size (), 3);
  EXPECT_TRUE (grabber.getTimestampAtIndex (0, frame0_microsec));
  EXPECT_TRUE (grabber.getTimestampAtIndex (1, frame1_microsec));
  std::uint64_t timediff = frame1_microsec - frame0_microsec;
  EXPECT_EQ (timediff, 254471); // 20121214T142256.068683 - 20121214T142255.814212 
}

TEST (PCL, ImageGrabberSetIntrinsicsTIFF)
{
  pcl::ImageGrabber<PointT> grabber (tiff_dir_, 0, false, false);

  // Get all clouds from the grabber
  std::vector <CloudT::ConstPtr> tiff_clouds;
  CloudT::ConstPtr cloud_buffer;
  bool signal_received = false;
  std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>
    fxn = [&] (const CloudT::ConstPtr& input_cloud) { cloud_callback (signal_received, cloud_buffer, input_cloud); };
  grabber.registerCallback (fxn);
  grabber.start ();
  // Change the camera parameters
  double fx_multiplier = 1.2;
  double fy_multiplier = 0.5;
  double cx_multiplier = 0.8;
  double cy_multiplier = 1.3;
  double fx_old, fy_old, cx_old, cy_old;
  //grabber.getCameraIntrinsics (fx_old, fy_old, cx_old, cy_old); Need to use old intrinsics, can't trust defaults
  fx_old = 525;
  fy_old = 525;
  cx_old = 320;
  cy_old = 240;
  double fx_new = fx_multiplier * fx_old;
  double fy_new = fy_multiplier * fy_old;
  double cx_new = cx_multiplier * cx_old;
  double cy_new = cy_multiplier * cy_old;
  grabber.setCameraIntrinsics (fx_new, fy_new, cx_new, cy_new);
  // Collect the clouds
  for (std::size_t i = 0; i < grabber.size (); i++)
  {
    grabber.trigger ();
    std::size_t niter = 0;
    while (!signal_received)
    {
      std::this_thread::sleep_for(10ms);
      if (++niter > 100)
      {
        #ifdef PCL_BUILT_WITH_VTK
          FAIL ();
        #endif
        return;
      }
    }
    tiff_clouds.push_back (cloud_buffer);
    signal_received = false;
  }

  // Make sure they match
  EXPECT_EQ (pcds_.size (), tiff_clouds.size ());
  for (std::size_t i = 0; i < pcds_.size (); i++)
  {
    EXPECT_EQ (pcds_[i]->width, tiff_clouds[i]->width);
    EXPECT_EQ (pcds_[i]->height, tiff_clouds[i]->height);
    for (std::uint32_t x = 0; x < pcds_[i]->width; x++)
    {
      for (std::uint32_t y = 0; y < pcds_[i]->height; y++)
      {
        const PointT &pcd_pt = pcds_[i]->operator()(x,y);
        const PointT &tiff_pt = tiff_clouds[i]->operator()(x,y);
        if (std::isnan (pcd_pt.x))
          EXPECT_TRUE (std::isnan (tiff_pt.x));
        else
          EXPECT_NEAR ( pcd_pt.x * (x-cx_new), tiff_pt.x * fx_multiplier * (x-cx_old), 1E-4);
        if (std::isnan (pcd_pt.y))
          EXPECT_TRUE (std::isnan (tiff_pt.y));
        else
          EXPECT_NEAR ( pcd_pt.y * (y-cy_new), tiff_pt.y * fy_multiplier * (y-cy_old), 1E-4);
        if (std::isnan (pcd_pt.z))
          EXPECT_TRUE (std::isnan (tiff_pt.z));
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
  std::vector <CloudT::ConstPtr> pclzf_clouds;
  CloudT::ConstPtr cloud_buffer;
  bool signal_received = false;
  std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>
    fxn = [&] (const CloudT::ConstPtr& input_cloud) { cloud_callback (signal_received, cloud_buffer, input_cloud); };
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
  for (std::size_t i = 0; i < grabber.size (); i++)
  {
    grabber.trigger ();
    std::size_t niter = 0;
    while (!signal_received)
    {
      std::this_thread::sleep_for(10ms);
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
  for (std::size_t i = 0; i < pcds_.size (); i++)
  {
    EXPECT_EQ (pcds_[i]->width, pclzf_clouds[i]->width);
    EXPECT_EQ (pcds_[i]->height, pclzf_clouds[i]->height);
    for (std::uint32_t x = 0; x < pcds_[i]->width; x++)
    {
      for (std::uint32_t y = 0; y < pcds_[i]->height; y++)
      {
        const PointT &pcd_pt = pcds_[i]->operator()(x,y);
        const PointT &pclzf_pt = pclzf_clouds[i]->operator()(x,y);
        if (std::isnan (pcd_pt.x))
          EXPECT_TRUE (std::isnan (pclzf_pt.x));
        else
          EXPECT_NEAR ( pcd_pt.x * (x-cx_new), pclzf_pt.x * fx_multiplier * (x-cx_old), 1E-4);
        if (std::isnan (pcd_pt.y))
          EXPECT_TRUE (std::isnan (pclzf_pt.y));
        else
          EXPECT_NEAR ( pcd_pt.y * (y-cy_new), pclzf_pt.y * fy_multiplier * (y-cy_old), 1E-4);
        if (std::isnan (pcd_pt.z))
          EXPECT_TRUE (std::isnan (pclzf_pt.z));
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
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (pcd_dir_); itr != end_itr; ++itr)
  {
    if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ())) == ".PCD" )
    {
      pcd_files_.push_back (itr->path ().string ());
      std::cout << "added: " << itr->path ().string () << std::endl;
    }
  }
  sort (pcd_files_.begin (), pcd_files_.end ());
  // And load them
  for (const auto &pcd_file : pcd_files_)
  {
    CloudT::Ptr cloud (new CloudT);
    pcl::io::loadPCDFile (pcd_file, *cloud); 
    pcds_.emplace_back(cloud);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
