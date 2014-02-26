/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *  Author: Bernhard Zeisl, (myname.mysurname@inf.ethz.ch)
 */


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/containers/initialization.h>
#include "openni_capture.h"

#include "tsdf_volume.h"
#include "tsdf_volume.hpp"

using namespace std;
namespace pc = pcl::console;

typedef pcl::PointXYZ    PointT;
typedef float            VoxelT;
typedef short            WeightT;

string cloud_file  = "cloud.pcd";
string volume_file = "tsdf_volume.dat";

double min_trunc_dist = 30.0f;

bool quit = false, save = false;
bool extract_cloud_volume = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////
/// DEVICE_VOLUME

/** \brief Class for storing and handling the TSDF Volume on the GPU */
class DeviceVolume
{
public:

  typedef boost::shared_ptr<DeviceVolume>       Ptr;
  typedef boost::shared_ptr<const DeviceVolume> ConstPtr;

  /** \brief Constructor
   * param[in] volume_size size of the volume in mm
   * param[in] volume_res volume grid resolution (typically device::VOLUME_X x device::VOLUME_Y x device::VOLUME_Z)
   */
  DeviceVolume (const Eigen::Vector3f &volume_size, const Eigen::Vector3i &volume_res)
    : volume_size_ (volume_size)
  {
    // initialize GPU
    device_volume_.create (volume_res[1] * volume_res[2], volume_res[0]); // (device::VOLUME_Y * device::VOLUME_Z, device::VOLUME_X)
    pcl::device::initVolume (device_volume_);

    // truncation distance
    Eigen::Vector3f voxel_size = volume_size.array() / volume_res.array().cast<float>();
    trunc_dist_ = max ((float)min_trunc_dist, 2.1f * max (voxel_size[0], max (voxel_size[1], voxel_size[2])));
  };

  /** \brief Creates the TSDF volume on the GPU
   * param[in] depth depth readings from the sensor
   * param[in] intr camaera intrinsics
   */
  void
  createFromDepth (const pcl::device::PtrStepSz<const unsigned short> &depth, const pcl::device::Intr &intr);

  /** \brief Downloads the volume from the GPU
   * param[out] volume volume structure where the data is written to (size needs to be appropriately set beforehand (is checked))
   */
  bool
  getVolume (pcl::TSDFVolume<VoxelT, WeightT>::Ptr &volume);

  /** \brief Generates and returns a point cloud form the implicit surface in the TSDF volume
   * param[out] cloud point cloud containing the surface
   */
  bool
  getCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

private:

  template<class D, class Matx> D&
  device_cast (Matx& matx)
  {
    return (*reinterpret_cast<D*>(matx.data ()));
  };

  pcl::gpu::DeviceArray2D<int> device_volume_;

  Eigen::Vector3f volume_size_;

  float trunc_dist_;

};


void
DeviceVolume::createFromDepth (const pcl::device::PtrStepSz<const unsigned short> &depth, const pcl::device::Intr &intr)
{
  using namespace pcl;

  typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;

  const int rows = 480;
  const int cols = 640;

  // scale depth values
  gpu::DeviceArray2D<float> device_depth_scaled;
  device_depth_scaled.create (rows, cols);

  // upload depth map on GPU
  pcl::gpu::KinfuTracker::DepthMap device_depth;
  device_depth.upload (depth.data, depth.step, depth.rows, depth.cols);

  // initial camera rotation and translation
  Matrix3frm      init_Rcam = Eigen::Matrix3f::Identity ();
  Eigen::Vector3f init_tcam = volume_size_ * 0.5f - Eigen::Vector3f (0, 0, volume_size_(2)/2 * 1.2f);

  Matrix3frm init_Rcam_inv = init_Rcam.inverse ();
  device::Mat33&  device_Rcam_inv = device_cast<device::Mat33> (init_Rcam_inv);
  float3& device_tcam = device_cast<float3> (init_tcam);

  // integrate depth values into volume
  float3 device_volume_size = device_cast<float3> (volume_size_);
  device::integrateTsdfVolume (device_depth, intr, device_volume_size, device_Rcam_inv, device_tcam, trunc_dist_,
                               device_volume_, device_depth_scaled);
}


bool
DeviceVolume::getVolume (pcl::TSDFVolume<VoxelT, WeightT>::Ptr &volume)
{
  int volume_size = device_volume_.rows() * device_volume_.cols();

  if ((size_t)volume_size != volume->size())
  {
    pc::print_error ("Device volume size (%d) and tsdf volume size (%d) don't match. ABORTING!\n", volume_size, volume->size());
    return false;
  }

  vector<VoxelT>&  volume_vec  = volume->volumeWriteable();
  vector<WeightT>& weights_vec = volume->weightsWriteable();

  device_volume_.download (&volume_vec[0], device_volume_.cols() * sizeof(int));

  #pragma omp parallel for
  for(int i = 0; i < (int) volume->size(); ++i)
  {
    short2 *elem = (short2*)&volume_vec[i];
    volume_vec[i]  = (float)(elem->x)/pcl::device::DIVISOR;
    weights_vec[i] = (short)(elem->y);
  }

  return true;
}


bool
DeviceVolume::getCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  const int DEFAULT_VOLUME_CLOUD_BUFFER_SIZE = 10 * 1000 * 1000;

  // point buffer on the device
  pcl::gpu::DeviceArray<pcl::PointXYZ> device_cloud_buffer (DEFAULT_VOLUME_CLOUD_BUFFER_SIZE);

  // do the extraction
  float3 device_volume_size = device_cast<float3> (volume_size_);
  /*size_t size =*/ pcl::device::extractCloud (device_volume_, device_volume_size, device_cloud_buffer);

  // write into point cloud structure
  device_cloud_buffer.download (cloud->points);
  cloud->width = (int)cloud->points.size ();
  cloud->height = 1;

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/// HELPER FUNCTIONS

/** \brief Converts depth and RGB sensor readings into a point cloud
 * param[in] depth depth data from sensor
 * param[in] rgb24 color data from sensor
 * param[in] intr camera intrinsics
 * param[out] cloud the generated point cloud
 * \note: position in mm is converted to m
 * \note: RGB reading not working!
 */
//TODO implement correct color reading (how does rgb24 look like?)
bool
convertDepthRGBToCloud (const pcl::device::PtrStepSz<const unsigned short> &depth, const pcl::device::PtrStepSz<const pcl::gpu::KinfuTracker::PixelRGB> &rgb24, const pcl::device::Intr &intr,
                pcl::PointCloud<PointT>::Ptr &cloud)
{
  // resize point cloud if it doesn't fit
  if (depth.rows != (int)cloud->height || depth.cols != (int)cloud->width)
    cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT> (depth.cols, depth.rows));

  // std::cout << "step = " << rgb24.step << std::endl;
  // std::cout << "elem size = " << rgb24.elem_size << std::endl;

  // iterate over all depth and rgb values
  for (int y = 0; y < depth.rows; ++y)
  {
    // get pointers to the values in one row
    const unsigned short *depth_row_ptr = depth.ptr(y);
    // const pcl::gpu::KinfuTracker::RGB *rgb24_row_ptr = rgb24.ptr(y);
    // const char* rgb24_row_ptr = (const char*) rgb24.ptr(y);

    // iterate over row and store values
    for (int x = 0; x < depth.cols; ++x)
    {
      float u = (x - intr.cx) / intr.fx;
      float v = (y - intr.cy) / intr.fy;

      PointT &point = cloud->at (x, y);

      point.z = depth_row_ptr[x] / 1000.0f;
      point.x = u * point.z;
      point.y = v * point.z;

/*      uint8_t r = *(rgb24_row_ptr + 0);
      uint8_t g = *(rgb24_row_ptr + 1);
      uint8_t b = *(rgb24_row_ptr + 2);
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      point.rgb = *reinterpret_cast<float*>(&rgb);

      point.r = *((const char*)rgb24.data + y*rgb24.step + x*rgb24.elem_size);
      point.g = *((const char*)rgb24.data + y*rgb24.step + x*rgb24.elem_size + 1);
      point.b = *((const char*)rgb24.data + y*rgb24.step + x*rgb24.elem_size + 2);
*/
    }
  }

  cloud->is_dense = false;

  return true;
}

/** \brief Captures data from a sensor and generates a point cloud from it
 * param[in] capture capturing device object
 * param[out] depth the depth reading
 * param[out] rgb24 the color reading
 * param[out] intr camera intrinsics for this reading
 * param[out] cloud point cloud generated from the readings
 */
bool
captureCloud (pcl::gpu::CaptureOpenNI &capture,
              pcl::device::PtrStepSz<const unsigned short> &depth, pcl::device::PtrStepSz<const pcl::gpu::KinfuTracker::PixelRGB> &rgb24,
              pcl::device::Intr &intr, pcl::PointCloud<PointT>::Ptr &cloud)
{
  // capture frame
  if (!capture.grab (depth, rgb24))
  {
    pc::print_error ("Can't capture via sensor.\n");
    return false;
  }

  // get intrinsics from capture
  float f = capture.depth_focal_length_VGA;
  intr = pcl::device::Intr (f, f, depth.cols/2, depth.rows/2);

  // generate point cloud
  cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT> (depth.cols, depth.rows));
  if (!convertDepthRGBToCloud (depth, rgb24, intr, cloud))
  {
    pc::print_error ("Conversion depth --> cloud was not successful!\n");
    return false;
  }

  return true;
}

/** \brief callback function for the PCLvisualizer */
void
keyboard_callback (const pcl::visualization::KeyboardEvent &event, void *cookie)
{
  if (event.keyDown())
  {
    switch (event.getKeyCode())
    {
      case 27:
      case (int)'q': case (int)'Q':
      case (int)'e': case (int)'E':
        cout << "Exiting program" << endl;
        quit = true;
        break;
      case (int)'s': case (int)'S':
        cout << "Saving volume and cloud" << endl;
        save = true;
        break;
      default:
        break;
    }
  }
}

/** \brief prints usage information for the executable */
void
printUsage (char* argv[])
{
  pc::print_error ("usage: %s <options>\n\n", argv[0]);

  pc::print_info  ("  where options are:\n");
  pc::print_info  ("                    -cf = cloud filename (default: ");
  pc::print_value ("%s)", cloud_file.c_str()); pc::print_info (")\n");
  pc::print_info  ("                    -vf = volume filename (default: ");
  pc::print_value ("%s", volume_file.c_str()); pc::print_info (")\n");
  pc::print_info  ("                    -ec = extract cloud from generated volume (default: ");
  pc::print_value ("0 (false)", volume_file.c_str()); pc::print_info (")\n");
  pc::print_info  ("                    -td = minimal truncation distance (default: ");
  pc::print_value ("%f", min_trunc_dist); pc::print_info (")\n");
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/// MAIN

/** \brief main loop for the program */
int
main (int argc, char* argv[])
{
  pc::print_info ("Records a 2.5 point cloud (organized = depth map) as TSDF volume. For more information, use: %s -h\n", argv[0]);

  /***
   * PARSE COMMAND LINE
   */

  // check for help
  if (pc::find_argument (argc, argv, "-h") > 0)
  {
    pc::print_warn ("Showing help: \n\n");
    printUsage (argv);
    return (EXIT_SUCCESS);
  }

  // parse input cloud file
  pc::parse_argument (argc, argv, "-cf", cloud_file);

  // pase output volume file
  pc::parse_argument (argc, argv, "-vf", volume_file);

  // parse options to extract and save cloud from volume
  pc::parse_argument (argc, argv, "-ec", extract_cloud_volume);

  // parse minimual truncation distance
  pc::parse_argument (argc, argv, "-td", min_trunc_dist);

  /***
   * SET UP AND VISUALIZATION
   */

  pc::print_info (" -------------------- START OF ALGORITHM --------------------\n");

  pcl::gpu::setDevice (0);
  pcl::gpu::printShortCudaDeviceInfo (0);

  pcl::gpu::CaptureOpenNI capture (0);  // first OpenNI device;
  pcl::device::PtrStepSz<const unsigned short> depth;
  pcl::device::PtrStepSz<const pcl::gpu::KinfuTracker::PixelRGB> rgb24;

  pcl::PointCloud<PointT>::Ptr cloud; // (new pcl::PointCloud<PointT>);
  pcl::device::Intr intr;

  // capture first frame
  if (!captureCloud (capture, depth, rgb24, intr, cloud))
    return EXIT_FAILURE;

  // start visualizer
  pcl::visualization::PCLVisualizer visualizer;
  // pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_handler (cloud);
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (cloud, 0.5, 0.5, 0.5);
  visualizer.addPointCloud<PointT> (cloud); //, color_handler, "cloud");
  visualizer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
  visualizer.addCoordinateSystem (1);
  visualizer.initCameraParameters();
  visualizer.registerKeyboardCallback (keyboard_callback);
  visualizer.spinOnce();

  /***
   * CAPTURING DATA AND GENERATING CLOUD
   */

  pc::print_highlight ("Capturing data ... \n");

  while (!quit && !save)
  {
    // capture data and convert to point cloud
    if (!captureCloud (capture, depth, rgb24, intr, cloud))
      return EXIT_FAILURE;

    // update visualization
    visualizer.updatePointCloud<PointT> (cloud); //, color_handler, "cloud");
    visualizer.spinOnce();
  }

  if (quit)
    return (EXIT_SUCCESS);


  /***
   * GENERATE VOLUME
   */

  // create volume object
  pcl::TSDFVolume<VoxelT, WeightT>::Ptr volume (new pcl::TSDFVolume<VoxelT, WeightT>);
  Eigen::Vector3i resolution (pcl::device::VOLUME_X, pcl::device::VOLUME_Y, pcl::device::VOLUME_Z);
  Eigen::Vector3f volume_size = Eigen::Vector3f::Constant (3000);
  volume->resize (resolution, volume_size);

  DeviceVolume::Ptr device_volume (new DeviceVolume (volume->volumeSize(), volume->gridResolution()));


  // integrate depth in device volume
  pc::print_highlight ("Converting depth map to volume ... "); cout << flush;
  device_volume->createFromDepth (depth, intr);

  // get volume from device
  if (!device_volume->getVolume (volume))
  {
    pc::print_error ("Coudln't get volume from device!\n");
    return (EXIT_FAILURE);
  }
  pc::print_info ("done [%d voxels]\n", volume->size());


  // generating TSDF cloud
  pc::print_highlight ("Generating tsdf volume cloud ... "); cout << flush;
  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  volume->convertToTsdfCloud (tsdf_cloud);
  pc::print_info ("done [%d points]\n", tsdf_cloud->size());


  // get cloud from volume
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_volume (new pcl::PointCloud<pcl::PointXYZ>);
  if (extract_cloud_volume)
  {
    pc::print_highlight ("Generating cloud from volume ... "); cout << flush;
    if (!device_volume->getCloud (cloud_volume))
    {
      pc::print_error ("Cloudn't get cloud from device volume!\n");
      return (EXIT_FAILURE);
    }
    pc::print_info ("done [%d points]\n", cloud_volume->size());
  }


  /***
   * STORE RESULTS
   */

  pc::print_highlight ("Storing results:\n");

  // point cloud
  pc::print_info ("Saving captured cloud to "); pc::print_value ("%s", cloud_file.c_str()); pc::print_info (" ... ");
  if (pcl::io::savePCDFile (cloud_file, *cloud, true) < 0)
  {
    cout << endl;
    pc::print_error ("Cloudn't save the point cloud to file %s.\n", cloud_file.c_str());
  }
  else
    pc::print_info ("done [%d points].\n", cloud->size());

  // volume
  if (!volume->save (volume_file, true))
    pc::print_error ("Cloudn't save the volume to file %s.\n", volume_file.c_str());

  // TSDF point cloud
  string tsdf_cloud_file (pcl::getFilenameWithoutExtension(volume_file) + "_cloud.pcd");
  pc::print_info ("Saving volume cloud to "); pc::print_value ("%s", tsdf_cloud_file.c_str()); pc::print_info (" ... ");
  if (pcl::io::savePCDFile (tsdf_cloud_file, *tsdf_cloud, true) < 0)
  {
    cout << endl;
    pc::print_error ("Cloudn't save the volume point cloud to file %s.\n", tsdf_cloud_file.c_str());
  }
  else
    pc::print_info ("done [%d points].\n", tsdf_cloud->size());

  // point cloud from volume
  if (extract_cloud_volume)
  {
    string cloud_volume_file (pcl::getFilenameWithoutExtension(cloud_file) + "_from_volume.pcd");
    pc::print_info ("Saving cloud from volume to "); pc::print_value ("%s", cloud_volume_file.c_str()); pc::print_info (" ... ");
    if (pcl::io::savePCDFile (cloud_volume_file, *cloud_volume, true) < 0)
    {
      cout << endl;
      pc::print_error ("Cloudn't save the point cloud to file %s.\n", cloud_volume_file.c_str());
    }
    else
      pc::print_info ("done [%d points].\n", cloud_volume->size());
  }

  pc::print_info (" --------------------  END OF ALGORITHM  --------------------\n");

}
