/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 */

#include <pcl/apps/dem_ground_segmentation.h>

#include <cstdlib>
#include <ctime>

#include <string>
#include <sstream>
#include <vector>
#include <utility>
#include <numeric>
#include <algorithm>
#include <fstream>
#include <limits>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/eigen.h>
#include <pcl/stereo/digital_elevation_map.h>
#include <pcl/io/pcd_io.h>

/** \brief Class for testing the resulting algorithm of the second Honda Research Institute code sprint. */
class HRCSRoadSegmentation
{
public:
  HRCSRoadSegmentation (const std::string &left_image_path,
      const std::string &disparity_path, const std::string &positions_file_path,
      const std::string &labeling_path);
  ~HRCSRoadSegmentation () {}

  void 
  run ();

private:
  bool
  loadDir (const std::string &dir, std::vector<std::string> &container);

  bool
  loadPositionFile (const std::string &positions_file_path);

  bool
  loadImage (const std::string name, pcl::PointCloud<pcl::RGB>::Ptr &cloud); 

  void
  computeDEM (const std::string &left_image_name, const std::string &disparity,
      pcl::PointCloud<pcl::PointDEM>::Ptr &dem);

  bool
  loadDisparityMap (
      const std::string &file_name, std::vector<float> &disparity_map);

  void
  saveLabeling (const pcl::PointCloud<pcl::Label>::Ptr &labels,
      const pcl::PointCloud<pcl::PointDEM>::Ptr &dem,
      const std::vector<double> &left_curb, 
      const std::vector<double> &right_curb, 
      const std::vector<double> &road,
      const size_t frame_idx);

  const float DISPARITY_MIN;
  const float DISPARITY_MAX;
  const size_t DEM_RESOLUTION_COLUMN;
  const size_t DEM_RESOLUTION_DISPARITY;
  const float CENTER_X;
  const float CENTER_Y;
  const float FOCAL_LENGTH;
  const float BASELINE;
  const float HEIGHT_THRESHOLD;

  // Minimum number of the available data.
  size_t size_;
  std::vector<std::string> left_images_;
  std::vector<std::string> disparity_maps_;

  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > positions_;

  std::string labeling_path_;

  bool is_consistent_;

  // Image size.
  float width_;
  float height_;
};

int main (int argc, char *argv[])
{
  // Parse the command line.
  namespace po = boost::program_options;

  std::string left_image_path;
  std::string disparity_path;
  std::string positions_file_path;
  std::string labeling_path;

  po::options_description description ("Options");
  description.add_options ()
      ("help,h", "print this help")
      ("labeling,g", po::value<std::string> (& labeling_path), 
          "Path to the folder with the resulting labeling.")
      ("left,l", po::value<std::string> (&left_image_path), 
          "Path to the folder with the left images.")
      ("disparity,d", po::value<std::string> (&disparity_path), 
          "Path to the folder with the disparity maps.")
      ("position,p", po::value<std::string> (&positions_file_path), 
          "Path to the file with the position data.")
  ;

  try
  {
    po::variables_map map;
    po::store (po::command_line_parser (argc, argv).
        options (description).run (), map);
    po::notify(map);

    if (map.count ("help")) 
    {
      std::cout << description << std::endl;
      return 0;
    }
  }
  catch (...)
  {
    std::cout << "Error during parsing the parameters!" << std::endl;
    std::cout << description << std::endl;
    return 0;
  }

  // Run the segmentation.
  HRCSRoadSegmentation alg (left_image_path, disparity_path, positions_file_path, labeling_path);
  alg.run ();

  return (0);
}

HRCSRoadSegmentation::HRCSRoadSegmentation(const std::string &left_image_path,
    const std::string &disparity_path, const std::string &positions_file_path,
    const std::string &labeling_path) : 
    DISPARITY_MIN (15.0f),
    DISPARITY_MAX (80.0f),
    DEM_RESOLUTION_COLUMN (64),
    DEM_RESOLUTION_DISPARITY (32),
    CENTER_X (318.112200f),
    CENTER_Y (224.334900f),
    FOCAL_LENGTH (368.534700f),
    BASELINE (0.8387445f),
    HEIGHT_THRESHOLD (0.05f),
    size_ (0),
    labeling_path_ (labeling_path),
    is_consistent_ (true)
{

  // Load the left images's pathes.
  is_consistent_ = loadDir (left_image_path, left_images_);
  if (!is_consistent_) return;

  // Load the disparity's pathes.
  is_consistent_ = loadDir (disparity_path, disparity_maps_);
  if (!is_consistent_) return;

  size_ = std::min (disparity_maps_.size (), left_images_.size ());
 
  // Load the position file.
  is_consistent_ = loadPositionFile (positions_file_path);
  if (!is_consistent_) return;

  size_ = std::min (size_, positions_.size ());
}

void
HRCSRoadSegmentation::run ()
{
  // Check, if class is consistent.
  if (!is_consistent_)
  {
    return;
  }
  
  // Init the data.
  pcl::PointCloud<pcl::PointDEM>::Ptr dem (new pcl::PointCloud<pcl::PointDEM>);
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  Eigen::Matrix4f transformation;
  Eigen::Matrix4f previous_transformation;
  previous_transformation.setIdentity ();

  // Init the segmentator.
  pcl::DEMGroundSegmentation segmentator;

  // For each frame.
  for (size_t frame_idx = 0; frame_idx < size_; ++frame_idx)
  {
    computeDEM (left_images_ [frame_idx], disparity_maps_ [frame_idx], dem);

    segmentator.setInputCloud (dem);

    // Compute transformation matrix from this frame to the previous.
    Eigen::Vector3f rotation = positions_[frame_idx].second;
    Eigen::Vector3f translation = positions_[frame_idx].first;

    // Transformation from the starting point.
    transformation.block (0, 0, 3, 3) = (Eigen::Matrix3f)
        Eigen::AngleAxisf(rotation (0), Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(rotation (1), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(rotation (2), Eigen::Vector3f::UnitZ());
    transformation.block (0, 3, 3, 1) = translation;
    transformation (3, 0) = 0.0f;
    transformation (3, 1) = 0.0f;
    transformation (3, 2) = 0.0f;
    transformation (3, 3) = 1.0f;

    segmentator.setTransformation (previous_transformation * transformation.inverse ());

    previous_transformation = transformation;

    std::vector<double> left_curb, right_curb, road;

    segmentator.segment (*labels, left_curb, right_curb, road);
    
    // Convert labeling from per DEM cell to per pixel and save it.
    saveLabeling (labels, dem, left_curb, right_curb, road, frame_idx);
  } // frame_idx
}

bool
HRCSRoadSegmentation::loadDir (const std::string &dir, std::vector<std::string> &container)
{
  boost::filesystem::directory_iterator end_itr;

  if (!boost::filesystem::is_directory(dir))
  {
    std::cout << "Is not a directory: " << dir << std::endl;
    return false;
  }
  for (boost::filesystem::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    container.push_back (itr->path ().string ());
  }
  sort (container.begin (), container.end ());

  return true;
}

bool 
HRCSRoadSegmentation::loadPositionFile (const std::string &positions_file_path)
{
  std::fstream positions_file;

  // Open the disparity file
  positions_file.open (positions_file_path.c_str(), std::fstream::in);
  if (!positions_file.is_open ())
  {
    std::cout << "Can't load the file: " << positions_file_path << std::endl;
    return false;
  }

  while (!positions_file.eof () && !positions_file.fail ())
  {
    std::string input_string;
    std::stringstream input_ss;
    std::getline (positions_file, input_string);

    if (input_string.size () > 0 && input_string[0] != '#')
    {
      input_ss.str (input_string);
      size_t image_number;
      input_ss >> image_number;

      std::pair<Eigen::Vector3f, Eigen::Vector3f> position;

      // Load a translation vector.
      input_ss >> position.first(0);
      input_ss >> position.first(1);
      input_ss >> position.first(2);
      // Load a rotation vector.
      input_ss >> position.second(0);
      input_ss >> position.second(1);
      input_ss >> position.second(2);

      positions_.push_back (position);
    }
  }

  return true;
}

bool
HRCSRoadSegmentation::loadImage (const std::string name, pcl::PointCloud<pcl::RGB>::Ptr &cloud)
{
  // This function was copied from tools/png2pcd.cpp
  vtkSmartPointer<vtkImageData> image_data;
  vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New ();
  reader->SetFileName (name.c_str());
  image_data = reader->GetOutput ();
  image_data->Update ();

  int components = image_data->GetNumberOfScalarComponents();

  double* pixel = new double [4];
  memset (pixel, 0, sizeof(double) * 4);

  int dimensions[3];
  image_data->GetDimensions (dimensions);
  cloud->width = dimensions[0];
  cloud->height = dimensions[1]; // This indicates that the point cloud is organized

  width_ = dimensions[0];
  height_ = dimensions[1];

  cloud->is_dense = true;
  cloud->points.resize (cloud->width * cloud->height);

  for (int z = 0; z < dimensions[2]; z++)
  {
    for (int y = 0; y < dimensions[1]; y++)
    {
      for (int x = 0; x < dimensions[0]; x++)
      {
        for (int c = 0; c < components; c++)
          pixel[c] = image_data->GetScalarComponentAsDouble(x, y, 0, c);

        pcl::RGB color;
        color.r = 0;
        color.g = 0;
        color.b = 0;
        color.a = 0;
        color.rgb = 0.0f;
        color.rgba = 0;

        int rgb;
        int rgba;
        switch (components)
        {
        case 1:  color.r = static_cast<uint8_t> (pixel[0]);
          color.g = static_cast<uint8_t> (pixel[0]);
          color.b = static_cast<uint8_t> (pixel[0]);

          rgb = (static_cast<int> (color.r)) << 16 |
            (static_cast<int> (color.g)) << 8 |
            (static_cast<int> (color.b));

          rgba = rgb;
          color.rgb = static_cast<float> (rgb);
          color.rgba = static_cast<uint32_t> (rgba);
          break;

        case 3:  color.r = static_cast<uint8_t> (pixel[0]);
          color.g = static_cast<uint8_t> (pixel[1]);
          color.b = static_cast<uint8_t> (pixel[2]);

          rgb = (static_cast<int> (color.r)) << 16 |
            (static_cast<int> (color.g)) << 8 |
            (static_cast<int> (color.b));

          rgba = rgb;
          color.rgb = static_cast<float> (rgb);
          color.rgba = static_cast<uint32_t> (rgba);
          break;

        case 4:  color.r = static_cast<uint8_t> (pixel[0]);
          color.g = static_cast<uint8_t> (pixel[1]);
          color.b = static_cast<uint8_t> (pixel[2]);
          color.a = static_cast<uint8_t> (pixel[3]);

          rgb = (static_cast<int> (color.r)) << 16 |
            (static_cast<int> (color.g)) << 8 |
            (static_cast<int> (color.b));
          rgba = (static_cast<int> (color.a)) << 24 |
            (static_cast<int> (color.r)) << 16 |
            (static_cast<int> (color.g)) << 8 |
            (static_cast<int> (color.b));

          color.rgb = static_cast<float> (rgb);
          color.rgba = static_cast<uint32_t> (rgba);
          break;
        }

        cloud->at (x, dimensions[1] - y - 1) = color;

      }
    }
  }
  return false;
}

void
HRCSRoadSegmentation::computeDEM (const std::string &left_image_name, const std::string &disparity,
    pcl::PointCloud<pcl::PointDEM>::Ptr &dem)
{
  pcl::DigitalElevationMapBuilder<pcl::PointDEM> demb;
  demb.setBaseline (BASELINE);
  demb.setFocalLength (FOCAL_LENGTH);
  demb.setImageCenterX (CENTER_X);
  demb.setImageCenterY (CENTER_Y);
  demb.setDisparityThresholdMin (DISPARITY_MIN);
  demb.setDisparityThresholdMax (DISPARITY_MAX);
  demb.setResolution (DEM_RESOLUTION_COLUMN, DEM_RESOLUTION_DISPARITY);
  demb.setMinPointsInCell (3);

  // Left view of the scene. 
  pcl::PointCloud<pcl::RGB>::Ptr left_image (new
      pcl::PointCloud<pcl::RGB>);
  loadImage(left_image_name, left_image);
  demb.setImage (left_image);

  // Disparity map of the scene.
  demb.loadDisparityMap (disparity);
  
  demb.compute(*dem);
}

bool
HRCSRoadSegmentation::loadDisparityMap (
    const std::string &file_name, std::vector<float> &disparity_map)
{
  std::fstream disparity_file;

  // Open the disparity file
  disparity_file.open (file_name.c_str(), std::fstream::in);
  if (!disparity_file.is_open ())
  {
    return false;
  }

  // Allocate memory for the disparity map.
  disparity_map.resize (width_ * height_);

  // Reading the disparity map.
  for (size_t row = 0; row < height_; ++row)
  {
    for (size_t column = 0; column < width_; ++column)
    {
      float disparity;
      disparity_file >> disparity;

      disparity_map[column + row * width_] = disparity;
    } // column
  } // row

  return true;
}

void
HRCSRoadSegmentation::saveLabeling (
    const pcl::PointCloud<pcl::Label>::Ptr &labels,
    const pcl::PointCloud<pcl::PointDEM>::Ptr &dem,
    const std::vector<double> &left_curb, 
    const std::vector<double> &right_curb,
    const std::vector<double> &road,
    const size_t frame_idx)
{
  std::vector<float> disparity_map;
  if (!loadDisparityMap (disparity_maps_[frame_idx], disparity_map))
  {
    std::cout << "Couldn't open the disparity file." << std::endl;
    return;
  }

  boost::filesystem::path left_image_path = left_images_[frame_idx];
  std::string labeling_path = labeling_path_ + "/" +
      left_image_path.filename().replace_extension().string() + ".txt";

  std::fstream labeling_file (labeling_path.c_str(), std::fstream::out | std::fstream::trunc);

  if (!labeling_file.is_open ())
  {
    std::cout << "Couldn't create the labeling file." << std::endl;
    return;
  }

  const size_t column_step = (width_ - 1) / DEM_RESOLUTION_COLUMN + 1;
  const float disparity_step = (DISPARITY_MAX - DISPARITY_MIN) / 
      DEM_RESOLUTION_DISPARITY;
  
  // Save labeling.
  for (size_t row = 0; row < height_; ++row)
  {
    for (size_t column = 0; column < width_; ++column)
    {
      float disparity = disparity_map[column + row * width_];
      if (DISPARITY_MIN < disparity && disparity < DISPARITY_MAX)
      {
        // Compute 3D-coordinates based on the image coordinates, the disparity and the camera parameters.
        float z_value = FOCAL_LENGTH * BASELINE / disparity;
        float x_value = (static_cast<float>(column) - CENTER_X) * (z_value / FOCAL_LENGTH);
        float y_value = (static_cast<float>(row) - CENTER_Y) * (z_value / FOCAL_LENGTH);
        // Compute x coordinates of the curbs.
        float x_left = -std::numeric_limits<float>::max();
        float x_right = std::numeric_limits<float>::max();
        if (!left_curb.empty())
        {
          x_left  = left_curb.at (0) * z_value * z_value * z_value +
                    left_curb.at (1) * z_value * z_value +
                    left_curb.at (2) * z_value +
                    left_curb.at (3);
        }
        if (!right_curb.empty())
        {
          x_right = right_curb.at (0) * z_value * z_value * z_value +
                    right_curb.at (1) * z_value * z_value +
                    right_curb.at (2) * z_value +
                    right_curb.at (3);
        }
        // Compute y coordinates of the road surface.
        float y_road = road[0] * x_value * x_value +
                       road[1] * z_value * z_value +
                       road[2] * 2.0 * x_value * z_value +
                       road[3] * x_value +
                       road[4] * z_value +
                       road[5];
        // Find the corresponding label.
        size_t index_column = column / column_step;
        size_t index_disparity = static_cast<size_t>(
            (disparity - DISPARITY_MIN) / disparity_step);
        pcl::Label label = labels->at (index_column, index_disparity);

        if (std::abs (y_value - y_road) > HEIGHT_THRESHOLD)
        {
          label.label = pcl::DEMGroundSegmentation::UNASSIGNED;
        }

        if (label.label != pcl::DEMGroundSegmentation::UNASSIGNED &&
            x_left < x_right)
        {
          if (x_value < x_left)
          {
            label.label = pcl::DEMGroundSegmentation::LEFT_SIDEWALK;
          }
          else if (x_value > x_right)
          {
            label.label = pcl::DEMGroundSegmentation::RIGHT_SIDEWALK;
          }
          else
          {
            label.label = pcl::DEMGroundSegmentation::ROAD;
          }
        }

        labeling_file << label.label;
      }
      else
      {
        labeling_file << 0;
      }
      if (column != width_ - 1)
      {
        labeling_file << " ";
      }
    }//column
    labeling_file << std::endl;
  }//row

  // Save curbs pixel-relative coordinates.
  if (!boost::filesystem::is_directory (labeling_path_ + "/Left"))
  {
    boost::filesystem::create_directory (labeling_path_ + "/Left");
  }
  if (!boost::filesystem::is_directory (labeling_path_ + "/Right"))
  {
    boost::filesystem::create_directory (labeling_path_ + "/Right");
  }

  if (!left_curb.empty())
  {
    std::string left_curb_path = labeling_path_ + "/Left/" +
        left_image_path.filename().replace_extension().string() + ".txt";
    std::fstream left_curb_file (left_curb_path.c_str(), std::fstream::out | std::fstream::trunc);
    
    for (size_t row = 0; row < dem->height; ++row)
    {
      float z_value = dem->at (0, row).z;
      float x_value = left_curb.at (0) * z_value * z_value * z_value +
                     left_curb.at (1) * z_value * z_value +
                     left_curb.at (2) * z_value +
                     left_curb.at (3);
      float y_value = road[0] * x_value * x_value +
                      road[1] * z_value * z_value +
                      road[2] * 2.0 * x_value * z_value +
                      road[3] * x_value +
                      road[4] * z_value +
                      road[5];

      if (y_value == y_value)
      {
        int x = std::floor(x_value * FOCAL_LENGTH / z_value + CENTER_X + 0.5f);
        int y = std::floor(y_value * FOCAL_LENGTH / z_value + CENTER_Y + 0.5f);

        left_curb_file << x << " " << y << std::endl;
      }
    }
  }

  if (!right_curb.empty())
  {
    std::string right_curb_path = labeling_path_ + "/Right/" +
        left_image_path.filename().replace_extension().string() + ".txt";
    std::fstream right_curb_file (right_curb_path.c_str(), std::fstream::out | std::fstream::trunc);
    
    for (size_t row = 0; row < dem->height; ++row)
    {
      float z_value = dem->at (0, row).z;
      float x_value = right_curb.at (0) * z_value * z_value * z_value +
                     right_curb.at (1) * z_value * z_value +
                     right_curb.at (2) * z_value +
                     right_curb.at (3);
      float y_value = road[0] * x_value * x_value +
                      road[1] * z_value * z_value +
                      road[2] * 2.0 * x_value * z_value +
                      road[3] * x_value +
                      road[4] * z_value +
                      road[5];

      if (y_value == y_value)
      {
        int x = std::floor(x_value * FOCAL_LENGTH / z_value + CENTER_X + 0.5f);
        int y = std::floor(y_value * FOCAL_LENGTH / z_value + CENTER_Y + 0.5f);

        right_curb_file << x << " " << y << std::endl;
      }
    }
  }
}
