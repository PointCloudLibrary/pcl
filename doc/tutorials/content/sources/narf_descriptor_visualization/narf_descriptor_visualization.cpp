/* \author Bastian Steder */

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/narf.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension

float angular_resolution = 0.5f;
int rotation_invariant = 0;
float support_size = 0.3f;
int descriptor_size = 36;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

typedef pcl::PointXYZ PointType;

void 
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
            << "-s <float>   support size for the interest points (diameter of the used sphere - "
            <<                                                     "default "<<support_size<<")\n"
            << "-d <int>     descriptor size (default "<<descriptor_size<<")\n"
            << "-c <int>     coordinate frame of the input point cloud (default "<< (int)coordinate_frame<<")\n"
            << "-o <0/1>     switch rotational invariant version of the feature on/off"
            <<               " (default "<< (int)rotation_invariant<<")\n"
            << "-m           set unseen pixels to max range\n"
            << "-h           this help\n"
            << "\n\n";
}

int 
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    setUnseenToMaxRange = true;
    std::cout << "Setting unseen values in range image to maximum range readings.\n";
  }
  if (pcl::console::parse (argc, argv, "-o", rotation_invariant) >= 0)
    std::cout << "Switching rotation invariant feature version "<< (rotation_invariant ? "on" : "off")<<".\n";
  int tmp_coordinate_frame;
  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
    std::cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
  }
  if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
    std::cout << "Setting support size to "<<support_size<<".\n";
  if (pcl::console::parse (argc, argv, "-d", descriptor_size) >= 0)
    std::cout << "Setting descriptor size to "<<descriptor_size<<".\n";
  if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
    std::cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
  angular_resolution = pcl::deg2rad (angular_resolution);
  

  // -----------------------
  // -----Read pcd file-----
  // -----------------------
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
    {
      std::cout << "Was not able to open file \""<<filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
    std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
    if (pcl::io::loadPCDFile (far_ranges_filename.c_str (), far_ranges) == -1)
      std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
  }
  else
  {
    std::cout << "\nNo *.pcd file for scene given.\n\n";
    printUsage (argv[0]);
    return 1;
  }
  
  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();
  
  // Extract NARF features:
  std::cout << "Now extracting NARFs in every image point.\n";
  std::vector<std::vector<pcl::Narf*> > narfs;
  narfs.resize (range_image.size ());
  unsigned int last_percentage=0;
  for (unsigned int y=0; y<range_image.height; ++y)
  {
    for (unsigned int x=0; x<range_image.width; ++x)
    {
      const auto index = y*range_image.width+x;
      const auto percentage = ((100*index) / range_image.size ());
      if (percentage > last_percentage)
      {
        std::cout << percentage<<"% "<<std::flush;
        last_percentage = percentage;
      }
      pcl::Narf::extractFromRangeImageAndAddToList (range_image, x, y, descriptor_size,
                                                    support_size, rotation_invariant != 0, narfs[index]);
      //std::cout << "Extracted "<<narfs[index].size ()<<" features for pixel "<<x<<","<<y<<".\n";
    }
  }
  std::cout << "100%\n";
  std::cout << "Done.\n\n Now you can click on points in the image to visualize how the descriptor is "
       << "extracted and see the descriptor distances to every other point..\n";
  
  //---------------------
  // -----Show image-----
  // --------------------
  pcl::visualization::RangeImageVisualizer range_image_widget ("Scene range image"),
                                           surface_patch_widget("Descriptor's surface patch"),
                                           descriptor_widget("Descriptor"),
                                           descriptor_distances_widget("descriptor distances");
  range_image_widget.showRangeImage (range_image);
  //range_image_widget.visualize_selected_point = true;

  //--------------------
  // -----Main loop-----
  //--------------------
  while (true) 
  {
    range_image_widget.spinOnce ();  // process GUI events
    surface_patch_widget.spinOnce ();  // process GUI events
    descriptor_widget.spinOnce ();  // process GUI events
    pcl_sleep(0.01);
    
    //if (!range_image_widget.mouse_click_happened)
      continue;
    //range_image_widget.mouse_click_happened = false;
    //float clicked_pixel_x_f = range_image_widget.last_clicked_point_x,
          //clicked_pixel_y_f = range_image_widget.last_clicked_point_y;
    int clicked_pixel_x, clicked_pixel_y;
    //range_image.real2DToInt2D (clicked_pixel_x_f, clicked_pixel_y_f, clicked_pixel_x, clicked_pixel_y);
    if (!range_image.isValid (clicked_pixel_x, clicked_pixel_y))
      continue;
      //Vector3f clicked_3d_point;
      //range_image.getPoint (clicked_pixel_x, clicked_pixel_y, clicked_3d_point);
    
    //surface_patch_widget.show (false);
    //descriptor_widget.show (false);"
    
    int selected_index = clicked_pixel_y*range_image.width + clicked_pixel_x;
    pcl::Narf narf;
    if (!narf.extractFromRangeImage (range_image, clicked_pixel_x, clicked_pixel_y,
                                                                         descriptor_size, support_size))
    {
      continue;
    }
    
    int surface_patch_pixel_size = narf.getSurfacePatchPixelSize ();
    float surface_patch_world_size = narf.getSurfacePatchWorldSize ();
    surface_patch_widget.showFloatImage (narf.getSurfacePatch (), surface_patch_pixel_size, surface_patch_pixel_size,
                                         -0.5f*surface_patch_world_size, 0.5f*surface_patch_world_size, true);
    /*float surface_patch_rotation = narf.getSurfacePatchRotation ();
    float patch_middle = 0.5f* (float (surface_patch_pixel_size-1));
    float angle_step_size = pcl::deg2rad (360.0f)/narf.getDescriptorSize ();
    float cell_size = surface_patch_world_size/float (surface_patch_pixel_size),
          cell_factor = 1.0f/cell_size,
          max_dist = 0.5f*surface_patch_world_size,
          line_length = cell_factor* (max_dist-0.5f*cell_size);
    for (int descriptor_value_idx=0; descriptor_value_idx<narf.getDescriptorSize (); ++descriptor_value_idx)
    {
      float angle = descriptor_value_idx*angle_step_size + surface_patch_rotation;
      //surface_patch_widget.markLine (patch_middle, patch_middle, patch_middle+line_length*sinf (angle),
                                     //patch_middle+line_length*-std::cos (angle), pcl::visualization::Vector3ub (0,255,0));
    }
    std::vector<float> rotations, strengths;
    narf.getRotations (rotations, strengths);
    float radius = 0.5f*surface_patch_pixel_size;
    for (unsigned int i=0; i<rotations.size (); ++i)
    {
      //surface_patch_widget.markLine (radius-0.5, radius-0.5, radius-0.5f + 2.0f*radius*sinf (rotations[i]),
                                                //radius-0.5f - 2.0f*radius*std::cos (rotations[i]), pcl::visualization::Vector3ub (255,0,0));
    }*/
    
    descriptor_widget.showFloatImage (narf.getDescriptor (), narf.getDescriptorSize (), 1, -0.1f, 0.3f, true);

    //===================================
    //=====Compare with all features=====
    //===================================
    const std::vector<pcl::Narf*>& narfs_of_selected_point = narfs[selected_index];
    if (narfs_of_selected_point.empty ())
      continue;
    
    //descriptor_distances_widget.show (false);
    float* descriptor_distance_image = new float[range_image.size ()];
    for (unsigned int point_index=0; point_index<range_image.size (); ++point_index)
    {
      float& descriptor_distance = descriptor_distance_image[point_index];
      descriptor_distance = std::numeric_limits<float>::infinity ();
      std::vector<pcl::Narf*>& narfs_of_current_point = narfs[point_index];
      if (narfs_of_current_point.empty ())
        continue;
      for (unsigned int i=0; i<narfs_of_selected_point.size (); ++i)
      {
        for (unsigned int j=0; j<narfs_of_current_point.size (); ++j)
        {
          descriptor_distance = (std::min)(descriptor_distance,
                                           narfs_of_selected_point[i]->getDescriptorDistance (*narfs_of_current_point[j]));
        }
      }
    }
    descriptor_distances_widget.showFloatImage (descriptor_distance_image, range_image.width, range_image.height,
                                               -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), true);
    delete[] descriptor_distance_image;
  }
}
