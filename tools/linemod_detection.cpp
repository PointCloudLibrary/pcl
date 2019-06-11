
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

using PointCloudXYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd  min_depth  max_depth  max_height  output_template.lmt\n", argv[0]);
  print_info ("  where options are:\n");
}

void printElapsedTimeAndNumberOfPoints (double t, int w, int h=1)
{
  print_info ("[done, "); print_value ("%g", t); print_info (" ms : "); 
  print_value ("%d", w*h); print_info (" points]\n");
}

bool
loadCloud (const std::string & filename, PointCloudXYZRGBA & cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);

  printElapsedTimeAndNumberOfPoints (tt.toc (), cloud.width, cloud.height);

  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}



/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Train one or more linemod templates. For more information, use: %s -h\n", argv[0]);

  // If no arguments are given, print the help text
  if (argc == 1)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the gradient magnitude threshold
  float grad_mag_thresh = 10.0f;
  parse_argument (argc, argv, "-grad_mag_thresh", grad_mag_thresh);

  // Parse the detection threshold
  float detect_thresh = 0.75f;
  parse_argument (argc, argv, "-detect_thresh", detect_thresh);

  // Parse the command line arguments for .lmt files
  std::vector<int> lmt_file_indices;
  lmt_file_indices = parse_file_extension_argument (argc, argv, ".lmt");
  if (lmt_file_indices.empty ())
  {
    print_error ("Need at least one input LMT file.\n");
    return (-1);
  }

  LineRGBD<PointXYZRGBA> line_rgbd;
  line_rgbd.setGradientMagnitudeThreshold (grad_mag_thresh);
  line_rgbd.setDetectionThreshold (detect_thresh);

  // Load the template LMT and PCD files
  for (const int &lmt_file_index : lmt_file_indices)
  {
    // Load the LMT file
    std::string lmt_filename = argv[lmt_file_index];
    line_rgbd.loadTemplates (lmt_filename);
  }

  // Load the input PCD file
  std::string input_filename;
  if (parse_argument (argc, argv, "-input", input_filename) < 0)
    return (-1);
  PointCloudXYZRGBA::Ptr cloud (new PointCloudXYZRGBA);
  if (!loadCloud (input_filename, *cloud)) 
    return (-1);

  // Detect objects
  line_rgbd.setInputCloud (cloud);
  line_rgbd.setInputColors (cloud);

  std::vector<LineRGBD<PointXYZRGBA>::Detection> detections;
  line_rgbd.detect (detections);

  for (const auto &d : detections)
  {
    const BoundingBoxXYZ & bb = d.bounding_box;
    print_info ("%lu %lu %f (%f %f %f) (%f %f %f)\n", 
                d.detection_id, d.template_id, d.response,
                bb.x, bb.y, bb.z, bb.width, bb.height, bb.depth);
  }
}
