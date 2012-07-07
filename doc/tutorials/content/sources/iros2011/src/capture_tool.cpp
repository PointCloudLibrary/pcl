#include "openni_capture.h"

#include <string>
#include <pcl/console/parse.h>

int 
main (int argc, char ** argv)
{
  if (argc < 2) 
  {
    pcl::console::print_info ("Syntax is: %s output_filename <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    -n nr_frames ... Number of frames to capture (default = 1)\n");
    pcl::console::print_info ("  Note: The output filename should be provided without the .pcd extension.\n");
    pcl::console::print_info ("        File names will have frame numbers and extension appended to them.\n");
    pcl::console::print_info ("        (e.g., 'output' will become 'output_1.pcd', 'output_2.pcd', etc.)\n");
    return (1);
  }

  // Determine the number of frames to capture
  int nr_frames = 1; // Default value
  pcl::console::parse_argument (argc, argv, "-n", nr_frames);

  OpenNICapture camera;
  camera.setTriggerMode (true);
  for (int i = 0; i < nr_frames; ++i)
  {
    std::string foo = argv[1];
    std::stringstream filename (foo);
    filename << argv[1] << "_" << i << ".pcd";
    PCL_INFO ("%s\n", filename.str().c_str());
    camera.snapAndSave (filename.str ());
  }

  return (0);
}
