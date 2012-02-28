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
 * Author: Nico Blodow (blodow@cs.tum.edu)
 *         Christian Potthast (potthast@usc.edu)
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

using namespace pcl::console;

std::string default_format = "bc";
bool default_noend = false;
bool set_file_name = false;

class OpenNIGrabFrame
{
  public:
    OpenNIGrabFrame () : w (), no_frame (true), noend (false), file (), format ()
    {
    }

    void 
    cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      if (!no_frame)
        return;

      std::string output_dir;
      if (output_dir.empty ())
        output_dir = ".";

      std::stringstream ss;
      if (noend || !set_file_name)
      {
        ss << output_dir << "/frame_" << boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());
        file = ss.str ();
      }

      if (strcmp (format.c_str (), "ascii") == 0)
      {
        w.writeASCII<pcl::PointXYZRGBA> (file + ".pcd", *cloud);
        std::cerr << "Data saved in ASCII format to " << file << ".pcd" << std::endl;
      }        
      else if (strcmp (format.c_str (), "b") == 0)
      {
        w.writeBinary<pcl::PointXYZRGBA> (file + ".pcd", *cloud);
        std::cerr << "Data saved in BINARY format to " << file << ".pcd" << std::endl;
      }
      else if (strcmp (format.c_str (), "bc") == 0)
      {
        w.writeBinaryCompressed<pcl::PointXYZRGBA> (file + ".pcd", *cloud);
        std::cerr << "Data saved in BINARY COMPRESSED format to " << file << ".pcd" << std::endl;
      }
        else if (strcmp (format.c_str (), "all") == 0)
      {
        w.writeASCII<pcl::PointXYZRGBA> (file + "_ascii.pcd", *cloud);
        w.writeBinary<pcl::PointXYZRGBA> (file + "_binary.pcd", *cloud);
        w.writeBinaryCompressed<pcl::PointXYZRGBA> (file + "_binary_compressed.pcd", *cloud);
        std::cerr << "Data saved as ASCII, BINARY and BINARY COMPRESSED to " << file << "_{ascii,binary,binary_compresed}.pcd" << std::endl;
      }
              
      if (!noend)
        no_frame = false;
    }
    
    void 
    run ()
    {
      // create a new grabber for OpenNI devices
      pcl::Grabber* interface = new pcl::OpenNIGrabber ();

      // make callback function from member function
      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind (&OpenNIGrabFrame::cloud_cb_, this, _1);

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c = interface->registerCallback (f);

      // start receiving point clouds
      interface->start ();

      // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
      while (no_frame)
        boost::this_thread::sleep (boost::posix_time::seconds (1));
   
      // stop the grabber
      interface->stop ();
    }

    void
    setOptions (std::string filename, std::string pcd_format, bool loop)
    {
      if (strcmp (filename.c_str (), "") != 0)
      {  
        set_file_name = true;

        size_t index = filename.find (".");
        file = filename.substr (0, index);
      }
      noend = loop;
      format = pcd_format;
    }

    pcl::PCDWriter w;
    bool no_frame;
    bool noend;
    std::string file;
    std::string format;  
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <filename> <options>\n\n";
  
  print_info ("  filename: if no filename is provided a generic timestamp will be set as filename\n\n");
  print_info ("  where options are:\n");
  print_info ("                    -format = PCD file format (b=binary; bc=binary compressed; ascii=ascii; all=all) (default: ");
  print_value ("%s", default_format.c_str()); print_info(")\n");
  print_info ("                    -noend  = Store files to disk until the user quits the program with Ctrl-C (default: ") ;
  print_value ("%d", default_noend); print_info(")\n");
}

int 
main (int argc, char** argv)
{
  OpenNIGrabFrame v;

  std::string arg;
  if (argc > 1)
    arg = std::string (argv[1]);

  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return 1;
  }

  std::string format = default_format;
  bool noend = default_noend;
  std::string filename;
  if (argc > 1)
  {
    // Parse the command line arguments for .pcd file
    std::vector<int> p_file_indices;
    p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    if (p_file_indices.size () > 0)
      filename = argv[p_file_indices[0]];
    
    // Command line parsing
    parse_argument (argc, argv, "-format", format);
    if (find_argument (argc, argv, "-noend") != -1)
      noend = true; 
  }

  v.setOptions (filename, format, noend);
  v.run ();
  return (0);
}

