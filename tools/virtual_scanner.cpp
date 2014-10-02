/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id$
 *
 */

/**
  * \author Radu Bogdan Rusu
  *
  * @b virtual_scanner takes in a .ply or a .vtk file of an object model, and virtually scans it
  * in a raytracing fashion, saving the end results as PCD (Point Cloud Data) files. In addition, 
  * it noisifies the PCD models, and downsamples them. 
  * The viewpoint can be set to 1 or multiple views on a sphere.
  */
#include <string>
#include <pcl/register_point_struct.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/vtk.h>
#include "boost.h"

using namespace pcl;

#define EPS 0.00001

struct ScanParameters
{
  int nr_scans;             // number of steps for sweep movement
  int nr_points_in_scans;   // number of laser beam measurements per scan
  double vert_res;          // vertical resolution (res. of sweep) in degrees
  double hor_res;           // horizontal  resolution (of laser beam) in degrees
  double max_dist;          // maximum distance in units.
};


/** \brief Loads a 3D point cloud from a given fileName, and returns: a
  * vtkDataSet object containing the point cloud.
  * \param file_name the name of the file containing the dataset
  */
vtkPolyData*
loadDataSet (const char* file_name)
{
  std::string extension = boost::filesystem::extension (file_name);
  if (extension == ".ply")
  {
    vtkPLYReader* reader = vtkPLYReader::New ();
    reader->SetFileName (file_name);
    reader->Update ();
    return (reader->GetOutput ());
  }
  else if (extension == ".vtk")
  {
    vtkPolyDataReader* reader = vtkPolyDataReader::New ();
    reader->SetFileName (file_name);
    reader->Update ();
    return (reader->GetOutput ());
  }
  else
  {
    PCL_ERROR ("Needs a VTK/PLY file to continue.\n");
    return (NULL);
  }
}

int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    PCL_INFO ("Usage %s [options] <model.ply | model.vtk>\n", argv[0]);
    PCL_INFO (" * where options are:\n"
              "         -object_coordinates <0|1> : save the dataset in object coordinates (1) or camera coordinates (0)\n"
              "         -single_view <0|1>        : take a single snapshot (1) or record a lot of camera poses on a view sphere (0)\n"
              "         -view_point <x,y,z>       : set the camera viewpoint from where the acquisition will take place\n"
              "         -target_point <x,y,z>     : the target point that the camera should look at (default: 0, 0, 0)\n"
              "         -organized <0|1>          : create an organized, grid-like point cloud of width x height (1), or keep it unorganized with height = 1 (0)\n"
              "         -noise <0|1>              : add gausian noise (1) or keep the model noiseless (0)\n"
              "         -noise_std <x>            : use X times the standard deviation\n"
              "");
    return (-1);
  }
  std::string filename;
  // Parse the command line arguments for .vtk or .ply files
  std::vector<int> p_file_indices_vtk = console::parse_file_extension_argument (argc, argv, ".vtk");
  std::vector<int> p_file_indices_ply = console::parse_file_extension_argument (argc, argv, ".ply");
  bool object_coordinates = true;
  console::parse_argument (argc, argv, "-object_coordinates", object_coordinates);
  bool single_view = false;
  console::parse_argument (argc, argv, "-single_view", single_view);
  double vx = 0, vy = 0, vz = 0;
  console::parse_3x_arguments (argc, argv, "-view_point", vx, vy, vz);
  double tx = 0, ty = 0, tz = 0;
  console::parse_3x_arguments (argc, argv, "-target_point", tx, ty, tz);
  int organized = 0;
  console::parse_argument (argc, argv, "-organized", organized);
  if (organized)
    PCL_INFO ("Saving an organized dataset.\n");
  else
    PCL_INFO ("Saving an unorganized dataset.\n");

  vtkSmartPointer<vtkPolyData> data;
  // Loading PLY/VTK file
  if (p_file_indices_ply.empty () && p_file_indices_vtk.empty ())
  {
    PCL_ERROR ("Error: no .PLY or .VTK files given!\n");
    return (-1);
  }
  
  std::stringstream filename_stream;
  if (!p_file_indices_ply.empty ())
    filename_stream << argv[p_file_indices_ply.at (0)];
  else
    filename_stream << argv[p_file_indices_vtk.at (0)];

  filename = filename_stream.str ();
  
  data = loadDataSet (filename.c_str ());
  
  PCL_INFO ("Loaded model with %d vertices/points.\n", data->GetNumberOfPoints ());

  // Default scan parameters
  ScanParameters sp;
  sp.nr_scans           = 900;
  console::parse_argument (argc, argv, "-nr_scans", sp.nr_scans);
  sp.nr_points_in_scans = 900;
  console::parse_argument (argc, argv, "-pts_in_scan", sp.nr_points_in_scans);
  sp.max_dist           = 30000;   // maximum distance (in mm)
  sp.vert_res           = 0.25;
  console::parse_argument (argc, argv, "-vert_res", sp.vert_res);
  sp.hor_res            = 0.25;
  console::parse_argument (argc, argv, "-hor_res", sp.hor_res);

  int noise_model = 0;               // set the default noise level to none
  console::parse_argument (argc, argv, "-noise", noise_model);
  float noise_std = 0.05f;           // 0.5 standard deviations by default
  console::parse_argument (argc, argv, "-noise_std", noise_std);
  if (noise_model)
    PCL_INFO ("Adding Gaussian noise to the final model with %f x std_dev.\n", noise_std);
  else
    PCL_INFO ("Not adding any noise to the final model.\n");

  int subdiv_level = 1;
  double scan_dist = 3;
  std::string fname, base;
  char seq[256];

  // Compute start/stop for vertical and horizontal
  double vert_start = - (static_cast<double> (sp.nr_scans - 1) / 2.0) * sp.vert_res;
  double vert_end   = + ((sp.nr_scans-1) * sp.vert_res) + vert_start;
  double hor_start  = - (static_cast<double> (sp.nr_points_in_scans - 1) / 2.0) * sp.hor_res;
  double hor_end    = + ((sp.nr_points_in_scans-1) * sp.hor_res) + hor_start;

  // Prepare the point cloud data
  pcl::PointCloud<pcl::PointWithViewpoint> cloud;

  // Prepare the leaves for downsampling
  pcl::VoxelGrid<pcl::PointWithViewpoint> grid;
  grid.setLeafSize (2.5, 2.5, 2.5);    // @note: this value should be given in mm!

  // Reset and set a random seed for the Global Random Number Generator
  boost::mt19937 rng (static_cast<unsigned int> (std::time (0)));
  boost::normal_distribution<float> normal_distrib (0.0f, noise_std * noise_std);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > gaussian_rng (rng, normal_distrib);

  std::vector<std::string> st;
  // Virtual camera parameters
  double eye[3]     = {0.0, 0.0, 0.0};
  double viewray[3] = {0.0, 0.0, 0.0};
  double up[3]      = {0.0, 0.0, 0.0};
  double right[3]  = {0.0, 0.0, 0.0};
  double x_axis[3] = {1.0, 0.0, 0.0};
  double z_axis[3] = {0.0, 0.0, 1.0};
  double bounds[6];
  double temp_beam[3], beam[3], p[3];
  double p_coords[3], x[3], t;
  int subId;
 
  // Create a Icosahedron at center in origin and radius of 1
  vtkSmartPointer<vtkPlatonicSolidSource> icosa = vtkSmartPointer<vtkPlatonicSolidSource>::New ();
  icosa->SetSolidTypeToIcosahedron ();

  // Tesselize the source icosahedron (subdivide each triangular face
  // of the icosahedron into smaller triangles)
  vtkSmartPointer<vtkLoopSubdivisionFilter> subdivide = vtkSmartPointer<vtkLoopSubdivisionFilter>::New ();
  subdivide->SetNumberOfSubdivisions (subdiv_level);
  subdivide->SetInputConnection (icosa->GetOutputPort ());
  subdivide->Update ();

  // Get camera positions
  vtkPolyData *sphere = subdivide->GetOutput ();
  if (!single_view)
    PCL_INFO ("Created %ld camera position points.\n", sphere->GetNumberOfPoints ());

  // Build a spatial locator for our dataset
  vtkSmartPointer<vtkCellLocator> tree = vtkSmartPointer<vtkCellLocator>::New ();
  tree->SetDataSet (data);
  tree->CacheCellBoundsOn ();
  tree->SetTolerance (0.0);
  tree->SetNumberOfCellsPerBucket (1);
  tree->AutomaticOn ();
  tree->BuildLocator ();
  tree->Update ();

  // Get the min-max bounds of data
  data->GetBounds (bounds);

  // if single view is required iterate over loop only once
  int number_of_points = static_cast<int> (sphere->GetNumberOfPoints ());
  if (single_view)
    number_of_points = 1;

  int sid = -1;
  for (int i = 0; i < number_of_points; i++)
  {
    sphere->GetPoint (i, eye);
    if (fabs(eye[0]) < EPS) eye[0] = 0;
    if (fabs(eye[1]) < EPS) eye[1] = 0;
    if (fabs(eye[2]) < EPS) eye[2] = 0;

    viewray[0] = -eye[0];
    viewray[1] = -eye[1];
    viewray[2] = -eye[2];
    eye[0] *= scan_dist;
    eye[1] *= scan_dist;
    eye[2] *= scan_dist;
    //Change here if only single view point is required
    if (single_view)
    {
      eye[0] = vx;//0.0;
      eye[1] = vy;//-0.26;
      eye[2] = vz;//-0.86;
      viewray[0] = tx - vx;
      viewray[1] = ty - vy;
      viewray[2] = tz - vz;
      double len = sqrt (viewray[0]*viewray[0] + viewray[1]*viewray[1] + viewray[2]*viewray[2]);
      if (len == 0)
      {
        PCL_ERROR ("The single_view option is enabled but the view_point and the target_point are the same!\n");
        break;
      }
      viewray[0] /= len;
      viewray[1] /= len;
      viewray[2] /= len;
    }

    if ((viewray[0] == 0) && (viewray[1] == 0))
      vtkMath::Cross (viewray, x_axis, right);
    else
      vtkMath::Cross (viewray, z_axis, right);
    if (fabs(right[0]) < EPS) right[0] = 0;
    if (fabs(right[1]) < EPS) right[1] = 0;
    if (fabs(right[2]) < EPS) right[2] = 0;

    vtkMath::Cross (viewray, right, up);
    if (fabs(up[0]) < EPS) up[0] = 0;
    if (fabs(up[1]) < EPS) up[1] = 0;
    if (fabs(up[2]) < EPS) up[2] = 0;

    if (!object_coordinates)
    {
      // Normalization
      double right_len = sqrt (right[0]*right[0] + right[1]*right[1] + right[2]*right[2]);
      right[0] /= right_len;
      right[1] /= right_len;
      right[2] /= right_len;
      double up_len = sqrt (up[0]*up[0] + up[1]*up[1] + up[2]*up[2]);
      up[0] /= up_len;
      up[1] /= up_len;
      up[2] /= up_len;
    
      // Output resulting vectors
      cerr << "Viewray Right Up:" << endl;
      cerr << viewray[0] << " " << viewray[1] << " " << viewray[2] << " " << endl;
      cerr << right[0] << " " << right[1] << " " << right[2] << " " << endl;
      cerr << up[0] << " " << up[1] << " " << up[2] << " " << endl;
    }

    // Create a transformation
    vtkGeneralTransform* tr1 = vtkGeneralTransform::New ();
    vtkGeneralTransform* tr2 = vtkGeneralTransform::New ();

    // right = viewray x up
    vtkMath::Cross (viewray, up, right);

    // Sweep vertically
    for (double vert = vert_start; vert <= vert_end; vert += sp.vert_res)
    {
      sid++;

      tr1->Identity ();
      tr1->RotateWXYZ (vert, right);
      tr1->InternalTransformPoint (viewray, temp_beam);

      // Sweep horizontally
      int pid = -1;
      for (double hor = hor_start; hor <= hor_end; hor += sp.hor_res)
      {
        pid ++;
      
        // Create a beam vector with (lat,long) angles (vert, hor) with the viewray
        tr2->Identity ();
        tr2->RotateWXYZ (hor, up);
        tr2->InternalTransformPoint (temp_beam, beam);
        vtkMath::Normalize (beam);

        // Find point at max range: p = eye + beam * max_dist
        for (int d = 0; d < 3; d++)
          p[d] = eye[d] + beam[d] * sp.max_dist;

        // Put p_coords into laser scan at packetid = vert, scan id = hor
        vtkIdType cellId;
        if (tree->IntersectWithLine (eye, p, 0, t, x, p_coords, subId, cellId))
        {
          pcl::PointWithViewpoint pt;
          if (object_coordinates)
          {
            pt.x = static_cast<float> (x[0]); 
            pt.y = static_cast<float> (x[1]); 
            pt.z = static_cast<float> (x[2]);
            pt.vp_x = static_cast<float> (eye[0]); 
            pt.vp_y = static_cast<float> (eye[1]); 
            pt.vp_z = static_cast<float> (eye[2]);
          }
          else
          {
            // z axis is the viewray
            // y axis is up
            // x axis is -right (negative because z*y=-x but viewray*up=right)
            pt.x = static_cast<float> (-right[0]*x[1] + up[0]*x[2] + viewray[0]*x[0] + eye[0]);
            pt.y = static_cast<float> (-right[1]*x[1] + up[1]*x[2] + viewray[1]*x[0] + eye[1]);
            pt.z = static_cast<float> (-right[2]*x[1] + up[2]*x[2] + viewray[2]*x[0] + eye[2]);
            pt.vp_x = pt.vp_y = pt.vp_z = 0.0f;
          }
          cloud.points.push_back (pt);
        }
        else
          if (organized)
          {
            pcl::PointWithViewpoint pt;
            pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
            pt.vp_x = static_cast<float> (eye[0]);
            pt.vp_y = static_cast<float> (eye[1]);
            pt.vp_z = static_cast<float> (eye[2]);
            cloud.points.push_back (pt);
          }
      } // Horizontal
    } // Vertical

    // Noisify each point in the dataset
    // \note: we might decide to noisify along the ray later
    for (size_t cp = 0; cp < cloud.points.size (); ++cp)
    {
      // Add noise ?
      switch (noise_model)
      {
        // Gaussian
        case 1: { cloud.points[cp].x += gaussian_rng (); cloud.points[cp].y += gaussian_rng (); cloud.points[cp].z += gaussian_rng (); break; }
      }
    }

    // Downsample and remove silly point duplicates
    pcl::PointCloud<pcl::PointWithViewpoint> cloud_downsampled;
    grid.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointWithViewpoint> > (cloud));
    //grid.filter (cloud_downsampled);

    // Saves the point cloud data to disk
    sprintf (seq, "%d", i);
    boost::trim (filename);
    boost::split (st, filename, boost::is_any_of ("/\\"), boost::token_compress_on);

    std::stringstream ss;
    std::string output_dir = st.at (st.size () - 1);
    ss << output_dir << "_output";

    boost::filesystem::path outpath (ss.str ());
    if (!boost::filesystem::exists (outpath))
    {
      if (!boost::filesystem::create_directories (outpath))
      {
        PCL_ERROR ("Error creating directory %s.\n", ss.str ().c_str ());
        return (-1);
      }
      PCL_INFO ("Creating directory %s\n", ss.str ().c_str ());
    }

    fname = ss.str () + "/" + seq + ".pcd";

    if (organized)
    {
      cloud.height = 1 + static_cast<uint32_t> ((vert_end - vert_start) / sp.vert_res);
      cloud.width = 1 + static_cast<uint32_t> ((hor_end - hor_start) / sp.hor_res);
    }
    else
    {
      cloud.width = static_cast<uint32_t> (cloud.points.size ());
      cloud.height = 1;
    }

    pcl::PCDWriter writer;
    PCL_INFO ("Wrote %lu points (%d x %d) to %s\n", cloud.points.size (), cloud.width, cloud.height, fname.c_str ());
    writer.writeBinaryCompressed (fname.c_str (), cloud);
  } // sphere
  return (0);
}
/* ]--- */
