/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
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
 *  Author: Raphael Favier, Technical University Eindhoven, (r.mysurname <aT> tue.nl)
 */


#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <fstream>
#include <iostream>
#include <sstream>

#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/texture_mapping.h>

#include <pcl/io/vtk_lib_io.h>

using namespace pcl;

/** \brief Save a textureMesh object to obj file */
int
saveOBJFile (const std::string &file_name,
             const pcl::TextureMesh &tex_mesh, unsigned precision)
{
  if (tex_mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
    return (-1);
  }

  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  // Define material file
  std::string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
  // Strip path for "mtllib" command
  std::string mtl_file_name_nopath = mtl_file_name;
  mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);

  /* Write 3D information */
  // number of points
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = tex_mesh.cloud.data.size () / nr_points;

  // mesh size
  int nr_meshes = tex_mesh.tex_polygons.size ();
  // number of faces for header
  int nr_faces = 0;
  for (int m = 0; m < nr_meshes; ++m)
    nr_faces += tex_mesh.tex_polygons[m].size ();

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  fs << "# Faces: " <<nr_faces << std::endl;
  fs << "# Material information:" << std::endl;
  fs << "mtllib " << mtl_file_name_nopath << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "v" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                tex_mesh.cloud.fields[d].name == "x" ||
                tex_mesh.cloud.fields[d].name == "y" ||
                tex_mesh.cloud.fields[d].name == "z"))
      {
        if (!v_written)
        {
            // write vertices beginning with v
            fs << "v ";
            v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
            break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << std::endl;
  }
  fs << "# "<< nr_points <<" vertices" << std::endl;

  // Write vertex normals
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "vn" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
      count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
      tex_mesh.cloud.fields[d].name == "normal_x" ||
      tex_mesh.cloud.fields[d].name == "normal_y" ||
      tex_mesh.cloud.fields[d].name == "normal_z"))
      {
        if (!v_written)
        {
          // write vertices beginning with vn
          fs << "vn ";
          v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
    return (-2);
    }
    fs << std::endl;
  }
  // Write vertex texture with "vt" (adding latter)

  for (int m = 0; m < nr_meshes; ++m)
  {
    if(tex_mesh.tex_coordinates.size() == 0)
      continue;

    PCL_INFO ("%d vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size (), m);
    fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m <<  std::endl;
    for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size (); ++i)
    {
      fs << "vt ";
      fs <<  tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
    }
  }

  int f_idx = 0;

  // int idx_vt =0;
  PCL_INFO ("Writting faces...\n");
  for (int m = 0; m < nr_meshes; ++m)
  {
    if (m > 0) 
      f_idx += tex_mesh.tex_polygons[m-1].size ();

    if(tex_mesh.tex_materials.size() !=0)
    {
      fs << "# The material will be used for mesh " << m << std::endl;
      //TODO pbl here with multi texture and unseen faces
      fs << "usemtl " <<  tex_mesh.tex_materials[m].tex_name << std::endl;
      fs << "# Faces" << std::endl;
    }
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      // Write faces with "f"
      fs << "f";
      size_t j = 0;
      // There's one UV per vertex per face, i.e., the same vertex can have
      // different UV depending on the face.
      for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
        fs << " " << idx
        << "/" << 3*(i+f_idx) +j+1
        << "/" << idx; // vertex index in obj file format starting with 1
      }
      fs << std::endl;
    }
    PCL_INFO ("%d faces in mesh %d \n", tex_mesh.tex_polygons[m].size () , m);
    fs << "# "<< tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
  }
  fs << "# End of File";

  // Close obj file
  PCL_INFO ("Closing obj file\n");
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file
  PCL_INFO ("Writing material files\n");
  //dont do it if no material to write
  if(tex_mesh.tex_materials.size() ==0)
    return (0);

  std::ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << std::endl;
  m_fs << "# Wavefront material file" << std::endl;
  m_fs << "#" << std::endl;
  for(int m = 0; m < nr_meshes; ++m)
  {
    m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
    m_fs << "Ka "<< tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
    m_fs << "Kd "<< tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
    m_fs << "Ks "<< tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
    m_fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
    m_fs << "Ns "<< tex_mesh.tex_materials[m].tex_Ns  << std::endl; // defines the shininess of the material to be s.
    m_fs << "illum "<< tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
    // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
    // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
    m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
    m_fs << "###" << std::endl;
  }
  m_fs.close ();
  return (0);
}

/** \brief Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses */
void showCameras (pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

  // visualization object
  pcl::visualization::PCLVisualizer visu ("cameras");

  // add a visual for each camera at the correct pose
  for(int i = 0 ; i < cams.size () ; ++i)
  {
    // read current camera
    pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
    double focal = cam.focal_length;
    double height = cam.height;
    double width = cam.width;
    
    // create a 5-point visual for each camera
    pcl::PointXYZ p1, p2, p3, p4, p5;
    p1.x=0; p1.y=0; p1.z=0;
    double angleX = RAD2DEG (2.0 * atan (width / (2.0*focal)));
    double angleY = RAD2DEG (2.0 * atan (height / (2.0*focal)));
    double dist = 0.75;
    double minX, minY, maxX, maxY;
    maxX = dist*tan (atan (width / (2.0*focal)));
    minX = -maxX;
    maxY = dist*tan (atan (height / (2.0*focal)));
    minY = -maxY;
    p2.x=minX; p2.y=minY; p2.z=dist;
    p3.x=maxX; p3.y=minY; p3.z=dist;
    p4.x=maxX; p4.y=maxY; p4.z=dist;
    p5.x=minX; p5.y=maxY; p5.z=dist;
    p1=pcl::transformPoint (p1, cam.pose);
    p2=pcl::transformPoint (p2, cam.pose);
    p3=pcl::transformPoint (p3, cam.pose);
    p4=pcl::transformPoint (p4, cam.pose);
    p5=pcl::transformPoint (p5, cam.pose);
    std::stringstream ss;
    ss << "Cam #" << i+1;
    visu.addText3D(ss.str (), p1, 0.1, 1.0, 1.0, 1.0, ss.str ());

    ss.str ("");
    ss << "camera_" << i << "line1";
    visu.addLine (p1, p2,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line2";
    visu.addLine (p1, p3,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line3";
    visu.addLine (p1, p4,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line4";
    visu.addLine (p1, p5,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line5";
    visu.addLine (p2, p5,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line6";
    visu.addLine (p5, p4,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line7";
    visu.addLine (p4, p3,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line8";
    visu.addLine (p3, p2,ss.str ());
  }
  
  // add a coordinate system
  visu.addCoordinateSystem (1.0, "global");
  
  // add the mesh's cloud (colored on Z axis)
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloud, "z");
  visu.addPointCloud (cloud, color_handler, "cloud");
  
  // reset camera
  visu.resetCamera ();
  
  // wait for user input
  visu.spin ();
}

/** \brief Helper function that jump to a specific line of a text file */
std::ifstream& GotoLine(std::ifstream& file, unsigned int num)
{
  file.seekg (std::ios::beg);
  for(int i=0; i < num - 1; ++i)
  {
    file.ignore (std::numeric_limits<std::streamsize>::max (),'\n');
  }
  return (file);
}

/** \brief Helper function that reads a camera file outputed by Kinfu */
bool readCamPoseFile(std::string filename, pcl::TextureMapping<pcl::PointXYZ>::Camera &cam)
{
  ifstream myReadFile;
  myReadFile.open(filename.c_str (), ios::in);
  if(!myReadFile.is_open ())
  {
    PCL_ERROR ("Error opening file %d\n", filename.c_str ());
    return false;
  }
  myReadFile.seekg(ios::beg);

  char current_line[1024];
  double val;
  
  // go to line 2 to read translations
  GotoLine(myReadFile, 2);
  myReadFile >> val; cam.pose (0,3)=val; //TX
  myReadFile >> val; cam.pose (1,3)=val; //TY
  myReadFile >> val; cam.pose (2,3)=val; //TZ

  // go to line 7 to read rotations
  GotoLine(myReadFile, 7);

  myReadFile >> val; cam.pose (0,0)=val;
  myReadFile >> val; cam.pose (0,1)=val;
  myReadFile >> val; cam.pose (0,2)=val;

  myReadFile >> val; cam.pose (1,0)=val;
  myReadFile >> val; cam.pose (1,1)=val;
  myReadFile >> val; cam.pose (1,2)=val;

  myReadFile >> val; cam.pose (2,0)=val;
  myReadFile >> val; cam.pose (2,1)=val;
  myReadFile >> val; cam.pose (2,2)=val;

  cam.pose (3,0) = 0.0;
  cam.pose (3,1) = 0.0;
  cam.pose (3,2) = 0.0;
  cam.pose (3,3) = 1.0; //Scale
  
  // go to line 12 to read camera focal length and size
  GotoLine (myReadFile, 12);
  myReadFile >> val; cam.focal_length=val; 
  myReadFile >> val; cam.height=val;
  myReadFile >> val; cam.width=val;  
  
  // close file
  myReadFile.close ();

  return true;

}

int
main (int argc, char** argv)
{

  // read mesh from plyfile
  PCL_INFO ("\nLoading mesh from file %s...\n", argv[1]);
  pcl::PolygonMesh triangles;
  pcl::io::loadPolygonFilePLY(argv[1], triangles);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

  // Create the texturemesh object that will contain our UV-mapped mesh
  TextureMesh mesh;
  mesh.cloud = triangles.cloud;
  std::vector< pcl::Vertices> polygon_1;
  
  // push faces into the texturemesh object
  polygon_1.resize (triangles.polygons.size ());
  for(size_t i =0; i < triangles.polygons.size (); ++i)
  {
    polygon_1[i] = triangles.polygons[i];
  }
  mesh.tex_polygons.push_back(polygon_1);
  PCL_INFO ("\tInput mesh contains %d faces and %d vertices\n", mesh.tex_polygons[0].size (), cloud->points.size ());
  PCL_INFO ("...Done.\n");
  
  // Load textures and cameras poses and intrinsics
  PCL_INFO ("\nLoading textures and camera poses...\n");
  pcl::texture_mapping::CameraVector my_cams;
  
  const boost::filesystem::path base_dir (".");
  std::string extension (".txt");
  int cpt_cam = 0;
  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if(boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
      readCamPoseFile(it->path ().string (), cam);
      cam.texture_file = boost::filesystem::basename (it->path ()) + ".png";
      my_cams.push_back (cam);
      cpt_cam++ ;
    }
  }
  PCL_INFO ("\tLoaded %d textures.\n", my_cams.size ());
  PCL_INFO ("...Done.\n");
  
  // Display cameras to user
  PCL_INFO ("\nDisplaying cameras. Press \'q\' to continue texture mapping\n");
  showCameras(my_cams, cloud);


  // Create materials for each texture (and one extra for occluded faces)
  mesh.tex_materials.resize (my_cams.size () + 1);
  for(int i = 0 ; i <= my_cams.size() ; ++i)
  {
    pcl::TexMaterial mesh_material;
    mesh_material.tex_Ka.r = 0.2f;
    mesh_material.tex_Ka.g = 0.2f;
    mesh_material.tex_Ka.b = 0.2f;

    mesh_material.tex_Kd.r = 0.8f;
    mesh_material.tex_Kd.g = 0.8f;
    mesh_material.tex_Kd.b = 0.8f;

    mesh_material.tex_Ks.r = 1.0f;
    mesh_material.tex_Ks.g = 1.0f;
    mesh_material.tex_Ks.b = 1.0f;

    mesh_material.tex_d = 1.0f;
    mesh_material.tex_Ns = 75.0f;
    mesh_material.tex_illum = 2;

    std::stringstream tex_name;
    tex_name << "material_" << i;
    tex_name >> mesh_material.tex_name;

    if(i < my_cams.size ())
      mesh_material.tex_file = my_cams[i].texture_file;
    else
      mesh_material.tex_file = "occluded.jpg";

    mesh.tex_materials[i] = mesh_material;
  }


  // Sort faces
  PCL_INFO ("\nSorting faces by cameras...\n");
  pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
  tm.textureMeshwithMultipleCameras(mesh, my_cams);
  
  
  PCL_INFO ("Sorting faces by cameras done.\n");
  for(int i = 0 ; i <= my_cams.size() ; ++i)
  {
    PCL_INFO ("\tSub mesh %d contains %d faces and %d UV coordinates.\n", i, mesh.tex_polygons[i].size (), mesh.tex_coordinates[i].size ());
  }


  // compute normals for the mesh
  PCL_INFO ("\nEstimating normals...\n");
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  PCL_INFO ("...Done.\n");

  pcl::toPCLPointCloud2 (*cloud_with_normals, mesh.cloud);

  PCL_INFO ("\nSaving mesh to textured_mesh.obj\n");

  saveOBJFile ("textured_mesh.obj", mesh, 5);

  return (0);
}
