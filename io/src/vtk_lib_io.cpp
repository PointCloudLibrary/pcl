/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Dirk Holz, University of Bonn.
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
 * $Id$
 *
 */

#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/impl/vtk_lib_io.hpp>
#include <pcl/PCLPointCloud2.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkImageData.h>
#include <vtkImageShiftScale.h>
#include <vtkPNGWriter.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::loadPolygonFile (const std::string &file_name, pcl::PolygonMesh& mesh)
{
  std::string extension = file_name.substr (file_name.find_last_of (".") + 1);

  if (extension == "pcd") // no Polygon, but only a point cloud
  {
    pcl::io::loadPCDFile (file_name, mesh.cloud);
    mesh.polygons.resize (0);
    return (static_cast<int> (mesh.cloud.width * mesh.cloud.height));
  }
  else if (extension == "vtk")
   return (pcl::io::loadPolygonFileVTK (file_name, mesh));
  else if (extension == "ply")
   return (pcl::io::loadPolygonFilePLY (file_name, mesh));
  else if (extension == "obj")
    return (pcl::io::loadPolygonFileOBJ (file_name, mesh));
  else if (extension == "stl" )
    return (pcl::io::loadPolygonFileSTL (file_name, mesh));
  else
  {
    PCL_ERROR ("[pcl::io::loadPolygonFile]: Unsupported file type (%s)\n", extension.c_str ());
    return (0);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::savePolygonFile (const std::string &file_name, const pcl::PolygonMesh& mesh)
{
  // TODO: what about binary/ASCII modes?!?!?!
  // TODO: what about sensor position and orientation?!?!?!?
  // TODO: how to adequately catch exceptions thrown by the vtk writers?!
  std::string extension = file_name.substr (file_name.find_last_of (".") + 1);
  if (extension == "pcd") // no Polygon, but only a point cloud
  {
    int error_code = pcl::io::savePCDFile (file_name, mesh.cloud);
    if (error_code != 0)
      return (0);
    return (static_cast<int> (mesh.cloud.width * mesh.cloud.height));
  }
  else if (extension == "vtk")
   return (pcl::io::savePolygonFileVTK (file_name, mesh));
  else if (extension == "ply")
   return (pcl::io::savePolygonFilePLY (file_name, mesh));
  else if (extension == "stl" )
    return (pcl::io::savePolygonFileSTL (file_name, mesh));
  else
  {
    PCL_ERROR ("[pcl::io::savePolygonFile]: Unsupported file type (%s)\n", extension.c_str ());
    return (0);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::loadPolygonFileVTK (const std::string &file_name, pcl::PolygonMesh& mesh)
{
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New ();

  vtkSmartPointer<vtkPolyDataReader> ply_reader = vtkSmartPointer<vtkPolyDataReader>::New ();
  ply_reader->SetFileName (file_name.c_str ());
  ply_reader->Update ();
  poly_data = ply_reader->GetOutput ();

  return (pcl::io::vtk2mesh (poly_data, mesh));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::loadPolygonFilePLY (const std::string &file_name, pcl::PolygonMesh& mesh)
{
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();

  vtkSmartPointer<vtkPLYReader> ply_reader = vtkSmartPointer<vtkPLYReader>::New();
  ply_reader->SetFileName (file_name.c_str ());
  ply_reader->Update ();
  poly_data = ply_reader->GetOutput ();

  return (pcl::io::vtk2mesh (poly_data, mesh));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::loadPolygonFileOBJ (const std::string &file_name, pcl::PolygonMesh& mesh)
{
  vtkSmartPointer<vtkOBJReader> ply_reader = vtkSmartPointer<vtkOBJReader>::New ();
  ply_reader->SetFileName (file_name.c_str ());
  ply_reader->Update ();

  vtkSmartPointer<vtkPolyData> poly_data = ply_reader->GetOutput ();

  return (pcl::io::vtk2mesh (poly_data, mesh));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::loadPolygonFileOBJ (const std::string &file_name, pcl::TextureMesh& mesh)
{
  vtkSmartPointer<vtkOBJReader> ply_reader = vtkSmartPointer<vtkOBJReader>::New ();
  ply_reader->SetFileName (file_name.c_str ());
  ply_reader->Update ();

  vtkSmartPointer<vtkPolyData> poly_data = ply_reader->GetOutput ();

  return (pcl::io::vtk2mesh (poly_data, mesh));
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::loadPolygonFileSTL (const std::string &file_name, pcl::PolygonMesh& mesh)
{
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New ();

  vtkSmartPointer<vtkSTLReader> ply_reader = vtkSmartPointer<vtkSTLReader>::New ();
  ply_reader->SetFileName (file_name.c_str ());
  ply_reader->Update ();
  poly_data = ply_reader->GetOutput ();

  return (pcl::io::vtk2mesh (poly_data, mesh));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::savePolygonFileVTK (const std::string &file_name, const pcl::PolygonMesh& mesh)
{
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New ();

  pcl::io::mesh2vtk (mesh, poly_data);

  vtkSmartPointer<vtkPolyDataWriter> poly_writer = vtkSmartPointer<vtkPolyDataWriter>::New ();
  poly_writer->SetInput (poly_data);
  poly_writer->SetFileName (file_name.c_str ());
  poly_writer->Write ();

  return (static_cast<int> (mesh.cloud.width * mesh.cloud.height));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::savePolygonFilePLY (const std::string &file_name, const pcl::PolygonMesh& mesh)
{
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New ();

  pcl::io::mesh2vtk (mesh, poly_data);

  vtkSmartPointer<vtkPLYWriter> poly_writer = vtkSmartPointer<vtkPLYWriter>::New ();
  poly_writer->SetInput (poly_data);
  poly_writer->SetFileName (file_name.c_str ());
	poly_writer->SetArrayName ("Colors");
  poly_writer->Write ();

  return (static_cast<int> (mesh.cloud.width * mesh.cloud.height));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::savePolygonFileSTL (const std::string &file_name, const pcl::PolygonMesh& mesh)
{
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New ();

  pcl::io::mesh2vtk (mesh, poly_data);
  poly_data->Update ();
  vtkSmartPointer<vtkSTLWriter> poly_writer = vtkSmartPointer<vtkSTLWriter>::New ();
  poly_writer->SetInput (poly_data);
  poly_writer->SetFileName (file_name.c_str ());
  poly_writer->Write ();

  return (static_cast<int> (mesh.cloud.width * mesh.cloud.height));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::vtk2mesh (const vtkSmartPointer<vtkPolyData>& poly_data, pcl::PolygonMesh& mesh)
{
  mesh.polygons.resize (0);
  mesh.cloud.data.clear ();
  mesh.cloud.width = mesh.cloud.height = 0;
  mesh.cloud.is_dense = true;

  vtkSmartPointer<vtkPoints> mesh_points = poly_data->GetPoints ();
  vtkIdType nr_points = mesh_points->GetNumberOfPoints ();
  vtkIdType nr_polygons = poly_data->GetNumberOfPolys ();

  if (nr_points == 0)
    return (0);


  // First get the xyz information
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  xyz_cloud->points.resize (nr_points);
  xyz_cloud->width = static_cast<uint32_t> (xyz_cloud->points.size ());
  xyz_cloud->height = 1;
  xyz_cloud->is_dense = true;
  double point_xyz[3];
  for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); i++)
  {
    mesh_points->GetPoint (i, &point_xyz[0]);
    xyz_cloud->points[i].x = static_cast<float> (point_xyz[0]);
    xyz_cloud->points[i].y = static_cast<float> (point_xyz[1]);
    xyz_cloud->points[i].z = static_cast<float> (point_xyz[2]);
  }
  // And put it in the mesh cloud
  pcl::toPCLPointCloud2 (*xyz_cloud, mesh.cloud);


  // Then the color information, if any
  vtkUnsignedCharArray* poly_colors = NULL;
  if (poly_data->GetPointData() != NULL)
    poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("Colors"));

  // some applications do not save the name of scalars (including PCL's native vtk_io)
  if (!poly_colors && poly_data->GetPointData () != NULL)
    poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("scalars"));

  if (!poly_colors && poly_data->GetPointData () != NULL)
    poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("RGB"));

  // TODO: currently only handles rgb values with 3 components
  if (poly_colors && (poly_colors->GetNumberOfComponents () == 3))
  {
    pcl::PointCloud<pcl::RGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::RGB> ());
    rgb_cloud->points.resize (nr_points);
    rgb_cloud->width = static_cast<uint32_t> (rgb_cloud->points.size ());
    rgb_cloud->height = 1;
    rgb_cloud->is_dense = true;

    unsigned char point_color[3];
    for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); i++)
    {
      poly_colors->GetTupleValue (i, &point_color[0]);
      rgb_cloud->points[i].r = point_color[0];
      rgb_cloud->points[i].g = point_color[1];
      rgb_cloud->points[i].b = point_color[2];
    }

    pcl::PCLPointCloud2 rgb_cloud2;
    pcl::toPCLPointCloud2 (*rgb_cloud, rgb_cloud2);
    pcl::PCLPointCloud2 aux;
    pcl::concatenateFields (rgb_cloud2, mesh.cloud, aux);
    mesh.cloud = aux;
  }


  // Then handle the normals, if any
  vtkFloatArray* normals = NULL;
  if (poly_data->GetPointData () != NULL)
    normals = vtkFloatArray::SafeDownCast (poly_data->GetPointData ()->GetNormals ());
  if (normals != NULL)
  {
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal> ());
    normal_cloud->resize (nr_points);
    normal_cloud->width = static_cast<uint32_t> (xyz_cloud->points.size ());
    normal_cloud->height = 1;
    normal_cloud->is_dense = true;

    for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); i++)
    {
      float normal[3];
      normals->GetTupleValue (i, normal);
      normal_cloud->points[i].normal_x = normal[0];
      normal_cloud->points[i].normal_y = normal[1];
      normal_cloud->points[i].normal_z = normal[2];
    }

    pcl::PCLPointCloud2 normal_cloud2;
    pcl::toPCLPointCloud2 (*normal_cloud, normal_cloud2);
    pcl::PCLPointCloud2 aux;
    pcl::concatenateFields (normal_cloud2, mesh.cloud, aux);
    mesh.cloud = aux;
  }



  // Now handle the polygons
  mesh.polygons.resize (nr_polygons);
  vtkIdType* cell_points;
  vtkIdType nr_cell_points;
  vtkCellArray * mesh_polygons = poly_data->GetPolys ();
  mesh_polygons->InitTraversal ();
  int id_poly = 0;
  while (mesh_polygons->GetNextCell (nr_cell_points, cell_points))
  {
    mesh.polygons[id_poly].vertices.resize (nr_cell_points);
    for (int i = 0; i < nr_cell_points; ++i)
      mesh.polygons[id_poly].vertices[i] = static_cast<int> (cell_points[i]);
    ++id_poly;
  }

  return (static_cast<int> (nr_points));
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::vtk2mesh (const vtkSmartPointer<vtkPolyData>& poly_data, pcl::TextureMesh& mesh)
{
  /// TODO avoid copying here
  PolygonMesh polygon_mesh;
  vtk2mesh (poly_data, polygon_mesh);

  mesh.cloud = polygon_mesh.cloud;
  mesh.header = polygon_mesh.header;
  /// TODO check for sub-meshes
  mesh.tex_polygons.push_back (polygon_mesh.polygons);

  // Add dummy material
  mesh.tex_materials.push_back (pcl::TexMaterial ());
  std::vector<Eigen::Vector2f> dummy;
  mesh.tex_coordinates.push_back (dummy);

  vtkIdType nr_points = poly_data->GetNumberOfPoints ();

  // Handle the texture coordinates
  vtkFloatArray* texture_coords = NULL;
  if (poly_data->GetPointData () != NULL)
    texture_coords = vtkFloatArray::SafeDownCast (poly_data->GetPointData ()->GetTCoords ());

  if (texture_coords != NULL)
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      float tex[2];
      texture_coords->GetTupleValue (i, tex);
      mesh.tex_coordinates.front ().push_back (Eigen::Vector2f (tex[0], tex[1]));
    }
  }
  else
    PCL_ERROR ("Could not find texture coordinates in the polydata\n");

  return (static_cast<int> (nr_points));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::mesh2vtk (const pcl::PolygonMesh& mesh, vtkSmartPointer<vtkPolyData>& poly_data)
{
  unsigned int nr_points = mesh.cloud.width * mesh.cloud.height;
  unsigned int nr_polygons = static_cast<unsigned int> (mesh.polygons.size ());

  // reset vtkPolyData object
  poly_data = vtkSmartPointer<vtkPolyData>::New (); // OR poly_data->Reset();
  vtkSmartPointer<vtkPoints> vtk_mesh_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> vtk_mesh_polygons = vtkSmartPointer<vtkCellArray>::New ();
  poly_data->SetPoints (vtk_mesh_points);

  // get field indices for x, y, z (as well as rgb and/or rgba)
  int idx_x = -1, idx_y = -1, idx_z = -1, idx_rgb = -1, idx_rgba = -1, idx_normal_x = -1, idx_normal_y = -1, idx_normal_z = -1;
  for (int d = 0; d < static_cast<int> (mesh.cloud.fields.size ()); ++d)
  {
    if (mesh.cloud.fields[d].name == "x") idx_x = d;
    else if (mesh.cloud.fields[d].name == "y") idx_y = d;
    else if (mesh.cloud.fields[d].name == "z") idx_z = d;
    else if (mesh.cloud.fields[d].name == "rgb") idx_rgb = d;
    else if (mesh.cloud.fields[d].name == "rgba") idx_rgba = d;
    else if (mesh.cloud.fields[d].name == "normal_x") idx_normal_x = d;
    else if (mesh.cloud.fields[d].name == "normal_y") idx_normal_y = d;
    else if (mesh.cloud.fields[d].name == "normal_z") idx_normal_z = d;
  }
  if ( ( idx_x == -1 ) || ( idx_y == -1 ) || ( idx_z == -1 ) )
    nr_points = 0;

  // copy point data
  vtk_mesh_points->SetNumberOfPoints (nr_points);
  if (nr_points > 0)
  {
    Eigen::Vector4f pt = Eigen::Vector4f::Zero ();
    Eigen::Array4i xyz_offset (mesh.cloud.fields[idx_x].offset, mesh.cloud.fields[idx_y].offset, mesh.cloud.fields[idx_z].offset, 0);
    for (vtkIdType cp = 0; cp < static_cast<vtkIdType> (nr_points); ++cp, xyz_offset += mesh.cloud.point_step)
    {
      memcpy(&pt[0], &mesh.cloud.data[xyz_offset[0]], sizeof (float));
      memcpy(&pt[1], &mesh.cloud.data[xyz_offset[1]], sizeof (float));
      memcpy(&pt[2], &mesh.cloud.data[xyz_offset[2]], sizeof (float));
      vtk_mesh_points->InsertPoint (cp, pt[0], pt[1], pt[2]);
    }
  }

  // copy polygon data
  if (nr_polygons > 0)
  {
    for (unsigned int i = 0; i < nr_polygons; i++)
    {
      unsigned int nr_points_in_polygon = static_cast<unsigned int> (mesh.polygons[i].vertices.size ());
      vtk_mesh_polygons->InsertNextCell (nr_points_in_polygon);
      for (unsigned int j = 0; j < nr_points_in_polygon; j++)
        vtk_mesh_polygons->InsertCellPoint (mesh.polygons[i].vertices[j]);
    }
    poly_data->SetPolys (vtk_mesh_polygons);
  }

  // copy color information
  if (idx_rgb != -1 || idx_rgba != -1)
  {
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");
    pcl::RGB rgb;
    int offset = (idx_rgb != -1) ? mesh.cloud.fields[idx_rgb].offset : mesh.cloud.fields[idx_rgba].offset;
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      memcpy (&rgb, &mesh.cloud.data[cp * mesh.cloud.point_step + offset], sizeof (RGB));
      const unsigned char color[3] = {rgb.r, rgb.g, rgb.b};
      colors->InsertNextTupleValue (color);
    }
    poly_data->GetPointData ()->SetScalars (colors);
  }

  // copy normal information
  if (( idx_normal_x != -1 ) && ( idx_normal_y != -1 ) && ( idx_normal_z != -1 ))
  {
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New ();
    normals->SetNumberOfComponents (3);
    float nx = 0.0f, ny = 0.0f, nz = 0.0f;
    for (vtkIdType cp = 0; cp < nr_points; ++cp)
    {
      memcpy (&nx, &mesh.cloud.data[cp*mesh.cloud.point_step+mesh.cloud.fields[idx_normal_x].offset], sizeof (float));
      memcpy (&ny, &mesh.cloud.data[cp*mesh.cloud.point_step+mesh.cloud.fields[idx_normal_y].offset], sizeof (float));
      memcpy (&nz, &mesh.cloud.data[cp*mesh.cloud.point_step+mesh.cloud.fields[idx_normal_z].offset], sizeof (float));
      const float normal[3] = {nx, ny, nz};
      normals->InsertNextTupleValue (normal);
    }
    poly_data->GetPointData()->SetNormals (normals);
  }

  if (poly_data->GetPoints () == NULL)
    return (0);
  return (static_cast<int> (poly_data->GetPoints ()->GetNumberOfPoints ()));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::saveRangeImagePlanarFilePNG (
    const std::string &file_name, const pcl::RangeImagePlanar& range_image)
{
  vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
  image->SetDimensions(range_image.width, range_image.height, 1);
  image->SetNumberOfScalarComponents(1);
  image->SetScalarTypeToFloat();
  image->AllocateScalars();

  int* dims = image->GetDimensions();

  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      float* pixel = static_cast<float*>(image->GetScalarPointer(x,y,0));
      pixel[0] = range_image(y,x).range;
      }
    }

  // Compute the scaling
  float oldRange = static_cast<float> (image->GetScalarRange()[1] - image->GetScalarRange()[0]);
  float newRange = 255; // We want the output [0,255]

  vtkSmartPointer<vtkImageShiftScale> shiftScaleFilter = vtkSmartPointer<vtkImageShiftScale>::New();
  shiftScaleFilter->SetOutputScalarTypeToUnsignedChar();
  shiftScaleFilter->SetInputConnection(image->GetProducerPort());
  shiftScaleFilter->SetShift(-1.0f * image->GetScalarRange()[0]); // brings the lower bound to 0
  shiftScaleFilter->SetScale(newRange/oldRange);
  shiftScaleFilter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(file_name.c_str());
  writer->SetInputConnection(shiftScaleFilter->GetOutputPort());
  writer->Write();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::pointCloudTovtkPolyData(const pcl::PCLPointCloud2Ptr& cloud, vtkSmartPointer<vtkPolyData>& poly_data)
{
  if (!poly_data.GetPointer())
    poly_data = vtkSmartPointer<vtkPolyData>::New (); // OR poly_data->Reset();


  // Add Points
  size_t x_idx = pcl::getFieldIndex (*cloud, std::string ("x") );
  vtkSmartPointer<vtkPoints> cloud_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cloud_vertices = vtkSmartPointer<vtkCellArray>::New ();

  vtkIdType pid[1];
  for (size_t point_idx = 0; point_idx < cloud->width * cloud->height; point_idx ++)
  {
    float point[3];

    int point_offset = (int (point_idx) * cloud->point_step);
    int offset = point_offset + cloud->fields[x_idx].offset;
    memcpy (&point, &cloud->data[offset], sizeof (float)*3);

    pid[0] = cloud_points->InsertNextPoint (point);
    cloud_vertices->InsertNextCell (1, pid);
  }

  //set the points and vertices we created as the geometry and topology of the polydata
  poly_data->SetPoints (cloud_points);
  poly_data->SetVerts (cloud_vertices);

  // Add RGB
  int rgb_idx = pcl::getFieldIndex (*cloud, "rgb");
  if (rgb_idx != -1)
  {
    //std::cout << "Adding rgb" << std::endl;
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();

    colors->SetNumberOfComponents (3);
    colors->SetName ("rgb");

    for (size_t point_idx = 0; point_idx < cloud->width * cloud->height; point_idx ++)
    {
      unsigned char bgr[3];

      int point_offset = (int (point_idx) * cloud->point_step);
      int offset = point_offset + cloud->fields[rgb_idx].offset;
      memcpy (&bgr, &cloud->data[offset], sizeof (unsigned char)*3);

      colors->InsertNextTuple3(bgr[2], bgr[1], bgr[0]);
    }

    poly_data->GetCellData()->SetScalars(colors);
  }

  // Add Intensity
  int intensity_idx = pcl::getFieldIndex (*cloud, "intensity");
  if (intensity_idx != -1)
  {
    //std::cout << "Adding intensity" << std::endl;
    vtkSmartPointer<vtkFloatArray> cloud_intensity = vtkSmartPointer<vtkFloatArray>::New ();
    cloud_intensity->SetNumberOfComponents (1);
    cloud_intensity->SetName("intensity");

    for (size_t point_idx = 0; point_idx < cloud->width * cloud->height; point_idx ++)
    {
      float intensity;

      int point_offset = (int (point_idx) * cloud->point_step);
      int offset = point_offset + cloud->fields[intensity_idx].offset;
      memcpy (&intensity, &cloud->data[offset], sizeof(float));

      cloud_intensity->InsertNextValue(intensity);
    }

    poly_data->GetCellData()->AddArray(cloud_intensity);
    if (rgb_idx == -1)
      poly_data->GetCellData()->SetActiveAttribute("intensity", vtkDataSetAttributes::SCALARS);
  }

  // Add Normals
  int normal_x_idx = pcl::getFieldIndex (*cloud, std::string ("normal_x") );
  if (normal_x_idx != -1)
  {
    //std::cout << "Adding normals" << std::endl;
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
    normals->SetNumberOfComponents(3); //3d normals (ie x,y,z)
    normals->SetName("normals");

    for (size_t point_idx = 0; point_idx < cloud->width * cloud->height; point_idx ++)
    {
      float normal[3];

      int point_offset = (int (point_idx) * cloud->point_step);
      int offset = point_offset + cloud->fields[normal_x_idx].offset;
      memcpy (&normal, &cloud->data[offset], sizeof (float)*3);

      normals->InsertNextTuple(normal);
    }

    poly_data->GetCellData()->SetNormals(normals);
    //poly_data->GetCellData()->SetActiveAttribute("normals", vtkDataSetAttributes::SCALARS);
  }
}
