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

#ifndef PCL_SURFACE_IMPL_TEXTURE_MAPPING_HPP_
#define PCL_SURFACE_IMPL_TEXTURE_MAPPING_HPP_

#include <pcl/surface/texture_mapping.h>

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> std::vector<Eigen::Vector2f>
pcl::TextureMapping<PointInT>::mapTexture2Face (
    const Eigen::Vector3f &p1, 
    const Eigen::Vector3f &p2, 
    const Eigen::Vector3f &p3)
{
  std::vector<Eigen::Vector2f> tex_coordinates;
  // process for each face
  Eigen::Vector3f p1p2 (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
  Eigen::Vector3f p1p3 (p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]);
  Eigen::Vector3f p2p3 (p3[0] - p2[0], p3[1] - p2[1], p3[2] - p2[2]);

  // Normalize
  p1p2 = p1p2 / std::sqrt (p1p2.dot (p1p2));
  p1p3 = p1p3 / std::sqrt (p1p3.dot (p1p3));
  p2p3 = p2p3 / std::sqrt (p2p3.dot (p2p3));

  // compute vector normal of a face
  Eigen::Vector3f f_normal = p1p2.cross (p1p3);
  f_normal = f_normal / std::sqrt (f_normal.dot (f_normal));

  // project vector field onto the face: vector v1_projected = v1 - Dot(v1, n) * n;
  Eigen::Vector3f f_vector_field = vector_field_ - vector_field_.dot (f_normal) * f_normal;

  // Normalize
  f_vector_field = f_vector_field / std::sqrt (f_vector_field.dot (f_vector_field));

  // texture coordinates
  Eigen::Vector2f tp1, tp2, tp3;

  double alpha = std::acos (f_vector_field.dot (p1p2));

  // distance between 3 vertices of triangles
  double e1 = (p2 - p3).norm () / f_;
  double e2 = (p1 - p3).norm () / f_;
  double e3 = (p1 - p2).norm () / f_;

  // initialize
  tp1[0] = 0.0;
  tp1[1] = 0.0;

  tp2[0] = static_cast<float> (e3);
  tp2[1] = 0.0;

  // determine texture coordinate tp3;
  double cos_p1 = (e2 * e2 + e3 * e3 - e1 * e1) / (2 * e2 * e3);
  double sin_p1 = sqrt (1 - (cos_p1 * cos_p1));

  tp3[0] = static_cast<float> (cos_p1 * e2);
  tp3[1] = static_cast<float> (sin_p1 * e2);

  // rotating by alpha (angle between V and pp1 & pp2)
  Eigen::Vector2f r_tp2, r_tp3;
  r_tp2[0] = static_cast<float> (tp2[0] * std::cos (alpha) - tp2[1] * std::sin (alpha));
  r_tp2[1] = static_cast<float> (tp2[0] * std::sin (alpha) + tp2[1] * std::cos (alpha));

  r_tp3[0] = static_cast<float> (tp3[0] * std::cos (alpha) - tp3[1] * std::sin (alpha));
  r_tp3[1] = static_cast<float> (tp3[0] * std::sin (alpha) + tp3[1] * std::cos (alpha));

  // shifting
  tp1[0] = tp1[0];
  tp2[0] = r_tp2[0];
  tp3[0] = r_tp3[0];
  tp1[1] = tp1[1];
  tp2[1] = r_tp2[1];
  tp3[1] = r_tp3[1];

  float min_x = tp1[0];
  float min_y = tp1[1];
  if (min_x > tp2[0])
    min_x = tp2[0];
  if (min_x > tp3[0])
    min_x = tp3[0];
  if (min_y > tp2[1])
    min_y = tp2[1];
  if (min_y > tp3[1])
    min_y = tp3[1];

  if (min_x < 0)
  {
    tp1[0] = tp1[0] - min_x;
    tp2[0] = tp2[0] - min_x;
    tp3[0] = tp3[0] - min_x;
  }
  if (min_y < 0)
  {
    tp1[1] = tp1[1] - min_y;
    tp2[1] = tp2[1] - min_y;
    tp3[1] = tp3[1] - min_y;
  }

  tex_coordinates.push_back (tp1);
  tex_coordinates.push_back (tp2);
  tex_coordinates.push_back (tp3);
  return tex_coordinates;
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::mapTexture2Mesh (pcl::TextureMesh &tex_mesh)
{
  // mesh information
  int nr_points = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = static_cast<int> (tex_mesh.cloud.data.size ()) / nr_points;

  // temporary PointXYZ
  float x, y, z;
  // temporary face
  Eigen::Vector3f facet[3];

  // texture coordinates for each mesh
  std::vector<std::vector<Eigen::Vector2f> > texture_map;

  for (size_t m = 0; m < tex_mesh.tex_polygons.size (); ++m)
  {
    // texture coordinates for each mesh
    std::vector<Eigen::Vector2f> texture_map_tmp;

    // processing for each face
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size (); ++i)
    {
      size_t idx;

      // get facet information
      for (size_t j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        idx = tex_mesh.tex_polygons[m][i].vertices[j];
        memcpy (&x, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[0].offset], sizeof(float));
        memcpy (&y, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[1].offset], sizeof(float));
        memcpy (&z, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[2].offset], sizeof(float));
        facet[j][0] = x;
        facet[j][1] = y;
        facet[j][2] = z;
      }

      // get texture coordinates of each face
      std::vector<Eigen::Vector2f> tex_coordinates = mapTexture2Face (facet[0], facet[1], facet[2]);
      for (size_t n = 0; n < tex_coordinates.size (); ++n)
        texture_map_tmp.push_back (tex_coordinates[n]);
    }// end faces

    // texture materials
    std::stringstream tex_name;
    tex_name << "material_" << m;
    tex_name >> tex_material_.tex_name;
    tex_material_.tex_file = tex_files_[m];
    tex_mesh.tex_materials.push_back (tex_material_);

    // texture coordinates
    tex_mesh.tex_coordinates.push_back (texture_map_tmp);
  }// end meshes
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::mapTexture2MeshUV (pcl::TextureMesh &tex_mesh)
{
  // mesh information
  int nr_points = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = static_cast<int> (tex_mesh.cloud.data.size ()) / nr_points;

  float x_lowest = 100000;
  float x_highest = 0;
  float y_lowest = 100000;
  //float y_highest = 0 ;
  float z_lowest = 100000;
  float z_highest = 0;
  float x_, y_, z_;

  for (int i = 0; i < nr_points; ++i)
  {
    memcpy (&x_, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[0].offset], sizeof(float));
    memcpy (&y_, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[1].offset], sizeof(float));
    memcpy (&z_, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[2].offset], sizeof(float));
    // x
    if (x_ <= x_lowest)
      x_lowest = x_;
    if (x_ > x_lowest)
      x_highest = x_;

    // y
    if (y_ <= y_lowest)
      y_lowest = y_;
    //if (y_ > y_lowest) y_highest = y_;

    // z
    if (z_ <= z_lowest)
      z_lowest = z_;
    if (z_ > z_lowest)
      z_highest = z_;
  }
  // x
  float x_range = (x_lowest - x_highest) * -1;
  float x_offset = 0 - x_lowest;
  // x
  //float y_range = (y_lowest - y_highest)*-1;
  //float y_offset = 0 - y_lowest;
  // z
  float z_range = (z_lowest - z_highest) * -1;
  float z_offset = 0 - z_lowest;

  // texture coordinates for each mesh
  std::vector<std::vector<Eigen::Vector2f> > texture_map;

  for (size_t m = 0; m < tex_mesh.tex_polygons.size (); ++m)
  {
    // texture coordinates for each mesh
    std::vector<Eigen::Vector2f> texture_map_tmp;

    // processing for each face
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size (); ++i)
    {
      size_t idx;
      Eigen::Vector2f tmp_VT;
      for (size_t j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        idx = tex_mesh.tex_polygons[m][i].vertices[j];
        memcpy (&x_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[0].offset], sizeof(float));
        memcpy (&y_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[1].offset], sizeof(float));
        memcpy (&z_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[2].offset], sizeof(float));

        // calculate uv coordinates
        tmp_VT[0] = (x_ + x_offset) / x_range;
        tmp_VT[1] = (z_ + z_offset) / z_range;
        texture_map_tmp.push_back (tmp_VT);
      }
    }// end faces

    // texture materials
    std::stringstream tex_name;
    tex_name << "material_" << m;
    tex_name >> tex_material_.tex_name;
    tex_material_.tex_file = tex_files_[m];
    tex_mesh.tex_materials.push_back (tex_material_);

    // texture coordinates
    tex_mesh.tex_coordinates.push_back (texture_map_tmp);
  }// end meshes
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::mapMultipleTexturesToMeshUV (pcl::TextureMesh &tex_mesh, std::vector<Camera> &cams)
{

  if (tex_mesh.tex_polygons.size () != cams.size () + 1)
  {
    PCL_ERROR ("The mesh should be divided into nbCamera+1 sub-meshes.\n");
    std::cerr << "You provided " << cams.size () << " cameras and a mesh containing "
        << tex_mesh.tex_polygons.size () << " sub-meshes." << std::endl;
    return;
  }

  std::cout << "You provided " << cams.size () << " cameras and a mesh containing " << tex_mesh.tex_polygons.size ()
      << " sub-meshes." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //convert mesh's cloud to pcl format for ease
  pcl::fromROSMsg (tex_mesh.cloud, *originalCloud);

  // texture coordinates for each mesh
  std::vector<std::vector<Eigen::Vector2f> > texture_map;

  for (size_t m = 0; m < cams.size (); ++m)
  {
    // get current camera parameters
    Camera current_cam = cams[m];

    // get camera transform
    Eigen::Affine3f cam_trans = current_cam.pose;

    // transform cloud into current camera frame
    pcl::transformPointCloud (*originalCloud, *camera_transformed_cloud, cam_trans.inverse ());

    //save transformed cloud
    //pcl::io::savePCDFile("cloud_cam.pcd", *camera_transformed_cloud);

    // vector of texture coordinates for each face
    std::vector<Eigen::Vector2f> texture_map_tmp;

    // processing each face visible by this camera
    pcl::PointXYZ pt;
    size_t idx;
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size (); ++i)
    {
      Eigen::Vector2f tmp_VT;
      // for each point of this face
      for (size_t j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        // get point
        idx = tex_mesh.tex_polygons[m][i].vertices[j];
        pt = camera_transformed_cloud->points[idx];

        // compute UV coordinates for this point
        getPointUVCoordinates (pt, current_cam, tmp_VT);
        texture_map_tmp.push_back (tmp_VT);

      }//end points
    }// end faces

    // texture materials
    std::stringstream tex_name;
    tex_name << "material_" << m;
    tex_name >> tex_material_.tex_name;
    tex_material_.tex_file = current_cam.texture_file;
    tex_mesh.tex_materials.push_back (tex_material_);

    // texture coordinates
    tex_mesh.tex_coordinates.push_back (texture_map_tmp);
  }// end cameras

  // push on extra empty UV map (for unseen faces) so that obj writer does not crash!
  std::vector<Eigen::Vector2f> texture_map_tmp;
  for (size_t i = 0; i < tex_mesh.tex_polygons[cams.size ()].size (); ++i)
    for (size_t j = 0; j < tex_mesh.tex_polygons[cams.size ()][i].vertices.size (); ++j)
    {
      Eigen::Vector2f tmp_VT;
      tmp_VT[0] = -1;
      tmp_VT[1] = -1;
      texture_map_tmp.push_back (tmp_VT);
    }

  tex_mesh.tex_coordinates.push_back (texture_map_tmp);

  //push on an extra dummy material for the same reason
  std::stringstream tex_name;
  tex_name << "material_" << cams.size ();
  tex_name >> tex_material_.tex_name;
  tex_material_.tex_file = "occluded.jpg";
  tex_mesh.tex_materials.push_back (tex_material_);

}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> bool
pcl::TextureMapping<PointInT>::isPointOccluded (const pcl::PointXYZ &pt, OctreePtr octree) const
{
  Eigen::Vector3f direction;
  direction (0) = pt.x;
  direction (1) = pt.y;
  direction (2) = pt.z;

  std::vector<int> indices;

  PointCloudConstPtr cloud (new PointCloud());
  cloud = octree->getInputCloud();

  double distance_threshold = octree->getResolution();

  //raytrace
  octree->getIntersectedVoxelIndices(direction, -direction, indices);

  int nbocc = static_cast<int> (indices.size ());
  for (size_t j = 0; j < indices.size (); j++)
  {
   //if intersected point is on the over side of the camera
   if (pt.z * cloud->points[indices[j]].z < 0)
   {
     nbocc--;
     continue;
   }

   if (fabs (cloud->points[indices[j]].z - pt.z) <= distance_threshold)
   {
     //points are very close to each-other, we do not consider the occlusion
     nbocc--;
   }
  }

  if (nbocc == 0)
   return (false);
  else
   return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::removeOccludedPoints (PointCloudPtr &input_cloud,
                                                     PointCloudPtr &filtered_cloud,
                                                     double octree_voxel_size, std::vector<int> &visible_indices,
                                                     std::vector<int> &occluded_indices)
{
  //variable used to filter occluded points by depth
  double maxDeltaZ = octree_voxel_size;

  //create an octree to perform rayTracing
  OctreePtr octree (new Octree (octree_voxel_size));
  //create octree structure
  octree->setInputCloud (input_cloud);
  //update bounding box automatically
  octree->defineBoundingBox ();
  //add points in the tree
  octree->addPointsFromInputCloud ();

  visible_indices.clear ();

  //for each point of the cloud, raycast toward camera and check intersected voxels.
  Eigen::Vector3f direction;
  std::vector<int> indices;
  for (size_t i = 0; i < input_cloud->points.size (); ++i)
  {
    direction (0) = input_cloud->points[i].x;
    direction (1) = input_cloud->points[i].y;
    direction (2) = input_cloud->points[i].z;

    //if point is not occluded
    octree->getIntersectedVoxelIndices (direction, -direction, indices);

    int nbocc = static_cast<int> (indices.size ());
    for (size_t j = 0; j < indices.size (); j++)
    {
      //if intersected point is on the over side of the camera
      if (input_cloud->points[i].z * input_cloud->points[indices[j]].z < 0)
      {
        nbocc--;
        continue;
      }

      if (fabs (input_cloud->points[indices[j]].z - input_cloud->points[i].z) <= maxDeltaZ)
      {
        //points are very close to each-other, we do not consider the occlusion
        nbocc--;
      }
    }

    if (nbocc == 0)
    {
      //point is added in the filtered mesh
      filtered_cloud->points.push_back (input_cloud->points[i]);
      visible_indices.push_back (static_cast<int> (i));
    }
    else
    {
      occluded_indices.push_back (static_cast<int> (i));
    }
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::removeOccludedPoints (pcl::TextureMesh &tex_mesh, pcl::TextureMesh &cleaned_mesh, double octree_voxel_size)
{
  //copy mesh
  cleaned_mesh = tex_mesh;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //load points into a PCL format
  pcl::fromROSMsg (tex_mesh.cloud, *cloud);

  std::vector<int> visible, occluded;
  removeOccludedPoints (cloud, filtered_cloud, octree_voxel_size, visible, occluded);

  //Now that we know which points are visible, let's iterate over each face.
  //if the face has one invisible point => out!
  for (size_t polygons = 0; polygons < cleaned_mesh.tex_polygons.size (); ++polygons)
  {
    //remove all faces from cleaned mesh
    cleaned_mesh.tex_polygons[polygons].clear ();
    //iterate over faces
    for (size_t faces = 0; faces < tex_mesh.tex_polygons[polygons].size (); ++faces)
    {
      //check if all the face's points are visible
      bool faceIsVisible = true;
      std::vector<int>::iterator it;
      //iterate over face's vertex
      //      std::cout << "Face " << faces << " has " << tex_mesh.tex_polygons[polygons][faces].vertices.size() << " vertices" << std::endl;
      for (size_t points = 0; points < tex_mesh.tex_polygons[polygons][faces].vertices.size (); ++points)
      {

        //        std::cout << "Currently dealing with Polygon " << polygons << ", face " << faces << ", point of indice " << tex_mesh.tex_polygons[polygons][faces].vertices[points] << std::flush;
        it = find (occluded.begin (), occluded.end (), tex_mesh.tex_polygons[polygons][faces].vertices[points]);

        if (it == occluded.end ())
        {
          //point is not in the occluded vector
          //          std::cout << "  VISIBLE!" << std::endl;
        }
        else
        {
          //point was occluded
          //          std::cout << "  OCCLUDED!" << std::endl;
          faceIsVisible = false;
        }
      }

      if (faceIsVisible)
      {
        cleaned_mesh.tex_polygons[polygons].push_back (tex_mesh.tex_polygons[polygons][faces]);
      }

    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::removeOccludedPoints (pcl::TextureMesh &tex_mesh, PointCloudPtr &filtered_cloud,
                      double octree_voxel_size)
{
  PointCloudPtr cloud (new PointCloud);

  //load points into a PCL format
  pcl::fromROSMsg (tex_mesh.cloud, *cloud);

  std::vector<int> visible, occluded;
  removeOccludedPoints (cloud, filtered_cloud, octree_voxel_size, visible, occluded);

}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> int
pcl::TextureMapping<PointInT>::sortFacesByCamera (pcl::TextureMesh &tex_mesh, pcl::TextureMesh &sorted_mesh,
                                                  std::vector<Camera> &cameras, double octree_voxel_size,
                                                  PointCloud &visible_pts)
{
  if (tex_mesh.tex_polygons.size () != 1)
  {
    PCL_ERROR ("The mesh must contain only 1 sub-mesh!\n");
    return (-1);
  }

  if (cameras.size () == 0)
  {
    PCL_ERROR ("Must provide at least one camera info!\n");
    return (-1);
  }

  //copy mesh
  sorted_mesh = tex_mesh;
  //clear polygons from cleaned_mesh
  sorted_mesh.tex_polygons.clear ();

  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //load points into a PCL format
  pcl::fromROSMsg (tex_mesh.cloud, *original_cloud);

  //FOR EACH CAMERA INFO
  for (size_t cam = 0; cam < cameras.size (); ++cam)
  {
    //get camera pose as transform
    Eigen::Affine3f cam_trans = cameras[cam].pose;

    //transform original cloud in camera coordinates
    pcl::transformPointCloud (*original_cloud, *transformed_cloud, cam_trans.inverse ());

    //find occlusions on transformed cloud
    std::vector<int> visible, occluded;
    removeOccludedPoints (transformed_cloud, filtered_cloud, octree_voxel_size, visible, occluded);
    //TODO quick hack for debug. At least clear filtered_cloud before calling remove_occluded_points
    visible_pts = *filtered_cloud;

    //find visible faces => add them to polygon N for camera N
    //add polygon group for current camera in clean
    std::vector<pcl::Vertices> visibleFaces_currentCam;
    //iterate over the faces of the current mesh
    for (size_t faces = 0; faces < tex_mesh.tex_polygons[0].size (); ++faces)
    {
      //check if all the face's points are visible
      bool faceIsVisible = true;
      std::vector<int>::iterator it;

      //iterate over face's vertex
      for (size_t current_pt_indice = 0; faceIsVisible && current_pt_indice < tex_mesh.tex_polygons[0][faces].vertices.size (); ++current_pt_indice)
      {
        //TODO this is far too long! Better create an helper function that raycasts here.
        it = find (occluded.begin (), occluded.end (), tex_mesh.tex_polygons[0][faces].vertices[current_pt_indice]);

        if (it == occluded.end ())
        {
          //point is not occluded
          //does it land on the camera's image plane?
          pcl::PointXYZ pt = transformed_cloud->points[tex_mesh.tex_polygons[0][faces].vertices[current_pt_indice]];
          Eigen::Vector2f dummy_UV;
          if (!getPointUVCoordinates (pt, cameras[cam], dummy_UV))
          {
            //point is not visible by the camera
            faceIsVisible = false;
          }
        }
        else
        {
          faceIsVisible = false;
        }
      }

      if (faceIsVisible)
      {
        //push current visible face into the sorted mesh
        visibleFaces_currentCam.push_back (tex_mesh.tex_polygons[0][faces]);
        //remove it from the unsorted mesh
        tex_mesh.tex_polygons[0].erase (tex_mesh.tex_polygons[0].begin () + faces);
        faces--;
      }

    }
    sorted_mesh.tex_polygons.push_back (visibleFaces_currentCam);
  }

  //we should only have occluded and non-visible faces left in tex_mesh.tex_polygons[0]
  //we need to add them as an extra polygon in the sorted mesh
  sorted_mesh.tex_polygons.push_back (tex_mesh.tex_polygons[0]);
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::showOcclusions (PointCloudPtr &input_cloud,
                                               pcl::PointCloud<pcl::PointXYZI>::Ptr &colored_cloud,
                                               double octree_voxel_size, bool show_nb_occlusions,
                                               int max_occlusions)
                                               {
  //variable used to filter occluded points by depth
  double maxDeltaZ = octree_voxel_size * 2.0;

  //create an octree to perform rayTracing
  pcl::octree::OctreePointCloudSearch<PointInT> *octree;
  octree = new pcl::octree::OctreePointCloudSearch<PointInT> (octree_voxel_size);
  //create octree structure
  octree->setInputCloud (input_cloud);
  //update bounding box automatically
  octree->defineBoundingBox ();
  //add points in the tree
  octree->addPointsFromInputCloud ();

  //ray direction
  Eigen::Vector3f direction;

  std::vector<int> indices;
  //point from where we ray-trace
  pcl::PointXYZI pt;

  std::vector<double> zDist;
  std::vector<double> ptDist;
  //for each point of the cloud, ray-trace toward the camera and check intersected voxels.
  for (size_t i = 0; i < input_cloud->points.size (); ++i)
  {
    direction (0) = input_cloud->points[i].x;
    pt.x = input_cloud->points[i].x;
    direction (1) = input_cloud->points[i].y;
    pt.y = input_cloud->points[i].y;
    direction (2) = input_cloud->points[i].z;
    pt.z = input_cloud->points[i].z;

    //get number of occlusions for that point
    indices.clear ();
    int nbocc = octree->getIntersectedVoxelIndices (direction, -direction, indices);

    nbocc = static_cast<int> (indices.size ());

    //TODO need to clean this up and find tricks to get remove aliasaing effect on planes
    for (size_t j = 0; j < indices.size (); j++)
    {
      //if intersected point is on the over side of the camera
      if (pt.z * input_cloud->points[indices[j]].z < 0)
      {
        nbocc--;
      }
      else if (fabs (input_cloud->points[indices[j]].z - pt.z) <= maxDeltaZ)
      {
        //points are very close to each-other, we do not consider the occlusion
        nbocc--;
      }
      else
      {
        zDist.push_back (fabs (input_cloud->points[indices[j]].z - pt.z));
        ptDist.push_back (pcl::euclideanDistance (input_cloud->points[indices[j]], pt));
      }
    }

    if (show_nb_occlusions)
      (nbocc <= max_occlusions) ? (pt.intensity = static_cast<float> (nbocc)) : (pt.intensity = static_cast<float> (max_occlusions));
    else
      (nbocc == 0) ? (pt.intensity = 0) : (pt.intensity = 1);

    colored_cloud->points.push_back (pt);
  }

  if (zDist.size () >= 2)
  {
    std::sort (zDist.begin (), zDist.end ());
    std::sort (ptDist.begin (), ptDist.end ());
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::showOcclusions (pcl::TextureMesh &tex_mesh, pcl::PointCloud<pcl::PointXYZI>::Ptr &colored_cloud,
                  double octree_voxel_size, bool show_nb_occlusions, int max_occlusions)
{
  //load points into a PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (tex_mesh.cloud, *cloud);

  showOcclusions (cloud, colored_cloud, octree_voxel_size, show_nb_occlusions, max_occlusions);
}

#define PCL_INSTANTIATE_TextureMapping(T)                \
    template class PCL_EXPORTS pcl::TextureMapping<T>;

#endif /* TEXTURE_MAPPING_HPP_ */

