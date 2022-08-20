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

#include <pcl/common/distances.h>
#include <pcl/surface/texture_mapping.h>
#include <unordered_set>

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >
pcl::TextureMapping<PointInT>::mapTexture2Face (
    const Eigen::Vector3f &p1, 
    const Eigen::Vector3f &p2, 
    const Eigen::Vector3f &p3)
{
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > tex_coordinates;
  // process for each face
  Eigen::Vector3f p1p2 (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
  Eigen::Vector3f p1p3 (p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]);
  Eigen::Vector3f p2p3 (p3[0] - p2[0], p3[1] - p2[1], p3[2] - p2[2]);

  // Normalize
  p1p2 /= std::sqrt (p1p2.dot (p1p2));
  p1p3 /= std::sqrt (p1p3.dot (p1p3));
  p2p3 /= std::sqrt (p2p3.dot (p2p3));

  // compute vector normal of a face
  Eigen::Vector3f f_normal = p1p2.cross (p1p3);
  f_normal /= std::sqrt (f_normal.dot (f_normal));

  // project vector field onto the face: vector v1_projected = v1 - Dot(v1, n) * n;
  Eigen::Vector3f f_vector_field = vector_field_ - vector_field_.dot (f_normal) * f_normal;

  // Normalize
  f_vector_field /= std::sqrt (f_vector_field.dot (f_vector_field));

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
    tp1[0] -= min_x;
    tp2[0] -= min_x;
    tp3[0] -= min_x;
  }
  if (min_y < 0)
  {
    tp1[1] -= min_y;
    tp2[1] -= min_y;
    tp3[1] -= min_y;
  }

  tex_coordinates.push_back (tp1);
  tex_coordinates.push_back (tp2);
  tex_coordinates.push_back (tp3);
  return (tex_coordinates);
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
  std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > >texture_map;

  for (std::size_t m = 0; m < tex_mesh.tex_polygons.size (); ++m)
  {
    // texture coordinates for each mesh
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > texture_map_tmp;

    // processing for each face
    for (std::size_t i = 0; i < tex_mesh.tex_polygons[m].size (); ++i)
    {
      // get facet information
      for (std::size_t j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        std::size_t idx = tex_mesh.tex_polygons[m][i].vertices[j];
        memcpy (&x, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[0].offset], sizeof(float));
        memcpy (&y, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[1].offset], sizeof(float));
        memcpy (&z, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[2].offset], sizeof(float));
        facet[j][0] = x;
        facet[j][1] = y;
        facet[j][2] = z;
      }

      // get texture coordinates of each face
      std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > tex_coordinates = mapTexture2Face (facet[0], facet[1], facet[2]);
      for (const auto &tex_coordinate : tex_coordinates)
        texture_map_tmp.push_back (tex_coordinate);
    }// end faces

    // texture materials
    tex_material_.tex_name = "material_" + std::to_string(m);
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
  // float y_range = (y_lowest - y_highest)*-1;
  // float y_offset = 0 - y_lowest;
  // z
  float z_range = (z_lowest - z_highest) * -1;
  float z_offset = 0 - z_lowest;

  // texture coordinates for each mesh
  std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > >texture_map;

  for (std::size_t m = 0; m < tex_mesh.tex_polygons.size (); ++m)
  {
    // texture coordinates for each mesh
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > texture_map_tmp;

    // processing for each face
    for (std::size_t i = 0; i < tex_mesh.tex_polygons[m].size (); ++i)
    {
      Eigen::Vector2f tmp_VT;
      for (std::size_t j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        std::size_t idx = tex_mesh.tex_polygons[m][i].vertices[j];
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
    tex_material_.tex_name = "material_" + std::to_string(m);
    tex_material_.tex_file = tex_files_[m];
    tex_mesh.tex_materials.push_back (tex_material_);

    // texture coordinates
    tex_mesh.tex_coordinates.push_back (texture_map_tmp);
  }// end meshes
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::mapMultipleTexturesToMeshUV (pcl::TextureMesh &tex_mesh, pcl::texture_mapping::CameraVector &cams)
{

  if (tex_mesh.tex_polygons.size () != cams.size () + 1)
  {
    PCL_ERROR ("The mesh should be divided into nbCamera+1 sub-meshes.\n");
    PCL_ERROR ("You provided %d cameras and a mesh containing %d sub-meshes.\n", cams.size (), tex_mesh.tex_polygons.size ());
    return;
  }

  PCL_INFO ("You provided %d  cameras and a mesh containing %d sub-meshes.\n", cams.size (), tex_mesh.tex_polygons.size ());

  typename pcl::PointCloud<PointInT>::Ptr originalCloud (new pcl::PointCloud<PointInT>);
  typename pcl::PointCloud<PointInT>::Ptr camera_transformed_cloud (new pcl::PointCloud<PointInT>);

  // convert mesh's cloud to pcl format for ease
  pcl::fromPCLPointCloud2 (tex_mesh.cloud, *originalCloud);

  for (std::size_t m = 0; m < cams.size (); ++m)
  {
    // get current camera parameters
    Camera current_cam = cams[m];

    // get camera transform
    Eigen::Affine3f cam_trans = current_cam.pose;

    // transform cloud into current camera frame
    pcl::transformPointCloud (*originalCloud, *camera_transformed_cloud, cam_trans.inverse ());

    // vector of texture coordinates for each face
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > texture_map_tmp;

    // processing each face visible by this camera
    for (const auto &tex_polygon : tex_mesh.tex_polygons[m])
    {
      Eigen::Vector2f tmp_VT;
      // for each point of this face
      for (const auto &vertex : tex_polygon.vertices)
      {
        // get point
        PointInT pt = (*camera_transformed_cloud)[vertex];

        // compute UV coordinates for this point
        getPointUVCoordinates (pt, current_cam, tmp_VT);
        texture_map_tmp.push_back (tmp_VT);
      }// end points
    }// end faces

    // texture materials
    tex_material_.tex_name = "material_" + std::to_string(m);
    tex_material_.tex_file = current_cam.texture_file;
    tex_mesh.tex_materials.push_back (tex_material_);

    // texture coordinates
    tex_mesh.tex_coordinates.push_back (texture_map_tmp);
  }// end cameras

  // push on extra empty UV map (for unseen faces) so that obj writer does not crash!
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > texture_map_tmp;
  for (const auto &tex_polygon : tex_mesh.tex_polygons[cams.size ()])
    for (std::size_t j = 0; j < tex_polygon.vertices.size (); ++j)
    {
      Eigen::Vector2f tmp_VT;
      tmp_VT[0] = -1;
      tmp_VT[1] = -1;
      texture_map_tmp.push_back (tmp_VT);
    }

  tex_mesh.tex_coordinates.push_back (texture_map_tmp);

  // push on an extra dummy material for the same reason
  tex_material_.tex_name = "material_" + std::to_string(cams.size());
  tex_material_.tex_file = "occluded.jpg";
  tex_mesh.tex_materials.push_back (tex_material_);
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> bool
pcl::TextureMapping<PointInT>::isPointOccluded (const PointInT &pt, OctreePtr octree)
{
  Eigen::Vector3f direction;
  direction (0) = pt.x;
  direction (1) = pt.y;
  direction (2) = pt.z;

  pcl::Indices indices;

  PointCloudConstPtr cloud (new PointCloud());
  cloud = octree->getInputCloud();

  double distance_threshold = octree->getResolution();

  // raytrace
  octree->getIntersectedVoxelIndices(direction, -direction, indices);

  int nbocc = static_cast<int> (indices.size ());
  for (const auto &index : indices)
  {
   // if intersected point is on the over side of the camera
   if (pt.z * (*cloud)[index].z < 0)
   {
     nbocc--;
     continue;
   }

   if (std::fabs ((*cloud)[index].z - pt.z) <= distance_threshold)
   {
     // points are very close to each-other, we do not consider the occlusion
     nbocc--;
   }
  }

  return (nbocc != 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::removeOccludedPoints (const PointCloudPtr &input_cloud,
                                                     PointCloudPtr &filtered_cloud,
                                                     const double octree_voxel_size, pcl::Indices &visible_indices,
                                                     pcl::Indices &occluded_indices)
{
  // variable used to filter occluded points by depth
  double maxDeltaZ = octree_voxel_size;

  // create an octree to perform rayTracing
  Octree octree (octree_voxel_size);
  // create octree structure
  octree.setInputCloud (input_cloud);
  // update bounding box automatically
  octree.defineBoundingBox ();
  // add points in the tree
  octree.addPointsFromInputCloud ();

  visible_indices.clear ();

  // for each point of the cloud, raycast toward camera and check intersected voxels.
  Eigen::Vector3f direction;
  pcl::Indices indices;
  for (std::size_t i = 0; i < input_cloud->size (); ++i)
  {
    direction (0) = (*input_cloud)[i].x;
    direction (1) = (*input_cloud)[i].y;
    direction (2) = (*input_cloud)[i].z;

    // if point is not occluded
    octree.getIntersectedVoxelIndices (direction, -direction, indices);

    int nbocc = static_cast<int> (indices.size ());
    for (const auto &index : indices)
    {
      // if intersected point is on the over side of the camera
      if ((*input_cloud)[i].z * (*input_cloud)[index].z < 0)
      {
        nbocc--;
        continue;
      }

      if (std::fabs ((*input_cloud)[index].z - (*input_cloud)[i].z) <= maxDeltaZ)
      {
        // points are very close to each-other, we do not consider the occlusion
        nbocc--;
      }
    }

    if (nbocc == 0)
    {
      // point is added in the filtered mesh
      filtered_cloud->points.push_back ((*input_cloud)[i]);
      visible_indices.push_back (static_cast<pcl::index_t> (i));
    }
    else
    {
      occluded_indices.push_back (static_cast<pcl::index_t> (i));
    }
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::removeOccludedPoints (const pcl::TextureMesh &tex_mesh, pcl::TextureMesh &cleaned_mesh, const double octree_voxel_size)
{
  // copy mesh
  cleaned_mesh = tex_mesh;

  typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT>);
  typename pcl::PointCloud<PointInT>::Ptr filtered_cloud (new pcl::PointCloud<PointInT>);

  // load points into a PCL format
  pcl::fromPCLPointCloud2 (tex_mesh.cloud, *cloud);

  pcl::Indices visible, occluded;
  removeOccludedPoints (cloud, filtered_cloud, octree_voxel_size, visible, occluded);

  // Now that we know which points are visible, let's iterate over each face.
  // if the face has one invisible point => out!
  for (std::size_t polygons = 0; polygons < cleaned_mesh.tex_polygons.size (); ++polygons)
  {
    // remove all faces from cleaned mesh
    cleaned_mesh.tex_polygons[polygons].clear ();
    // iterate over faces
    for (std::size_t faces = 0; faces < tex_mesh.tex_polygons[polygons].size (); ++faces)
    {
      // check if all the face's points are visible
      bool faceIsVisible = true;

      // iterate over face's vertex
      for (const auto &vertex : tex_mesh.tex_polygons[polygons][faces].vertices)
      {
        if (find (occluded.begin (), occluded.end (), vertex) == occluded.end ())
        {
          // point is not in the occluded vector
          // PCL_INFO ("  VISIBLE!\n");
        }
        else
        {
          // point was occluded
          // PCL_INFO("  OCCLUDED!\n");
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
pcl::TextureMapping<PointInT>::removeOccludedPoints (const pcl::TextureMesh &tex_mesh, PointCloudPtr &filtered_cloud,
                      const double octree_voxel_size)
{
  PointCloudPtr cloud (new PointCloud);

  // load points into a PCL format
  pcl::fromPCLPointCloud2 (tex_mesh.cloud, *cloud);

  pcl::Indices visible, occluded;
  removeOccludedPoints (cloud, filtered_cloud, octree_voxel_size, visible, occluded);

}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> int
pcl::TextureMapping<PointInT>::sortFacesByCamera (pcl::TextureMesh &tex_mesh, pcl::TextureMesh &sorted_mesh,
                                                  const pcl::texture_mapping::CameraVector &cameras, const double octree_voxel_size,
                                                  PointCloud &visible_pts)
{
  if (tex_mesh.tex_polygons.size () != 1)
  {
    PCL_ERROR ("The mesh must contain only 1 sub-mesh!\n");
    return (-1);
  }

  if (cameras.empty ())
  {
    PCL_ERROR ("Must provide at least one camera info!\n");
    return (-1);
  }

  // copy mesh
  sorted_mesh = tex_mesh;
  // clear polygons from cleaned_mesh
  sorted_mesh.tex_polygons.clear ();

  typename pcl::PointCloud<PointInT>::Ptr original_cloud (new pcl::PointCloud<PointInT>);
  typename pcl::PointCloud<PointInT>::Ptr transformed_cloud (new pcl::PointCloud<PointInT>);
  typename pcl::PointCloud<PointInT>::Ptr filtered_cloud (new pcl::PointCloud<PointInT>);

  // load points into a PCL format
  pcl::fromPCLPointCloud2 (tex_mesh.cloud, *original_cloud);

  // for each camera
  for (const auto &camera : cameras)
  {
    // get camera pose as transform
    Eigen::Affine3f cam_trans = camera.pose;

    // transform original cloud in camera coordinates
    pcl::transformPointCloud (*original_cloud, *transformed_cloud, cam_trans.inverse ());

    // find occlusions on transformed cloud
    pcl::Indices visible, occluded;
    removeOccludedPoints (transformed_cloud, filtered_cloud, octree_voxel_size, visible, occluded);
    visible_pts = *filtered_cloud;

    // pushing occluded idxs into a set for faster lookup
    std::unordered_set<index_t> occluded_set(occluded.cbegin(), occluded.cend());

    // find visible faces => add them to polygon N for camera N
    // add polygon group for current camera in clean
    std::vector<pcl::Vertices> visibleFaces_currentCam;
    // iterate over the faces of the current mesh
    for (std::size_t faces = 0; faces < tex_mesh.tex_polygons[0].size (); ++faces)
    {
      // check if all the face's points are visible
      // iterate over face's vertex
      const auto faceIsVisible = std::all_of(tex_mesh.tex_polygons[0][faces].vertices.cbegin(),
                                             tex_mesh.tex_polygons[0][faces].vertices.cend(),
                                             [&](const auto& vertex)
      {
          if (occluded_set.find(vertex) != occluded_set.cend()) {
            return false;  // point is occluded
          }
          // is the point visible to the camera?
          Eigen::Vector2f dummy_UV;
          return this->getPointUVCoordinates ((*transformed_cloud)[vertex], camera, dummy_UV);
      });

      if (faceIsVisible)
      {
        // push current visible face into the sorted mesh
        visibleFaces_currentCam.push_back (tex_mesh.tex_polygons[0][faces]);
        // remove it from the unsorted mesh
        tex_mesh.tex_polygons[0].erase (tex_mesh.tex_polygons[0].begin () + faces);
        faces--;
      }

    }
    sorted_mesh.tex_polygons.push_back (visibleFaces_currentCam);
  }

  // we should only have occluded and non-visible faces left in tex_mesh.tex_polygons[0]
  // we need to add them as an extra polygon in the sorted mesh
  sorted_mesh.tex_polygons.push_back (tex_mesh.tex_polygons[0]);
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::showOcclusions (const PointCloudPtr &input_cloud,
                                               pcl::PointCloud<pcl::PointXYZI>::Ptr &colored_cloud,
                                               const double octree_voxel_size, const bool show_nb_occlusions,
                                               const int max_occlusions)
                                               {
  // variable used to filter occluded points by depth
  double maxDeltaZ = octree_voxel_size * 2.0;

  // create an octree to perform rayTracing
  Octree octree (octree_voxel_size);
  // create octree structure
  octree.setInputCloud (input_cloud);
  // update bounding box automatically
  octree.defineBoundingBox ();
  // add points in the tree
  octree.addPointsFromInputCloud ();

  // ray direction
  Eigen::Vector3f direction;

  pcl::Indices indices;
  // point from where we ray-trace
  pcl::PointXYZI pt;

  std::vector<double> zDist;
  std::vector<double> ptDist;
  // for each point of the cloud, ray-trace toward the camera and check intersected voxels.
  for (const auto& point: *input_cloud)
  {
    direction = pt.getVector3fMap() = point.getVector3fMap();

    // get number of occlusions for that point
    indices.clear ();
    int nbocc = octree.getIntersectedVoxelIndices (direction, -direction, indices);

    nbocc = static_cast<int> (indices.size ());

    // TODO need to clean this up and find tricks to get remove aliasaing effect on planes
    for (const auto &index : indices)
    {
      // if intersected point is on the over side of the camera
      if (pt.z * (*input_cloud)[index].z < 0)
      {
        nbocc--;
      }
      else if (std::fabs ((*input_cloud)[index].z - pt.z) <= maxDeltaZ)
      {
        // points are very close to each-other, we do not consider the occlusion
        nbocc--;
      }
      else
      {
        zDist.push_back (std::fabs ((*input_cloud)[index].z - pt.z));
        ptDist.push_back (pcl::euclideanDistance ((*input_cloud)[index], pt));
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
  // load points into a PCL format
  typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT>);
  pcl::fromPCLPointCloud2 (tex_mesh.cloud, *cloud);

  showOcclusions (cloud, colored_cloud, octree_voxel_size, show_nb_occlusions, max_occlusions);
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> void
pcl::TextureMapping<PointInT>::textureMeshwithMultipleCameras (pcl::TextureMesh &mesh, const pcl::texture_mapping::CameraVector &cameras)
{

  if (mesh.tex_polygons.size () != 1)
    return;

  typename pcl::PointCloud<PointInT>::Ptr mesh_cloud (new pcl::PointCloud<PointInT>);

  pcl::fromPCLPointCloud2 (mesh.cloud, *mesh_cloud);

  std::vector<pcl::Vertices> faces;

  for (int current_cam = 0; current_cam < static_cast<int> (cameras.size ()); ++current_cam)
  {
    PCL_INFO ("Processing camera %d of %d.\n", current_cam+1, cameras.size ());
    
    // transform mesh into camera's frame
    typename pcl::PointCloud<PointInT>::Ptr camera_cloud (new pcl::PointCloud<PointInT>);
    pcl::transformPointCloud (*mesh_cloud, *camera_cloud, cameras[current_cam].pose.inverse ());

    // CREATE UV MAP FOR CURRENT FACES
    pcl::PointCloud<pcl::PointXY>::Ptr projections (new pcl::PointCloud<pcl::PointXY>);
    std::vector<bool> visibility;
    visibility.resize (mesh.tex_polygons[current_cam].size ());
    std::vector<UvIndex> indexes_uv_to_points;
    // for each current face

    //TODO change this
    pcl::PointXY nan_point;
    nan_point.x = std::numeric_limits<float>::quiet_NaN ();
    nan_point.y = std::numeric_limits<float>::quiet_NaN ();
    UvIndex u_null;
    u_null.idx_cloud = -1;
    u_null.idx_face = -1;

    int cpt_invisible=0;
    for (int idx_face = 0; idx_face <  static_cast<int> (mesh.tex_polygons[current_cam].size ()); ++idx_face)
    {
      //project each vertice, if one is out of view, stop
      pcl::PointXY uv_coord1;
      pcl::PointXY uv_coord2;
      pcl::PointXY uv_coord3;

      if (isFaceProjected (cameras[current_cam],
                           (*camera_cloud)[mesh.tex_polygons[current_cam][idx_face].vertices[0]],
                           (*camera_cloud)[mesh.tex_polygons[current_cam][idx_face].vertices[1]],
                           (*camera_cloud)[mesh.tex_polygons[current_cam][idx_face].vertices[2]],
                           uv_coord1,
                           uv_coord2,
                           uv_coord3))
       {
        // face is in the camera's FOV

        // add UV coordinates
        projections->points.push_back (uv_coord1);
        projections->points.push_back (uv_coord2);
        projections->points.push_back (uv_coord3);

        // remember corresponding face
        UvIndex u1, u2, u3;
        u1.idx_cloud = mesh.tex_polygons[current_cam][idx_face].vertices[0];
        u2.idx_cloud = mesh.tex_polygons[current_cam][idx_face].vertices[1];
        u3.idx_cloud = mesh.tex_polygons[current_cam][idx_face].vertices[2];
        u1.idx_face = idx_face; u2.idx_face = idx_face; u3.idx_face = idx_face;
        indexes_uv_to_points.push_back (u1);
        indexes_uv_to_points.push_back (u2);
        indexes_uv_to_points.push_back (u3);

        //keep track of visibility
        visibility[idx_face] = true;
      }
      else
      {
        projections->points.push_back (nan_point);
        projections->points.push_back (nan_point);
        projections->points.push_back (nan_point);
        indexes_uv_to_points.push_back (u_null);
        indexes_uv_to_points.push_back (u_null);
        indexes_uv_to_points.push_back (u_null);
        //keep track of visibility
        visibility[idx_face] = false;
        cpt_invisible++;
      }
    }

    // projections contains all UV points of the current faces
    // indexes_uv_to_points links a uv point to its point in the camera cloud
    // visibility contains tells if a face was in the camera FOV (false = skip)

    // TODO handle case were no face could be projected
    if (visibility.size () - cpt_invisible !=0)
    {
        //create kdtree
        pcl::KdTreeFLANN<pcl::PointXY> kdtree;
        kdtree.setInputCloud (projections);

        pcl::Indices idxNeighbors;
        std::vector<float> neighborsSquaredDistance;
        // af first (idx_pcan < current_cam), check if some of the faces attached to previous cameras occlude the current faces
        // then (idx_pcam == current_cam), check for self occlusions. At this stage, we skip faces that were already marked as occluded
        cpt_invisible = 0;
        for (int idx_pcam = 0 ; idx_pcam <= current_cam ; ++idx_pcam)
        {
          // project all faces
          for (int idx_face = 0; idx_face <  static_cast<int> (mesh.tex_polygons[idx_pcam].size ()); ++idx_face)
          {

            if (idx_pcam == current_cam && !visibility[idx_face])
            {
              // we are now checking for self occlusions within the current faces
              // the current face was already declared as occluded.
              // therefore, it cannot occlude another face anymore => we skip it
              continue;
            }

            // project each vertice, if one is out of view, stop
            pcl::PointXY uv_coord1;
            pcl::PointXY uv_coord2;
            pcl::PointXY uv_coord3;

            if (isFaceProjected (cameras[current_cam],
                                 (*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[0]],
                                 (*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[1]],
                                 (*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[2]],
                                 uv_coord1,
                                 uv_coord2,
                                 uv_coord3))
             {
              // face is in the camera's FOV
              //get its circumsribed circle
              double radius;
              pcl::PointXY center;
              // getTriangleCircumcenterAndSize (uv_coord1, uv_coord2, uv_coord3, center, radius);
              getTriangleCircumcscribedCircleCentroid(uv_coord1, uv_coord2, uv_coord3, center, radius); // this function yields faster results than getTriangleCircumcenterAndSize

              // get points inside circ.circle
              if (kdtree.radiusSearch (center, radius, idxNeighbors, neighborsSquaredDistance) > 0 )
              {
                // for each neighbor
                for (const auto &idxNeighbor : idxNeighbors)
                {
                  if (std::max ((*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[0]].z,
                                std::max ((*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[1]].z, 
                                          (*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[2]].z))
                     < (*camera_cloud)[indexes_uv_to_points[idxNeighbor].idx_cloud].z)
                  {
                    // neighbor is farther than all the face's points. Check if it falls into the triangle
                    if (checkPointInsideTriangle(uv_coord1, uv_coord2, uv_coord3, (*projections)[idxNeighbor]))
                    {
                      // current neighbor is inside triangle and is closer => the corresponding face
                      visibility[indexes_uv_to_points[idxNeighbor].idx_face] = false;
                      cpt_invisible++;
                      //TODO we could remove the projections of this face from the kd-tree cloud, but I fond it slower, and I need the point to keep ordered to querry UV coordinates later
                    }
                  }
                }
              }
             }
          }
        }
    }

    // now, visibility is true for each face that belongs to the current camera
    // if a face is not visible, we push it into the next one.

    if (static_cast<int> (mesh.tex_coordinates.size ()) <= current_cam)
    {
      std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > dummy_container;
      mesh.tex_coordinates.push_back (dummy_container);
    }
    mesh.tex_coordinates[current_cam].resize (3 * visibility.size ());

    std::vector<pcl::Vertices> occluded_faces;
    occluded_faces.resize (visibility.size ());
    std::vector<pcl::Vertices> visible_faces;
    visible_faces.resize (visibility.size ());

    int cpt_occluded_faces = 0;
    int cpt_visible_faces = 0;

    for (std::size_t idx_face = 0 ; idx_face < visibility.size () ; ++idx_face)
    {
      if (visibility[idx_face])
      {
        // face is visible by the current camera copy UV coordinates
        mesh.tex_coordinates[current_cam][cpt_visible_faces * 3](0) = (*projections)[idx_face*3].x;
        mesh.tex_coordinates[current_cam][cpt_visible_faces * 3](1) = (*projections)[idx_face*3].y;

        mesh.tex_coordinates[current_cam][cpt_visible_faces * 3 + 1](0) = (*projections)[idx_face*3 + 1].x;
        mesh.tex_coordinates[current_cam][cpt_visible_faces * 3 + 1](1) = (*projections)[idx_face*3 + 1].y;

        mesh.tex_coordinates[current_cam][cpt_visible_faces * 3 + 2](0) = (*projections)[idx_face*3 + 2].x;
        mesh.tex_coordinates[current_cam][cpt_visible_faces * 3 + 2](1) = (*projections)[idx_face*3 + 2].y;

        visible_faces[cpt_visible_faces] = mesh.tex_polygons[current_cam][idx_face];

        cpt_visible_faces++;
      }
      else
      {
        // face is occluded copy face into temp vector
        occluded_faces[cpt_occluded_faces] = mesh.tex_polygons[current_cam][idx_face];
        cpt_occluded_faces++;
      }
    }
    mesh.tex_coordinates[current_cam].resize (cpt_visible_faces*3);

    occluded_faces.resize (cpt_occluded_faces);
    mesh.tex_polygons.push_back (occluded_faces);

    visible_faces.resize (cpt_visible_faces);
    mesh.tex_polygons[current_cam].clear ();
    mesh.tex_polygons[current_cam] = visible_faces;
  }

  // we have been through all the cameras.
  // if any faces are left, they were not visible by any camera
  // we still need to produce uv coordinates for them

  if (mesh.tex_coordinates.size() <= cameras.size ())
  {
   std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > dummy_container;
   mesh.tex_coordinates.push_back(dummy_container);
   }


  for(std::size_t idx_face = 0 ; idx_face < mesh.tex_polygons[cameras.size()].size() ; ++idx_face)
  {
    Eigen::Vector2f UV1, UV2, UV3;
    UV1(0) = -1.0; UV1(1) = -1.0;
    UV2(0) = -1.0; UV2(1) = -1.0;
    UV3(0) = -1.0; UV3(1) = -1.0;
    mesh.tex_coordinates[cameras.size()].push_back(UV1);
    mesh.tex_coordinates[cameras.size()].push_back(UV2);
    mesh.tex_coordinates[cameras.size()].push_back(UV3);
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> inline void
pcl::TextureMapping<PointInT>::getTriangleCircumcenterAndSize(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circomcenter, double &radius)
{
  // we simplify the problem by translating the triangle's origin to its first point
  pcl::PointXY ptB, ptC;
  ptB.x = p2.x - p1.x; ptB.y = p2.y - p1.y; // B'=B-A
  ptC.x = p3.x - p1.x; ptC.y = p3.y - p1.y; // C'=C-A

  double D = 2.0*(ptB.x*ptC.y - ptB.y*ptC.x); // D'=2(B'x*C'y - B'y*C'x)

  // Safety check to avoid division by zero
  if(D == 0)
  {
    circomcenter.x = p1.x;
    circomcenter.y = p1.y;
  }
  else
  {
    // compute squares once
    double bx2 = ptB.x * ptB.x; // B'x^2
    double by2 = ptB.y * ptB.y; // B'y^2
    double cx2 = ptC.x * ptC.x; // C'x^2
    double cy2 = ptC.y * ptC.y; // C'y^2

    // compute circomcenter's coordinates (translate back to original coordinates)
    circomcenter.x = static_cast<float> (p1.x + (ptC.y*(bx2 + by2) - ptB.y*(cx2 + cy2)) / D);
    circomcenter.y = static_cast<float> (p1.y + (ptB.x*(cx2 + cy2) - ptC.x*(bx2 + by2)) / D);
  }

  radius = std::sqrt( (circomcenter.x - p1.x)*(circomcenter.x - p1.x)  + (circomcenter.y - p1.y)*(circomcenter.y - p1.y));//2.0* (p1.x*(p2.y - p3.y)  + p2.x*(p3.y - p1.y) + p3.x*(p1.y - p2.y));
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> inline void
pcl::TextureMapping<PointInT>::getTriangleCircumcscribedCircleCentroid ( const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius)
{
  // compute centroid's coordinates (translate back to original coordinates)
  circumcenter.x = static_cast<float> (p1.x + p2.x + p3.x ) / 3;
  circumcenter.y = static_cast<float> (p1.y + p2.y + p3.y ) / 3;
  double r1 = (circumcenter.x - p1.x) * (circumcenter.x - p1.x) + (circumcenter.y - p1.y) * (circumcenter.y - p1.y)  ;
  double r2 = (circumcenter.x - p2.x) * (circumcenter.x - p2.x) + (circumcenter.y - p2.y) * (circumcenter.y - p2.y)  ;
  double r3 = (circumcenter.x - p3.x) * (circumcenter.x - p3.x) + (circumcenter.y - p3.y) * (circumcenter.y - p3.y)  ;

  // radius
  radius = std::sqrt( std::max( r1, std::max( r2, r3) )) ;
}


///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> inline bool
pcl::TextureMapping<PointInT>::getPointUVCoordinates(const PointInT &pt, const Camera &cam, pcl::PointXY &UV_coordinates)
{
  if (pt.z > 0)
  {
    // compute image center and dimension
    double sizeX = cam.width;
    double sizeY = cam.height;
    double cx, cy;
    if (cam.center_w > 0)
      cx = cam.center_w;
    else
      cx = sizeX / 2.0;
    if (cam.center_h > 0)
      cy = cam.center_h;
    else
      cy = sizeY / 2.0;

    double focal_x, focal_y; 
    if (cam.focal_length_w > 0)
      focal_x = cam.focal_length_w;
    else
      focal_x = cam.focal_length;
    if (cam.focal_length_h > 0)
      focal_y = cam.focal_length_h;
    else
      focal_y = cam.focal_length;

    // project point on camera's image plane
    UV_coordinates.x = static_cast<float> ((focal_x * (pt.x / pt.z) + cx) / sizeX); //horizontal
    UV_coordinates.y = 1.0f - static_cast<float> ((focal_y * (pt.y / pt.z) + cy) / sizeY); //vertical

    // point is visible!
    if (UV_coordinates.x >= 0.0 && UV_coordinates.x <= 1.0 && UV_coordinates.y >= 0.0 && UV_coordinates.y <= 1.0)
      return (true); // point was visible by the camera
  }

  // point is NOT visible by the camera
  UV_coordinates.x = -1.0f;
  UV_coordinates.y = -1.0f;
  return (false); // point was not visible by the camera
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> inline bool
pcl::TextureMapping<PointInT>::checkPointInsideTriangle(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, const pcl::PointXY &pt)
{
   // Compute vectors
   Eigen::Vector2d v0, v1, v2;
   v0(0) = p3.x - p1.x; v0(1) = p3.y - p1.y; // v0= C - A
   v1(0) = p2.x - p1.x; v1(1) = p2.y - p1.y; // v1= B - A
   v2(0) = pt.x - p1.x; v2(1) = pt.y - p1.y; // v2= P - A

   // Compute dot products
   double dot00 = v0.dot(v0); // dot00 = dot(v0, v0)
   double dot01 = v0.dot(v1); // dot01 = dot(v0, v1)
   double dot02 = v0.dot(v2); // dot02 = dot(v0, v2)
   double dot11 = v1.dot(v1); // dot11 = dot(v1, v1)
   double dot12 = v1.dot(v2); // dot12 = dot(v1, v2)

   // Compute barycentric coordinates
   double invDenom = 1.0 / (dot00*dot11 - dot01*dot01);
   double u = (dot11*dot02 - dot01*dot12) * invDenom;
   double v = (dot00*dot12 - dot01*dot02) * invDenom;

   // Check if point is in triangle
   return ((u >= 0) && (v >= 0) && (u + v < 1));
}

///////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT> inline bool
pcl::TextureMapping<PointInT>::isFaceProjected (const Camera &camera, const PointInT &p1, const PointInT &p2, const PointInT &p3, pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3)
{
  return (getPointUVCoordinates(p1, camera, proj1)
      &&
      getPointUVCoordinates(p2, camera, proj2)
      &&
      getPointUVCoordinates(p3, camera, proj3)
  );
}

#define PCL_INSTANTIATE_TextureMapping(T)                \
    template class PCL_EXPORTS pcl::TextureMapping<T>;

#endif /* TEXTURE_MAPPING_HPP_ */
