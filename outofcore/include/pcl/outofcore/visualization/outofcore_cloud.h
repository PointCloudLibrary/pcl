#ifndef PCL_OUTOFCORE_OUTOFCORE_CLOUD_H_
#define PCL_OUTOFCORE_OUTOFCORE_CLOUD_H_

// PCL
//#include <pcl/common/time.h>
//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL - outofcore
#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

// VTK
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkAppendPolyData.h>
#include <vtkDataSetMapper.h>
//#include <vtkCamera.h>
//#include <vtkCameraActor.h>
//#include <vtkHull.h>
//#include <vtkPlanes.h>
#include <vtkPolyData.h>
//#include <vtkPolyDataMapper.h>
//#include <vtkProperty.h>
#include <vtkSmartPointer.h>

class OutofcoreCloud : public Object
{

  // Typedefs
  // -----------------------------------------------------------------------------
  typedef pcl::PointXYZ PointT;
  typedef pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointT>, PointT> octree_disk;
  typedef pcl::outofcore::OutofcoreOctreeBaseNode<pcl::outofcore::OutofcoreOctreeDiskContainer<PointT>, PointT> octree_disk_node;
  typedef boost::shared_ptr<octree_disk> OctreeDiskPtr;
  typedef Eigen::aligned_allocator<PointT> AlignedPointT;

  typedef std::map<std::string, vtkSmartPointer<vtkActor> > CloudActorMap;

public:

  // Operators
  // -----------------------------------------------------------------------------
  OutofcoreCloud (std::string name, boost::filesystem::path& tree_root);

  // Methods
  // -----------------------------------------------------------------------------
  void
  updateVoxelData ();
  void
  updateCloudData ();

  void
  updateView (double frustum[24], const Eigen::Vector3d &eye, const Eigen::Matrix4d &view_projection_matrix);

  // Accessors
  // -----------------------------------------------------------------------------
  OctreeDiskPtr
  getOctree ()
  {
    return octree_;
  }

  inline vtkSmartPointer<vtkActor>
  getVoxelActor () const
  {
    return voxel_actor_;
  }

  inline vtkSmartPointer<vtkActorCollection>
  getCloudActors () const
  {
    return cloud_actors_;
  }

  void
  setDisplayDepth (int displayDepth)
  {
    if (displayDepth < 0)
    {
      displayDepth = 0;
    }
    else if (displayDepth > octree_->getDepth ())
    {
      displayDepth = octree_->getDepth ();
    }

    if (display_depth_ != displayDepth)
    {
      display_depth_ = displayDepth;
      updateVoxelData ();
      //updateCloudData();
    }
  }

  int
  getDisplayDepth ()
  {
    return display_depth_;
  }

  uint64_t
  getPointsLoaded ()
  {
    return points_loaded_;
  }

  Eigen::Vector3d
  getBoundingBoxMin ()
  {
    return bbox_min_;
  }

  Eigen::Vector3d
  getBoundingBoxMax ()
  {
    return bbox_max_;
  }

  void
  setDisplayVoxels (bool display_voxels)
  {
    voxel_actor_->SetVisibility (display_voxels);
  }

  bool
  setFrustum (double *frustum)
  {
    if (!frustum_)
      frustum_ = new double[24];

    bool frustum_changed = false;
    for (int i = 0; i < 24; i++)
    {
      if (frustum_[i] != frustum[i])
        frustum_changed = true;
      frustum_[i] = frustum[i];
    }

//    if (frustum_changed)
//        updateCloudData();

    return frustum_changed;
  }

  void
  setModelViewMatrix (const Eigen::Matrix4d &model_view_matrix)
  {
    model_view_matrix_ = model_view_matrix;
  }

  void
  setProjectionMatrix (const Eigen::Matrix4d &projection_matrix)
  {
    projection_matrix_ = projection_matrix;
  }

private:

  // Members
  // -----------------------------------------------------------------------------
  OctreeDiskPtr octree_;

  uint64_t display_depth_;
  uint64_t points_loaded_;

  Eigen::Vector3d bbox_min_, bbox_max_;

  double *frustum_;
  Eigen::Matrix4d model_view_matrix_;
  Eigen::Matrix4d projection_matrix_;

  vtkSmartPointer<vtkActor> voxel_actor_;

  std::map<std::string, vtkSmartPointer<vtkActor> > cloud_actors_map_;
  vtkSmartPointer<vtkActorCollection> cloud_actors_;

};

#endif
