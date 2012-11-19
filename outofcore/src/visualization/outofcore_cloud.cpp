// PCL
//#include <pcl/common/time.h>
//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/pcl_macros.h>

// PCL - outofcore
#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>
#include <pcl/outofcore/impl/monitor_queue.hpp>

#include <pcl/outofcore/visualization/camera.h>
#include <pcl/outofcore/visualization/common.h>
#include <pcl/outofcore/visualization/object.h>
#include <pcl/outofcore/visualization/scene.h>
#include <pcl/outofcore/visualization/outofcore_cloud.h>

// PCL - visualziation
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/vtk/vtkVertexBufferObjectMapper.h>

// VTK
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

// Boost
//#include <boost/date_time.hpp>
//#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

// Forward Declarations

boost::shared_ptr<boost::thread> OutofcoreCloud::pcd_reader_thread;
MonitorQueue<std::string> OutofcoreCloud::pcd_queue;
std::map<std::string, vtkSmartPointer<vtkPolyData> > OutofcoreCloud::cloud_data_map;
boost::mutex OutofcoreCloud::cloud_data_map_mutex;

void
OutofcoreCloud::pcdReaderThread ()
{

//  Scene *scene = Scene::instance ();
//  Camera *octree_camera = scene->getCamera ("octree");
//  OutofcoreCloud *cloud = static_cast<OutofcoreCloud*> (scene->getObjectByName ("my_octree"));
//
  std::string pcd_file;

  while (true)
  {
    pcd_file = pcd_queue.pop();

    if (cloud_data_map.find (pcd_file) == cloud_data_map.end ())
    {
      vtkSmartPointer<vtkPolyData> cloud_data = vtkSmartPointer<vtkPolyData>::New ();

      sensor_msgs::PointCloud2Ptr cloud (new sensor_msgs::PointCloud2);
      pcl::io::loadPCDFile (pcd_file, *cloud);
      pcl::io::pointCloudTovtkPolyData (cloud, cloud_data);

      cloud_data_map_mutex.lock();
      cloud_data_map[pcd_file] = cloud_data;
      cloud_data_map_mutex.unlock();
      std::cout << "Loaded: " << pcd_file << std::endl;
    }




//    {
//      boost::mutex::scoped_lock lock (camera_changed_mutex);
//      camera_changed.wait (lock);
//    }
//
//    double frustum[24];
//    octree_camera->getFrustum (frustum);
//
//    Eigen::Vector3d eye = octree_camera->getPosition ();
//    Eigen::Matrix4d view_projection_matrix = octree_camera->getViewProjectionMatrix ();
//
//    //cloud->updateView (frustum, eye, view_projection_matrix);
//
//    window->Render ();
//
////    std::list<std::string> file_names;
////    cloud->getOctree()->queryFrustum(frustum, file_names, cloud->getDisplayDepth());
////    cout << "bins: " << file_names.size() << endl;
  }
}


// Operators
// -----------------------------------------------------------------------------
OutofcoreCloud::OutofcoreCloud (std::string name, boost::filesystem::path& tree_root) :
    Object (name), display_depth_ (1), points_loaded_ (0), render_camera_(NULL)//, frustum_ (NULL), model_view_matrix_ (
//        Eigen::Matrix4d::Identity ()), projection_matrix_ (Eigen::Matrix4d::Identity ())
{

  // Create the pcd reader thread once for all outofcore nodes
  if (OutofcoreCloud::pcd_reader_thread.get() == NULL)
  {
    OutofcoreCloud::pcd_reader_thread = boost::shared_ptr<boost::thread>(new boost::thread(OutofcoreCloud::pcdReaderThread));
  }


  octree_.reset (new OctreeDisk (tree_root, true));
  octree_->getBoundingBox (bbox_min_, bbox_max_);

  voxel_actor_ = vtkSmartPointer<vtkActor>::New ();

  updateVoxelData ();

//    cloud_data_ = vtkSmartPointer<vtkPolyData>::New ();
//    cloud_actor_ = vtkSmartPointer<vtkActor>::New ();
  cloud_actors_ = vtkSmartPointer<vtkActorCollection>::New ();

  //updateCloudData();

//    if (!cloud_actor_)
//      cout << "Not initialized" << endl;
//
//    if (!cloud_actor_.GetPointer())
//      cout << "GetPointer - Not initialized" << endl;
//
//    if (cloud_actor_.GetPointer())
//          cout << "GetPointer - Initialized" << endl;

//poly_data.GetPointer()

//pcl::io::pointCloudTovtkPolyData(const sensor_msgs::PointCloud2Ptr& cloud, vtkSmartPointer<vtkPolyData>& poly_data)
  addActor (voxel_actor_);
}

// Methods
// -----------------------------------------------------------------------------
void
OutofcoreCloud::updateVoxelData ()
{
  std::vector<PointT, AlignedPointT> voxel_centers;
  vtkSmartPointer<vtkAppendPolyData> voxel_data = vtkSmartPointer<vtkAppendPolyData>::New ();
  vtkSmartPointer<vtkDataSetMapper> voxel_mapper = vtkSmartPointer<vtkDataSetMapper>::New ();

  voxel_centers.clear ();
  voxel_data->RemoveAllInputs ();

  octree_->getOccupiedVoxelCenters (voxel_centers, display_depth_);
  double voxel_side_length = octree_->getVoxelSideLength (display_depth_);

  double s = voxel_side_length / 2;
  for (size_t i = 0; i < voxel_centers.size (); i++)
  {
    double x = voxel_centers[i].x;
    double y = voxel_centers[i].y;
    double z = voxel_centers[i].z;

    voxel_data->AddInput (getVtkCube (x - s, x + s, y - s, y + s, z - s, z + s));
  }

  voxel_mapper->SetInput (voxel_data->GetOutput ());

  voxel_actor_->SetMapper (voxel_mapper);
  voxel_actor_->GetProperty ()->SetRepresentationToWireframe ();
  voxel_actor_->GetProperty ()->SetColor (0.0, 1.0, 0.0);
  voxel_actor_->GetProperty ()->SetLighting (false);
}

void
OutofcoreCloud::updateView (double frustum[24], const Eigen::Vector3d &eye, const Eigen::Matrix4d &view_projection_matrix)
{

//void
//OutofcoreCloud::updateCloudData()
//{

//  std::cout << "updateCloud - Removing" << std::endl;
  //Remove existing cloud actors

  std::list<std::string> bins;
  cloud_actors_->RemoveAllItems ();

  //for(uint64_t depth = 0; depth <= display_depth_; depth++)
  //{
//  if(!frustum_)
//    return;

//  std::cout << "updateCloud - Querying" << std::endl;
  //octree_->queryBBIntersects(bbox_min_, bbox_max_, depth, bins);
  //octree_->queryFrustum(frustum, bins, display_depth_);
  octree_->queryFrustum (frustum, eye, view_projection_matrix, bins, display_depth_);

  //std::cout << "updateCloud - Adding [" << bins.size() << "]" << std::endl;

  std::list<std::string>::iterator it_bins;
  for (it_bins = bins.begin (); it_bins != bins.end (); it_bins++)
  {
    //std::string node_meta_data = *it_bins;

    pcl::outofcore::OutofcoreOctreeNodeMetadata node_meta_data;
    node_meta_data.loadMetadataFromDisk (*it_bins);

    std::string cloud_file = node_meta_data.getPCDFilename ().string ();

    if (cloud_actors_map_.find (cloud_file) == cloud_actors_map_.end ())
    {
      vtkSmartPointer<vtkPolyData> cloud_data = vtkSmartPointer<vtkPolyData>::New ();
      vtkSmartPointer<vtkActor> cloud_actor = vtkSmartPointer<vtkActor>::New ();
      vtkSmartPointer<vtkVertexBufferObjectMapper> mapper = vtkSmartPointer<vtkVertexBufferObjectMapper>::New ();

      sensor_msgs::PointCloud2Ptr cloud (new sensor_msgs::PointCloud2);
      pcl::io::loadPCDFile (cloud_file, *cloud);
      pcl::io::pointCloudTovtkPolyData (cloud, cloud_data);

      mapper->SetInput (cloud_data);
      cloud_actor->SetMapper (mapper);
      cloud_actor->GetProperty ()->SetColor (0.0, 0.0, 1.0);
      cloud_actor->GetProperty ()->SetPointSize (1);
      cloud_actor->GetProperty ()->SetLighting (0);

      cloud_actors_map_[cloud_file] = cloud_actor;
    }

    if (!hasActor (cloud_actors_map_[cloud_file]))
      points_loaded_ += cloud_actors_map_[cloud_file]->GetMapper ()->GetInput ()->GetNumberOfPoints ();

    cloud_actors_->AddItem (cloud_actors_map_[cloud_file]);
    addActor (cloud_actors_map_[cloud_file]);
  }

  //vtkSmartPointer<vtkActor> actors = getActors();
  std::vector<vtkActor*> actors_to_remove;
  {
    boost::mutex::scoped_lock lock (actors_mutex_);

    actors_to_remove.clear ();
    actors_->InitTraversal ();
    for (vtkIdType i = 0; i < actors_->GetNumberOfItems (); i++)
    {
      vtkActor* actor = actors_->GetNextActor ();
      if (actor != voxel_actor_.GetPointer ())
      {
        bool actor_found = false;
        cloud_actors_->InitTraversal ();
        for (vtkIdType j = 0; j < cloud_actors_->GetNumberOfItems (); j++)
        {
          vtkActor* cloud_actor = cloud_actors_->GetNextActor ();
          if (actor == cloud_actor)
          {
            actor_found = true;
            break;
          }
        }

        if (!actor_found)
        {
          actors_to_remove.push_back (actor);
        }
      }
    }
  }

  for (int i = 0; i < actors_to_remove.size (); i++)
  {
    points_loaded_ -= actors_to_remove.back ()->GetMapper ()->GetInput ()->GetNumberOfPoints ();
    removeActor (actors_to_remove.back ());
    actors_to_remove.pop_back ();
  }
}

void
OutofcoreCloud::render (vtkRenderer* renderer)
{
  vtkSmartPointer<vtkCamera> active_camera = renderer->GetActiveCamera ();

  Scene *scene = Scene::instance ();
  Camera *camera = scene->getCamera (active_camera);

  if (render_camera_ != NULL && render_camera_->getName() == camera->getName ())
  {
    renderer->ComputeAspect ();
    //double *aspect = renderer->GetAspect ();
    int *size = renderer->GetSize ();

//    Eigen::Matrix4d projection_matrix = pcl::visualization::vtkToEigen (
//        active_camera->GetProjectionTransformMatrix (aspect[0] / aspect[1], 0.0, 1.0));
//
//    Eigen::Matrix4d model_view_matrix = pcl::visualization::vtkToEigen (
//        active_camera->GetModelViewTransformMatrix ());

    OctreeDisk::BreadthFirstIterator breadth_first_it (*octree_);
    breadth_first_it.setMaxDepth(display_depth_);

    double frustum[24];
    camera->getFrustum(frustum);

    Eigen::Vector3d eye = camera->getPosition();
    Eigen::Matrix4d view_projection_matrix = camera->getViewProjectionMatrix();
    //Eigen::Matrix4d view_projection_matrix = projection_matrix * model_view_matrix;//camera->getViewProjectionMatrix();

    cloud_actors_->RemoveAllItems ();

    while ( *breadth_first_it !=0 )
    {
      OctreeDiskNode *node = *breadth_first_it;

      Eigen::Vector3d min_bb, max_bb;
      node->getBoundingBox(min_bb, max_bb);

      // Frustum culling
      if (pcl::visualization::cullFrustum(frustum, min_bb, max_bb) == pcl::visualization::PCL_OUTSIDE_FRUSTUM)
      {
        breadth_first_it.skipChildVoxels();
        breadth_first_it++;
        continue;
      }

      // Bounding box lod projection
      float coverage = pcl::visualization::viewScreenArea(eye, min_bb, max_bb, view_projection_matrix, size[0], size[1]);
      if (coverage <= 10000)
      {
        breadth_first_it.skipChildVoxels();
      }

      std::string pcd_file = node->getPCDFilename ().string ();

      cloud_data_map_mutex.lock();
      CloudDataCacheIterator cloud_data_it = cloud_data_map.find (pcd_file);

      if (cloud_data_it == cloud_data_map.end ())
      {
        pcd_queue.push(pcd_file);
      }
      else
      {
        if (cloud_actors_map_.find (pcd_file) == cloud_actors_map_.end ())
        {

          vtkSmartPointer<vtkActor> cloud_actor = vtkSmartPointer<vtkActor>::New ();
          vtkSmartPointer<vtkVertexBufferObjectMapper> mapper = vtkSmartPointer<vtkVertexBufferObjectMapper>::New ();

          mapper->SetInput (cloud_data_it->second);
          cloud_actor->SetMapper (mapper);
          cloud_actor->GetProperty ()->SetColor (0.0, 0.0, 1.0);
          cloud_actor->GetProperty ()->SetPointSize (1);
          cloud_actor->GetProperty ()->SetLighting (0);

          cloud_actors_map_[pcd_file] = cloud_actor;
        }

        if (!hasActor (cloud_actors_map_[pcd_file]))
          points_loaded_ += cloud_actors_map_[pcd_file]->GetMapper ()->GetInput ()->GetNumberOfPoints ();

        cloud_actors_->AddItem (cloud_actors_map_[pcd_file]);
        addActor (cloud_actors_map_[pcd_file]);
      }

      cloud_data_map_mutex.unlock();

      breadth_first_it++;
    }

    std::vector<vtkActor*> actors_to_remove;
    {
      actors_to_remove.clear ();
      actors_->InitTraversal ();
      for (vtkIdType i = 0; i < actors_->GetNumberOfItems (); i++)
      {
        vtkActor* actor = actors_->GetNextActor ();
        if (actor != voxel_actor_.GetPointer ())
        {
          bool actor_found = false;
          cloud_actors_->InitTraversal ();
          for (vtkIdType j = 0; j < cloud_actors_->GetNumberOfItems (); j++)
          {
            vtkActor* cloud_actor = cloud_actors_->GetNextActor ();
            if (actor == cloud_actor)
            {
              actor_found = true;
              break;
            }
          }

          if (!actor_found)
          {
            actors_to_remove.push_back (actor);
          }
        }
      }
    }

    for (int i = 0; i < actors_to_remove.size (); i++)
    {
      points_loaded_ -= actors_to_remove.back ()->GetMapper ()->GetInput ()->GetNumberOfPoints ();
      removeActor (actors_to_remove.back ());
      actors_to_remove.pop_back ();
    }
  }

  Object::render(renderer);
}

