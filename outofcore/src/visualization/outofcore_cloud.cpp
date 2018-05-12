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
#include <pcl/outofcore/impl/lru_cache.hpp>

#include <pcl/outofcore/visualization/camera.h>
#include <pcl/outofcore/visualization/common.h>
#include <pcl/outofcore/visualization/object.h>
#include <pcl/outofcore/visualization/scene.h>
#include <pcl/outofcore/visualization/outofcore_cloud.h>

// PCL - visualziation
#include <pcl/visualization/common/common.h>

#if VTK_RENDERING_BACKEND_OPENGL_VERSION < 2
#include <pcl/visualization/vtk/vtkVertexBufferObjectMapper.h>
#endif

// VTK
#include <vtkVersion.h>
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

// Boost
//#include <boost/date_time.hpp>
//#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

// Forward Declarations

boost::condition OutofcoreCloud::pcd_queue_ready;
boost::mutex OutofcoreCloud::pcd_queue_mutex;

boost::shared_ptr<boost::thread> OutofcoreCloud::pcd_reader_thread;
//MonitorQueue<std::string> OutofcoreCloud::pcd_queue;

//std::map<std::string, vtkSmartPointer<vtkPolyData> > OutofcoreCloud::cloud_data_cache;
OutofcoreCloud::CloudDataCache OutofcoreCloud::cloud_data_cache(524288);
boost::mutex OutofcoreCloud::cloud_data_cache_mutex;

OutofcoreCloud::PcdQueue OutofcoreCloud::pcd_queue;

void
OutofcoreCloud::pcdReaderThread ()
{

  std::string pcd_file;
  size_t timestamp = 0;

  while (true)
  {
    //{
      //boost::mutex::scoped_lock lock (pcd_queue_mutex);
      //pcd_queue_mutex.wait (lock);
      pcd_queue_ready.wait(pcd_queue_mutex);
    //}
    //pcd_queue_ready

    int queue_size = pcd_queue.size ();
    for (int i=0; i < queue_size; i++)
    {
      const PcdQueueItem *pcd_queue_item = &pcd_queue.top();

      if (cloud_data_cache.hasKey(pcd_queue_item->pcd_file))
      {
        CloudDataCacheItem *cloud_data_cache_item = &cloud_data_cache.get(pcd_queue_item->pcd_file);
        cloud_data_cache_item->timestamp = timestamp;
      }
      else
      {
        vtkSmartPointer<vtkPolyData> cloud_data = vtkSmartPointer<vtkPolyData>::New ();

        pcl::PCLPointCloud2Ptr cloud (new pcl::PCLPointCloud2);

        pcl::io::loadPCDFile (pcd_queue_item->pcd_file, *cloud);
        pcl::io::pointCloudTovtkPolyData (cloud, cloud_data);

        CloudDataCacheItem cloud_data_cache_item(pcd_queue_item->pcd_file, pcd_queue_item->coverage, cloud_data, timestamp);

        cloud_data_cache_mutex.lock();
        bool inserted = cloud_data_cache.insert(pcd_queue_item->pcd_file, cloud_data_cache_item);
        cloud_data_cache_mutex.unlock();

        if (!inserted)
        {
          //std::cout << "CACHE FILLED" << std::endl;
          while (!pcd_queue.empty())
          {
            //const PcdQueueItem *pcd_queue_item = &pcd_queue.top();
            //std::cout << "Skipping: " << pcd_queue_item->pcd_file <<std::endl;
            pcd_queue.pop();
          }
          break;
        }
        //std::cout << "Loaded: " << pcd_queue_item->pcd_file << std::endl;
      }

      pcd_queue.pop();
    }

    timestamp++;
  }
}


// Operators
// -----------------------------------------------------------------------------
OutofcoreCloud::OutofcoreCloud (std::string name, boost::filesystem::path& tree_root) :
    Object (name), display_depth_ (1), points_loaded_ (0), data_loaded_(0), render_camera_(NULL), lod_pixel_threshold_(10000)
{

  // Create the pcd reader thread once for all outofcore nodes
  if (OutofcoreCloud::pcd_reader_thread.get() == NULL)
  {
//    OutofcoreCloud::pcd_reader_thread = boost::shared_ptr<boost::thread>(new boost::thread(&OutofcoreCloud::pcdReaderThread, this));
    OutofcoreCloud::pcd_reader_thread = boost::shared_ptr<boost::thread>(new boost::thread(&OutofcoreCloud::pcdReaderThread));
  }


  octree_.reset (new OctreeDisk (tree_root, true));
  octree_->getBoundingBox (bbox_min_, bbox_max_);

  voxel_actor_ = vtkSmartPointer<vtkActor>::New ();

  updateVoxelData ();

  cloud_actors_ = vtkSmartPointer<vtkActorCollection>::New ();

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

#if VTK_MAJOR_VERSION < 6
    voxel_data->AddInput (getVtkCube (x - s, x + s, y - s, y + s, z - s, z + s));
#else
    voxel_data->AddInputData (getVtkCube (x - s, x + s, y - s, y + s, z - s, z + s));
#endif
  }

#if VTK_MAJOR_VERSION < 6
  voxel_mapper->SetInput (voxel_data->GetOutput ());
#else
  voxel_mapper->SetInputData (voxel_data->GetOutput ());
#endif

  voxel_actor_->SetMapper (voxel_mapper);
  voxel_actor_->GetProperty ()->SetRepresentationToWireframe ();
  voxel_actor_->GetProperty ()->SetColor (0.0, 1.0, 0.0);
  voxel_actor_->GetProperty ()->SetLighting (false);
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
      if (coverage <= lod_pixel_threshold_)
      {
        breadth_first_it.skipChildVoxels();
      }

//      for (int i=0; i < node->getDepth(); i++)
//        std::cout << " ";
//      std::cout << coverage << "-" << pcd_file << endl;//" : " << (coverage > (size[0] * size[1])) << endl;

      std::string pcd_file = node->getPCDFilename ().string ();

      cloud_data_cache_mutex.lock();

      PcdQueueItem pcd_queue_item(pcd_file, coverage);

      // If we can lock the queue add another item
      if (pcd_queue_mutex.try_lock())
      {
        pcd_queue.push(pcd_queue_item);
        pcd_queue_mutex.unlock();
      }

      if (cloud_data_cache.hasKey(pcd_file))
      {
        //std::cout << "Has Key for: " << pcd_file << std::endl;
        if (cloud_actors_map_.find (pcd_file) == cloud_actors_map_.end ())
        {

          vtkSmartPointer<vtkActor> cloud_actor = vtkSmartPointer<vtkActor>::New ();
          CloudDataCacheItem *cloud_data_cache_item = &cloud_data_cache.get(pcd_file);

#if VTK_RENDERING_BACKEND_OPENGL_VERSION < 2
          vtkSmartPointer<vtkVertexBufferObjectMapper> mapper = vtkSmartPointer<vtkVertexBufferObjectMapper>::New ();
          mapper->SetInput (cloud_data_cache_item->item);
#else
          vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
          // Usually we choose between SetInput and SetInputData based on VTK version. But OpenGL ≥ 2 automatically
          // means VTK version is ≥ 6.3
          mapper->SetInputData (cloud_data_cache_item->item);
#endif

          cloud_actor->SetMapper (mapper);
          cloud_actor->GetProperty ()->SetColor (0.0, 0.0, 1.0);
          cloud_actor->GetProperty ()->SetPointSize (1);
          cloud_actor->GetProperty ()->SetLighting (0);

          cloud_actors_map_[pcd_file] = cloud_actor;
        }

        if (!hasActor (cloud_actors_map_[pcd_file]))
        {
          points_loaded_ += cloud_actors_map_[pcd_file]->GetMapper ()->GetInput ()->GetNumberOfPoints ();
          data_loaded_ += cloud_actors_map_[pcd_file]->GetMapper ()->GetInput ()->GetActualMemorySize();
        }

        //std::cout << "Adding Actor: " << pcd_file << std::endl;

        cloud_actors_->AddItem (cloud_actors_map_[pcd_file]);
        addActor (cloud_actors_map_[pcd_file]);
      }

      cloud_data_cache_mutex.unlock();

      breadth_first_it++;
    }

    // We're done culling, notify the pcd_reader thread the queue is read
    pcd_queue_ready.notify_one();

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

    for (size_t i = 0; i < actors_to_remove.size (); i++)
    {
      points_loaded_ -= actors_to_remove.back ()->GetMapper ()->GetInput ()->GetNumberOfPoints ();
      data_loaded_ -= actors_to_remove.back ()->GetMapper ()->GetInput ()->GetActualMemorySize();
      removeActor (actors_to_remove.back ());
      actors_to_remove.pop_back ();
    }
  }

  Object::render(renderer);
}

