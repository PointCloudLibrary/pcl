#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

// PCL
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

// PCL - outofcore
#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>
#include <pcl/outofcore/impl/lru_cache.hpp>

// PCL
#include "camera.h"

// VTK
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkAppendPolyData.h>
#include <vtkDataSetMapper.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

//class Camera;

class OutofcoreCloud : public Object
{
    // Typedefs
    // -----------------------------------------------------------------------------
    using PointT = pcl::PointXYZ;
//    using octree_disk = pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointT>, PointT>;
//    using octree_disk_node = pcl::outofcore::OutofcoreOctreeBaseNode<pcl::outofcore::OutofcoreOctreeDiskContainer<PointT>, PointT>;

    using OctreeDisk = pcl::outofcore::OutofcoreOctreeBase<>;
    using OctreeDiskNode = pcl::outofcore::OutofcoreOctreeBaseNode<>;
//    using OctreeBreadthFirstIterator = pcl::outofcore::OutofcoreBreadthFirstIterator<>;

    using OctreeDiskPtr = OctreeDisk::Ptr;
    using AlignedPointT = Eigen::aligned_allocator<PointT>;



    using CloudActorMap = std::map<std::string, vtkSmartPointer<vtkActor> >;

  public:

//    using CloudDataCache = std::map<std::string, vtkSmartPointer<vtkPolyData> >;
//    using CloudDataCacheIterator = std::map<std::string, vtkSmartPointer<vtkPolyData> >::iterator;


    static std::shared_ptr<std::thread> pcd_reader_thread;
    //static MonitorQueue<std::string> pcd_queue;

    struct PcdQueueItem
    {
      PcdQueueItem (std::string pcd_file, float coverage)
      {
       this->pcd_file = pcd_file;
       this->coverage = coverage;
      }

      bool operator< (const PcdQueueItem& rhs) const
      {
       return coverage < rhs.coverage;
      }

      std::string pcd_file;
      float coverage;
    };

    using PcdQueue = std::priority_queue<PcdQueueItem>;
    static PcdQueue pcd_queue;
    static std::mutex pcd_queue_mutex;
    static std::condition_variable pcd_queue_ready;

    class CloudDataCacheItem : public LRUCacheItem< vtkSmartPointer<vtkPolyData> >
    {
    public:

      CloudDataCacheItem (std::string pcd_file, float coverage, vtkSmartPointer<vtkPolyData> cloud_data, std::size_t timestamp)
      {
       this->pcd_file = pcd_file;
       this->coverage = coverage;
       this->item = cloud_data;
       this->timestamp = timestamp;
      }

      std::size_t
      sizeOf() const override
      {
        return item->GetActualMemorySize();
      }

      std::string pcd_file;
      float coverage;
    };


//    static CloudDataCache cloud_data_map;
//    static std::mutex cloud_data_map_mutex;
    using CloudDataCache = LRUCache<std::string, CloudDataCacheItem>;
    static CloudDataCache cloud_data_cache;
    static std::mutex cloud_data_cache_mutex;

    static void pcdReaderThread();

    // Operators
    // -----------------------------------------------------------------------------
    OutofcoreCloud (std::string name, boost::filesystem::path& tree_root);

    // Methods
    // -----------------------------------------------------------------------------
    void
    updateVoxelData ();

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
      else if (static_cast<unsigned int> (displayDepth) > octree_->getDepth ())
      {
        displayDepth = octree_->getDepth ();
      }

      if (display_depth_ != static_cast<std::uint64_t> (displayDepth))
      {
        display_depth_ = displayDepth;
        updateVoxelData ();
        //updateCloudData();
      }
    }

    int
    getDisplayDepth () const
    {
      return display_depth_;
    }

    std::uint64_t
    getPointsLoaded () const
    {
      return points_loaded_;
    }

    std::uint64_t
    getDataLoaded () const
    {
      return data_loaded_;
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
    getDisplayVoxels()
    {
      return voxel_actor_->GetVisibility ();
    }

    void
    setRenderCamera(Camera *render_camera)
    {
      render_camera_ = render_camera;
    }

    int
    getLodPixelThreshold () const
    {
      return lod_pixel_threshold_;
    }

    void
    setLodPixelThreshold (int lod_pixel_threshold)
    {
      if (lod_pixel_threshold <= 1000)
        lod_pixel_threshold = 1000;

      lod_pixel_threshold_ = lod_pixel_threshold;
    }

    void
    increaseLodPixelThreshold ()
    {
      int value = 1000;

      if (lod_pixel_threshold_ >= 50000)
        value = 10000;
      if (lod_pixel_threshold_ >= 10000)
        value = 5000;
      else if (lod_pixel_threshold_ >= 1000)
        value = 100;

      lod_pixel_threshold_ += value;
      std::cout << "Increasing lod pixel threshold: " << lod_pixel_threshold_ << std::endl;
    }

    void
    decreaseLodPixelThreshold ()
    {
      int value = 1000;
      if (lod_pixel_threshold_ > 50000)
        value = 10000;
      else if (lod_pixel_threshold_ > 10000)
        value = 5000;
      else if (lod_pixel_threshold_ > 1000)
        value = 100;

      lod_pixel_threshold_ -= value;

      if (lod_pixel_threshold_ < 100)
        lod_pixel_threshold_ = 100;
      std::cout << "Decreasing lod pixel threshold: " << lod_pixel_threshold_ << std::endl;
    }

    void
    render (vtkRenderer* renderer) override;

  private:

    // Members
    // -----------------------------------------------------------------------------
    OctreeDiskPtr octree_;

    std::uint64_t display_depth_;
    std::uint64_t points_loaded_;
    std::uint64_t data_loaded_;

    Eigen::Vector3d bbox_min_, bbox_max_;

    Camera *render_camera_;

    int lod_pixel_threshold_;

    vtkSmartPointer<vtkActor> voxel_actor_;

    std::map<std::string, vtkSmartPointer<vtkActor> > cloud_actors_map_;
    vtkSmartPointer<vtkActorCollection> cloud_actors_;

  public:
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};
