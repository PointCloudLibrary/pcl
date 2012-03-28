#include <ui_organized_segmentation_demo.h>
// QT4
#include <QMainWindow>
#include <QMutex>
#include <QTimer>
// Boost
#include <boost/thread/thread.hpp>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

typedef pcl::PointXYZRGBA PointT;

// Useful macros
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

namespace Ui
{
  class MainWindow;
}

class OrganizedSegmentationDemo : public QMainWindow
{
  Q_OBJECT
  public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;
  

    OrganizedSegmentationDemo(pcl::Grabber& grabber);

    ~OrganizedSegmentationDemo ()
    {
      if(grabber_.isRunning())
        grabber_.stop();
    }
  
    void cloud_cb (const CloudConstPtr& cloud);
  
  protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    pcl::Grabber& grabber_;

    QMutex mtx_;
    QMutex vis_mtx_;
    Ui::MainWindow *ui_;
    QTimer *vis_timer_;
    pcl::PointCloud<PointT> prev_cloud_;
    pcl::PointCloud<pcl::Normal> prev_normals_;
    std::vector<pcl::PlanarRegion<PointT> > prev_regions_;
    float* prev_distance_map_;
    
    std::vector<pcl::PointCloud<PointT> > prev_clusters_;
    
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;

    bool capture_;
    bool data_modified_;
    size_t previous_data_size_;
    size_t previous_clusters_size_;

    bool display_normals_;
    bool display_curvature_;
    bool display_distance_map_;

    bool use_planar_refinement_;
    bool use_clustering_;

    pcl::PlaneCoefficientComparator<PointT, pcl::Normal>::Ptr plane_comparator_;
    pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr euclidean_comparator_;
    pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr rgb_comparator_;
    pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal> rgb_comp_;
    pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal>::Ptr edge_aware_comparator_;
    pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;

  public slots:
    void toggleCapturePressed()
    {
      capture_ = !capture_;
    }

    void usePlaneComparatorPressed ();
    void useEuclideanComparatorPressed ();
    void useRGBComparatorPressed ();
    void useEdgeAwareComparatorPressed ();
    
    void displayCurvaturePressed ();
    void displayDistanceMapPressed ();
    void displayNormalsPressed ();
                                 
    void disableRefinementPressed ()
    {
      use_planar_refinement_ = false;
    }
    
    void usePlanarRefinementPressed ()
    {
      use_planar_refinement_ = true;
    }

    void disableClusteringPressed ()
    {
      use_clustering_ = false;
    }

    void useEuclideanClusteringPressed ()
    {
      use_clustering_ = true;
    }
    

  private slots:
  void
    timeoutSlot();

  
};
