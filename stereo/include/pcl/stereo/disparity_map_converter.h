#ifndef PCL_DISPARITY_MAP_CONVERTER_H_
#define PCL_DISPARITY_MAP_CONVERTER_H_

#include <cstring>
#include <vector>

#pragma warning(disable : 4996 4521)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#pragma warning(default : 4996 4521)

namespace pcl
{
  // Compute point cloud from the disparity map.
  template <typename PointT>
  class DisparityMapConverter
  {
    protected:
      typedef typename PointCloud<PointT>::Ptr PointCloudPointer;

    public:
      // Public constructor.
      DisparityMapConverter ();
      // Public destructor.
      virtual ~DisparityMapConverter ();

      // Set x-coordinate of the image center.
      void
      setImageCenterX (const float center_x);

      // Get x-coordinate of the image center.
      float
      getImageCenterX () const;

      // Set y-coordinate of the image center.
      void
      setImageCenterY (const float center_y);

      // Get y-coordinate of the image center.
      float
      getImageCenterY () const;

      // Set focal length.
      void
      setFocalLength (const float focal_length);

      // Get focal length.
      float
      getFocalLength () const;

      // Set baseline.
      void
      setBaseline (const float baseline);

      // Get baseline.
      float
      getBaseline () const;

      // Set min disparity threshold.
      void
      setDisparityThresholdMin (const float disparity_threshold_min);

      // Get min disparity threshold.
      float
      getDisparityThresholdMin () const;

      // Set max disparity threshold.
      void
      setDisparityThresholdMax (const float disparity_threshold_max);

      // Get max disparity threshold.
      float
      getDisparityThresholdMax () const;

      // Load an image, that will be used for coloring of the output cloud.
      bool
      loadImage (const std::string &file_name);

      // Set an image, that will be used for coloring of the output cloud.
      void
      setImage (const PointCloud<RGB>::Ptr &image);

      // Load the disparity map.
      bool
      loadDisparityMap (const std::string &file_name);

      // Load the disparity map and initialize it's size.
      bool
      loadDisparityMap (const std::string &file_name, const size_t width, const size_t height);

      // Set the disparity map.
      void
      setDisparityMap(const std::vector<float> &disparity_map);

      // Set the disparity map and initialize it's size.
      void
      setDisparityMap(const std::vector<float> &disparity_map, const size_t width, const size_t height);

      // Compute the output cloud.
      virtual void
      compute (PointCloudPointer &out_cloud);

    protected:
      // X-coordinate of the image center.
      float center_x_;
      // Y-coordinate of the image center.
      float center_y_;
      // Focal length.
      float focal_length_;
      // Baseline.
      float baseline_;
      
      // Color image of the scene.
      pcl::PointCloud<pcl::RGB>::Ptr image_;

      // Vector for the disparity map.
      std::vector<float> disparity_map_;
      // X-size of the disparity map.
      size_t disparity_map_width_;
      // Y-size of the disparity map.
      size_t disparity_map_height_;

      // Thresholds of the disparity.
      float disparity_threshold_min_;
      float disparity_threshold_max_;

      // Translate point from image coordinates and disparity to 3D-coordinates
      PointXYZ 
      translateCoordinates (size_t row, size_t column, float disparity) const;
  };

}
#endif // PCL_DISPARITY_MAP_CONVERTER_H_