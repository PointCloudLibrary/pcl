#ifndef PCL_DIGITAL_ELEVATION_MAP_H_
#define PCL_DIGITAL_ELEVATION_MAP_H_

#pragma warning(disable : 4996)
#include <pcl/point_types.h>
#include <pcl/stereo/disparity_map_converter.h>
#pragma warning(default : 4996)

namespace pcl
{
  // Point type for Digital Elevation Map
  struct EIGEN_ALIGN16 _PointDEM
  {
    PCL_ADD_POINT4D;
    float intensity;
    float height_variance;
    float intensity_variance;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct PointDEM : public _PointDEM
  {
    inline PointDEM()
    {
      x = y = z = 0.0f;
      intensity = 0.0f;
      height_variance = intensity_variance = 0.0f;
    }
    inline bool
    isValid()
    {
      return (height_variance >= 0.0f && intensity_variance >= 0.0f);
    }
  };

  // Type for histograms.
  class FeatureHistogram
  {
    public:
      // Public constructor.
      FeatureHistogram (size_t number_of_bins);
      // Public destructor.
      virtual ~FeatureHistogram ();

      // Set min and max thresholds.
      void
      setThresholds(float min, float max);

      // Get the lower threshold.
      float
      getThresholdMin() const;

      // Get the upper threshold.
      float
      getThresholdMax() const;

      // Get number of elements was added to the histogram.
      size_t
      getNumberOfElements() const;

      // Get number of bins in the histogram.
      size_t
      getNumberOfBins() const;

      // Increase a bin, that corresponds the value.
      void
      addValue (float value);

      // Get value, corresponds to the greatest bin.
      float
      meanValue ();

      // Get variance of the value.
      float
      variance (float mean);

    protected:
      // Vector, that contain the histogram.
      std::vector <unsigned> histogram_;

      // Thresholds.
      float threshold_min_;
      float threshold_max_;
      // "Width" of a bin.
      float step_;

      // Number of values was added to the histogram.
      size_t number_of_elements_;

      // Number of bins.
      size_t number_of_bins_;
  };

  // Build a Digital Elevation Map in the column-disparity space 
  // from a disparity map and color image of the scene.
  template <typename PointT>
  class DigitalElevationMapBuilder : public DisparityMapConverter <PointT>
  {
    public:
      // Public constructor.
      DigitalElevationMapBuilder ();
      // Public destructor.
      virtual ~DigitalElevationMapBuilder ();

      // Set resolution of the DEM.
      void
      setResolution (size_t resolution_column, size_t resolution_disparity);

      // Get column resolution od the DEM.
      size_t
      getColumnResolution () const;

      // Get disparity resolution od the DEM.
      size_t
      getDisparityResolution () const;

      // Set minimum amount of points in a DEM's cell.
      void
      setMinPointsInCell(size_t min_points_in_cell);

      // Get minimum amount of points in a DEM's cell.
      size_t
      getMinPointsInCell() const;

      // Compute Digital Elevation Map.
      void 
      compute (PointCloudPointer &out_cloud);

    protected:
      // Resolution of the DEM.
      size_t resolution_column_;
      size_t resolution_disparity_;

      // Minimum amount of points in a DEM's cell.
      size_t min_points_in_cell_;
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT (_PointDEM,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, height_variance, height_variance)
                                    (float, intensity_variance, intensity_variance)
 )



#endif // PCL_DIGITAL_ELEVATION_MAP_H_