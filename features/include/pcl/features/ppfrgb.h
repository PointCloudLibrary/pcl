#ifndef PCL_PPFRGB_H_
#define PCL_PPFRGB_H_

#include <pcl/features/feature.h>
#include <pcl/features/boost.h>

namespace pcl
{
  template <typename PointInT, typename PointNT, typename PointOutT>
  class PPFRGBEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using PCLBase<PointInT>::indices_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef pcl::PointCloud<PointOutT> PointCloudOut;

      /**
        * \brief Empty Constructor
        */
      PPFRGBEstimation ();


    private:
      /** \brief The method called for actually doing the computations
        * \param output the resulting point cloud (which should be of type pcl::PPFRGBSignature);
        */
      void
      computeFeature (PointCloudOut &output);

      /** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
        * \param[out] output the output point cloud 
        */
      void 
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &) {}
  };

  template <typename PointInT, typename PointNT, typename PointOutT>
  class PPFRGBRegionEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using PCLBase<PointInT>::indices_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::getClassName;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef pcl::PointCloud<PointOutT> PointCloudOut;

      PPFRGBRegionEstimation ();

    private:
      void
      computeFeature (PointCloudOut &output);

      /** \brief Make the computeFeature (pcl::PointCloud<Eigen::MatrixXf> &output); inaccessible from outside the class
        * \param[out] output the output point cloud 
        */
      void 
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &) {}
  };
}

#endif // PCL_PPFRGB_H_
