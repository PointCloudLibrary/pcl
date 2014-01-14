%nspace pcl::NormalEstimation;

%{
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
%}

%include "../PointCloud.i"

namespace pcl
{
	 template <typename PointInT, typename PointOutT>
  class NormalEstimation: public Feature<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<NormalEstimation<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const NormalEstimation<PointInT, PointOutT> > ConstPtr;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      
      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudConstPtr PointCloudConstPtr;
      
      NormalEstimation ();
      
      
      virtual ~NormalEstimation ();
      void computePointNormal (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
                          Eigen::Vector4f &plane_parameters, float &curvature);

      
	  void computePointNormal (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
                          float &nx, float &ny, float &nz, float &curvature);

      virtual void 
      setInputCloud (const PointCloudConstPtr &cloud);
      
      void setViewPoint (float vpx, float vpy, float vpz);
      void getViewPoint (float &vpx, float &vpy, float &vpz);
      void useSensorOriginAsViewPoint ();
      
    protected:
      void computeFeature (PointCloudOut &output);
      float vpx_, vpy_, vpz_;
      Eigen::Matrix3f covariance_matrix_;
      Eigen::Vector4f xyz_centroid_;
      bool use_sensor_origin_;
  };
}


//making instances of the template to be used within java
%import "../point_types/Normal.i"
%template (NormalEstimation_PointXYZ_Normal) pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>;
