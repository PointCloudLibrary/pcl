#ifndef PCL_SURFLET_H_
#define PCL_SURFLET_H_

#include <pcl/features/feature.h>

/// @TODO: use forward declaration for unordered_multimap ??
#include <boost/unordered_map.hpp>


namespace pcl
{
  template <typename PointInT, typename PointOutT>
  class SurfletModelEstimation : public Feature<PointInT, PointOutT>
  {
  public:
    typedef typename PointCloud<PointInT>::ConstPtr PointCloudIn;
    typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    /// slight hack to enable the usage of the boost::hash <pair <A, B> >
    struct HashKeyStruct : public std::pair <int, std::pair <int, std::pair <int, int> > > {
      HashKeyStruct(int a, int b, int c, int d)
      {
        this->first = a;
        this->second.first = b;
        this->second.second.first = c;
        this->second.second.second = d;
      }
    };

    typedef boost::unordered_multimap<HashKeyStruct, std::pair<size_t, size_t> > FeatureHashMapType;
    typedef boost::shared_ptr<FeatureHashMapType> FeatureHashMapTypePtr;


    SurfletModelEstimation (float a_angle_discretization_step = 30.0 / 180 * M_PI, float a_distance_discretization_step = 0.05)
    {
      angle_discretization_step = a_angle_discretization_step;
      distance_discretization_step = a_distance_discretization_step;
    }

    FeatureHashMapTypePtr
    computeSurfletModel (const pcl::PointCloud<PointInT> &cloud /* output goes here */);

    std::vector < std::pair <Eigen::Affine3f, float> >
    registerModelToScene (const pcl::PointCloud<PointInT> &cloud_model, const pcl::PointCloud<PointInT> &cloud_scene, FeatureHashMapTypePtr feature_hashmap_model);


    protected:
    void
    computeFeature (PointCloudOut &output);

    private:
    float angle_discretization_step, distance_discretization_step;
  };
}

#endif /* PCL_SURFLET_H_ */
