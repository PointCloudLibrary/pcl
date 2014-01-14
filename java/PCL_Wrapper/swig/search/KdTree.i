%nspace pcl::search::KdTree;

%include <stdint.i>
%include <std_vector.i>

%{
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
%}

//make shared pointers definition here
%shared_ptr(pcl::search::KdTree<pcl::PointXYZ>) //will map boost::shared_ptr<pcl::search::KdTree<pcl::PointXYZ> > and boost::shared_ptr<const pcl::search::KdTree<pcl::PointXYZ> > to pcl::search::KdTree<pcl::PointXYZ> which is a good thing, elaminates the use of pointers  , elaminates incomplete type SWIGTYPE_p_boost_ptr....

namespace pcl
{
  template <typename T> class PointRepresentation;

  namespace search
  {
    template<typename PointT>
    class KdTree: public Search<PointT>
    {
      public:
        typedef typename Search<PointT>::PointCloud PointCloud;
        typedef typename Search<PointT>::PointCloudConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        using pcl::search::Search<PointT>::indices_;
        using pcl::search::Search<PointT>::input_;
        using pcl::search::Search<PointT>::getIndices;
        using pcl::search::Search<PointT>::getInputCloud;
        using pcl::search::Search<PointT>::nearestKSearch;
        using pcl::search::Search<PointT>::radiusSearch;
        using pcl::search::Search<PointT>::sorted_results_;

        typedef boost::shared_ptr<KdTree<PointT> > Ptr;
        typedef boost::shared_ptr<const KdTree<PointT> > ConstPtr;

        typedef boost::shared_ptr<pcl::KdTreeFLANN<PointT> > KdTreeFLANNPtr;
        typedef boost::shared_ptr<const pcl::KdTreeFLANN<PointT> > KdTreeFLANNConstPtr;
        typedef boost::shared_ptr<const PointRepresentation<PointT> > PointRepresentationConstPtr;

        KdTree (bool sorted = true); 

        virtual
        ~KdTree ();

        void
        setPointRepresentation (const PointRepresentationConstPtr &point_representation);

        
        inline PointRepresentationConstPtr
        getPointRepresentation () const;
        
	void 
        setSortedResults (bool sorted_results);
        
        void
        setEpsilon (float eps);

        inline float
        getEpsilon () const;

        void
        setInputCloud (const PointCloudConstPtr& cloud, 
                       const IndicesConstPtr& indices = IndicesConstPtr ());

        int
        nearestKSearch (const PointT &point, int k, 
                        std::vector<int> &k_indices, 
                        std::vector<float> &k_sqr_distances) const;

        int
        radiusSearch (const PointT& point, double radius, 
                      std::vector<int> &k_indices, 
                      std::vector<float> &k_sqr_distances,
                      unsigned int max_nn = 0) const;
      protected:
        KdTreeFLANNPtr tree_;
    };
  }
}

#define PCL_INSTANTIATE_KdTree(T) template class PCL_EXPORTS pcl::search::KdTree<T>;


%import "swig/point_types/PointXYZ.i"
%template (KdTree_PointXYZ) pcl::search::KdTree<pcl::PointXYZ>;
//don't forget to make shared pointers out of this class at line 11
