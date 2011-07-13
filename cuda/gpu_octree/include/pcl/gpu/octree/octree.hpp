#ifndef PCL_GPU_OCTREE_
#define PCL_GPU_OCTREE_

#include <vector>
#include <boost/shared_ptr.hpp>

#include "pcl/pcl_macros.h"
#include "pcl/gpu/common/device_array.hpp"

namespace pcl
{

    namespace gpu
    {        
		struct PointXYZ
		{
			float x, y, z;           
		};

        class PCL_EXPORTS Octree
        {
        public:
            Octree();
            virtual ~Octree();


            /* Types */

            typedef DeviceArray_<PointXYZ> PointCloud;
            typedef boost::shared_ptr<PointCloud> PointCloudPtr;
            typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

            typedef boost::shared_ptr<Octree> Ptr;
			typedef boost::shared_ptr<const Octree> ConstPtr;

            typedef DeviceArray_<PointXYZ> BatchQueries;
            typedef DeviceArray2D_<int> BatchResult;

            /*  Methods */            
            void setCloud(const PointCloud& cloud_arg);

			void build();

            void internalDownload();
            void radiusSearchHost(const PointXYZ& center, float radius, std::vector<int>& out);

            void radiusSearchBatchGPU(const BatchQueries& centers, float radius, BatchResult& out, DeviceArray_<int>& out_sizes) const;
        private:
            void *impl;            
        };        
    }
}

#endif /* PCL_GPU_OCTREE_ */