#include "pcl/gpu/octree/octree.hpp"

#include "cuda_interface.hpp"
#include "pcl/gpu/common/timers_cuda.hpp"


pcl::gpu::CudaOctree* cast(void *ptr) { return static_cast<pcl::gpu::CudaOctree*>(ptr); }

pcl::gpu::Octree::Octree() : impl(0)
{
    impl = new pcl::gpu::Octree2();    
    //impl = new Octree_host();
}

pcl::gpu::Octree::~Octree() 
{
    if (impl)
        delete cast(impl);
}

void pcl::gpu::Octree::setCloud(const PointCloud& cloud_arg)
{
    const DeviceArray_<float3>& cloud = (const DeviceArray_<float3>&)cloud_arg;
    cast(impl)->setCloud(cloud);
}

void pcl::gpu::Octree::build()
{
    cast(impl)->build();    
}

void pcl::gpu::Octree::internalDownload()
{
    static_cast<Octree2*>(impl)->internalDownload();
    //pcl::gpu::internalDownload(cast(impl));
}

void pcl::gpu::Octree::radiusSearchHost(const PointXYZ& center, float radius, std::vector<int>& out)
{
    if (!static_cast<Octree2*>(impl)->downloaded)
        internalDownload();

    float3 c = *(float3*)(&center.x);
    static_cast<Octree2*>(impl)->radiusSearch2(c, radius, out);
}

void pcl::gpu::Octree::radiusSearchBatchGPU(const BatchQueries& centers, float radius, BatchResult& output, DeviceArray_<int>& out_sizes) const
{
    const DeviceArray_<float3>& queries = (const DeviceArray_<float3>&)centers;

    printf("out_sizes = %d\n", out_sizes.size());


    static_cast<Octree2*>(impl)->radiusSearchBatch(queries, radius, output, out_sizes);
}
        