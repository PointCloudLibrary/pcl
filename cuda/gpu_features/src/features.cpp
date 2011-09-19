#include "pcl/gpu/features/features.hpp"
#include "pcl/gpu/utils/device/static_check.hpp"
#include "internal.hpp"

using namespace pcl::device;

/////////////////////////////////////////////////////////////////////////
/// PFHEstimation

pcl::gpu::PFHEstimation::PFHEstimation()
{
    Static<sizeof(PFHEstimation:: PointType) == sizeof(device:: PointType)>::check();
    Static<sizeof(PFHEstimation::NormalType) == sizeof(device::NormalType)>::check();    

    impl = new PfhImpl();
}

pcl::gpu::PFHEstimation::~PFHEstimation()
{
    delete static_cast<PfhImpl*>(impl);
}

void pcl::gpu::PFHEstimation::compute(const PointCloud& cloud, const Normals& normals, const NeighborIndices& indices, DeviceArray2D<PFHSignature125>& features)
{
    //assert( cloud.size() == normals.size() );    
    //assert( indices.validate(cloud.size()) );

    const device::PointCloud& c = (const device::PointCloud&)cloud;
    const device::Normals&    n = (const device::Normals&)normals;    

    features.create(indices.sizes.size(), 1);    

    PfhImpl& est = *static_cast<PfhImpl*>(impl);
    est.cloud = c;
    est.normals = n;
    est.neighbours = indices;

    DeviceArray2D<device::PFHSignature125>& f = (DeviceArray2D<device::PFHSignature125>&)features;

    est.compute(f);
}

void pcl::gpu::PFHEstimation::setInputCloud(const PointCloud& cloud) { cloud_ = cloud; }
void pcl::gpu::PFHEstimation::setInputNormals(const Normals& normals) { normals_ = normals; }
void pcl::gpu::PFHEstimation::setSearchSurface(const PointCloud& surface) { surface_ = surface; }
void pcl::gpu::PFHEstimation::setIndices(const Indices& indices) { indices_ = indices; }
void pcl::gpu::PFHEstimation::setRadiusSearch(float radius, int max_results) { radius_ = radius; max_results_ = max_results; }

void pcl::gpu::PFHEstimation::compute(DeviceArray2D<PFHSignature125>& features)
{
    PointCloud& surface = surface_.empty() ? cloud_ : surface_;
    
    octree_.setCloud(surface);
    octree_.build();

    assert( cloud_.size() == normals_.size());

    if (indices_.empty() || (!indices_.empty() && indices_.size() == cloud_.size()))
    {        
        octree_.radiusSearch(cloud_, radius_, max_results_, nn_indices_);       
        compute(surface, normals_, nn_indices_, features);        
    }
    else
    {        
        octree_.radiusSearch(cloud_, indices_, radius_, max_results_, nn_indices_);                
        compute(surface, normals_, nn_indices_, features);
    }

}



/////////////////////////////////////////////////////////////////////////
/// FPFHEstimation

pcl::gpu::FPFHEstimation::FPFHEstimation()
{
    Static<sizeof(FPFHEstimation:: PointType) == sizeof(device:: PointType)>::check();
    Static<sizeof(FPFHEstimation::NormalType) == sizeof(device::NormalType)>::check();    
}

pcl::gpu::FPFHEstimation::~FPFHEstimation()
{
}

void pcl::gpu::FPFHEstimation::compute(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighbours, DeviceArray2D<FPFHSignature33>& features)
{   
    assert( cloud.size() == normals.size() );    
    assert( neighbours.validate(cloud.size()) );

    const device::PointCloud& c = (const device::PointCloud&)cloud;
    const device::Normals&    n = (const device::Normals&)normals;    

    features.create(cloud.size(), 1);    
    spfh.create(cloud.size(), 1);

    DeviceArray2D<device::FPFHSignature33>& s = (DeviceArray2D<device::FPFHSignature33>&)spfh;
    DeviceArray2D<device::FPFHSignature33>& f = (DeviceArray2D<device::FPFHSignature33>&)features;

    device::computeSPFH(c, n, device::Indices(), neighbours, s);
    device::computeFPFH(c, neighbours, s, f);    
}

void pcl::gpu::FPFHEstimation::setInputCloud(const PointCloud& cloud) { cloud_ = cloud; }
void pcl::gpu::FPFHEstimation::setInputNormals(const Normals& normals) { normals_ = normals; }
void pcl::gpu::FPFHEstimation::setSearchSurface(const PointCloud& surface) { surface_ = surface; }
void pcl::gpu::FPFHEstimation::setIndices(const Indices& indices) { indices_ = indices; }
void pcl::gpu::FPFHEstimation::setRadiusSearch(float radius, int max_results) { radius_ = radius; max_results_ = max_results; }

void pcl::gpu::FPFHEstimation::compute(DeviceArray2D<FPFHSignature33>& features)
{   
    bool hasInds = !indices_.empty() && indices_.size() != cloud_.size();
    bool hasSurf = !surface_.empty();

    features.create( hasInds ? indices_.size() : cloud_.size(), 1);

    if (!hasInds && !hasSurf)
    {
        features.create(cloud_.size(), 1);
        octree_.setCloud(cloud_);
        octree_.build();
        assert( cloud_.size() == normals_.size());    
        octree_.radiusSearch(cloud_, radius_, max_results_, nn_indices_);
        compute(cloud_, normals_, nn_indices_, features);
        return;
    }

    PointCloud& surface = surface_.empty() ? cloud_ : surface_;
    
    octree_.setCloud(surface);
    octree_.build();

    if (hasInds)
        octree_.radiusSearch(cloud_, indices_, radius_, max_results_, nn_indices_);
    else
        octree_.radiusSearch(cloud_, radius_, max_results_, nn_indices_);

    int total = computeUniqueIndices(surface.size(), nn_indices_, unique_indices_storage, lookup);
    
    DeviceArray<int> unique_indices(unique_indices_storage.ptr(), total);

    octree_.radiusSearch(surface, unique_indices, radius_, max_results_, nn_indices2_);
    
    DeviceArray2D<device::FPFHSignature33>& spfh33 = (DeviceArray2D<device::FPFHSignature33>&)spfh;
    const device::PointCloud& c = (const device::PointCloud&)cloud_;
    const device::PointCloud& s = (const device::PointCloud&)surface;
    const device::Normals&    n = (const device::Normals&)normals_; 
    device::computeSPFH(s, n, unique_indices, nn_indices2_, spfh33);

    std::vector<int> ui;
    unique_indices.download(ui);

    std::vector<int> szs;
    std::vector<int> data;
    nn_indices2_.data.download(data);
    nn_indices2_.sizes.download(szs);

    std::vector<int> nn(data.begin() + nn_indices2_.max_elems * 1, data.begin() + nn_indices2_.max_elems * 1 + szs[1]);
    
    DeviceArray2D<device::FPFHSignature33>& f = (DeviceArray2D<device::FPFHSignature33>&)features;
    device::computeFPFH(c, indices_, s, nn_indices_, lookup, spfh33, f);
}
