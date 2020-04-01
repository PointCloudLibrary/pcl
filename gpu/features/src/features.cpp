/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/utils/device/static_check.hpp>
#include "internal.hpp"

#include <pcl/exceptions.h>
#include <pcl/console/print.h>

using namespace pcl::device;

/////////////////////////////////////////////////////////////////////////
/// Feature

pcl::gpu::Feature::Feature() { radius_ = 0.f, max_results_ = 0; } 
void pcl::gpu::Feature::setInputCloud(const PointCloud& cloud) { cloud_ = cloud; }
void pcl::gpu::Feature::setSearchSurface(const PointCloud& surface) { surface_ = surface; }
void pcl::gpu::Feature::setIndices(const Indices& indices) { indices_ = indices; }
void pcl::gpu::Feature::setRadiusSearch(float radius, int max_results) { radius_ = radius; max_results_ = max_results; }

/////////////////////////////////////////////////////////////////////////
/// FeatureFromNormals
void pcl::gpu::FeatureFromNormals::setInputNormals(const Normals& normals)  { normals_ = normals; }


/////////////////////////////////////////////////////////////////////////
/// NormalEstimation
pcl::gpu::NormalEstimation::NormalEstimation() : vpx_(0), vpy_(0), vpz_(0) {}

void pcl::gpu::NormalEstimation::computeNormals(const PointCloud& cloud, const NeighborIndices& nn_indices, Normals& normals)
{       
    normals.create(nn_indices.neighboors_size());    

    const device::PointCloud& c = (const device::PointCloud&)cloud;
    device::Normals& n = (device::Normals&)normals;

    device::computeNormals(c, nn_indices, n); 
}

void pcl::gpu::NormalEstimation::flipNormalTowardsViewpoint(const PointCloud& cloud, float vp_x, float vp_y, float vp_z, Normals& normals)
{    
    const device::PointCloud& c = (const device::PointCloud&)cloud;
    device::Normals& n = (device::Normals&)normals;

    device::flipNormalTowardsViewpoint(c, make_float3(vp_x, vp_y, vp_z), n);
}

void pcl::gpu::NormalEstimation::flipNormalTowardsViewpoint(const PointCloud& cloud, const Indices& indices, float vp_x, float vp_y, float vp_z, Normals& normals)
{
    const device::PointCloud& c = (const device::PointCloud&)cloud;
    device::Normals& n = (device::Normals&)normals;

    device::flipNormalTowardsViewpoint(c, indices, make_float3(vp_x, vp_y, vp_z), n);
}


void pcl::gpu::NormalEstimation::setViewPoint (float vpx, float vpy, float vpz)
{
    vpx_ = vpx; vpy_ = vpy; vpz_ = vpz;
}

void pcl::gpu::NormalEstimation::getViewPoint (float &vpx, float &vpy, float &vpz) const
{
    vpx = vpx_; vpy = vpy_; vpz = vpz_;
}

void pcl::gpu::NormalEstimation::compute(Normals& normals)
{
    assert(!cloud_.empty());

    PointCloud& surface = surface_.empty() ? cloud_ : surface_;

    octree_.setCloud(surface);
    octree_.build();

    if (indices_.empty() || (!indices_.empty() && indices_.size() == cloud_.size()))
    {
        octree_.radiusSearch(cloud_, radius_, max_results_, nn_indices_);        
        computeNormals(surface, nn_indices_, normals);
        flipNormalTowardsViewpoint(cloud_, vpx_, vpy_, vpz_, normals);
    }
    else
    {
        octree_.radiusSearch(cloud_, indices_, radius_, max_results_, nn_indices_);
        computeNormals(surface, nn_indices_, normals);
        flipNormalTowardsViewpoint(cloud_, indices_, vpx_, vpy_, vpz_, normals);
    }    
}


/////////////////////////////////////////////////////////////////////////
/// PFHEstimation

void pcl::gpu::PFHEstimation::compute(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighbours, DeviceArray2D<PFHSignature125>& features)
{
    assert( cloud.size() == normals.size() );    
    assert( neighbours.validate(cloud.size()) );

    const device::PointCloud& c = (const device::PointCloud&)cloud;
    const device::Normals&    n = (const device::Normals&)normals;    

    features.create (static_cast<int> (neighbours.sizes.size ()), 1);

    DeviceArray2D<device::PFHSignature125>& f = (DeviceArray2D<device::PFHSignature125>&)features;

    repackToAosForPfh(c, n, neighbours, data_rpk, max_elems_rpk);
    computePfh125(data_rpk, max_elems_rpk, neighbours, f);
}

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

void pcl::gpu::PFHRGBEstimation::compute(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighbours, DeviceArray2D<PFHRGBSignature250>& features)
{
    assert( cloud.size() == normals.size() );    
    assert( neighbours.validate(cloud.size()) );

    const device::PointCloud& c = (const device::PointCloud&)cloud;
    const device::Normals&    n = (const device::Normals&)normals;    

    features.create (static_cast<int> (neighbours.sizes.size ()), 1);

    DeviceArray2D<device::PFHRGBSignature250>& f = (DeviceArray2D<device::PFHRGBSignature250>&)features;

    repackToAosForPfhRgb(c, n, neighbours, data_rpk, max_elems_rpk);
    computePfhRgb250(data_rpk, max_elems_rpk, neighbours, f);
}

void pcl::gpu::PFHRGBEstimation::compute(DeviceArray2D<PFHRGBSignature250>& features)
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

pcl::gpu::FPFHEstimation::~FPFHEstimation() {}


void pcl::gpu::FPFHEstimation::compute(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighbours, DeviceArray2D<FPFHSignature33>& features)
{   
    assert( cloud.size() == normals.size() );    
    assert( neighbours.validate(cloud.size()) );

    const device::PointCloud& c = (const device::PointCloud&)cloud;
    const device::Normals&    n = (const device::Normals&)normals;    

    features.create (static_cast<int> (cloud.size ()), 1);    
    spfh.create (static_cast<int> (cloud.size ()), 1);

    DeviceArray2D<device::FPFHSignature33>& s = (DeviceArray2D<device::FPFHSignature33>&)spfh;
    DeviceArray2D<device::FPFHSignature33>& f = (DeviceArray2D<device::FPFHSignature33>&)features;

    device::computeSPFH(c, n, device::Indices(), neighbours, s);
    device::computeFPFH(c, neighbours, s, f);    
}

void pcl::gpu::FPFHEstimation::compute(DeviceArray2D<FPFHSignature33>& features)
{   
    bool hasInds = !indices_.empty() && indices_.size() != cloud_.size();
    bool hasSurf = !surface_.empty();

    features.create (static_cast<int> (hasInds ? indices_.size () : cloud_.size ()), 1);

    if (!hasInds && !hasSurf)
    {
        features.create (static_cast<int> (cloud_.size ()), 1);
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

    DeviceArray2D<device::FPFHSignature33>& f = (DeviceArray2D<device::FPFHSignature33>&)features;
    device::computeFPFH(c, indices_, s, nn_indices_, lookup, spfh33, f);
}


/////////////////////////////////////////////////////////////////////////
/// PPFEstimation      

void pcl::gpu::PPFEstimation::compute(DeviceArray<PPFSignature>& features)
{
    Static<sizeof(PPFEstimation:: PointType) == sizeof(device:: PointType)>::check();
    Static<sizeof(PPFEstimation::NormalType) == sizeof(device::NormalType)>::check();    

    assert(this->surface_.empty() && !indices_.empty() && !cloud_.empty() && normals_.size() == cloud_.size());
    features.create(indices_.size () * cloud_.size ());

    const device::PointCloud& c = (const device::PointCloud&)cloud_;
    const device::Normals&    n = (const device::Normals&)normals_;

    DeviceArray<device::PPFSignature>& f = (DeviceArray<device::PPFSignature>&)features;    
    device::computePPF(c, n, indices_, f);    
}

/////////////////////////////////////////////////////////////////////////
/// PPFRGBEstimation      

void pcl::gpu::PPFRGBEstimation::compute(DeviceArray<PPFRGBSignature>& features)
{    
    Static<sizeof(PPFEstimation:: PointType) == sizeof(device:: PointType)>::check();
    Static<sizeof(PPFEstimation::NormalType) == sizeof(device::NormalType)>::check();    

    assert(this->surface_.empty() && !indices_.empty() && !cloud_.empty() && normals_.size() == cloud_.size());
    features.create(indices_.size () * cloud_.size ());

    const device::PointCloud&  c = (const device::PointCloud&)cloud_;
    const device::Normals&     n = (const device::Normals&)normals_;

    DeviceArray<device::PPFRGBSignature>& f = (DeviceArray<device::PPFRGBSignature>&)features;    
    device::computePPFRGB(c, n, indices_, f);    
}

/////////////////////////////////////////////////////////////////////////
/// PPFRGBRegionEstimation  

void pcl::gpu::PPFRGBRegionEstimation::compute(DeviceArray<PPFRGBSignature>& features)
{
    Static<sizeof(PPFRGBRegionEstimation:: PointType) == sizeof(device:: PointType)>::check();
    Static<sizeof(PPFRGBRegionEstimation::NormalType) == sizeof(device::NormalType)>::check();    

    assert(this->surface_.empty() && !indices_.empty() && !cloud_.empty() && normals_.size() == cloud_.size());

    features.create(indices_.size());

    octree_.setCloud(cloud_);
    octree_.build();

    octree_.radiusSearch(cloud_, indices_, radius_, max_results_, nn_indices_);

    const device::PointCloud& c = (const device::PointCloud&)cloud_;
    const device::Normals&    n = (const device::Normals&)normals_;

    DeviceArray<device::PPFRGBSignature>& f = (DeviceArray<device::PPFRGBSignature>&)features;        

    device::computePPFRGBRegion(c, n, indices_, nn_indices_, f);            
}

/////////////////////////////////////////////////////////////////////////
/// PrincipalCurvaturesEstimation

void pcl::gpu::PrincipalCurvaturesEstimation::compute(DeviceArray<PrincipalCurvatures>& features)
{
    Static<sizeof(PPFRGBRegionEstimation:: PointType) == sizeof(device:: PointType)>::check();
    Static<sizeof(PPFRGBRegionEstimation::NormalType) == sizeof(device::NormalType)>::check();    

    assert(/*!indices_.empty() && */!cloud_.empty() && max_results_ > 0 && radius_ > 0.f);
    assert(surface_.empty() ? normals_.size() == cloud_.size() : normals_.size() == surface_.size());

    PointCloud& surface = surface_.empty() ? cloud_ : surface_;

    octree_.setCloud(surface);
    octree_.build();

    if(indices_.empty())
        octree_.radiusSearch(cloud_, radius_, max_results_, nn_indices_);
    else
        octree_.radiusSearch(cloud_, indices_, radius_, max_results_, nn_indices_);

    const device::Normals& n = (const device::Normals&)normals_;

    features.create(normals_.size());

    DeviceArray<device::PrincipalCurvatures>& f = (DeviceArray<device::PrincipalCurvatures>&)features;

    device::computePointPrincipalCurvatures(n, indices_, nn_indices_, f, proj_normals_buf);
}


/////////////////////////////////////////////////////////////////////////
/// VFHEstimation

pcl::gpu::VFHEstimation::VFHEstimation()
{     
    vpx_ =  vpy_ =  vpz_ = 0.f;

    //default parameters to compute VFH
    use_given_normal_ = false;
    use_given_centroid_ = false;
    normalize_bins_ = true;
    normalize_distances_ = false;
    size_component_ = false;    
}

void pcl::gpu::VFHEstimation::setViewPoint(float  vpx, float  vpy, float  vpz) { vpx_ = vpx; vpy_ = vpy; vpz_ = vpz; }
void pcl::gpu::VFHEstimation::getViewPoint(float& vpx, float& vpy, float& vpz) const { vpx = vpx_; vpy = vpy_; vpz = vpz_; }      

void pcl::gpu::VFHEstimation::setUseGivenNormal (bool use) { use_given_normal_ = use; }
void pcl::gpu::VFHEstimation::setNormalToUse (const NormalType& normal)   { normal_to_use_ = normal; }
void pcl::gpu::VFHEstimation::setUseGivenCentroid (bool use) { use_given_centroid_ = use; }
void pcl::gpu::VFHEstimation::setCentroidToUse (const PointType& centroid)  { centroid_to_use_ = centroid; }

void pcl::gpu::VFHEstimation::setNormalizeBins (bool normalize)     { normalize_bins_ = normalize; }
void pcl::gpu::VFHEstimation::setNormalizeDistance (bool normalize) { normalize_distances_ = normalize; }
void pcl::gpu::VFHEstimation::setFillSizeComponent (bool fill_size) { size_component_ = fill_size; }

////////////////////////////////////////////////////////////////////////////////////////////

void pcl::gpu::VFHEstimation::compute(DeviceArray<VFHSignature308>& feature)
{   
    assert(!surface_.empty() && normals_.size() == surface_.size() && cloud_.empty());    

    Static<sizeof(VFHEstimation:: PointType) == sizeof(device:: PointType)>::check();
    Static<sizeof(VFHEstimation::NormalType) == sizeof(device::NormalType)>::check();

    feature.create(1);

    VFHEstimationImpl impl;

    const device::PointCloud& s = (const device::PointCloud&)surface_;
    const device::Normals& n = (const device::Normals&)normals_;

    if (use_given_centroid_) 
    {
        impl.xyz_centroid.x = centroid_to_use_.x;
        impl.xyz_centroid.y = centroid_to_use_.y;
        impl.xyz_centroid.z = centroid_to_use_.z;
    }
    else        
    {
        compute3DCentroid(s, indices_, impl.xyz_centroid);

    }
    if (use_given_normal_)
    {
        impl.normal_centroid.x = normal_to_use_.x;
        impl.normal_centroid.y = normal_to_use_.y;
        impl.normal_centroid.z = normal_to_use_.z;
    }
    else
        compute3DCentroid (n, indices_, impl.normal_centroid);

    impl.viewpoint = make_float3(vpx_, vpy_, vpz_);


    impl.indices = indices_;
    impl.points = s;
    impl.normals = n;

    impl.normalize_distances = normalize_distances_;
    impl.size_component = size_component_;
    impl.normalize_bins = normalize_bins_;

    DeviceArray<device::VFHSignature308>& f = (DeviceArray<device::VFHSignature308>&)feature;
    impl.compute(f);
}

/////////////////////////////////////////////////////////////////////////
/// SpinImageEstimation

void pcl::gpu::SpinImageEstimation::setImageWidth (unsigned int bin_count) { image_width_ = bin_count; }
void pcl::gpu::SpinImageEstimation::setSupportAngle (float support_angle_cos) 
{
    if (0.f > support_angle_cos || support_angle_cos > 1.f)  // may be permit negative cosine?
		pcl::gpu::error("Cosine of support angle should be between 0 and 1", __FILE__, __LINE__);
    support_angle_cos_ = support_angle_cos;
}

void pcl::gpu::SpinImageEstimation::setMinPointCountInNeighbourhood (unsigned int min_pts_neighb) { min_pts_neighb_ = min_pts_neighb; }
void pcl::gpu::SpinImageEstimation::setInputWithNormals(const PointCloud& input, const Normals& normals)
{
    setInputCloud (input);
    input_normals_ = normals;
}

void pcl::gpu::SpinImageEstimation::setSearchSurfaceWithNormals (const PointCloud& surface, const Normals& normals)
{
    setSearchSurface (surface);
    setInputNormals (normals);
}

void pcl::gpu::SpinImageEstimation::setRotationAxis (const NormalType& axis)
{
    rotation_axis_ = axis;
    use_custom_axis_ = true;
    use_custom_axes_cloud_ = false;
}

void pcl::gpu::SpinImageEstimation::setInputRotationAxes (const Normals& axes)
{
    rotation_axes_cloud_ = axes;
    use_custom_axes_cloud_ = true;
    use_custom_axis_ = false;
}

void pcl::gpu::SpinImageEstimation::useNormalsAsRotationAxis () {  use_custom_axis_ = false;  use_custom_axes_cloud_ = false; }
void pcl::gpu::SpinImageEstimation::setAngularDomain (bool is_angular) { is_angular_ = is_angular; }
void pcl::gpu::SpinImageEstimation::setRadialStructure (bool is_radial) { is_radial_ = is_radial; }

//////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::SpinImageEstimation::SpinImageEstimation (unsigned int image_width, double support_angle_cos, unsigned int min_pts_neighb)
: is_angular_(false), use_custom_axis_(false), use_custom_axes_cloud_(false), is_radial_(false),
image_width_(image_width), support_angle_cos_(support_angle_cos), min_pts_neighb_(min_pts_neighb)
{
    assert (support_angle_cos_ <= 1.0 && support_angle_cos_ >= 0.0);
}

////////////////////////////////////////////////////////////////////////////////////////////
void pcl::gpu::SpinImageEstimation::compute(DeviceArray2D<SpinImage>& features, DeviceArray<unsigned char>& mask)
{   
	assert(!indices_.empty());

	if (image_width_ != 8)
		pcl::gpu::error("Currently only image_width = 8 is supported (less is possible right now, more - need to allocate more memory)", __FILE__, __LINE__);
	
	Static<sizeof(SpinImageEstimation:: PointType) == sizeof(device:: PointType)>::check();
    Static<sizeof(SpinImageEstimation::NormalType) == sizeof(device::NormalType)>::check();

	features.create (static_cast<int> (indices_.size ()), 1);
	mask.create(indices_.size());

	//////////////////////////////
	if (!surface_)
	{
		surface_ = cloud_;
		normals_ = input_normals_;
		fake_surface_ = true;
	}
	
	assert(!(use_custom_axis_ && use_custom_axes_cloud_));

	if (!use_custom_axis_ && !use_custom_axes_cloud_ && !input_normals_)
		pcl::gpu::error("No normals for input cloud were given!", __FILE__, __LINE__);
	
	if ((is_angular_ || support_angle_cos_ > 0.0) && !input_normals_)
		pcl::gpu::error("No normals for input cloud were given!", __FILE__, __LINE__);
	
	if (use_custom_axes_cloud_ && rotation_axes_cloud_.size () != cloud_.size ())
		pcl::gpu::error("Rotation axis cloud have different size from input!", __FILE__, __LINE__);
	
	///////////////////////////////////////////////
	octree_.setCloud(surface_);
    octree_.build();
    octree_.radiusSearch(cloud_, indices_, radius_, max_results_, nn_indices_);

	// OK, we are interested in the points of the cylinder of height 2*r and base radius r, where r = m_dBinSize * in_iImageWidth
	// it can be embedded to the sphere of radius sqrt(2) * m_dBinSize * in_iImageWidth
	// suppose that points are uniformly distributed, so we lose ~40% // according to the volumes ratio
	float bin_size = radius_ / image_width_;
	if (!is_radial_)
		bin_size /= std::sqrt(2.f);


	const device::PointCloud& s = (const device::PointCloud&)surface_;
	const device::PointCloud& c = (const device::PointCloud&)cloud_;
	const device::Normals& in = (const device::Normals&)input_normals_;
    const device::Normals& n = (const device::Normals&)normals_;
	

	if (use_custom_axis_)
	{
		float3 axis = make_float3(rotation_axis_.x, rotation_axis_.y, rotation_axis_.z);
		computeSpinImagesCustomAxes(is_radial_, is_angular_, support_angle_cos_, indices_, c, in,
			s, n, nn_indices_, min_pts_neighb_, image_width_, bin_size, axis, features);
	}
	else if (use_custom_axes_cloud_)
	{
		const device::Normals& axes = (const device::Normals&)rotation_axes_cloud_;

		computeSpinImagesCustomAxesCloud(is_radial_, is_angular_, support_angle_cos_, indices_, c, in,
			s, n, nn_indices_, min_pts_neighb_, image_width_, bin_size, axes, features);
	}
	else
	{
		computeSpinImagesOrigigNormal(is_radial_, is_angular_, support_angle_cos_, indices_, c, in,
			s, n, nn_indices_, min_pts_neighb_, image_width_, bin_size, features);
	}
	
	computeMask(nn_indices_, min_pts_neighb_, mask);
}
