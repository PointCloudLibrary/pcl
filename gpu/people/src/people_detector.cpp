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
* @author: Koen Buys, Anatoly Baksheev
*/

#include <pcl/gpu/people/people_detector.h>
#include <pcl/gpu/people/label_common.h>

//#include <pcl/gpu/people/conversions.h>
//#include <pcl/gpu/people/label_conversion.h>
//#include <pcl/gpu/people/label_segment.h>
#include <pcl/gpu/people/label_tree.h>
#include <pcl/gpu/people/probability_processor.h>
#include <pcl/gpu/people/organized_plane_detector.h>
#include <pcl/console/print.h>
#include "internal.h"

#include <pcl/common/time.h>

#define AREA_THRES      200 // for euclidean clusterization 1 
#define AREA_THRES2     100 // for euclidean clusterization 2 
#define CLUST_TOL_SHS   0.05
#define DELTA_HUE_SHS   5

using namespace std;
using namespace pcl;
using namespace pcl::gpu::people;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::people::PeopleDetector::PeopleDetector() 
    : fx_(525.f), fy_(525.f), cx_(319.5f), cy_(239.5f), delta_hue_tolerance_(5)
{
  PCL_DEBUG ("[pcl::gpu::people::PeopleDetector] : (D) : Constructor called\n");

  // Create a new organized plane detector
  org_plane_detector_ = OrganizedPlaneDetector::Ptr (new OrganizedPlaneDetector());

  // Create a new probability_processor
  probability_processor_ = ProbabilityProcessor::Ptr (new ProbabilityProcessor());

  // Create a new person attribs
  person_attribs_ = PersonAttribs::Ptr (new PersonAttribs());

  // Just created, indicates first time callback (allows for tracking features to start from second frame)
  first_iteration_ = true;

  // allocation buffers with default sizes
  // if input size is other than the defaults, 
  // then the buffers will be reallocated at processing time.
  // This cause only penalty for first frame ( one reallocation of each buffer )
  allocate_buffers();
}

void
pcl::gpu::people::PeopleDetector::setIntrinsics (float fx, float fy, float cx, float cy)
{
  fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
}

/** @brief This function prepares the needed buffers on both host and device **/
void
pcl::gpu::people::PeopleDetector::allocate_buffers(int rows, int cols)
{ 
  device::Dilatation::prepareRect5x5Kernel(kernelRect5x5_);  

  cloud_host_.width  = cols;
  cloud_host_.height = rows;
  cloud_host_.points.resize(cols * rows);
  cloud_host_.is_dense = false;

  cloud_host_color_.width = cols;
  cloud_host_color_.height = rows;
  cloud_host_color_.resize(cols * rows);
  cloud_host_color_.is_dense = false;

  hue_host_.width  = cols;
  hue_host_.height = rows;
  hue_host_.points.resize(cols * rows);
  hue_host_.is_dense = false;

  depth_host_.width  = cols;
  depth_host_.height = rows;
  depth_host_.points.resize(cols * rows);
  depth_host_.is_dense = false;

  flowermat_host_.width  = cols;
  flowermat_host_.height = rows;
  flowermat_host_.points.resize(cols * rows);
  flowermat_host_.is_dense = false;
  
  cloud_device_.create(rows, cols);
  hue_device_.create(rows, cols);

  depth_device1_.create(rows, cols);
  depth_device2_.create(rows, cols);
  fg_mask_.create(rows, cols);
  fg_mask_grown_.create(rows, cols);
}

int
pcl::gpu::people::PeopleDetector::process(const Depth& depth, const Image& rgba)
{ 
  int cols;
  allocate_buffers(depth.rows(), depth.cols());

  depth_device1_ = depth;

  const device::Image& i = (const device::Image&)rgba;
  device::computeHueWithNans(i, depth_device1_, hue_device_);
  //TODO Hope this is temporary and after porting to GPU the download will be deleted  
  hue_device_.download(hue_host_.points, cols);
      
  device::Intr intr(fx_, fy_, cx_, cy_);
  intr.setDefaultPPIfIncorrect(depth.cols(), depth.rows());

  device::Cloud& c = (device::Cloud&)cloud_device_;
  device::computeCloud(depth, intr, c);  
  cloud_device_.download(cloud_host_.points, cols);    
    
  // uses cloud device, cloud host, depth device, hue device and other buffers
  return process();
}

int
pcl::gpu::people::PeopleDetector::process (const pcl::PointCloud<PointTC>::ConstPtr &cloud)
{
  allocate_buffers(cloud->height, cloud->width);

  const float qnan = std::numeric_limits<float>::quiet_NaN();

  for(size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud_host_.points[i].x = cloud->points[i].x;
    cloud_host_.points[i].y = cloud->points[i].y;
    cloud_host_.points[i].z = cloud->points[i].z;

    bool valid = isFinite(cloud_host_.points[i]);

    hue_host_.points[i] = !valid ? qnan : device::computeHue(cloud->points[i].rgba);
    depth_host_.points[i] = !valid ? 0 : static_cast<unsigned short>(cloud_host_.points[i].z * 1000); //m -> mm
  }
  cloud_device_.upload(cloud_host_.points, cloud_host_.width);
  hue_device_.upload(hue_host_.points, hue_host_.width);
  depth_device1_.upload(depth_host_.points, depth_host_.width);

  // uses cloud device, cloud host, depth device, hue device and other buffers
  return process();
}

int
pcl::gpu::people::PeopleDetector::process ()
{
  int cols = cloud_device_.cols();
  int rows = cloud_device_.rows();      
  
  rdf_detector_->process(depth_device1_, cloud_host_, AREA_THRES);

  const RDFBodyPartsDetector::BlobMatrix& sorted = rdf_detector_->getBlobMatrix();

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // if we found a neck display the tree, and continue with processing
  if(sorted[Neck].size() != 0)
  {
    int c = 0;
    Tree2 t;
    buildTree(sorted, cloud_host_, Neck, c, t);
    
    const std::vector<int>& seed = t.indices.indices;
        
    std::fill(flowermat_host_.points.begin(), flowermat_host_.points.end(), 0);
    {
      //ScopeTime time("shs");    
      shs5(cloud_host_, seed, &flowermat_host_.points[0]);
    }
    
    fg_mask_.upload(flowermat_host_.points, cols);
    device::Dilatation::invoke(fg_mask_, kernelRect5x5_, fg_mask_grown_);

    device::prepareForeGroundDepth(depth_device1_, fg_mask_grown_, depth_device2_);

    //// //////////////////////////////////////////////////////////////////////////////////////////////// //
    //// The second label evaluation    
        
    rdf_detector_->process(depth_device2_, cloud_host_, AREA_THRES2);    
    const RDFBodyPartsDetector::BlobMatrix& sorted2 = rdf_detector_->getBlobMatrix();

    //brief Test if the second tree is build up correctly
    if(sorted2[Neck].size() != 0)
    {      
      Tree2 t2;
      buildTree(sorted2, cloud_host_, Neck, c, t2);
      int par = 0;
      for(int f = 0; f < NUM_PARTS; f++)
      {
       /* if(t2.parts_lid[f] == NO_CHILD)
        {
          cerr << "1;";
          par++;
        }
        else
           cerr << "0;";*/
      }
      static int counter = 0; // TODO move this logging to PeopleApp
      //cerr << t2.nr_parts << ";" << par << ";" << t2.total_dist_error << ";" << t2.norm_dist_error << ";" << counter++ << ";" << endl;
      return 2;
    }
    return 1;
    //output: Tree2 and PointCloud<XYZRGBL> 
  }
  return 0;
}

int
pcl::gpu::people::PeopleDetector::processProb (const pcl::PointCloud<PointTC>::ConstPtr &cloud)
{
  allocate_buffers(cloud->height, cloud->width);

  const float qnan = std::numeric_limits<float>::quiet_NaN();

  for(size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud_host_color_.points[i].x  = cloud_host_.points[i].x = cloud->points[i].x;
    cloud_host_color_.points[i].y  = cloud_host_.points[i].y = cloud->points[i].y;
    cloud_host_color_.points[i].z  = cloud_host_.points[i].z = cloud->points[i].z;
    cloud_host_color_.points[i].rgba = cloud->points[i].rgba;

    bool valid = isFinite(cloud_host_.points[i]);

    hue_host_.points[i] = !valid ? qnan : device::computeHue(cloud->points[i].rgba);
    depth_host_.points[i] = !valid ? 0 : static_cast<unsigned short>(cloud_host_.points[i].z * 1000); //m -> mm
  }
  cloud_device_.upload(cloud_host_.points, cloud_host_.width);
  hue_device_.upload(hue_host_.points, hue_host_.width);
  depth_device1_.upload(depth_host_.points, depth_host_.width);

  // uses cloud device, cloud host, depth device, hue device and other buffers
  return processProb();
}

int
pcl::gpu::people::PeopleDetector::processProb ()
{
  int cols = cloud_device_.cols();
  int rows = cloud_device_.rows();

  PCL_DEBUG("[pcl::gpu::people::PeopleDetector::processProb] : (D) : called\n");

  // First iteration no tracking can take place
  if(first_iteration_)
  {
    //Process input pointcloud with RDF
    rdf_detector_->processProb(depth_device1_);

    probability_processor_->SelectLabel(depth_device1_, rdf_detector_->labels_, rdf_detector_->P_l_);
  }
  // Join probabilities from previous result
  else
  {
    // Backup P_l_1_ value in P_l_prev_1_;
    rdf_detector_->P_l_prev_1_.swap(rdf_detector_->P_l_1_);
    // Backup P_l_2_ value in P_l_prev_2_;
    rdf_detector_->P_l_prev_2_.swap(rdf_detector_->P_l_2_);

    //Process input pointcloud with RDF
    rdf_detector_->processProb(depth_device1_);

    // Create Gaussian Kernel for this iteration, in order to smooth P_l_2_
    float* kernel_ptr_host;
    int kernel_size = 5;
    float sigma = 1.0;
    kernel_ptr_host = probability_processor_->CreateGaussianKernel(sigma, kernel_size);
    DeviceArray<float> kernel_device(kernel_size * sizeof(float));
    kernel_device.upload(kernel_ptr_host, kernel_size * sizeof(float));

    // Output kernel for verification
    PCL_DEBUG("[pcl::gpu::people::PeopleDetector::processProb] : (D) : kernel:\n");
    for(int i = 0; i < kernel_size; i++)
      PCL_DEBUG("\t Entry %d \t: %lf\n", i, kernel_ptr_host[i]);

    if(probability_processor_->GaussianBlur(depth_device1_,rdf_detector_->P_l_2_, kernel_device, rdf_detector_->P_l_Gaus_Temp_ ,rdf_detector_->P_l_Gaus_) != 1)
      PCL_ERROR("[pcl::gpu::people::PeopleDetector::processProb] : (E) : Gaussian Blur failed\n");

    // merge with prior probabilities at this line
    probability_processor_->CombineProb(depth_device1_, rdf_detector_->P_l_Gaus_, 0.5, rdf_detector_->P_l_, 0.5, rdf_detector_->P_l_Gaus_Temp_);
    PCL_DEBUG("[pcl::gpu::people::PeopleDetector::processProb] : (D) : CombineProb called\n");

    // get labels
    probability_processor_->SelectLabel(depth_device1_, rdf_detector_->labels_, rdf_detector_->P_l_Gaus_Temp_);
  }

  // This executes the connected components
  rdf_detector_->processSmooth(depth_device1_, cloud_host_, AREA_THRES);
  // This creates the blobmatrix
  rdf_detector_->processRelations(person_attribs_);

  // Backup this value in P_l_1_;
  rdf_detector_->P_l_1_.swap(rdf_detector_->P_l_);

  const RDFBodyPartsDetector::BlobMatrix& sorted = rdf_detector_->getBlobMatrix();

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // if we found a neck display the tree, and continue with processing
  if(sorted[Neck].size() != 0)
  {
    int c = 0;
    Tree2 t;
    buildTree(sorted, cloud_host_, Neck, c, t, person_attribs_);

    const std::vector<int>& seed = t.indices.indices;

    std::fill(flowermat_host_.points.begin(), flowermat_host_.points.end(), 0);
    {
      //ScopeTime time("shs");
      shs5(cloud_host_, seed, &flowermat_host_.points[0]);
    }

    fg_mask_.upload(flowermat_host_.points, cols);
    device::Dilatation::invoke(fg_mask_, kernelRect5x5_, fg_mask_grown_);

    device::prepareForeGroundDepth(depth_device1_, fg_mask_grown_, depth_device2_);

    //// //////////////////////////////////////////////////////////////////////////////////////////////// //
    //// The second label evaluation

    rdf_detector_->processProb(depth_device2_);
    // TODO: merge with prior probabilities at this line

    // get labels
    probability_processor_->SelectLabel(depth_device1_, rdf_detector_->labels_, rdf_detector_->P_l_);
    // This executes the connected components
    rdf_detector_->processSmooth(depth_device2_, cloud_host_, AREA_THRES2);
    // This creates the blobmatrix
    rdf_detector_->processRelations(person_attribs_);

    // Backup this value in P_l_2_;
    rdf_detector_->P_l_2_.swap(rdf_detector_->P_l_);

    const RDFBodyPartsDetector::BlobMatrix& sorted2 = rdf_detector_->getBlobMatrix();

    //brief Test if the second tree is build up correctly
    if(sorted2[Neck].size() != 0)
    {
      Tree2 t2;
      buildTree(sorted2, cloud_host_, Neck, c, t2, person_attribs_);
      int par = 0;
      for(int f = 0; f < NUM_PARTS; f++)
      {
        if(t2.parts_lid[f] == NO_CHILD)
        {
          cerr << "1;";
          par++;
        }
        else
           cerr << "0;";
      }
      std::cerr << std::endl;
      static int counter = 0; // TODO move this logging to PeopleApp

      //cerr << t2.nr_parts << ";" << par << ";" << t2.total_dist_error << ";" << t2.norm_dist_error << ";" << counter++ << ";" << endl;
      first_iteration_ = false;
      return 2;
    }
    first_iteration_ = false;
    return 1;
    //output: Tree2 and PointCloud<XYZRGBL>
  }
  first_iteration_ = false;
  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace 
{
  void 
  getProjectedRadiusSearchBox (int rows, int cols, const pcl::device::Intr& intr, const pcl::PointXYZ& point, float squared_radius, 
                                  int &minX, int &maxX, int &minY, int &maxY)
  {  
    int min, max;

    float3 q;
    q.x = intr.fx * point.x + intr.cx * point.z;
    q.y = intr.fy * point.y + intr.cy * point.z;
    q.z = point.z;

    // http://www.wolframalpha.com/input/?i=%7B%7Ba%2C+0%2C+b%7D%2C+%7B0%2C+c%2C+d%7D%2C+%7B0%2C+0%2C+1%7D%7D+*+%7B%7Ba%2C+0%2C+0%7D%2C+%7B0%2C+c%2C+0%7D%2C+%7Bb%2C+d%2C+1%7D%7D

    float coeff8 = 1;                                   //K_KT_.coeff (8);
    float coeff7 = intr.cy;                             //K_KT_.coeff (7);
    float coeff4 = intr.fy * intr.fy + intr.cy*intr.cy; //K_KT_.coeff (4);

    float coeff6 = intr.cx;                             //K_KT_.coeff (6);
    float coeff0 = intr.fx * intr.fx + intr.cx*intr.cx; //K_KT_.coeff (0);

    float a = squared_radius * coeff8 - q.z * q.z;
    float b = squared_radius * coeff7 - q.y * q.z;
    float c = squared_radius * coeff4 - q.y * q.y;
    
    // a and c are multiplied by two already => - 4ac -> - ac
    float det = b * b - a * c;
  
    if (det < 0)
    {
      minY = 0;
      maxY = rows - 1;
    }
    else
    {
      float y1 = (b - sqrt (det)) / a;
      float y2 = (b + sqrt (det)) / a;

      min = (int)std::min(floor(y1), floor(y2));
      max = (int)std::max( ceil(y1),  ceil(y2));
      minY = std::min (rows - 1, std::max (0, min));
      maxY = std::max (std::min (rows - 1, max), 0);
    }

    b = squared_radius * coeff6 - q.x * q.z;
    c = squared_radius * coeff0 - q.x * q.x;

    det = b * b - a * c;
    if (det < 0)
    {
      minX = 0;
      maxX = cols - 1;
    }
    else
    {
      float x1 = (b - sqrt (det)) / a;
      float x2 = (b + sqrt (det)) / a;
 
      min = (int)std::min (floor(x1), floor(x2));
      max = (int)std::max ( ceil(x1),  ceil(x2));
      minX = std::min (cols- 1, std::max (0, min));
      maxX = std::max (std::min (cols - 1, max), 0);
    }
  }
 
  float 
  sqnorm(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
  {
    float dx = (p1.x - p2.x);
    float dy = (p1.y - p2.y);
    float dz = (p1.z - p2.z);
    return dx*dx + dy*dy + dz*dz;    
  }
}

void 
pcl::gpu::people::PeopleDetector::shs5(const pcl::PointCloud<PointT> &cloud, const std::vector<int>& indices, unsigned char *mask)
{
  pcl::device::Intr intr(fx_, fy_, cx_, cy_);
  intr.setDefaultPPIfIncorrect(cloud.width, cloud.height);
  
  const float *hue = &hue_host_.points[0];
  double squared_radius = CLUST_TOL_SHS * CLUST_TOL_SHS;

  std::vector< std::vector<int> > storage(100);

  // Process all points in the indices vector
  int total = static_cast<int> (indices.size ());
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int k = 0; k < total; ++k)
  {
    int i = indices[k];
    if (mask[i])
      continue;

    mask[i] = 255;

    int id = 0;
#ifdef _OPENMP
    id = omp_get_thread_num();
#endif
    std::vector<int>& seed_queue = storage[id];
    seed_queue.clear();
    seed_queue.reserve(cloud.size());
    int sq_idx = 0;
    seed_queue.push_back (i);

    PointT p = cloud.points[i];
    float h = hue[i];    

    while (sq_idx < (int)seed_queue.size ())
    {
      int index = seed_queue[sq_idx];
      const PointT& q = cloud.points[index];

      if(!pcl::isFinite (q))
        continue;

      // search window                  
      int left, right, top, bottom;
      getProjectedRadiusSearchBox(cloud.height, cloud.width, intr, q, squared_radius, left, right, top, bottom);
        
      int yEnd  = (bottom + 1) * cloud.width + right + 1;
      int idx  = top * cloud.width + left;
      int skip = cloud.width - right + left - 1;
      int xEnd = idx - left + right + 1;

      for (; xEnd < yEnd; idx += 2*skip, xEnd += 2*cloud.width)
        for (; idx < xEnd; idx += 2)
        {
          if (mask[idx])
            continue;

          if (sqnorm(cloud.points[idx], q) <= squared_radius)
          {
            float h_l = hue[idx];

            if (fabs(h_l - h) < DELTA_HUE_SHS)
            {                   
              seed_queue.push_back (idx);
              mask[idx] = 255;
            }
          }
        }
      
      sq_idx++;
    }        
  }
}

