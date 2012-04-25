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

#include <pcl/gpu/people/rdf_bodyparts_detector.h>
#include <cassert>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>
#include "internal.h"

using namespace std;

const int MAX_CLUST_SIZE = 25000;
const float CLUST_TOL = 0.05f;

pcl::gpu::people::RDFBodyPartsDetector::RDFBodyPartsDetector( const vector<string>& tree_files, 
                                                              int rows, 
                                                              int cols)
    : labels_(rows, cols), 
      labels_smoothed_(rows, cols), 
      max_cluster_size_(MAX_CLUST_SIZE), 
      cluster_tolerance_(CLUST_TOL)
{
  //TODO replace all asserts with exceptions
  assert(!tree_files.empty());

  impl_.reset( new device::MultiTreeLiveProc(rows, cols) );

  for(size_t i = 0; i < tree_files.size(); ++i)
  {
    // load the tree file
    vector<trees::Node>  nodes;
    vector<trees::Label> leaves;

    // this might throw but we haven't done any malloc yet
    int height = loadTree (tree_files[i], nodes, leaves );
    impl_->trees.push_back(device::CUDATree(height, nodes, leaves));
  }

  vector<pcl::RGB> rgba(LUT_COLOR_LABEL_LENGTH);

  for(int i = 0; i < LUT_COLOR_LABEL_LENGTH; ++i)
  {
      // !!!! generate in RGB format, not BGR
      rgba[i].r = LUT_COLOR_LABEL[i*3 + 2]; 
      rgba[i].g = LUT_COLOR_LABEL[i*3 + 1];
      rgba[i].b = LUT_COLOR_LABEL[i*3 + 0];
      rgba[i].a = 255;
  }
  color_map_.upload(rgba);
}

size_t 
pcl::gpu::people::RDFBodyPartsDetector::treesNumber() const
{
  return impl_->trees.size();
}

const pcl::gpu::people::RDFBodyPartsDetector::Labels& 
pcl::gpu::people::RDFBodyPartsDetector::getLabels() const
{
  return labels_smoothed_;
}

void 
pcl::gpu::people::RDFBodyPartsDetector::colorizeLabels(const Labels& labels, Image& color_labels) const
{
  color_labels.create(labels.rows(), labels.cols());

  const DeviceArray<uchar4>& map = (const DeviceArray<uchar4>&)color_map_;
  device::Image& img = (device::Image&)color_labels;
  device::colorLMap(labels, map, img);
}

void 
pcl::gpu::people::RDFBodyPartsDetector::computeLabels(const Depth& depth)
{
  // Process the depthimage (CUDA)
  impl_->process(depth, labels_);
  device::smoothLabelImage(labels_, depth, labels_smoothed_, NUM_PARTS, 5, 300);
}

//////////////////////////////////////////////////////////////////////
///////////////////// in development (dirty) /////////////////////////


#include <pcl/gpu/people/conversions.h>
#include <pcl/gpu/people/label_segment.h>
#include <pcl/gpu/people/label_tree.h>

void optimized_elec(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat& src_labels, float tolerance,
                    std::vector<std::vector<pcl::PointIndices> > &labeled_clusters,
                    unsigned int min_pts_per_cluster, unsigned int max_pts_per_cluster, unsigned int num_parts,
                    bool brute_force_border, float radius_scale);
void optimized_elec3(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat& src_labels, float tolerance,
                    std::vector<std::vector<pcl::PointIndices> > &labeled_clusters,
                    unsigned int min_pts_per_cluster, unsigned int max_pts_per_cluster, unsigned int num_parts,
                    bool brute_force_border, float radius_scale);

void 
pcl::gpu::people::RDFBodyPartsDetector::step2_selectBetterName(const PointCloud<PointXYZ>& cloud, int cluster_area_threshold, BlobMatrix& sorted)
{
  int cols = labels_smoothed_.cols();
  int rows = labels_smoothed_.rows();

  cv::Mat lmap(rows, cols, CV_8U);
  labels_smoothed_.download(lmap.data, lmap.step);

  // Make all the clusters
  vector<vector<pcl::PointIndices> > cluster_indices(NUM_PARTS);
  optimized_elec4(cloud, lmap, cluster_tolerance_, cluster_indices, cluster_area_threshold, max_cluster_size_, NUM_PARTS, false, 1.f);

  // Create a new struct to put the results in  
  sorted.clear();
  sorted.resize(NUM_PARTS);
  //create the blob2 matrix  
  label_skeleton::sortIndicesToBlob2 ( cloud, cluster_area_threshold, sorted, cluster_indices );
    //Build relationships between the blobs
  label_skeleton::buildRelations ( sorted );
}


#include "compute_search_radius.h"

template<typename PointT1, typename PointT2>
double sqnorm(const PointT1& p1, const PointT2& p2)
{
    return (p1.getVector3fMap () - p2.getVector3fMap ()).squaredNorm ();
}

template<typename It, typename Val>
void yota(It beg, It end, Val seed)
{
    for(It pos = beg; pos < end;)
        *pos++ = seed++;
}

void pcl::gpu::people::RDFBodyPartsDetector::optimized_elec4(const PointCloud<pcl::PointXYZ>& cloud, const cv::Mat& src_labels, float tolerance,
                    std::vector<std::vector<PointIndices> > &labeled_clusters,
                    unsigned int min_pts_per_cluster, unsigned int max_pts_per_cluster, unsigned int num_parts,
                    bool brute_force_border, float radius_scale)
{
  


    typedef unsigned char uchar;

    cv::Size sz = src_labels.size();

    cv::Mat dst_labels(sz, CV_32S, cv::Scalar(-1));
    cv::Mat wavefront(sz, CV_32SC2);
    cv::Mat region_sizes = cv::Mat::zeros(sz, CV_32S);

    cv::Point2i *wf = wavefront.ptr<cv::Point2i>();
    int *rsizes = region_sizes.ptr<int>();

    int cc = -1;

    float squared_radius = tolerance * tolerance;

    for(int j = 0; j < sz.height; ++j)
    {
        for(int i = 0; i < sz.width; ++i)
        {
            if (src_labels.at<uchar>(j, i) >= num_parts || dst_labels.at<int>(j, i) != -1) // invalid label && has been labeled
                continue;

            cv::Point2i* ws = wf; // initialize wavefront
            cv::Point2i p(i, j);  // current pixel

            cc++;	// next label

            dst_labels.at<int>(j, i) = cc;
            int count = 0;	// current region size

            // wavefront propagation
            while( ws >= wf ) // wavefront not empty
            {
                // put neighbors onto wavefront
                const uchar* sl = &src_labels.at<uchar>(p.y, p.x);
                int*         dl = &dst_labels.at<  int>(p.y, p.x);
                const pcl::PointXYZ *sp = &cloud.at(p.x, p.y);

                //right
                if( p.x < sz.width-1 && dl[+1] == -1 && sl[+1] == sl[0])
                    if (sqnorm(sp[0], sp[+1]) <= squared_radius)
                    {
                        dl[+1] = cc;
                        *ws++ = cv::Point2i(p.x+1, p.y);
                    }

                //left
                if( p.x > 0 && dl[-1] == -1 && sl[-1] == sl[0])
                    if (sqnorm(sp[0], sp[-1]) <= squared_radius)
                    {
                        dl[-1] = cc;
                        *ws++ = cv::Point2i(p.x-1, p.y);
                    }

                //top
                if( p.y < sz.height-1 && dl[+sz.width] == -1 && sl[+sz.width] == sl[0])
                    if (sqnorm(sp[0], sp[+sz.width]) <= squared_radius)
                    {
                        dl[+sz.width] = cc;
                        *ws++ = cv::Point2i(p.x, p.y+1);
                    }


                //top
                if( p.y > 0 && dl[-sz.width] == -1 && sl[-sz.width] == sl[0])
                    if (sqnorm(sp[0], sp[-sz.width]) <= squared_radius)
                    {
                        dl[-sz.width] = cc;
                        *ws++ = cv::Point2i(p.x, p.y-1);
                    }

                // pop most recent and propagate
                p = *--ws;
                count++;
            }

            rsizes[cc] = count;
        } /* for(int i = 0; i < sz.width; ++i) */
    } /* for(int j = 0; j < sz.height; ++j) */
    
#if 0
    if (brute_force_border)
    {
        pcl::ScopeTime time("border");


        Mat djset(sz, CV_32S);
        int* dj = djset.ptr<int>();
        yota(dj, dj + sz.area(), 0);

        int rows = cloud.height;
        int cols = cloud.width;
        device::Intr intr(525, 525, cols/2-0.5f, rows/2-0.5f);                

#define HT_USE_OMP

#ifdef HT_USE_OMP

        const int threads_num = 3;

        int range[threads_num + 1];
        for(int i = 0; i < threads_num+1; ++i)
            range[i] = i * num_parts/threads_num;

        #pragma omp parallel for num_threads(threads_num)
        for(int r = 0; r < threads_num; ++r)
        {
#endif
            for(int j = 1; j < sz.height-1; ++j)
            {
                int *dl = dst_labels.ptr<int>(j);

                for(int i = 1; i < sz.width-1; ++i)
                {
                    int cc = dl[i];

                    if (cc == -1)
                        continue;

#ifdef HT_USE_OMP
                    uchar lbl = src_labels.at<uchar>(j, i);
                    bool inRange =range[r] <= lbl && lbl < range[r+1];
                    if (!inRange)
                        continue;
#endif

                    if (cc == dl[i+1] && cc == dl[i-1] && cc == dl[i+sz.width] && cc == dl[i-sz.width])
                        continue; // inner point

                    int root = cc;
                    while(root != dj[root])
                        root = dj[root];

                    const pcl::PointXYZ& sp = cloud.at(i, j);
                    uchar sl = src_labels.at<uchar>(j, i);

                    if (!isFinite(sp))
                        continue;

                    int left, right, top, bottom;
                    getProjectedRadiusSearchBox(rows, cols, intr, sp, squared_radius, left, right, top, bottom);                    

                    if (radius_scale != 1.f)
                    {
                        left   = static_cast<unsigned int>(i - (i - left) * radius_scale);
                        right  = static_cast<unsigned int>(i + (right - i) * radius_scale);
                        top    = static_cast<unsigned int>(j - (j - top) * radius_scale);
                        bottom = static_cast<unsigned int>(j + (bottom - j) * radius_scale);
                    }

                    for(unsigned int y = top; y < bottom + 1; ++y)
                    {
                        const uchar *sl1  = src_labels.ptr<uchar>(y);
                        const int   *dl1 = dst_labels.ptr<int>(y);
                        const pcl::PointXYZ* sp1 = &cloud.at(0, y);

                        for(unsigned int x = left; x < right + 1; ++x)
                        {
                            int cc1 = dl1[x];
                            // not the same cc, the same label, and in radius
                            if (cc1 != cc && sl1[x] == sl && sqnorm(sp, sp1[x]) <= squared_radius)
                            {
                                while(cc1 != dj[cc1])
                                    cc1 = dj[cc1];

                                dj[cc1] = root;
                            }
                        }
                    }
                }
            }
#ifdef HT_USE_OMP
        }
#endif

        for(int j = 0; j < sz.height; ++j)
        {
            int *dl = dst_labels.ptr<int>(j);

            for(int i = 0; i < sz.width; ++i)
            {
                int cc = dl[i];
                if (cc == -1)
                    continue;

                while(cc != dj[cc]) //disjoint set
                    cc = dj[cc];
                dl[i] = cc;
                rsizes[cc]++;
            }
        }

    } /* if (brute_force_border)*/

#endif

    //convert to output format
    labeled_clusters.clear();
    labeled_clusters.resize(num_parts);

    vector<int> remap(sz.area(), -1);

    for(int j = 0; j < sz.height; ++j)
    {
        const uchar *sl = src_labels.ptr<uchar>(j);
        const int   *dl = dst_labels.ptr<int>(j);

        for(int i = 0; i < sz.width; ++i)
        {
            int part = sl[i];
            int cc   = dl[i];

            if (cc == -1)
                continue;

            if ((int)min_pts_per_cluster <= rsizes[cc] && rsizes[cc] <= (int)max_pts_per_cluster)
            {

                int ccindex = remap[cc];
                if (ccindex == -1)
                {
                    ccindex = (int)labeled_clusters[part].size();
                    labeled_clusters[part].resize(ccindex + 1);
                    remap[cc] = ccindex;
                }

                labeled_clusters[part][ccindex].indices.push_back(j*sz.width + i);
            }
        }
    }
}
