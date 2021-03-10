/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id: pfh.hpp 5027 2012-03-12 03:10:45Z rusu $
 *
 */

#ifndef PCL_FEATURES_IMPL_ESF_H_
#define PCL_FEATURES_IMPL_ESF_H_

#include <pcl/features/esf.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <ctime> // for time

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::ESFEstimation<PointInT, PointOutT>::computeESF (
    PointCloudIn &pc, std::vector<float> &hist)
{
  const int binsize = 64;
  unsigned int sample_size = 20000;
  // @TODO: Replace with c++ stdlib uniform_random_generator
  srand (static_cast<unsigned int> (time (nullptr)));
  const auto maxindex = pc.size ();

  std::vector<float> d2v, d1v, d3v, wt_d3;
  std::vector<int> wt_d2;
  d1v.reserve (sample_size);
  d2v.reserve (sample_size * 3);
  d3v.reserve (sample_size);
  wt_d2.reserve (sample_size * 3);
  wt_d3.reserve (sample_size);

  float h_in[binsize] = {0};
  float h_out[binsize] = {0};
  float h_mix[binsize] = {0};
  float h_mix_ratio[binsize] = {0};

  float h_a3_in[binsize] = {0};
  float h_a3_out[binsize] = {0};
  float h_a3_mix[binsize] = {0};

  float h_d3_in[binsize] = {0};
  float h_d3_out[binsize] = {0};
  float h_d3_mix[binsize] = {0};

  float ratio=0.0;
  float pih = static_cast<float>(M_PI) / 2.0f;
  float a,b,c,s;
  int th1,th2,th3;
  int vxlcnt = 0;
  int pcnt1,pcnt2,pcnt3;
  for (std::size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx)
  {
    // get a new random point
    int index1 = rand()%maxindex;
    int index2 = rand()%maxindex;
    int index3 = rand()%maxindex;

    if (index1==index2 || index1 == index3 || index2 == index3)
    {
      nn_idx--;
      continue;
    }

    Eigen::Vector4f p1 = pc[index1].getVector4fMap ();
    Eigen::Vector4f p2 = pc[index2].getVector4fMap ();
    Eigen::Vector4f p3 = pc[index3].getVector4fMap ();

    // A3
    Eigen::Vector4f v21 (p2 - p1);
    Eigen::Vector4f v31 (p3 - p1);
    Eigen::Vector4f v23 (p2 - p3);
    a = v21.norm (); b = v31.norm (); c = v23.norm (); s = (a+b+c) * 0.5f;
    if (s * (s-a) * (s-b) * (s-c) <= 0.001f)
    {
        nn_idx--;
        continue;
    }

    v21.normalize ();
    v31.normalize ();
    v23.normalize ();

    //TODO: .dot gives nan's
    th1 = static_cast<int> (pcl_round (std::acos (std::abs (v21.dot (v31))) / pih * (binsize-1)));
    th2 = static_cast<int> (pcl_round (std::acos (std::abs (v23.dot (v31))) / pih * (binsize-1)));
    th3 = static_cast<int> (pcl_round (std::acos (std::abs (v23.dot (v21))) / pih * (binsize-1)));
    if (th1 < 0 || th1 >= binsize)
    {
      nn_idx--;
      continue;
    }
    if (th2 < 0 || th2 >= binsize)
    {
      nn_idx--;
      continue;
    }
    if (th3 < 0 || th3 >= binsize)
    {
      nn_idx--;
      continue;
    }

    // D2
    d2v.push_back (pcl::euclideanDistance (pc[index1], pc[index2]));
    d2v.push_back (pcl::euclideanDistance (pc[index1], pc[index3]));
    d2v.push_back (pcl::euclideanDistance (pc[index2], pc[index3]));

    int vxlcnt_sum = 0;
    int p_cnt = 0;
    // IN, OUT, MIXED, Ratio line tracing, index1->index2
    {
      const int xs = p1[0] < 0.0? static_cast<int>(std::floor(p1[0])+GRIDSIZE_H): static_cast<int>(std::ceil(p1[0])+GRIDSIZE_H-1);
      const int ys = p1[1] < 0.0? static_cast<int>(std::floor(p1[1])+GRIDSIZE_H): static_cast<int>(std::ceil(p1[1])+GRIDSIZE_H-1);
      const int zs = p1[2] < 0.0? static_cast<int>(std::floor(p1[2])+GRIDSIZE_H): static_cast<int>(std::ceil(p1[2])+GRIDSIZE_H-1);
      const int xt = p2[0] < 0.0? static_cast<int>(std::floor(p2[0])+GRIDSIZE_H): static_cast<int>(std::ceil(p2[0])+GRIDSIZE_H-1);
      const int yt = p2[1] < 0.0? static_cast<int>(std::floor(p2[1])+GRIDSIZE_H): static_cast<int>(std::ceil(p2[1])+GRIDSIZE_H-1);
      const int zt = p2[2] < 0.0? static_cast<int>(std::floor(p2[2])+GRIDSIZE_H): static_cast<int>(std::ceil(p2[2])+GRIDSIZE_H-1);
      wt_d2.push_back (this->lci (xs, ys, zs, xt, yt, zt, ratio, vxlcnt, pcnt1));
      if (wt_d2.back () == 2)
        h_mix_ratio[static_cast<int> (pcl_round (ratio * (binsize-1)))]++;
      vxlcnt_sum += vxlcnt;
      p_cnt += pcnt1;
    }
    // IN, OUT, MIXED, Ratio line tracing, index1->index3
    {
      const int xs = p1[0] < 0.0? static_cast<int>(std::floor(p1[0])+GRIDSIZE_H): static_cast<int>(std::ceil(p1[0])+GRIDSIZE_H-1);
      const int ys = p1[1] < 0.0? static_cast<int>(std::floor(p1[1])+GRIDSIZE_H): static_cast<int>(std::ceil(p1[1])+GRIDSIZE_H-1);
      const int zs = p1[2] < 0.0? static_cast<int>(std::floor(p1[2])+GRIDSIZE_H): static_cast<int>(std::ceil(p1[2])+GRIDSIZE_H-1);
      const int xt = p3[0] < 0.0? static_cast<int>(std::floor(p3[0])+GRIDSIZE_H): static_cast<int>(std::ceil(p3[0])+GRIDSIZE_H-1);
      const int yt = p3[1] < 0.0? static_cast<int>(std::floor(p3[1])+GRIDSIZE_H): static_cast<int>(std::ceil(p3[1])+GRIDSIZE_H-1);
      const int zt = p3[2] < 0.0? static_cast<int>(std::floor(p3[2])+GRIDSIZE_H): static_cast<int>(std::ceil(p3[2])+GRIDSIZE_H-1);
      wt_d2.push_back (this->lci (xs, ys, zs, xt, yt, zt, ratio, vxlcnt, pcnt2));
      if (wt_d2.back () == 2)
        h_mix_ratio[static_cast<int>(pcl_round (ratio * (binsize-1)))]++;
      vxlcnt_sum += vxlcnt;
      p_cnt += pcnt2;
    }
    // IN, OUT, MIXED, Ratio line tracing, index2->index3
    {
      const int xs = p2[0] < 0.0? static_cast<int>(std::floor(p2[0])+GRIDSIZE_H): static_cast<int>(std::ceil(p2[0])+GRIDSIZE_H-1);
      const int ys = p2[1] < 0.0? static_cast<int>(std::floor(p2[1])+GRIDSIZE_H): static_cast<int>(std::ceil(p2[1])+GRIDSIZE_H-1);
      const int zs = p2[2] < 0.0? static_cast<int>(std::floor(p2[2])+GRIDSIZE_H): static_cast<int>(std::ceil(p2[2])+GRIDSIZE_H-1);
      const int xt = p3[0] < 0.0? static_cast<int>(std::floor(p3[0])+GRIDSIZE_H): static_cast<int>(std::ceil(p3[0])+GRIDSIZE_H-1);
      const int yt = p3[1] < 0.0? static_cast<int>(std::floor(p3[1])+GRIDSIZE_H): static_cast<int>(std::ceil(p3[1])+GRIDSIZE_H-1);
      const int zt = p3[2] < 0.0? static_cast<int>(std::floor(p3[2])+GRIDSIZE_H): static_cast<int>(std::ceil(p3[2])+GRIDSIZE_H-1);
      wt_d2.push_back (this->lci (xs,ys,zs,xt,yt,zt,ratio,vxlcnt,pcnt3));
      if (wt_d2.back () == 2)
        h_mix_ratio[static_cast<int>(pcl_round(ratio * (binsize-1)))]++;
      vxlcnt_sum += vxlcnt;
      p_cnt += pcnt3;
    }

    // D3 ( herons formula )
    d3v.push_back (std::sqrt (std::sqrt (s * (s-a) * (s-b) * (s-c))));
    if (vxlcnt_sum <= 21)
    {
      wt_d3.push_back (0);
      h_a3_out[th1] += static_cast<float> (pcnt3) / 32.0f;
      h_a3_out[th2] += static_cast<float> (pcnt1) / 32.0f;
      h_a3_out[th3] += static_cast<float> (pcnt2) / 32.0f;
    }
    else
      if (p_cnt - vxlcnt_sum < 4)
      {
        h_a3_in[th1] += static_cast<float> (pcnt3) / 32.0f;
        h_a3_in[th2] += static_cast<float> (pcnt1) / 32.0f;
        h_a3_in[th3] += static_cast<float> (pcnt2) / 32.0f;
        wt_d3.push_back (1);
      }
      else
      {
        h_a3_mix[th1] += static_cast<float> (pcnt3) / 32.0f;
        h_a3_mix[th2] += static_cast<float> (pcnt1) / 32.0f;
        h_a3_mix[th3] += static_cast<float> (pcnt2) / 32.0f;
        wt_d3.push_back (static_cast<float> (vxlcnt_sum) / static_cast<float> (p_cnt));
      }
  }
  // Normalizing, get max
  float maxd2 = 0;
  float maxd3 = 0;

  for (std::size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx)
  {
    // get max of Dx
    if (d2v[nn_idx] > maxd2)
      maxd2 = d2v[nn_idx];
    if (d2v[sample_size + nn_idx] > maxd2)
      maxd2 = d2v[sample_size + nn_idx];
    if (d2v[sample_size*2 +nn_idx] > maxd2)
      maxd2 = d2v[sample_size*2 +nn_idx];
    if (d3v[nn_idx] > maxd3)
      maxd3 = d3v[nn_idx];
  }

  // Normalize and create histogram
  int index;
  for (std::size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx)
  {
    if (wt_d3[nn_idx] >= 0.999) // IN
    {
      index = static_cast<int>(pcl_round (d3v[nn_idx] / maxd3 * (binsize-1)));
      if (index >= 0 && index < binsize)
        h_d3_in[index]++;
    }
    else
    {
      if (wt_d3[nn_idx] <= 0.001) // OUT
      {
        index = static_cast<int>(pcl_round (d3v[nn_idx] / maxd3 * (binsize-1)));
        if (index >= 0 && index < binsize)
          h_d3_out[index]++ ;
      }
      else
      {
        index = static_cast<int>(pcl_round (d3v[nn_idx] / maxd3 * (binsize-1)));
        if (index >= 0 && index < binsize)
          h_d3_mix[index]++;
      }
    }
  }
  //normalize and create histogram
  for (std::size_t nn_idx = 0; nn_idx < d2v.size(); ++nn_idx )
  {
    if (wt_d2[nn_idx] == 0)
      h_in[static_cast<int>(pcl_round (d2v[nn_idx] / maxd2 * (binsize-1)))]++ ;
    if (wt_d2[nn_idx] == 1)
      h_out[static_cast<int>(pcl_round (d2v[nn_idx] / maxd2 * (binsize-1)))]++;
    if (wt_d2[nn_idx] == 2)
      h_mix[static_cast<int>(pcl_round (d2v[nn_idx] / maxd2 * (binsize-1)))]++ ;
  }

  //float weights[10] = {1,  1,  1,  1,  1,  1,  1,  1 , 1 ,  1};
  float weights[10] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 1.0f,  1.0f, 2.0f, 2.0f, 2.0f};

  hist.reserve (binsize * 10);
  for (const float &i : h_a3_in)
    hist.push_back (i * weights[0]);
  for (const float &i : h_a3_out)
    hist.push_back (i * weights[1]);
  for (const float &i : h_a3_mix)
    hist.push_back (i * weights[2]);

  for (const float &i : h_d3_in)
    hist.push_back (i * weights[3]);
  for (const float &i : h_d3_out)
    hist.push_back (i * weights[4]);
  for (const float &i : h_d3_mix)
    hist.push_back (i * weights[5]);

  for (const float &i : h_in)
    hist.push_back (i*0.5f * weights[6]);
  for (const float &i : h_out)
    hist.push_back (i * weights[7]);
  for (const float &i : h_mix)
    hist.push_back (i * weights[8]);
  for (const float &i : h_mix_ratio)
    hist.push_back (i*0.5f * weights[9]);

  float sm = 0;
  for (const float &i : hist)
    sm += i;

  for (float &i : hist)
    i /= sm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> int
pcl::ESFEstimation<PointInT, PointOutT>::lci (
    const int x1, const int y1, const int z1, 
    const int x2, const int y2, const int z2, 
    float &ratio, int &incnt, int &pointcount)
{
  int voxelcount = 0;
  int voxel_in = 0;
  int act_voxel[3];
  act_voxel[0] = x1;
  act_voxel[1] = y1;
  act_voxel[2] = z1;
  int x_inc, y_inc, z_inc;
  int dx = x2 - x1;
  int dy = y2 - y1;
  int dz = z2 - z1;
  if (dx < 0)
    x_inc = -1;
  else
    x_inc = 1;
  int l = std::abs (dx);
  if (dy < 0)
    y_inc = -1 ;
  else
    y_inc = 1;
  int m = std::abs (dy);
  if (dz < 0)
    z_inc = -1 ;
  else
    z_inc = 1;
  int n = std::abs (dz);
  int dx2 = 2 * l;
  int dy2 = 2 * m;
  int dz2 = 2 * n;
  if ((l >= m) & (l >= n))
  {
    int err_1 = dy2 - l;
    int err_2 = dz2 - l;
    for (int i = 1; i<l; i++)
    {
      voxelcount++;
      voxel_in +=  static_cast<int>(lut_[act_voxel[0]][act_voxel[1]][act_voxel[2]] == 1);
      if (err_1 > 0)
      {
        act_voxel[1] += y_inc;
        err_1 -=  dx2;
      }
      if (err_2 > 0)
      {
        act_voxel[2] += z_inc;
        err_2 -= dx2;
      }
      err_1 += dy2;
      err_2 += dz2;
      act_voxel[0] += x_inc;
    }
  }
  else if ((m >= l) & (m >= n))
  {
    int err_1 = dx2 - m;
    int err_2 = dz2 - m;
    for (int i=1; i<m; i++)
    {
      voxelcount++;
      voxel_in +=  static_cast<int>(lut_[act_voxel[0]][act_voxel[1]][act_voxel[2]] == 1);
      if (err_1 > 0)
      {
        act_voxel[0] +=  x_inc;
        err_1 -= dy2;
      }
      if (err_2 > 0)
      {
        act_voxel[2] += z_inc;
        err_2 -= dy2;
      }
      err_1 += dx2;
      err_2 += dz2;
      act_voxel[1] += y_inc;
    }
  }
  else
  {
    int err_1 = dy2 - n;
    int err_2 = dx2 - n;
    for (int i=1; i<n; i++)
    {
      voxelcount++;
      voxel_in +=  static_cast<int>(lut_[act_voxel[0]][act_voxel[1]][act_voxel[2]] == 1);
      if (err_1 > 0)
      {
        act_voxel[1] += y_inc;
        err_1 -= dz2;
      }
      if (err_2 > 0)
      {
        act_voxel[0] += x_inc;
        err_2 -= dz2;
      }
      err_1 += dy2;
      err_2 += dx2;
      act_voxel[2] += z_inc;
    }
  }
  voxelcount++;
  voxel_in +=  static_cast<int>(lut_[act_voxel[0]][act_voxel[1]][act_voxel[2]] == 1);
  incnt = voxel_in;
  pointcount = voxelcount;

  if (voxel_in >=  voxelcount-1)
    return (0);

  if (voxel_in <= 7)
    return (1);

  ratio = static_cast<float>(voxel_in) / static_cast<float>(voxelcount);
  return (2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::ESFEstimation<PointInT, PointOutT>::voxelize9 (PointCloudIn &cluster)
{
  for (const auto& point: cluster)
  {
    int xx = point.x<0.0? static_cast<int>(std::floor(point.x)+GRIDSIZE_H) : static_cast<int>(std::ceil(point.x)+GRIDSIZE_H-1);
    int yy = point.y<0.0? static_cast<int>(std::floor(point.y)+GRIDSIZE_H) : static_cast<int>(std::ceil(point.y)+GRIDSIZE_H-1);
    int zz = point.z<0.0? static_cast<int>(std::floor(point.z)+GRIDSIZE_H) : static_cast<int>(std::ceil(point.z)+GRIDSIZE_H-1);

    for (int x = -1; x < 2; x++)
      for (int y = -1; y < 2; y++)
        for (int z = -1; z < 2; z++)
        {
          int xi = xx + x;
          int yi = yy + y;
          int zi = zz + z;

          if (yi >= GRIDSIZE || xi >= GRIDSIZE || zi>=GRIDSIZE || yi < 0 || xi < 0 || zi < 0)
          {
            ;
          }
          else
            this->lut_[xi][yi][zi] = 1;
        }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::ESFEstimation<PointInT, PointOutT>::cleanup9 (PointCloudIn &cluster)
{
  for (const auto& point: cluster)
  {
    int xx = point.x<0.0? static_cast<int>(std::floor(point.x)+GRIDSIZE_H) : static_cast<int>(std::ceil(point.x)+GRIDSIZE_H-1);
    int yy = point.y<0.0? static_cast<int>(std::floor(point.y)+GRIDSIZE_H) : static_cast<int>(std::ceil(point.y)+GRIDSIZE_H-1);
    int zz = point.z<0.0? static_cast<int>(std::floor(point.z)+GRIDSIZE_H) : static_cast<int>(std::ceil(point.z)+GRIDSIZE_H-1);

    for (int x = -1; x < 2; x++)
      for (int y = -1; y < 2; y++)
        for (int z = -1; z < 2; z++)
        {
          int xi = xx + x;
          int yi = yy + y;
          int zi = zz + z;

          if (yi >= GRIDSIZE || xi >= GRIDSIZE || zi>=GRIDSIZE || yi < 0 || xi < 0 || zi < 0)
          {
            ;
          }
          else
            this->lut_[xi][yi][zi] = 0;
        }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::ESFEstimation<PointInT, PointOutT>::scale_points_unit_sphere (
    const pcl::PointCloud<PointInT> &pc, float scalefactor, Eigen::Vector4f& centroid)
{
  pcl::compute3DCentroid (pc, centroid);
  pcl::demeanPointCloud (pc, centroid, local_cloud_);

  float max_distance = 0;
  pcl::PointXYZ cog (0, 0, 0);

  for (const auto& point: local_cloud_)
  {
    float d = pcl::euclideanDistance(cog,point);
    if (d > max_distance)
      max_distance = d;
  }

  float scale_factor = 1.0f / max_distance * scalefactor;

  Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
  matrix.scale (scale_factor);
  pcl::transformPointCloud (local_cloud_, local_cloud_, matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT> void
pcl::ESFEstimation<PointInT, PointOutT>::compute (PointCloudOut &output)
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    output.width = output.height = 0;
    output.clear ();
    return;
  }
  // Copy the header
  output.header = input_->header;

  // Resize the output dataset
  // Important! We should only allocate precisely how many elements we will need, otherwise
  // we risk at pre-allocating too much memory which could lead to bad_alloc 
  // (see http://dev.pointclouds.org/issues/657)
  output.width = output.height = 1;
  output.is_dense = input_->is_dense;
  output.resize (1);

  // Perform the actual feature computation
  computeFeature (output);

  Feature<PointInT, PointOutT>::deinitCompute ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::ESFEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  Eigen::Vector4f xyz_centroid;
  std::vector<float> hist;
  scale_points_unit_sphere (*surface_, static_cast<float>(GRIDSIZE_H), xyz_centroid);
  this->voxelize9 (local_cloud_);
  this->computeESF (local_cloud_, hist);
  this->cleanup9 (local_cloud_);

  // We only output _1_ signature
  output.resize (1);
  output.width = 1;
  output.height = 1;

  for (std::size_t d = 0; d < hist.size (); ++d)
    output[0].histogram[d] = hist[d];
}

#define PCL_INSTANTIATE_ESFEstimation(T,OutT) template class PCL_EXPORTS pcl::ESFEstimation<T,OutT>;

#endif    // PCL_FEATURES_IMPL_ESF_H_

