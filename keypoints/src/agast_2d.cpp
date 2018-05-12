/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#include <pcl/keypoints/agast_2d.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::AgastKeypoint2D<pcl::PointXYZ, pcl::PointUV>::detectKeypoints (pcl::PointCloud<pcl::PointUV> &output)
{
  // image size
  const size_t width = input_->width;
  const size_t height = input_->height;

  // destination for intensity data; will be forwarded to AGAST
  std::vector<float> image_data (width * height);

  for (size_t i = 0; i < image_data.size (); ++i)
    image_data[i] = static_cast<float> (intensity_ ((*input_)[i]));

  if (!detector_)
    detector_.reset (new pcl::keypoints::agast::AgastDetector7_12s (width, height, threshold_, bmax_));

  detector_->setMaxKeypoints (nr_max_keypoints_);

  if (apply_non_max_suppression_)
  {
    pcl::PointCloud<pcl::PointUV> tmp_cloud;

    detector_->detectKeypoints (image_data, tmp_cloud);
    detector_->applyNonMaxSuppression (image_data, tmp_cloud, output);
  }
  else
  {
    detector_->detectKeypoints (image_data, output);
  }

  // we don not change the denseness
  output.is_dense = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AbstractAgastDetector::detectKeypoints (
    const std::vector<unsigned char> & intensity_data, 
    pcl::PointCloud<pcl::PointUV> &output)
{
  detect (&(intensity_data[0]), output.points);

  output.height = 1;
  output.width = static_cast<uint32_t> (output.points.size ());
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AbstractAgastDetector::detectKeypoints (
    const std::vector<float> & intensity_data, 
    pcl::PointCloud<pcl::PointUV> &output)
{
  detect (&(intensity_data[0]), output.points);

  output.height = 1;
  output.width = static_cast<uint32_t> (output.points.size ());
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AbstractAgastDetector::applyNonMaxSuppression (
    const pcl::PointCloud<pcl::PointUV> &input,
    const std::vector<ScoreIndex> &scores,
    pcl::PointCloud<pcl::PointUV> &output)
{
  std::vector<int> nms_flags;

  const std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > & corners_all = input.points;
  std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > & corners_nms = output.points;

  int curr_corner_ind;
  int lastRow = 0, next_lastRow = 0;
  std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> >::const_iterator curr_corner;
  int lastRowCorner_ind = 0, next_lastRowCorner_ind = 0;
  std::vector<int>::iterator nms_flags_p;
  std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> >::iterator curr_corner_nms;
  int j;
  int num_corners_all = int (corners_all.size ());
  int n_max_corners = int (corners_nms.capacity ());

  curr_corner = corners_all.begin ();

  if (num_corners_all > n_max_corners)
  {
    if (n_max_corners == 0)
    {
      n_max_corners = 512 > num_corners_all ? 512 : num_corners_all;
      corners_nms.reserve (n_max_corners);
      nms_flags.reserve (n_max_corners);
    }
    else
    {
      n_max_corners *= 2;
      if (num_corners_all > n_max_corners)
        n_max_corners = num_corners_all;
      corners_nms.reserve (n_max_corners);
      nms_flags.reserve (n_max_corners);
    }
  }
  corners_nms.resize (num_corners_all);
  nms_flags.resize (num_corners_all);

  nms_flags_p = nms_flags.begin ();
  curr_corner_nms = corners_nms.begin ();

  // set all flags to MAXIMUM
  for (j = num_corners_all; j > 0; j--)
    *nms_flags_p++= -1;
  nms_flags_p = nms_flags.begin ();

  for (curr_corner_ind = 0; curr_corner_ind < num_corners_all; curr_corner_ind++)
  {
    int t;

    // check above
    if (lastRow + 1 < curr_corner->v)
    {
      lastRow=next_lastRow;
      lastRowCorner_ind=next_lastRowCorner_ind;
    }
    if (next_lastRow != curr_corner->v)
    {
      next_lastRow = int (curr_corner->v);
      next_lastRowCorner_ind = curr_corner_ind;
    }
    if (lastRow+1 == curr_corner->v)
    {
      //find the corner above the current one
      while ((corners_all[lastRowCorner_ind].u < curr_corner->u) && (corners_all[lastRowCorner_ind].v == lastRow))
        lastRowCorner_ind++;

      if ((corners_all[lastRowCorner_ind].u == curr_corner->u) && (lastRowCorner_ind!=curr_corner_ind))
      {
        int t = lastRowCorner_ind;
        while (nms_flags[t] != -1) //find the maximum in this block
          t = nms_flags[t];

        if (scores[curr_corner_ind].score < scores[t].score)
        {
          nms_flags[curr_corner_ind] = t;
        }
        else
          nms_flags[t] = curr_corner_ind;
      }
    }

    //check left
    t = curr_corner_ind - 1;
    if ((curr_corner_ind!=0) && (corners_all[t].v == curr_corner->v) && (corners_all[t].u+1 == curr_corner->u))
    {
      int curr_cornerMaxAbove_ind = nms_flags[curr_corner_ind];

      while (nms_flags[t] != -1) //find the maximum in that area
        t = nms_flags[t];

      if (curr_cornerMaxAbove_ind == -1) //no maximum above
      {
        if (t != curr_corner_ind)
        {
          if (scores[curr_corner_ind].score < scores[t].score)
            nms_flags[curr_corner_ind] = t;
          else
            nms_flags[t] = curr_corner_ind;
        }
      }
      else  //maximum above
      {
        if (t != curr_cornerMaxAbove_ind)
        {
          if (scores[curr_cornerMaxAbove_ind].score < scores[t].score)
          {
            nms_flags[curr_cornerMaxAbove_ind] = t;
            nms_flags[curr_corner_ind] = t;
          }
          else
          {
            nms_flags[t] = curr_cornerMaxAbove_ind;
            nms_flags[curr_corner_ind] = curr_cornerMaxAbove_ind;
          }
        }
      }
    }

    ++curr_corner;
  }

  // collecting maximum corners
  corners_nms.resize (0);
  for (curr_corner_ind = 0; curr_corner_ind < num_corners_all; curr_corner_ind++)
  {
    if (*nms_flags_p++ == -1)
      corners_nms.push_back (corners_all[curr_corner_ind]);
  }

  output.height = 1;
  output.width = static_cast<uint32_t> (output.points.size ());
  output.is_dense = input.is_dense;
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AbstractAgastDetector::applyNonMaxSuppression (
  const std::vector<unsigned char>& intensity_data, 
  const pcl::PointCloud<pcl::PointUV> &input,
  pcl::PointCloud<pcl::PointUV> &output)
{
  std::vector<ScoreIndex> scores;
  computeCornerScores (&(intensity_data[0]), input.points, scores);
  
  // If a threshold for the maximum number of keypoints is given
  if (nr_max_keypoints_ <= scores.size ()) //std::numeric_limits<unsigned int>::max ())
  {
    std::sort (scores.begin (), scores.end (), CompareScoreIndex ());

    scores.resize (nr_max_keypoints_);

    // Need to copy the points
    pcl::PointCloud<pcl::PointUV> best_input;
    best_input.resize (nr_max_keypoints_);
    for (size_t i = 0; i < scores.size (); ++i)
      best_input[i] = input[scores[i].idx];
    applyNonMaxSuppression (best_input, scores, output);
  }
  else
    applyNonMaxSuppression (input, scores, output);
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AbstractAgastDetector::applyNonMaxSuppression (
  const std::vector<float>& intensity_data, 
  const pcl::PointCloud<pcl::PointUV> &input,
  pcl::PointCloud<pcl::PointUV> &output)
{
  std::vector<ScoreIndex> scores;
  computeCornerScores (&(intensity_data[0]), input.points, scores);
  
  // If a threshold for the maximum number of keypoints is given
  if (nr_max_keypoints_ <= scores.size ()) //std::numeric_limits<unsigned int>::max ())
  {
    std::sort (scores.begin (), scores.end (), CompareScoreIndex ());

    scores.resize (nr_max_keypoints_);

    // Need to copy the points
    pcl::PointCloud<pcl::PointUV> best_input;
    best_input.resize (nr_max_keypoints_);
    for (size_t i = 0; i < scores.size (); ++i)
      best_input[i] = input[scores[i].idx];
    applyNonMaxSuppression (best_input, scores, output);
  }
  else
    applyNonMaxSuppression (input, scores, output);
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AbstractAgastDetector::computeCornerScores (
  const unsigned char* im,
  const std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &corners_all, 
  std::vector<ScoreIndex> &scores)
{
  unsigned int n = 0;
  unsigned int num_corners = static_cast<unsigned int> (corners_all.size ());

  if (num_corners > scores.capacity ())
  {
    if (scores.capacity () == 0)
      scores.reserve (512 > num_corners ? 512 : num_corners);
    else
    {
      unsigned int nScores = static_cast<unsigned int> (scores.capacity ()) * 2;
      if (num_corners > nScores)
        nScores = num_corners;
      scores.reserve (nScores);
    }
  }
  scores.resize (num_corners);

  for (; n < num_corners; n++)
  {
    scores[n].idx   = n;
    scores[n].score = computeCornerScore (im + static_cast<size_t> (corners_all[n].v) * width_ + static_cast<size_t> (corners_all[n].u));
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AbstractAgastDetector::computeCornerScores (
  const float* im,
  const std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &corners_all, 
  std::vector<ScoreIndex> &scores)
{
  unsigned int n = 0;
  unsigned int num_corners = static_cast<unsigned int> (corners_all.size ());

  if (num_corners > scores.capacity ())
  {
    if (scores.capacity () == 0)
      scores.reserve (512 > num_corners ? 512 : num_corners);
    else
    {
      unsigned int nScores = static_cast<unsigned int> (scores.capacity ()) * 2;
      if (num_corners > nScores)
        nScores = num_corners;
      scores.reserve (nScores);
    }
  }
  scores.resize (num_corners);

  for (; n < num_corners; n++)
  {
    scores[n].idx   = n;
    scores[n].score = computeCornerScore (im + static_cast<size_t> (corners_all[n].v) * width_ + static_cast<size_t> (corners_all[n].u));
  }
}


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
namespace pcl
{
  namespace keypoints
  {
    namespace agast
    {
      ///////////////////////////////////////////////////////////////////////////////////
      // Helper method for AgastDetector7_12s::detect
      template <typename T1, typename T2> void
      AgastDetector7_12s_detect (
          const T1* im, 
          int img_width, int img_height,
          double threshold,
          int_fast16_t s_offset0,
          int_fast16_t s_offset1,
          int_fast16_t s_offset2,
          int_fast16_t s_offset3,
          int_fast16_t s_offset4,
          int_fast16_t s_offset5,
          int_fast16_t s_offset6,
          int_fast16_t s_offset7,
          int_fast16_t s_offset8,
          int_fast16_t s_offset9,
          int_fast16_t s_offset10,
          int_fast16_t s_offset11,
          std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> >& corners)
      {
        int total = 0;
        int n_expected_corners = int (corners.capacity ());
        pcl::PointUV h;
        int x, y;
        int width_b  = img_width - 3; //2, +1 due to faster test x>width_b
        int height_b = img_height - 2;
        int_fast16_t offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7, offset8, offset9, offset10, offset11;
        int width;

        corners.resize (0);

        offset0  = s_offset0;
        offset1  = s_offset1;
        offset2  = s_offset2;
        offset3  = s_offset3;
        offset4  = s_offset4;
        offset5  = s_offset5;
        offset6  = s_offset6;
        offset7  = s_offset7;
        offset8  = s_offset8;
        offset9  = s_offset9;
        offset10 = s_offset10;
        offset11 = s_offset11;
        width    = img_width;

        for (y = 2; y < height_b; y++)
        {                    
          x = 1;
          while (1)              
          {                  
      homogeneous:
      {
            x++;      
            if (x > width_b)  
              break;
            else
            {
              const T1* const p = im + y * width + x;
              const T2 cb = *p + T2 (threshold);
              const T2 c_b = *p - T2 (threshold);
              if (p[offset0] > cb)
                if (p[offset2] > cb)
                if (p[offset5] > cb)
                  if (p[offset9] > cb)
                  if (p[offset7] > cb)
                    if (p[offset1] > cb)
                    if (p[offset6] > cb)
                      if (p[offset3] > cb)
                      if (p[offset4] > cb)
                        goto success_structured;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto homogeneous;
                      else
                      if (p[offset8] > cb)
                        if (p[offset10] > cb)
                        if (p[offset4] > cb)
                          goto success_structured;
                        else
                          if (p[offset11] > cb)
                          goto success_structured;
                          else
                          goto structured;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset11] > cb)
                      if (p[offset3] > cb)
                        if (p[offset4] > cb)
                        goto success_structured;
                        else
                        if (p[offset10] > cb)
                          goto success_structured;
                        else
                          goto homogeneous;
                      else
                        if (p[offset8] > cb)
                        if (p[offset10] > cb)
                          goto success_structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    if (p[offset6] > cb)
                      if (p[offset8] > cb)
                      if (p[offset4] > cb)
                        if (p[offset3] > cb)
                        goto success_structured;
                        else
                        if (p[offset10] > cb)
                          goto success_structured;
                        else
                          goto homogeneous;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    if (p[offset1] > cb)
                    if (p[offset11] > cb)
                      if (p[offset3] > cb)
                      if (p[offset4] > cb)
                        goto success_homogeneous;
                      else
                        if (p[offset10] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset8] > cb)
                        if (p[offset10] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset6] > cb)
                      if (p[offset3] > cb)
                        if (p[offset4] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  if (p[offset3] > cb)
                    if (p[offset4] > cb)
                    if (p[offset7] > cb)
                      if (p[offset1] > cb)
                      if (p[offset6] > cb)
                        goto success_homogeneous;
                      else
                        if (p[offset11] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset6] > cb)
                        if (p[offset8] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset1] > cb)
                      if (p[offset6] > cb)
                        goto success_homogeneous;
                      else
                        if (p[offset11] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                else
                  if (p[offset9] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset5] < c_b)
                    if (p[offset1] > cb)
                      if (p[offset4] > cb)
                      if (p[offset10] > cb)
                        if (p[offset3] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto homogeneous;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                          if (p[offset11] < c_b)
                          if (p[offset10] < c_b)
                            goto success_structured;
                          else
                            goto structured;
                          else
                          goto structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                          if (p[offset4] < c_b)
                          goto success_structured;
                          else
                          if (p[offset11] < c_b)
                            goto success_structured;
                          else
                            goto structured;
                        else
                          if (p[offset3] < c_b)
                          if (p[offset4] < c_b)
                            goto success_structured;
                          else
                            goto structured;
                          else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset6] < c_b)
                      if (p[offset8] < c_b)
                        if (p[offset4] < c_b)
                        if (p[offset3] < c_b)
                          goto success_structured;
                        else
                          if (p[offset10] < c_b)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                          goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    if (p[offset1] > cb)
                      if (p[offset3] > cb)
                      if (p[offset4] > cb)
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    if (p[offset1] > cb)
                    if (p[offset3] > cb)
                      if (p[offset4] > cb)
                      if (p[offset10] > cb)
                        if (p[offset11] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  if (p[offset10] > cb)
                    if (p[offset11] > cb)
                    if (p[offset9] > cb)
                      if (p[offset7] > cb)
                      if (p[offset1] > cb)
                        if (p[offset3] > cb)
                        goto success_homogeneous;
                        else
                        if (p[offset8] > cb)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                      else
                        if (p[offset6] > cb)
                        if (p[offset8] > cb)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset1] > cb)
                        if (p[offset3] > cb)
                        goto success_homogeneous;
                        else
                        if (p[offset8] > cb)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset1] > cb)
                      if (p[offset3] > cb)
                        if (p[offset4] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                else
                if (p[offset7] > cb)
                  if (p[offset9] > cb)
                  if (p[offset8] > cb)
                    if (p[offset5] > cb)
                    if (p[offset1] > cb)
                      if (p[offset10] > cb)
                      if (p[offset11] > cb)
                        goto success_homogeneous;
                      else
                        if (p[offset6] > cb)
                        if (p[offset4] > cb)
                          goto success_structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset6] > cb)
                        if (p[offset3] > cb)
                        if (p[offset4] > cb)
                          goto success_structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset6] > cb)
                      if (p[offset4] > cb)
                        if (p[offset3] > cb)
                        goto success_homogeneous;
                        else
                        if (p[offset10] > cb)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    if (p[offset10] > cb)
                      if (p[offset11] > cb)
                      if (p[offset1] > cb)
                        goto success_homogeneous;
                      else
                        if (p[offset6] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                  else
                  goto homogeneous;
                else
                  if (p[offset7] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset2] < c_b)
                    if (p[offset6] < c_b)
                      if (p[offset4] < c_b)
                      if (p[offset3] < c_b)
                        if (p[offset1] < c_b)
                        goto success_homogeneous;
                        else
                        if (p[offset8] < c_b)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                      else
                        if (p[offset9] < c_b)
                        if (p[offset8] < c_b)
                          if (p[offset10] < c_b)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset9] < c_b)
                        if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    if (p[offset9] < c_b)
                      if (p[offset6] < c_b)
                      if (p[offset8] < c_b)
                        if (p[offset4] < c_b)
                        if (p[offset3] < c_b)
                          goto success_homogeneous;
                        else
                          if (p[offset10] < c_b)
                          goto success_homogeneous;
                          else
                          goto homogeneous;
                        else
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          goto success_homogeneous;
                          else
                          goto homogeneous;
                        else
                          goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                  else
                  goto homogeneous;
              else if (p[offset0] < c_b)
                if (p[offset2] < c_b)
                if (p[offset9] < c_b)
                  if (p[offset5] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset1] < c_b)
                    if (p[offset6] < c_b)
                      if (p[offset3] < c_b)
                      if (p[offset4] < c_b)
                        goto success_structured;
                      else
                        if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto homogeneous;
                      else
                      if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                        if (p[offset4] < c_b)
                          goto success_structured;
                        else
                          if (p[offset11] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset11] < c_b)
                      if (p[offset3] < c_b)
                        if (p[offset4] < c_b)
                        goto success_structured;
                        else
                        if (p[offset10] < c_b)
                          goto success_structured;
                        else
                          goto homogeneous;
                      else
                        if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                          goto success_structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    if (p[offset6] < c_b)
                      if (p[offset8] < c_b)
                      if (p[offset4] < c_b)
                        if (p[offset3] < c_b)
                        goto success_structured;
                        else
                        if (p[offset10] < c_b)
                          goto success_structured;
                        else
                          goto homogeneous;
                      else
                        if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          goto success_structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    if (p[offset1] < c_b)
                    if (p[offset11] < c_b)
                      if (p[offset3] < c_b)
                      if (p[offset4] < c_b)
                        goto success_homogeneous;
                      else
                        if (p[offset10] < c_b)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset6] < c_b)
                      if (p[offset3] < c_b)
                        if (p[offset4] < c_b)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  if (p[offset10] < c_b)
                    if (p[offset11] < c_b)
                    if (p[offset7] < c_b)
                      if (p[offset1] < c_b)
                      if (p[offset3] < c_b)
                        goto success_homogeneous;
                      else
                        if (p[offset8] < c_b)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset1] < c_b)
                      if (p[offset3] < c_b)
                        goto success_homogeneous;
                      else
                        if (p[offset8] < c_b)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                else
                  if (p[offset9] > cb)
                  if (p[offset5] > cb)
                    if (p[offset7] > cb)
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                      if (p[offset10] < c_b)
                        if (p[offset3] < c_b)
                        if (p[offset11] < c_b)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto homogeneous;
                      else
                        if (p[offset6] > cb)
                        if (p[offset8] > cb)
                          if (p[offset11] > cb)
                          if (p[offset10] > cb)
                            goto success_structured;
                          else
                            goto structured;
                          else
                          goto structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset6] > cb)
                        if (p[offset8] > cb)
                        if (p[offset10] > cb)
                          if (p[offset4] > cb)
                          goto success_structured;
                          else
                          if (p[offset11] > cb)
                            goto success_structured;
                          else
                            goto structured;
                        else
                          if (p[offset3] > cb)
                          if (p[offset4] > cb)
                            goto success_structured;
                          else
                            goto structured;
                          else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset6] > cb)
                      if (p[offset8] > cb)
                        if (p[offset4] > cb)
                        if (p[offset3] > cb)
                          goto success_structured;
                        else
                          if (p[offset10] > cb)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                        if (p[offset10] > cb)
                          if (p[offset11] > cb)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                          goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    if (p[offset1] < c_b)
                      if (p[offset3] < c_b)
                      if (p[offset4] < c_b)
                        if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          goto success_structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    if (p[offset3] < c_b)
                    if (p[offset4] < c_b)
                      if (p[offset5] < c_b)
                      if (p[offset7] < c_b)
                        if (p[offset1] < c_b)
                        if (p[offset6] < c_b)
                          goto success_structured;
                        else
                          if (p[offset11] < c_b)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                        if (p[offset6] < c_b)
                          if (p[offset8] < c_b)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                          goto homogeneous;
                      else
                        if (p[offset1] < c_b)
                        if (p[offset6] < c_b)
                          goto success_homogeneous;
                        else
                          if (p[offset11] < c_b)
                          goto success_homogeneous;
                          else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset1] < c_b)
                        if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  if (p[offset3] < c_b)
                    if (p[offset4] < c_b)
                    if (p[offset5] < c_b)
                      if (p[offset7] < c_b)
                      if (p[offset1] < c_b)
                        if (p[offset6] < c_b)
                        goto success_homogeneous;
                        else
                        if (p[offset11] < c_b)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset1] < c_b)
                        if (p[offset6] < c_b)
                        goto success_homogeneous;
                        else
                        if (p[offset11] < c_b)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset1] < c_b)
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                else
                if (p[offset7] > cb)
                  if (p[offset5] > cb)
                  if (p[offset2] > cb)
                    if (p[offset6] > cb)
                    if (p[offset4] > cb)
                      if (p[offset3] > cb)
                      if (p[offset1] > cb)
                        goto success_homogeneous;
                      else
                        if (p[offset8] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset9] > cb)
                        if (p[offset8] > cb)
                        if (p[offset10] > cb)
                          goto success_structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset9] > cb)
                      if (p[offset8] > cb)
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    if (p[offset9] > cb)
                    if (p[offset6] > cb)
                      if (p[offset8] > cb)
                      if (p[offset4] > cb)
                        if (p[offset3] > cb)
                        goto success_homogeneous;
                        else
                        if (p[offset10] > cb)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  goto homogeneous;
                else
                  if (p[offset7] < c_b)
                  if (p[offset9] < c_b)
                    if (p[offset8] < c_b)
                    if (p[offset5] < c_b)
                      if (p[offset1] < c_b)
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                        goto success_homogeneous;
                        else
                        if (p[offset6] < c_b)
                          if (p[offset4] < c_b)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                          goto homogeneous;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset3] < c_b)
                          if (p[offset4] < c_b)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset6] < c_b)
                        if (p[offset4] < c_b)
                        if (p[offset3] < c_b)
                          goto success_homogeneous;
                        else
                          if (p[offset10] < c_b)
                          goto success_homogeneous;
                          else
                          goto homogeneous;
                        else
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          goto success_homogeneous;
                          else
                          goto homogeneous;
                        else
                          goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset10] < c_b)
                      if (p[offset11] < c_b)
                        if (p[offset1] < c_b)
                        goto success_homogeneous;
                        else
                        if (p[offset6] < c_b)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                  else
                  goto homogeneous;
              else
                if (p[offset5] > cb)
                if (p[offset7] > cb)
                  if (p[offset9] > cb)
                  if (p[offset6] > cb)
                    if (p[offset4] > cb)
                    if (p[offset3] > cb)
                      if (p[offset8] > cb)
                      goto success_homogeneous;
                      else
                      if (p[offset1] > cb)
                        if (p[offset2] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset8] > cb)
                      if (p[offset10] > cb)
                        goto success_homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    if (p[offset11] > cb)
                      if (p[offset8] > cb)
                      if (p[offset10] > cb)
                        goto success_homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                  else
                  if (p[offset2] > cb)
                    if (p[offset3] > cb)
                    if (p[offset4] > cb)
                      if (p[offset1] > cb)
                      if (p[offset6] > cb)
                        goto success_homogeneous;
                      else
                        goto homogeneous;
                      else
                      if (p[offset6] > cb)
                        if (p[offset8] > cb)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
                else
                if (p[offset5] < c_b)
                  if (p[offset7] < c_b)
                  if (p[offset9] < c_b)
                    if (p[offset6] < c_b)
                    if (p[offset4] < c_b)
                      if (p[offset3] < c_b)
                      if (p[offset8] < c_b)
                        goto success_homogeneous;
                      else
                        if (p[offset1] < c_b)
                        if (p[offset2] < c_b)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                    else
                      if (p[offset11] < c_b)
                      if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    if (p[offset2] < c_b)
                    if (p[offset3] < c_b)
                      if (p[offset4] < c_b)
                      if (p[offset1] < c_b)
                        if (p[offset6] < c_b)
                        goto success_homogeneous;
                        else
                        goto homogeneous;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                          goto success_homogeneous;
                        else
                          goto homogeneous;
                        else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  goto homogeneous;
                else
                  goto homogeneous;
            }
      }
      structured:
      {
            x++;      
            if (x > width_b)  
              break;    
            else
            {
              const T1* const p = im + y * width + x;
              const T2 cb = *p + T2 (threshold);
              const T2 c_b = *p - T2 (threshold);
              if (p[offset0] > cb)
                if (p[offset2] > cb)
                if (p[offset5] > cb)
                  if (p[offset9] > cb)
                  if (p[offset7] > cb)
                    if (p[offset1] > cb)
                    if (p[offset6] > cb)
                      if (p[offset3] > cb)
                      if (p[offset4] > cb)
                        goto success_structured;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      if (p[offset8] > cb)
                        if (p[offset10] > cb)
                        if (p[offset4] > cb)
                          goto success_structured;
                        else
                          if (p[offset11] > cb)
                          goto success_structured;
                          else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset11] > cb)
                      if (p[offset3] > cb)
                        if (p[offset4] > cb)
                        goto success_structured;
                        else
                        if (p[offset10] > cb)
                          goto success_structured;
                        else
                          goto structured;
                      else
                        if (p[offset8] > cb)
                        if (p[offset10] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      goto structured;
                    else
                    if (p[offset6] > cb)
                      if (p[offset8] > cb)
                      if (p[offset4] > cb)
                        if (p[offset3] > cb)
                        goto success_structured;
                        else
                        if (p[offset10] > cb)
                          goto success_structured;
                        else
                          goto structured;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                    if (p[offset1] > cb)
                    if (p[offset11] > cb)
                      if (p[offset3] > cb)
                      if (p[offset4] > cb)
                        goto success_structured;
                      else
                        if (p[offset10] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                      if (p[offset8] > cb)
                        if (p[offset10] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset6] > cb)
                      if (p[offset3] > cb)
                        if (p[offset4] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                    goto structured;
                  else
                  if (p[offset3] > cb)
                    if (p[offset4] > cb)
                    if (p[offset7] > cb)
                      if (p[offset1] > cb)
                      if (p[offset6] > cb)
                        goto success_structured;
                      else
                        if (p[offset11] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                      if (p[offset6] > cb)
                        if (p[offset8] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset1] > cb)
                      if (p[offset6] > cb)
                        goto success_structured;
                      else
                        if (p[offset11] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                      goto structured;
                    else
                    goto structured;
                  else
                    goto structured;
                else
                  if (p[offset7] < c_b)
                  if (p[offset9] < c_b)
                    if (p[offset5] < c_b)
                    if (p[offset1] > cb)
                      if (p[offset4] > cb)
                      if (p[offset10] > cb)
                        if (p[offset3] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                          if (p[offset11] < c_b)
                          if (p[offset10] < c_b)
                            goto success_structured;
                          else
                            goto structured;
                          else
                          goto structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                          if (p[offset4] < c_b)
                          goto success_structured;
                          else
                          if (p[offset11] < c_b)
                            goto success_structured;
                          else
                            goto structured;
                        else
                          if (p[offset3] < c_b)
                          if (p[offset4] < c_b)
                            goto success_structured;
                          else
                            goto structured;
                          else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset6] < c_b)
                      if (p[offset8] < c_b)
                        if (p[offset4] < c_b)
                        if (p[offset3] < c_b)
                          goto success_structured;
                        else
                          if (p[offset10] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                          goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                    if (p[offset1] > cb)
                      if (p[offset3] > cb)
                      if (p[offset4] > cb)
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                    if (p[offset10] > cb)
                    if (p[offset11] > cb)
                      if (p[offset9] > cb)
                      if (p[offset1] > cb)
                        if (p[offset3] > cb)
                        goto success_structured;
                        else
                        if (p[offset8] > cb)
                          goto success_structured;
                        else
                          goto structured;
                      else
                        goto structured;
                      else
                      if (p[offset1] > cb)
                        if (p[offset3] > cb)
                        if (p[offset4] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      goto structured;
                    else
                    goto structured;
                  else
                  if (p[offset10] > cb)
                    if (p[offset11] > cb)
                    if (p[offset9] > cb)
                      if (p[offset1] > cb)
                      if (p[offset3] > cb)
                        goto success_structured;
                      else
                        if (p[offset8] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                      if (p[offset6] > cb)
                        if (p[offset8] > cb)
                        if (p[offset7] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset1] > cb)
                      if (p[offset3] > cb)
                        if (p[offset4] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                    goto structured;
                  else
                    goto structured;
                else
                if (p[offset7] > cb)
                  if (p[offset9] > cb)
                  if (p[offset8] > cb)
                    if (p[offset5] > cb)
                    if (p[offset1] > cb)
                      if (p[offset10] > cb)
                      if (p[offset11] > cb)
                        goto success_structured;
                      else
                        if (p[offset6] > cb)
                        if (p[offset4] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      if (p[offset6] > cb)
                        if (p[offset3] > cb)
                        if (p[offset4] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset6] > cb)
                      if (p[offset4] > cb)
                        if (p[offset3] > cb)
                        goto success_structured;
                        else
                        if (p[offset10] > cb)
                          goto success_structured;
                        else
                          goto structured;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      goto structured;
                    else
                    if (p[offset10] > cb)
                      if (p[offset11] > cb)
                      if (p[offset1] > cb)
                        goto success_structured;
                      else
                        if (p[offset6] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                    goto structured;
                  else
                  goto structured;
                else
                  if (p[offset7] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset2] < c_b)
                    if (p[offset6] < c_b)
                      if (p[offset4] < c_b)
                      if (p[offset3] < c_b)
                        if (p[offset1] < c_b)
                        goto success_structured;
                        else
                        if (p[offset8] < c_b)
                          goto success_structured;
                        else
                          goto structured;
                      else
                        if (p[offset9] < c_b)
                        if (p[offset8] < c_b)
                          if (p[offset10] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      if (p[offset9] < c_b)
                        if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      goto structured;
                    else
                    if (p[offset9] < c_b)
                      if (p[offset6] < c_b)
                      if (p[offset8] < c_b)
                        if (p[offset4] < c_b)
                        if (p[offset3] < c_b)
                          goto success_structured;
                        else
                          if (p[offset10] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                          goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                    goto structured;
                  else
                  goto structured;
              else if (p[offset0] < c_b)
                if (p[offset2] < c_b)
                if (p[offset11] < c_b)
                  if (p[offset3] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset9] < c_b)
                    if (p[offset7] < c_b)
                      if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                        goto success_structured;
                      else
                        if (p[offset10] < c_b)
                        goto success_structured;
                        else
                        goto structured;
                      else
                      if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                        if (p[offset4] < c_b)
                          goto success_structured;
                        else
                          if (p[offset10] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                        goto success_structured;
                      else
                        if (p[offset10] < c_b)
                        goto success_structured;
                        else
                        goto structured;
                      else
                      goto structured;
                    else
                    if (p[offset4] < c_b)
                      if (p[offset7] < c_b)
                      if (p[offset1] < c_b)
                        goto success_structured;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      if (p[offset1] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto structured;
                  else
                    if (p[offset10] < c_b)
                    if (p[offset9] < c_b)
                      if (p[offset7] < c_b)
                      if (p[offset1] < c_b)
                        goto success_structured;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      if (p[offset1] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                    if (p[offset7] > cb)
                      if (p[offset9] > cb)
                      if (p[offset5] > cb)
                        if (p[offset4] > cb)
                        if (p[offset6] > cb)
                          if (p[offset8] > cb)
                          if (p[offset10] > cb)
                            goto success_structured;
                          else
                            goto structured;
                          else
                          goto structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                  if (p[offset9] < c_b)
                    if (p[offset8] < c_b)
                    if (p[offset10] < c_b)
                      if (p[offset7] < c_b)
                      if (p[offset1] < c_b)
                        goto success_structured;
                      else
                        if (p[offset6] < c_b)
                        goto success_structured;
                        else
                        goto structured;
                      else
                      if (p[offset1] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto structured;
                    else
                    goto structured;
                  else
                    if (p[offset5] > cb)
                    if (p[offset7] > cb)
                      if (p[offset9] > cb)
                      if (p[offset4] > cb)
                        if (p[offset6] > cb)
                        if (p[offset8] > cb)
                          if (p[offset3] > cb)
                          goto success_structured;
                          else
                          if (p[offset10] > cb)
                            goto success_structured;
                          else
                            goto structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                    else
                    goto structured;
                else
                  if (p[offset4] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset7] < c_b)
                    if (p[offset6] < c_b)
                      if (p[offset3] < c_b)
                      if (p[offset1] < c_b)
                        goto success_structured;
                      else
                        if (p[offset8] < c_b)
                        goto success_structured;
                        else
                        goto structured;
                      else
                      if (p[offset9] < c_b)
                        if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      goto structured;
                    else
                    if (p[offset1] < c_b)
                      if (p[offset6] < c_b)
                      if (p[offset3] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                    if (p[offset7] > cb)
                    if (p[offset9] > cb)
                      if (p[offset5] > cb)
                      if (p[offset6] > cb)
                        if (p[offset8] > cb)
                        if (p[offset10] > cb)
                          if (p[offset11] > cb)
                          goto success_structured;
                          else
                          goto structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                    else
                    goto structured;
                  else
                  if (p[offset5] > cb)
                    if (p[offset7] > cb)
                    if (p[offset9] > cb)
                      if (p[offset6] > cb)
                      if (p[offset8] > cb)
                        if (p[offset10] > cb)
                        if (p[offset4] > cb)
                          goto success_structured;
                        else
                          if (p[offset11] > cb)
                          goto success_structured;
                          else
                          goto homogeneous;
                        else
                        if (p[offset3] > cb)
                          if (p[offset4] > cb)
                          goto success_structured;
                          else
                          goto structured;
                        else
                          goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                    else
                    goto structured;
                  else
                    goto structured;
                else
                if (p[offset7] > cb)
                  if (p[offset5] > cb)
                  if (p[offset2] > cb)
                    if (p[offset6] > cb)
                    if (p[offset4] > cb)
                      if (p[offset3] > cb)
                      if (p[offset1] > cb)
                        goto success_structured;
                      else
                        if (p[offset8] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                      if (p[offset9] > cb)
                        if (p[offset8] > cb)
                        if (p[offset10] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset9] > cb)
                      if (p[offset8] > cb)
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                    goto structured;
                  else
                    if (p[offset9] > cb)
                    if (p[offset6] > cb)
                      if (p[offset8] > cb)
                      if (p[offset4] > cb)
                        if (p[offset3] > cb)
                        goto success_structured;
                        else
                        if (p[offset10] > cb)
                          goto success_structured;
                        else
                          goto structured;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                    else
                    goto structured;
                  else
                  goto structured;
                else
                  if (p[offset7] < c_b)
                  if (p[offset9] < c_b)
                    if (p[offset8] < c_b)
                    if (p[offset5] < c_b)
                      if (p[offset1] < c_b)
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                        goto success_structured;
                        else
                        if (p[offset6] < c_b)
                          if (p[offset4] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                          goto structured;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset3] < c_b)
                          if (p[offset4] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      if (p[offset6] < c_b)
                        if (p[offset4] < c_b)
                        if (p[offset3] < c_b)
                          goto success_structured;
                        else
                          if (p[offset10] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          goto success_structured;
                          else
                          goto structured;
                        else
                          goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset10] < c_b)
                      if (p[offset11] < c_b)
                        if (p[offset1] < c_b)
                        goto success_structured;
                        else
                        if (p[offset6] < c_b)
                          goto success_structured;
                        else
                          goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                    goto structured;
                  else
                    goto structured;
                  else
                  goto structured;
              else
                if (p[offset5] > cb)
                if (p[offset7] > cb)
                  if (p[offset9] > cb)
                  if (p[offset6] > cb)
                    if (p[offset4] > cb)
                    if (p[offset3] > cb)
                      if (p[offset8] > cb)
                      goto success_structured;
                      else
                      if (p[offset1] > cb)
                        if (p[offset2] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset8] > cb)
                      if (p[offset10] > cb)
                        goto success_structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                    if (p[offset11] > cb)
                      if (p[offset8] > cb)
                      if (p[offset10] > cb)
                        goto success_structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                    goto structured;
                  else
                  if (p[offset2] > cb)
                    if (p[offset3] > cb)
                    if (p[offset4] > cb)
                      if (p[offset1] > cb)
                      if (p[offset6] > cb)
                        goto success_structured;
                      else
                        goto structured;
                      else
                      if (p[offset6] > cb)
                        if (p[offset8] > cb)
                        goto success_structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      goto structured;
                    else
                    goto structured;
                  else
                    goto structured;
                else
                  goto structured;
                else
                if (p[offset5] < c_b)
                  if (p[offset7] < c_b)
                  if (p[offset9] < c_b)
                    if (p[offset6] < c_b)
                    if (p[offset4] < c_b)
                      if (p[offset3] < c_b)
                      if (p[offset8] < c_b)
                        goto success_structured;
                      else
                        if (p[offset1] < c_b)
                        if (p[offset2] < c_b)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                        goto success_structured;
                        else
                        goto structured;
                      else
                        goto structured;
                    else
                      if (p[offset11] < c_b)
                      if (p[offset8] < c_b)
                        if (p[offset10] < c_b)
                        goto success_structured;
                        else
                        goto structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                    goto structured;
                  else
                    if (p[offset2] < c_b)
                    if (p[offset3] < c_b)
                      if (p[offset4] < c_b)
                      if (p[offset1] < c_b)
                        if (p[offset6] < c_b)
                        goto success_structured;
                        else
                        goto structured;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset8] < c_b)
                          goto success_structured;
                        else
                          goto structured;
                        else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto structured;
                    else
                    goto structured;
                  else
                  goto structured;
                else
                  goto homogeneous;
            }
      }
      success_homogeneous:
            if (total == n_expected_corners)
            {
              if (n_expected_corners == 0)
              {
                n_expected_corners = 512;
                corners.reserve (n_expected_corners);
              }
              else
              {
                n_expected_corners *= 2;
                corners.reserve (n_expected_corners);
              }
            }
            h.u = float (x);
            h.v = float (y);
            corners.push_back (h);
            total++;
            goto homogeneous;
      success_structured:
            if (total == n_expected_corners)
            {
              if (n_expected_corners == 0)
              {
                n_expected_corners = 512;
                corners.reserve (n_expected_corners);
              }
              else
              {
                n_expected_corners *= 2;
                corners.reserve (n_expected_corners);
              }
            }
            h.u = float (x);
            h.v = float (y);
            corners.push_back (h);
            total++;            
            goto structured;        
          }                  
        }                    
      }

      ///////////////////////////////////////////////////////////////////////////////////
      // Helper method for AgastDetector7_12s::computeCornerScore
      template <typename T1, typename T2> int
      AgastDetector7_12s_computeCornerScore (
          const T1* p,
          double im_bmax,
          double score_threshold,
          int_fast16_t s_offset0,
          int_fast16_t s_offset1,
          int_fast16_t s_offset2,
          int_fast16_t s_offset3,
          int_fast16_t s_offset4,
          int_fast16_t s_offset5,
          int_fast16_t s_offset6,
          int_fast16_t s_offset7,
          int_fast16_t s_offset8,
          int_fast16_t s_offset9,
          int_fast16_t s_offset10,
          int_fast16_t s_offset11)
      {
        T2 bmin = T2 (score_threshold);
        T2 bmax = T2 (im_bmax); // 255;
        int b_test = int ((bmax + bmin) / 2);

        int_fast16_t offset0  = s_offset0;
        int_fast16_t offset1  = s_offset1;
        int_fast16_t offset2  = s_offset2;
        int_fast16_t offset3  = s_offset3;
        int_fast16_t offset4  = s_offset4;
        int_fast16_t offset5  = s_offset5;
        int_fast16_t offset6  = s_offset6;
        int_fast16_t offset7  = s_offset7;
        int_fast16_t offset8  = s_offset8;
        int_fast16_t offset9  = s_offset9;
        int_fast16_t offset10 = s_offset10;
        int_fast16_t offset11 = s_offset11;

        while (1)
        {
          const T2 cb = *p + T2 (b_test);
          const T2 c_b = *p - T2 (b_test);
          if (p[offset0] > cb)
            if (p[offset5] > cb)
              if (p[offset2] < c_b)
                if (p[offset7] > cb)
                  if (p[offset9] < c_b)
                    goto is_not_a_corner;
                  else
                    if (p[offset9] > cb)
                      if (p[offset1] < c_b)
                        if (p[offset6] < c_b)
                          goto is_not_a_corner;
                        else
                          if (p[offset6] > cb)
                            if (p[offset8] > cb)
                              if (p[offset4] > cb)
                                if (p[offset3] > cb)
                                  goto is_a_corner;
                                else
                                  if (p[offset10] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                if (p[offset10] > cb)
                                  if (p[offset11] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset1] > cb)
                          if (p[offset6] < c_b)
                            if (p[offset8] > cb)
                              if (p[offset10] > cb)
                                if (p[offset11] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset8] > cb)
                                if (p[offset4] > cb)
                                  if (p[offset3] > cb)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset8] > cb)
                                if (p[offset10] > cb)
                                  if (p[offset11] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                        else
                          if (p[offset6] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset8] > cb)
                                if (p[offset4] > cb)
                                  if (p[offset3] > cb)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                if (p[offset2] > cb)
                  if (p[offset7] < c_b)
                    if (p[offset9] < c_b)
                      if (p[offset1] < c_b)
                        goto is_not_a_corner;
                      else
                        if (p[offset1] > cb)
                          if (p[offset6] > cb)
                            if (p[offset3] > cb)
                              if (p[offset4] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              if (p[offset3] > cb)
                                if (p[offset4] > cb)
                                  if (p[offset11] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset3] > cb)
                                if (p[offset4] > cb)
                                  if (p[offset11] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset9] > cb)
                        if (p[offset1] < c_b)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] < c_b)
                              if (p[offset11] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                if (p[offset11] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset1] < c_b)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              if (p[offset3] > cb)
                                if (p[offset4] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                  else
                    if (p[offset9] < c_b)
                      if (p[offset7] > cb)
                        if (p[offset1] < c_b)
                          if (p[offset6] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset3] > cb)
                                if (p[offset4] > cb)
                                  if (p[offset8] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              if (p[offset3] > cb)
                                if (p[offset4] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset8] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        if (p[offset1] < c_b)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              if (p[offset3] > cb)
                                if (p[offset4] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                    else
                      if (p[offset7] > cb)
                        if (p[offset9] > cb)
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] < c_b)
                                if (p[offset11] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset4] > cb)
                                          goto is_a_corner;
                                        else
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset11] > cb)
                                    if (p[offset3] > cb)
                                      if (p[offset4] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset8] > cb)
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset8] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset8] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                      else
                        if (p[offset9] > cb)
                          if (p[offset1] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] < c_b)
                                if (p[offset11] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset11] > cb)
                                    if (p[offset3] > cb)
                                      if (p[offset4] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset8] > cb)
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                else
                  if (p[offset7] > cb)
                    if (p[offset9] < c_b)
                      goto is_not_a_corner;
                    else
                      if (p[offset9] > cb)
                        if (p[offset1] < c_b)
                          if (p[offset6] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset8] > cb)
                                if (p[offset4] > cb)
                                  if (p[offset3] > cb)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] < c_b)
                              if (p[offset8] > cb)
                                if (p[offset10] > cb)
                                  if (p[offset11] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset8] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
            else
              if (p[offset5] < c_b)
                if (p[offset9] < c_b)
                  if (p[offset7] > cb)
                    if (p[offset2] < c_b)
                      goto is_not_a_corner;
                    else
                      if (p[offset2] > cb)
                        if (p[offset1] < c_b)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              if (p[offset3] > cb)
                                if (p[offset4] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset7] < c_b)
                      if (p[offset2] < c_b)
                        if (p[offset1] > cb)
                          if (p[offset6] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              if (p[offset8] < c_b)
                                if (p[offset4] < c_b)
                                  if (p[offset3] < c_b)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset4] < c_b)
                                  if (p[offset3] < c_b)
                                    goto is_a_corner;
                                  else
                                    if (p[offset8] < c_b)
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        if (p[offset2] > cb)
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset3] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      if (p[offset8] < c_b)
                                        if (p[offset11] < c_b)
                                          if (p[offset10] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset4] < c_b)
                                          goto is_a_corner;
                                        else
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        if (p[offset3] < c_b)
                                          if (p[offset4] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                    else
                      if (p[offset2] < c_b)
                        goto is_not_a_corner;
                      else
                        if (p[offset2] > cb)
                          if (p[offset1] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                else
                  if (p[offset9] > cb)
                    if (p[offset7] < c_b)
                      if (p[offset2] > cb)
                        if (p[offset1] < c_b)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              if (p[offset10] > cb)
                                if (p[offset11] > cb)
                                  if (p[offset3] > cb)
                                    goto is_a_corner;
                                  else
                                    if (p[offset8] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset10] > cb)
                                  if (p[offset11] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset8] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset10] > cb)
                                  if (p[offset11] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset8] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset2] < c_b)
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset8] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset8] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset7] > cb)
                        if (p[offset2] < c_b)
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          if (p[offset2] > cb)
                            if (p[offset1] < c_b)
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset1] > cb)
                                if (p[offset6] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        if (p[offset3] > cb)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        if (p[offset3] > cb)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  goto is_not_a_corner;
                                else
                                  if (p[offset6] > cb)
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset1] > cb)
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  goto is_not_a_corner;
                                else
                                  if (p[offset6] > cb)
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                      else
                        if (p[offset2] < c_b)
                          goto is_not_a_corner;
                        else
                          if (p[offset2] > cb)
                            if (p[offset1] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] > cb)
                                if (p[offset6] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        if (p[offset3] > cb)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        if (p[offset3] > cb)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                  else
                    if (p[offset2] < c_b)
                      if (p[offset7] > cb)
                        goto is_not_a_corner;
                      else
                        if (p[offset7] < c_b)
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset8] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset8] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset2] > cb)
                        if (p[offset7] > cb)
                          if (p[offset1] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset7] < c_b)
                            if (p[offset1] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] > cb)
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset3] > cb)
                                      if (p[offset4] > cb)
                                        if (p[offset10] > cb)
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset3] > cb)
                                      if (p[offset4] > cb)
                                        if (p[offset10] > cb)
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] > cb)
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset3] > cb)
                                      if (p[offset4] > cb)
                                        if (p[offset10] > cb)
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset3] > cb)
                                      if (p[offset4] > cb)
                                        if (p[offset10] > cb)
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
              else
                if (p[offset2] < c_b)
                  if (p[offset7] > cb)
                    if (p[offset9] < c_b)
                      goto is_not_a_corner;
                    else
                      if (p[offset9] > cb)
                        if (p[offset1] < c_b)
                          if (p[offset6] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset8] > cb)
                                if (p[offset10] > cb)
                                  if (p[offset11] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              if (p[offset8] > cb)
                                if (p[offset10] > cb)
                                  if (p[offset11] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset8] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  if (p[offset2] > cb)
                    if (p[offset7] < c_b)
                      if (p[offset9] < c_b)
                        if (p[offset1] < c_b)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              if (p[offset3] > cb)
                                if (p[offset4] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset9] > cb)
                          if (p[offset1] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset10] > cb)
                                  if (p[offset11] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset8] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                    else
                      if (p[offset9] < c_b)
                        if (p[offset7] > cb)
                          if (p[offset1] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                      else
                        if (p[offset7] > cb)
                          if (p[offset9] > cb)
                            if (p[offset1] < c_b)
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset1] > cb)
                                if (p[offset6] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        if (p[offset3] > cb)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        if (p[offset3] > cb)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  goto is_not_a_corner;
                                else
                                  if (p[offset6] > cb)
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] > cb)
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset3] > cb)
                                      if (p[offset4] > cb)
                                        if (p[offset10] > cb)
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset3] > cb)
                                      if (p[offset4] > cb)
                                        if (p[offset10] > cb)
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                        else
                          if (p[offset9] > cb)
                            if (p[offset1] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] > cb)
                                if (p[offset6] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        if (p[offset3] > cb)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        if (p[offset3] > cb)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] > cb)
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset3] > cb)
                                      if (p[offset4] > cb)
                                        if (p[offset10] > cb)
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset3] > cb)
                                      if (p[offset4] > cb)
                                        if (p[offset10] > cb)
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                  else
                    if (p[offset7] > cb)
                      if (p[offset9] < c_b)
                        goto is_not_a_corner;
                      else
                        if (p[offset9] > cb)
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
          else if (p[offset0] < c_b)
            if (p[offset5] < c_b)
              if (p[offset9] > cb)
                if (p[offset2] > cb)
                  goto is_not_a_corner;
                else
                  if (p[offset2] < c_b)
                    if (p[offset7] > cb)
                      if (p[offset1] > cb)
                        goto is_not_a_corner;
                      else
                        if (p[offset1] < c_b)
                          if (p[offset6] < c_b)
                            if (p[offset3] < c_b)
                              if (p[offset4] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset3] < c_b)
                                if (p[offset4] < c_b)
                                  if (p[offset11] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset3] < c_b)
                                if (p[offset4] < c_b)
                                  if (p[offset11] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset7] < c_b)
                        if (p[offset1] > cb)
                          if (p[offset6] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              if (p[offset3] < c_b)
                                if (p[offset4] < c_b)
                                  if (p[offset8] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              if (p[offset3] < c_b)
                                if (p[offset4] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset8] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        if (p[offset1] > cb)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              if (p[offset3] < c_b)
                                if (p[offset4] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
              else
                if (p[offset9] < c_b)
                  if (p[offset7] > cb)
                    if (p[offset2] > cb)
                      goto is_not_a_corner;
                    else
                      if (p[offset2] < c_b)
                        if (p[offset1] > cb)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              if (p[offset11] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                if (p[offset11] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] < c_b)
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset7] < c_b)
                      if (p[offset2] > cb)
                        if (p[offset1] > cb)
                          if (p[offset6] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              if (p[offset8] < c_b)
                                if (p[offset4] < c_b)
                                  if (p[offset3] < c_b)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              if (p[offset8] < c_b)
                                if (p[offset10] < c_b)
                                  if (p[offset11] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        if (p[offset2] < c_b)
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset11] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] < c_b)
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset4] < c_b)
                                          goto is_a_corner;
                                        else
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset11] < c_b)
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset8] < c_b)
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                    else
                      if (p[offset2] > cb)
                        goto is_not_a_corner;
                      else
                        if (p[offset2] < c_b)
                          if (p[offset1] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset11] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] < c_b)
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset11] < c_b)
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset8] < c_b)
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                else
                  if (p[offset2] > cb)
                    goto is_not_a_corner;
                  else
                    if (p[offset2] < c_b)
                      if (p[offset7] > cb)
                        if (p[offset1] > cb)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              if (p[offset3] < c_b)
                                if (p[offset4] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset7] < c_b)
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset8] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset8] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
            else
              if (p[offset5] > cb)
                if (p[offset2] > cb)
                  if (p[offset7] < c_b)
                    if (p[offset9] > cb)
                      goto is_not_a_corner;
                    else
                      if (p[offset9] < c_b)
                        if (p[offset1] > cb)
                          if (p[offset6] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              if (p[offset8] < c_b)
                                if (p[offset10] < c_b)
                                  if (p[offset11] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              if (p[offset8] < c_b)
                                if (p[offset10] < c_b)
                                  if (p[offset11] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset7] > cb)
                      if (p[offset9] < c_b)
                        if (p[offset1] > cb)
                          if (p[offset6] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset3] > cb)
                                if (p[offset4] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset8] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset8] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        if (p[offset9] > cb)
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset8] > cb)
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset8] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset8] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  if (p[offset2] < c_b)
                    if (p[offset7] < c_b)
                      if (p[offset9] > cb)
                        if (p[offset1] > cb)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              if (p[offset3] < c_b)
                                if (p[offset4] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset9] < c_b)
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset10] < c_b)
                                  if (p[offset11] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset8] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                    else
                      if (p[offset7] > cb)
                        if (p[offset9] < c_b)
                          if (p[offset1] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset10] < c_b)
                                  if (p[offset11] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset8] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset9] > cb)
                            if (p[offset1] > cb)
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset1] < c_b)
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] > cb)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] > cb)
                                        if (p[offset8] > cb)
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        if (p[offset3] < c_b)
                                          if (p[offset11] < c_b)
                                            if (p[offset10] < c_b)
                                              goto is_a_corner;
                                            else
                                              goto is_not_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset8] > cb)
                                        if (p[offset10] > cb)
                                          if (p[offset4] > cb)
                                            goto is_a_corner;
                                          else
                                            if (p[offset11] > cb)
                                              goto is_a_corner;
                                            else
                                              goto is_not_a_corner;
                                        else
                                          if (p[offset3] > cb)
                                            if (p[offset4] > cb)
                                              goto is_a_corner;
                                            else
                                              goto is_not_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        if (p[offset10] < c_b)
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  goto is_not_a_corner;
                                else
                                  if (p[offset6] > cb)
                                    if (p[offset8] > cb)
                                      if (p[offset4] > cb)
                                        if (p[offset3] > cb)
                                          goto is_a_corner;
                                        else
                                          if (p[offset10] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          if (p[offset11] > cb)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] < c_b)
                                if (p[offset6] > cb)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        if (p[offset10] < c_b)
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        if (p[offset10] < c_b)
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        if (p[offset9] > cb)
                          if (p[offset1] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset9] < c_b)
                            if (p[offset1] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] < c_b)
                                if (p[offset6] > cb)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        if (p[offset3] < c_b)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        if (p[offset3] < c_b)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] < c_b)
                                if (p[offset6] > cb)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        if (p[offset10] < c_b)
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        if (p[offset10] < c_b)
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                  else
                    if (p[offset7] > cb)
                      if (p[offset9] < c_b)
                        goto is_not_a_corner;
                      else
                        if (p[offset9] > cb)
                          if (p[offset1] > cb)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset9] < c_b)
                        if (p[offset7] < c_b)
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
              else
                if (p[offset2] > cb)
                  if (p[offset7] < c_b)
                    if (p[offset9] > cb)
                      goto is_not_a_corner;
                    else
                      if (p[offset9] < c_b)
                        if (p[offset1] > cb)
                          if (p[offset6] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              if (p[offset8] < c_b)
                                if (p[offset10] < c_b)
                                  if (p[offset11] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              if (p[offset8] < c_b)
                                if (p[offset10] < c_b)
                                  if (p[offset11] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  if (p[offset2] < c_b)
                    if (p[offset7] > cb)
                      if (p[offset9] > cb)
                        if (p[offset1] > cb)
                          goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              if (p[offset3] < c_b)
                                if (p[offset4] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset9] < c_b)
                          if (p[offset1] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset10] < c_b)
                                  if (p[offset11] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset8] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                    else
                      if (p[offset9] > cb)
                        if (p[offset7] < c_b)
                          if (p[offset1] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                      else
                        if (p[offset7] < c_b)
                          if (p[offset9] < c_b)
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset1] < c_b)
                                if (p[offset6] > cb)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        if (p[offset3] < c_b)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        if (p[offset3] < c_b)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset8] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] < c_b)
                                if (p[offset6] > cb)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        if (p[offset10] < c_b)
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        if (p[offset10] < c_b)
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                        else
                          if (p[offset9] < c_b)
                            if (p[offset1] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] < c_b)
                                if (p[offset6] > cb)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset8] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        if (p[offset3] < c_b)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        if (p[offset3] < c_b)
                                          goto is_a_corner;
                                        else
                                          if (p[offset8] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset1] < c_b)
                                if (p[offset6] > cb)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset6] < c_b)
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        if (p[offset10] < c_b)
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    if (p[offset3] < c_b)
                                      if (p[offset4] < c_b)
                                        if (p[offset10] < c_b)
                                          if (p[offset11] < c_b)
                                            goto is_a_corner;
                                          else
                                            goto is_not_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                  else
                    if (p[offset7] < c_b)
                      if (p[offset9] > cb)
                        goto is_not_a_corner;
                      else
                        if (p[offset9] < c_b)
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                if (p[offset8] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
          else
            if (p[offset5] < c_b)
              if (p[offset7] > cb)
                goto is_not_a_corner;
              else
                if (p[offset7] < c_b)
                  if (p[offset2] > cb)
                    if (p[offset9] > cb)
                      goto is_not_a_corner;
                    else
                      if (p[offset9] < c_b)
                        if (p[offset1] > cb)
                          if (p[offset6] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              if (p[offset8] < c_b)
                                if (p[offset4] < c_b)
                                  if (p[offset3] < c_b)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset2] < c_b)
                      if (p[offset9] > cb)
                        if (p[offset1] < c_b)
                          if (p[offset6] > cb)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              if (p[offset3] < c_b)
                                if (p[offset4] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset8] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset8] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        if (p[offset9] < c_b)
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset8] < c_b)
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] < c_b)
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset3] < c_b)
                                  if (p[offset4] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset8] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset3] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset8] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                    else
                      if (p[offset9] > cb)
                        goto is_not_a_corner;
                      else
                        if (p[offset9] < c_b)
                          if (p[offset1] > cb)
                            if (p[offset6] > cb)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset4] < c_b)
                                    if (p[offset3] < c_b)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] < c_b)
                                      if (p[offset11] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] < c_b)
                                  if (p[offset8] < c_b)
                                    if (p[offset4] < c_b)
                                      if (p[offset3] < c_b)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] < c_b)
                                        if (p[offset11] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                else
                  goto is_not_a_corner;
            else
              if (p[offset5] > cb)
                if (p[offset7] > cb)
                  if (p[offset2] < c_b)
                    if (p[offset9] < c_b)
                      goto is_not_a_corner;
                    else
                      if (p[offset9] > cb)
                        if (p[offset1] > cb)
                          if (p[offset6] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset8] > cb)
                                if (p[offset4] > cb)
                                  if (p[offset3] > cb)
                                    goto is_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset2] > cb)
                      if (p[offset9] < c_b)
                        if (p[offset1] > cb)
                          if (p[offset6] < c_b)
                            goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset3] > cb)
                                if (p[offset4] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset8] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset8] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                      else
                        if (p[offset9] > cb)
                          if (p[offset1] < c_b)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] > cb)
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset8] > cb)
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset8] > cb)
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          if (p[offset1] > cb)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset3] > cb)
                                  if (p[offset4] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset8] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset3] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset8] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                    else
                      if (p[offset9] < c_b)
                        goto is_not_a_corner;
                      else
                        if (p[offset9] > cb)
                          if (p[offset1] > cb)
                            if (p[offset6] < c_b)
                              goto is_not_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset4] > cb)
                                    if (p[offset3] > cb)
                                      goto is_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    if (p[offset10] > cb)
                                      if (p[offset11] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset1] < c_b)
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset6] < c_b)
                                goto is_not_a_corner;
                              else
                                if (p[offset6] > cb)
                                  if (p[offset8] > cb)
                                    if (p[offset4] > cb)
                                      if (p[offset3] > cb)
                                        goto is_a_corner;
                                      else
                                        if (p[offset10] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                    else
                                      if (p[offset10] > cb)
                                        if (p[offset11] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                goto is_not_a_corner;

          is_a_corner:
            bmin = T2 (b_test);
            goto end;

          is_not_a_corner:
            bmax = T2 (b_test);
            goto end;

          end:

          if (bmin == bmax - 1 || bmin == bmax)
            return (int (bmin));
          b_test = int ((bmin + bmax) / 2);
        }
      }
    } // namespace agast
  } // namespace keypoints
} // namespace pcl

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AgastDetector7_12s::initPattern ()
{
  s_offset0_  = static_cast<int_fast16_t> ((-2) + (0)  * width_);
  s_offset1_  = static_cast<int_fast16_t> ((-2) + (-1) * width_);
  s_offset2_  = static_cast<int_fast16_t> ((-1) + (-2) * width_);
  s_offset3_  = static_cast<int_fast16_t> ((0)  + (-2) * width_);
  s_offset4_  = static_cast<int_fast16_t> ((1)  + (-2) * width_);
  s_offset5_  = static_cast<int_fast16_t> ((2)  + (-1) * width_);
  s_offset6_  = static_cast<int_fast16_t> ((2)  + (0)  * width_);
  s_offset7_  = static_cast<int_fast16_t> ((2)  + (1)  * width_);
  s_offset8_  = static_cast<int_fast16_t> ((1)  + (2)  * width_);
  s_offset9_  = static_cast<int_fast16_t> ((0)  + (2)  * width_);
  s_offset10_ = static_cast<int_fast16_t> ((-1) + (2)  * width_);
  s_offset11_ = static_cast<int_fast16_t> ((-2) + (1)  * width_);
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AgastDetector7_12s::detect (const unsigned char* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > & corners) const
{
  return (pcl::keypoints::agast::AgastDetector7_12s_detect<unsigned char, int> (
        im, int (width_), int (height_), threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_,
        s_offset8_,
        s_offset9_,
        s_offset10_,
        s_offset11_,
        corners));
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AgastDetector7_12s::detect (const float* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > & corners) const
{
  return (pcl::keypoints::agast::AgastDetector7_12s_detect<float, float> (
        im, int (width_), int (height_), threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_,
        s_offset8_,
        s_offset9_,
        s_offset10_,
        s_offset11_,
        corners));
}

/////////////////////////////////////////////////////////////////////////////////////////
int
pcl::keypoints::agast::AgastDetector7_12s::computeCornerScore (const unsigned char* p) const
{
  return (pcl::keypoints::agast::AgastDetector7_12s_computeCornerScore<unsigned char, int> (
        p, bmax_, threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_,
        s_offset8_,
        s_offset9_,
        s_offset10_,
        s_offset11_));
}

/////////////////////////////////////////////////////////////////////////////////////////
int
pcl::keypoints::agast::AgastDetector7_12s::computeCornerScore (const float* p) const
{
  return (pcl::keypoints::agast::AgastDetector7_12s_computeCornerScore<float, float> (
        p, bmax_, threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_,
        s_offset8_,
        s_offset9_,
        s_offset10_,
        s_offset11_));
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
namespace pcl
{
  namespace keypoints
  {
    namespace agast
    {
      ///////////////////////////////////////////////////////////////////////////////////
      // Helper method for AgastDetector5_8::detect
      template <typename T1, typename T2> void
      AgastDetector5_8_detect (
          const T1* im, 
          int img_width, int img_height,
          double threshold,
          int_fast16_t s_offset0,
          int_fast16_t s_offset1,
          int_fast16_t s_offset2,
          int_fast16_t s_offset3,
          int_fast16_t s_offset4,
          int_fast16_t s_offset5,
          int_fast16_t s_offset6,
          int_fast16_t s_offset7,
          std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> >& corners)
      {
        int total = 0;
        int n_expected_corners = int (corners.capacity ());
        pcl::PointUV h;
        int x, y;
        int xsize_b = int (img_width) - 2;
        int ysize_b = int (img_height) - 1;
        int_fast16_t offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7;
        int width;

        corners.resize (0);

        offset0 = s_offset0;
        offset1 = s_offset1;
        offset2 = s_offset2;
        offset3 = s_offset3;
        offset4 = s_offset4;
        offset5 = s_offset5;
        offset6 = s_offset6;
        offset7 = s_offset7;
        width   = int (img_width);

        for (y = 1; y < ysize_b; y++)
        {
          x = 0;
          while (1)
          { 
      homogeneous:
      {
            x++;
            if (x > xsize_b)
              break;
            else
            {
              const T1* const p = im + y * width + x;
              const T2 cb = *p + T2 (threshold);
              const T2 c_b = *p - T2 (threshold);
              if (p[offset0] > cb)
                if (p[offset2] > cb)
                if (p[offset3] > cb)
                  if (p[offset5] > cb)
                  if (p[offset1] > cb)
                    if (p[offset4] > cb)
                    goto success_structured;
                    else
                    if (p[offset7] > cb)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else
                    if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_structured;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  if (p[offset1] > cb)
                    if (p[offset4] > cb)
                    goto success_homogeneous;
                    else
                    if (p[offset7] > cb)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else
                  if (p[offset7] > cb)
                  if (p[offset6] > cb)
                    if (p[offset5] > cb)
                    if (p[offset1] > cb)
                      goto success_structured;
                    else
                      if (p[offset4] > cb)
                      goto success_structured;
                      else
                      goto homogeneous;
                    else
                    if (p[offset1] > cb)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                  else
                  if (p[offset5] < c_b)
                    if (p[offset3] < c_b)
                    if (p[offset7] < c_b)
                      if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                else
                if (p[offset5] > cb)
                  if (p[offset7] > cb)
                  if (p[offset6] > cb)
                    if (p[offset1] > cb)
                    goto success_homogeneous;
                    else
                    if (p[offset4] > cb)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                  else
                  goto homogeneous;
                else
                  if (p[offset5] < c_b)
                  if (p[offset3] < c_b)
                    if (p[offset2] < c_b)
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                      goto success_structured;
                      else
                      goto homogeneous;
                    else
                      if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    if (p[offset7] < c_b)
                      if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                  else
                  goto homogeneous;
              else if (p[offset0] < c_b)
                if (p[offset2] < c_b)
                if (p[offset7] > cb)
                  if (p[offset3] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto structured;
                    else
                    if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                      goto success_structured;
                      else
                      goto structured;
                    else
                      goto homogeneous;
                  else
                    if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  if (p[offset5] > cb)
                    if (p[offset3] > cb)
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                      goto success_structured;
                      else
                      goto structured;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                else
                  if (p[offset7] < c_b)
                  if (p[offset3] < c_b)
                    if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      goto success_structured;
                    else
                      if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                      else
                      goto homogeneous;
                    else
                    if (p[offset1] < c_b)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                  else
                    if (p[offset6] < c_b)
                    if (p[offset5] < c_b)
                      if (p[offset1] < c_b)
                      goto success_structured;
                      else
                      if (p[offset4] < c_b)
                        goto success_structured;
                      else
                        goto homogeneous;
                    else
                      if (p[offset1] < c_b)
                      goto success_homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  if (p[offset3] < c_b)
                    if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                      goto success_structured;
                      else
                      goto homogeneous;
                    else
                      if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                      goto success_homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else
                if (p[offset5] > cb)
                  if (p[offset3] > cb)
                  if (p[offset2] > cb)
                    if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto success_structured;
                    else
                      goto homogeneous;
                    else
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                      goto success_structured;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    if (p[offset7] > cb)
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                      goto success_structured;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  goto homogeneous;
                else
                  if (p[offset5] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset6] < c_b)
                    if (p[offset1] < c_b)
                      goto success_homogeneous;
                    else
                      if (p[offset4] < c_b)
                      goto success_homogeneous;
                      else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                  else
                  goto homogeneous;
              else
                if (p[offset3] > cb)
                if (p[offset5] > cb)
                  if (p[offset2] > cb)
                  if (p[offset1] > cb)
                    if (p[offset4] > cb)
                    goto success_homogeneous;
                    else
                    goto homogeneous;
                  else
                    if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  if (p[offset7] > cb)
                    if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
                else
                if (p[offset3] < c_b)
                  if (p[offset5] < c_b)
                  if (p[offset2] < c_b)
                    if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                    else
                    if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                      goto success_homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    if (p[offset7] < c_b)
                    if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                      goto success_homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  goto homogeneous;
                else
                  goto homogeneous;
            }
      }
      structured:
      {
            x++;      
            if (x > xsize_b)
              break;
            else
            {
              const T1* const p = im + y * width + x;
              const T2 cb = *p + T2 (threshold);
              const T2 c_b = *p - T2 (threshold);
              if (p[offset0] > cb)
                if (p[offset2] > cb)
                if (p[offset3] > cb)
                  if (p[offset5] > cb)
                  if (p[offset7] > cb)
                    if (p[offset1] > cb)
                    goto success_structured;
                    else
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                      goto success_structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                    if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto success_structured;
                    else
                      goto structured;
                    else
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                      goto success_structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                  if (p[offset7] > cb)
                    if (p[offset1] > cb)
                    goto success_structured;
                    else
                    goto structured;
                  else
                    if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto success_structured;
                    else
                      goto structured;
                    else
                    goto structured;
                else
                  if (p[offset7] > cb)
                  if (p[offset6] > cb)
                    if (p[offset5] > cb)
                    if (p[offset1] > cb)
                      goto success_structured;
                    else
                      if (p[offset4] > cb)
                      goto success_structured;
                      else
                      goto structured;
                    else
                    if (p[offset1] > cb)
                      goto success_structured;
                    else
                      goto structured;
                  else
                    goto structured;
                  else
                  if (p[offset5] < c_b)
                    if (p[offset3] < c_b)
                    if (p[offset7] < c_b)
                      if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto structured;
                else
                if (p[offset5] > cb)
                  if (p[offset7] > cb)
                  if (p[offset6] > cb)
                    if (p[offset1] > cb)
                    goto success_structured;
                    else
                    if (p[offset4] > cb)
                      goto success_structured;
                    else
                      goto structured;
                  else
                    goto structured;
                  else
                  goto structured;
                else
                  if (p[offset5] < c_b)
                  if (p[offset3] < c_b)
                    if (p[offset2] < c_b)
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                      goto success_structured;
                      else
                      goto structured;
                    else
                      if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                    if (p[offset7] < c_b)
                      if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto structured;
                  else
                  goto homogeneous;
              else if (p[offset0] < c_b)
                if (p[offset2] < c_b)
                if (p[offset7] > cb)
                  if (p[offset3] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto structured;
                    else
                    if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                      goto success_structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                    if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto structured;
                    else
                    goto structured;
                  else
                  if (p[offset5] > cb)
                    if (p[offset3] > cb)
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                      goto success_structured;
                      else
                      goto structured;
                    else
                      goto structured;
                    else
                    goto homogeneous;
                  else
                    goto structured;
                else
                  if (p[offset7] < c_b)
                  if (p[offset3] < c_b)
                    if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      goto success_structured;
                    else
                      if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                      else
                      goto structured;
                    else
                    if (p[offset1] < c_b)
                      goto success_structured;
                    else
                      goto structured;
                  else
                    if (p[offset6] < c_b)
                    if (p[offset5] < c_b)
                      if (p[offset1] < c_b)
                      goto success_structured;
                      else
                      if (p[offset4] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      if (p[offset1] < c_b)
                      goto success_structured;
                      else
                      goto structured;
                    else
                    goto structured;
                  else
                  if (p[offset3] < c_b)
                    if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                      goto success_homogeneous;
                      else
                      goto homogeneous;
                    else
                      if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_homogeneous;
                      else
                        goto homogeneous;
                      else
                      goto homogeneous;
                    else
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                      goto success_homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else
                if (p[offset5] > cb)
                  if (p[offset3] > cb)
                  if (p[offset2] > cb)
                    if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto success_structured;
                    else
                      goto structured;
                    else
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                      goto success_structured;
                      else
                      goto structured;
                    else
                      goto structured;
                  else
                    if (p[offset7] > cb)
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                      goto success_homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  goto structured;
                else
                  if (p[offset5] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset6] < c_b)
                    if (p[offset1] < c_b)
                      goto success_structured;
                    else
                      if (p[offset4] < c_b)
                      goto success_structured;
                      else
                      goto structured;
                    else
                    goto structured;
                  else
                    goto structured;
                  else
                  goto homogeneous;
              else
                if (p[offset3] > cb)
                if (p[offset5] > cb)
                  if (p[offset2] > cb)
                  if (p[offset1] > cb)
                    if (p[offset4] > cb)
                    goto success_homogeneous;
                    else
                    goto homogeneous;
                  else
                    if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  if (p[offset7] > cb)
                    if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
                else
                if (p[offset3] < c_b)
                  if (p[offset5] < c_b)
                  if (p[offset2] < c_b)
                    if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                    else
                    if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                      goto success_homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                  else
                    if (p[offset7] < c_b)
                    if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                      goto success_homogeneous;
                      else
                      goto homogeneous;
                    else
                      goto homogeneous;
                    else
                    goto homogeneous;
                  else
                  goto homogeneous;
                else
                  goto homogeneous;
            }
      }
      success_homogeneous:
            if (total == n_expected_corners)
            {
              if (n_expected_corners == 0)
              {
                n_expected_corners = 512;
                corners.reserve (n_expected_corners);
              }
              else
              {
                n_expected_corners *= 2;
                corners.reserve (n_expected_corners);
              }
            }
            h.u = float (x);
            h.v = float (y);
            corners.push_back (h);
            total++;
            goto homogeneous;
      success_structured:
            if (total == n_expected_corners)
            {
              if (n_expected_corners == 0)
              {
                n_expected_corners = 512;
                corners.reserve (n_expected_corners);
              }
              else
              {
                n_expected_corners *= 2;
                corners.reserve (n_expected_corners);
              }
            }
            h.u = float (x);
            h.v = float (y);
            corners.push_back (h);
            total++;
            goto structured;
          }
        }
      }

      ///////////////////////////////////////////////////////////////////////////////////
      // Helper method for AgastDetector5_8::computeCornerScore
      template <typename T1, typename T2> int
      AgastDetector5_8_computeCornerScore (
          const T1* p, 
          double im_bmax,
          double score_threshold,
          int_fast16_t s_offset0,
          int_fast16_t s_offset1,
          int_fast16_t s_offset2,
          int_fast16_t s_offset3,
          int_fast16_t s_offset4,
          int_fast16_t s_offset5,
          int_fast16_t s_offset6,
          int_fast16_t s_offset7)
      {
        T2 bmin = T2 (score_threshold);
        T2 bmax = T2 (im_bmax);
        int b_test = int ((bmax + bmin) / 2);

        int_fast16_t offset0 = s_offset0;
        int_fast16_t offset1 = s_offset1;
        int_fast16_t offset2 = s_offset2;
        int_fast16_t offset3 = s_offset3;
        int_fast16_t offset4 = s_offset4;
        int_fast16_t offset5 = s_offset5;
        int_fast16_t offset6 = s_offset6;
        int_fast16_t offset7 = s_offset7;

        while (1)
        {
          const T2 cb = *p + T2 (b_test);
          const T2 c_b = *p - T2 (b_test);
          if (p[offset0] > cb)
            if (p[offset2] > cb)
              if (p[offset3] > cb)
                if (p[offset5] > cb)
                  if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto is_a_corner;
                    else
                      if (p[offset7] > cb)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto is_a_corner;
                    else
                      if (p[offset7] > cb)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
              else
                if (p[offset7] > cb)
                  if (p[offset6] > cb)
                    if (p[offset5] > cb)
                      if (p[offset1] > cb)
                        goto is_a_corner;
                      else
                        if (p[offset4] > cb)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset1] > cb)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  if (p[offset5] < c_b)
                    if (p[offset3] < c_b)
                      if (p[offset7] < c_b)
                        if (p[offset4] < c_b)
                          if (p[offset6] < c_b)
                            goto is_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
            else
              if (p[offset5] > cb)
                if (p[offset7] > cb)
                  if (p[offset6] > cb)
                    if (p[offset1] > cb)
                      goto is_a_corner;
                    else
                      if (p[offset4] > cb)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                if (p[offset5] < c_b)
                  if (p[offset3] < c_b)
                    if (p[offset2] < c_b)
                      if (p[offset1] < c_b)
                        if (p[offset4] < c_b)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        if (p[offset4] < c_b)
                          if (p[offset6] < c_b)
                            goto is_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset7] < c_b)
                        if (p[offset4] < c_b)
                          if (p[offset6] < c_b)
                            goto is_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
          else if (p[offset0] < c_b)
            if (p[offset2] < c_b)
              if (p[offset7] > cb)
                if (p[offset3] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      if (p[offset4] < c_b)
                        if (p[offset6] < c_b)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  if (p[offset5] > cb)
                    if (p[offset3] > cb)
                      if (p[offset4] > cb)
                        if (p[offset6] > cb)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
              else
                if (p[offset7] < c_b)
                  if (p[offset3] < c_b)
                    if (p[offset5] < c_b)
                      if (p[offset1] < c_b)
                        goto is_a_corner;
                      else
                        if (p[offset4] < c_b)
                          if (p[offset6] < c_b)
                            goto is_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset1] < c_b)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset6] < c_b)
                      if (p[offset5] < c_b)
                        if (p[offset1] < c_b)
                          goto is_a_corner;
                        else
                          if (p[offset4] < c_b)
                            goto is_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset1] < c_b)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  if (p[offset3] < c_b)
                    if (p[offset5] < c_b)
                      if (p[offset1] < c_b)
                        if (p[offset4] < c_b)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        if (p[offset4] < c_b)
                          if (p[offset6] < c_b)
                            goto is_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset1] < c_b)
                        if (p[offset4] < c_b)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
            else
              if (p[offset5] > cb)
                if (p[offset3] > cb)
                  if (p[offset2] > cb)
                    if (p[offset1] > cb)
                      if (p[offset4] > cb)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      if (p[offset4] > cb)
                        if (p[offset6] > cb)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset7] > cb)
                      if (p[offset4] > cb)
                        if (p[offset6] > cb)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                if (p[offset5] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset6] < c_b)
                      if (p[offset1] < c_b)
                        goto is_a_corner;
                      else
                        if (p[offset4] < c_b)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
          else
            if (p[offset3] > cb)
              if (p[offset5] > cb)
                if (p[offset2] > cb)
                  if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto is_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  if (p[offset7] > cb)
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
              else
                goto is_not_a_corner;
            else
              if (p[offset3] < c_b)
                if (p[offset5] < c_b)
                  if (p[offset2] < c_b)
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                        goto is_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      if (p[offset4] < c_b)
                        if (p[offset6] < c_b)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset7] < c_b)
                      if (p[offset4] < c_b)
                        if (p[offset6] < c_b)
                          goto is_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                goto is_not_a_corner;

          is_a_corner:
            bmin = T2 (b_test);
            goto end;

          is_not_a_corner:
            bmax = T2 (b_test);
            goto end;

          end:

          if (bmin == bmax - 1 || bmin == bmax)
            return (int (bmin));
          b_test = int ((bmin + bmax) / 2);
        }
      }
    } // namespace agast
  } // namespace keypoints
} // namespace pcl

/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AgastDetector5_8::initPattern ()
{
  s_offset0_ = static_cast<int_fast16_t> ((-1) + (0)  * width_);
  s_offset1_ = static_cast<int_fast16_t> ((-1) + (-1) * width_);
  s_offset2_ = static_cast<int_fast16_t> ((0)  + (-1) * width_);
  s_offset3_ = static_cast<int_fast16_t> ((1)  + (-1) * width_);
  s_offset4_ = static_cast<int_fast16_t> ((1)  + (0)  * width_);
  s_offset5_ = static_cast<int_fast16_t> ((1)  + (1)  * width_);
  s_offset6_ = static_cast<int_fast16_t> ((0)  + (1)  * width_);
  s_offset7_ = static_cast<int_fast16_t> ((-1) + (1)  * width_);
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AgastDetector5_8::detect (const unsigned char* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > & corners) const
{
  return (pcl::keypoints::agast::AgastDetector5_8_detect<unsigned char, int> (
        im, int (width_), int (height_), threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_,
        corners));
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AgastDetector5_8::detect (const float* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > & corners) const
{
  return (pcl::keypoints::agast::AgastDetector5_8_detect<float, float> (
        im, int (width_), int (height_), threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_,
        corners));
}

/////////////////////////////////////////////////////////////////////////////////////////
int
pcl::keypoints::agast::AgastDetector5_8::computeCornerScore (const unsigned char* p) const
{
  return (pcl::keypoints::agast::AgastDetector5_8_computeCornerScore<unsigned char, int> (
        p, bmax_, threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_));
}

/////////////////////////////////////////////////////////////////////////////////////////
int
pcl::keypoints::agast::AgastDetector5_8::computeCornerScore (const float* p) const
{
  return (pcl::keypoints::agast::AgastDetector5_8_computeCornerScore<float, float> (
        p, bmax_, threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_));
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
namespace pcl
{
  namespace keypoints
  {
    namespace agast
    {
      ///////////////////////////////////////////////////////////////////////////////////
      // Helper method for OastDetector9_16::detect
      template <typename T1, typename T2> void
      OastDetector9_16_detect (
          const T1* im, 
          int img_width, int img_height,
          double threshold,
          int_fast16_t s_offset0,
          int_fast16_t s_offset1,
          int_fast16_t s_offset2,
          int_fast16_t s_offset3,
          int_fast16_t s_offset4,
          int_fast16_t s_offset5,
          int_fast16_t s_offset6,
          int_fast16_t s_offset7,
          int_fast16_t s_offset8,
          int_fast16_t s_offset9,
          int_fast16_t s_offset10,
          int_fast16_t s_offset11,
          int_fast16_t s_offset12,
          int_fast16_t s_offset13,
          int_fast16_t s_offset14,
          int_fast16_t s_offset15,
          std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> >& corners)
      {
        int total = 0;
        int n_expected_corners = int (corners.capacity ());
        pcl::PointUV h;
        int x, y;
        int xsize_b = int (img_width) - 4;
        int ysize_b = int (img_height) - 3;
        int_fast16_t offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7, offset8, offset9, offset10, offset11, offset12, offset13, offset14, offset15;
        int width;

        corners.resize (0);

        offset0  = s_offset0;
        offset1  = s_offset1;
        offset2  = s_offset2;
        offset3  = s_offset3;
        offset4  = s_offset4;
        offset5  = s_offset5;
        offset6  = s_offset6;
        offset7  = s_offset7;
        offset8  = s_offset8;
        offset9  = s_offset9;
        offset10 = s_offset10;
        offset11 = s_offset11;
        offset12 = s_offset12;
        offset13 = s_offset13;
        offset14 = s_offset14;
        offset15 = s_offset15;
        width    = int (img_width);

        for (y = 3; y < ysize_b; y++)
        {
          x = 2;
          while (1)
          {
            x++;
            if (x > xsize_b)
              break;
            else
            {
              const T1* const p = im + y * width + x;
              const T2 cb = *p + T2 (threshold);
              const T2 c_b = *p - T2 (threshold);
              if (p[offset0] > cb)
                if (p[offset2] > cb)
                if (p[offset4] > cb)
                  if (p[offset5] > cb)
                  if (p[offset7] > cb)
                    if (p[offset3] > cb)
                    if (p[offset1] > cb)
                      if (p[offset6] > cb)
                      if (p[offset8] > cb)
                        {}
                      else
                        if (p[offset15] > cb)
                        {}
                        else
                        continue;
                      else
                      if (p[offset13] > cb)
                        if (p[offset14] > cb)
                        if (p[offset15] > cb)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset8] > cb)
                      if (p[offset9] > cb)
                        if (p[offset10] > cb)
                        if (p[offset6] > cb)
                          {}
                        else
                          if (p[offset11] > cb)
                          if (p[offset12] > cb)
                            if (p[offset13] > cb)
                            if (p[offset14] > cb)
                              if (p[offset15] > cb)
                              {}
                              else
                              continue;
                            else
                              continue;
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    if (p[offset10] > cb)
                      if (p[offset11] > cb)
                      if (p[offset12] > cb)
                        if (p[offset8] > cb)
                        if (p[offset9] > cb)
                          if (p[offset6] > cb)
                          {}
                          else
                          if (p[offset13] > cb)
                            if (p[offset14] > cb)
                            if (p[offset15] > cb)
                              {}
                            else
                              continue;
                            else
                            continue;
                          else
                            continue;
                        else
                          if (p[offset1] > cb)
                          if (p[offset13] > cb)
                            if (p[offset14] > cb)
                            if (p[offset15] > cb)
                              {}
                            else
                              continue;
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                        if (p[offset1] > cb)
                          if (p[offset13] > cb)
                          if (p[offset14] > cb)
                            if (p[offset15] > cb)
                            {}
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                  else if (p[offset7] < c_b)
                    if (p[offset14] > cb)
                    if (p[offset15] > cb)
                      if (p[offset1] > cb)
                      if (p[offset3] > cb)
                        if (p[offset6] > cb)
                        {}
                        else
                        if (p[offset13] > cb)
                          {}
                        else
                          continue;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          if (p[offset12] > cb)
                          if (p[offset13] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset8] > cb)
                        if (p[offset9] > cb)
                        if (p[offset10] > cb)
                          if (p[offset11] > cb)
                          if (p[offset12] > cb)
                            if (p[offset13] > cb)
                            {}
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      continue;
                    else if (p[offset14] < c_b)
                    if (p[offset8] < c_b)
                      if (p[offset9] < c_b)
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                        if (p[offset12] < c_b)
                          if (p[offset13] < c_b)
                          if (p[offset6] < c_b)
                            {}
                          else
                            if (p[offset15] < c_b)
                            {}
                            else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    if (p[offset14] > cb)
                    if (p[offset15] > cb)
                      if (p[offset1] > cb)
                      if (p[offset3] > cb)
                        if (p[offset6] > cb)
                        {}
                        else
                        if (p[offset13] > cb)
                          {}
                        else
                          continue;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          if (p[offset12] > cb)
                          if (p[offset13] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset8] > cb)
                        if (p[offset9] > cb)
                        if (p[offset10] > cb)
                          if (p[offset11] > cb)
                          if (p[offset12] > cb)
                            if (p[offset13] > cb)
                            {}
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      continue;
                    else
                    continue;
                  else if (p[offset5] < c_b)
                  if (p[offset12] > cb)
                    if (p[offset13] > cb)
                    if (p[offset14] > cb)
                      if (p[offset15] > cb)
                      if (p[offset1] > cb)
                        if (p[offset3] > cb)
                        {}
                        else
                        if (p[offset10] > cb)
                          if (p[offset11] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset8] > cb)
                        if (p[offset9] > cb)
                          if (p[offset10] > cb)
                          if (p[offset11] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset6] > cb)
                        if (p[offset7] > cb)
                        if (p[offset8] > cb)
                          if (p[offset9] > cb)
                          if (p[offset10] > cb)
                            if (p[offset11] > cb)
                            {}
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      continue;
                    else
                    continue;
                  else if (p[offset12] < c_b)
                    if (p[offset7] < c_b)
                    if (p[offset8] < c_b)
                      if (p[offset9] < c_b)
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                        if (p[offset13] < c_b)
                          if (p[offset6] < c_b)
                          {}
                          else
                          if (p[offset14] < c_b)
                            if (p[offset15] < c_b)
                            {}
                            else
                            continue;
                          else
                            continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else
                  if (p[offset12] > cb)
                    if (p[offset13] > cb)
                    if (p[offset14] > cb)
                      if (p[offset15] > cb)
                      if (p[offset1] > cb)
                        if (p[offset3] > cb)
                        {}
                        else
                        if (p[offset10] > cb)
                          if (p[offset11] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset8] > cb)
                        if (p[offset9] > cb)
                          if (p[offset10] > cb)
                          if (p[offset11] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset6] > cb)
                        if (p[offset7] > cb)
                        if (p[offset8] > cb)
                          if (p[offset9] > cb)
                          if (p[offset10] > cb)
                            if (p[offset11] > cb)
                            {}
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      continue;
                    else
                    continue;
                  else if (p[offset12] < c_b)
                    if (p[offset7] < c_b)
                    if (p[offset8] < c_b)
                      if (p[offset9] < c_b)
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                        if (p[offset13] < c_b)
                          if (p[offset14] < c_b)
                          if (p[offset6] < c_b)
                            {}
                          else
                            if (p[offset15] < c_b)
                            {}
                            else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    continue;
                else if (p[offset4] < c_b)
                  if (p[offset11] > cb)
                  if (p[offset12] > cb)
                    if (p[offset13] > cb)
                    if (p[offset10] > cb)
                      if (p[offset14] > cb)
                      if (p[offset15] > cb)
                        if (p[offset1] > cb)
                        {}
                        else
                        if (p[offset8] > cb)
                          if (p[offset9] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset6] > cb)
                        if (p[offset7] > cb)
                          if (p[offset8] > cb)
                          if (p[offset9] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset5] > cb)
                        if (p[offset6] > cb)
                        if (p[offset7] > cb)
                          if (p[offset8] > cb)
                          if (p[offset9] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset1] > cb)
                      if (p[offset3] > cb)
                        if (p[offset14] > cb)
                        if (p[offset15] > cb)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else if (p[offset11] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset8] < c_b)
                    if (p[offset9] < c_b)
                      if (p[offset10] < c_b)
                      if (p[offset6] < c_b)
                        if (p[offset5] < c_b)
                        if (p[offset3] < c_b)
                          {}
                        else
                          if (p[offset12] < c_b)
                          {}
                          else
                          continue;
                        else
                        if (p[offset12] < c_b)
                          if (p[offset13] < c_b)
                          if (p[offset14] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset12] < c_b)
                        if (p[offset13] < c_b)
                          if (p[offset14] < c_b)
                          if (p[offset15] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else
                  continue;
                else
                  if (p[offset11] > cb)
                  if (p[offset12] > cb)
                    if (p[offset13] > cb)
                    if (p[offset10] > cb)
                      if (p[offset14] > cb)
                      if (p[offset15] > cb)
                        if (p[offset1] > cb)
                        {}
                        else
                        if (p[offset8] > cb)
                          if (p[offset9] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset6] > cb)
                        if (p[offset7] > cb)
                          if (p[offset8] > cb)
                          if (p[offset9] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset5] > cb)
                        if (p[offset6] > cb)
                        if (p[offset7] > cb)
                          if (p[offset8] > cb)
                          if (p[offset9] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset1] > cb)
                      if (p[offset3] > cb)
                        if (p[offset14] > cb)
                        if (p[offset15] > cb)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else if (p[offset11] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset8] < c_b)
                    if (p[offset9] < c_b)
                      if (p[offset10] < c_b)
                      if (p[offset12] < c_b)
                        if (p[offset13] < c_b)
                        if (p[offset6] < c_b)
                          if (p[offset5] < c_b)
                          {}
                          else
                          if (p[offset14] < c_b)
                            {}
                          else
                            continue;
                        else
                          if (p[offset14] < c_b)
                          if (p[offset15] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else
                  continue;
                else if (p[offset2] < c_b)
                if (p[offset9] > cb)
                  if (p[offset10] > cb)
                  if (p[offset11] > cb)
                    if (p[offset8] > cb)
                    if (p[offset12] > cb)
                      if (p[offset13] > cb)
                      if (p[offset14] > cb)
                        if (p[offset15] > cb)
                        {}
                        else
                        if (p[offset6] > cb)
                          if (p[offset7] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset5] > cb)
                        if (p[offset6] > cb)
                          if (p[offset7] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset4] > cb)
                        if (p[offset5] > cb)
                        if (p[offset6] > cb)
                          if (p[offset7] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset3] > cb)
                      if (p[offset4] > cb)
                        if (p[offset5] > cb)
                        if (p[offset6] > cb)
                          if (p[offset7] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    if (p[offset1] > cb)
                      if (p[offset12] > cb)
                      if (p[offset13] > cb)
                        if (p[offset14] > cb)
                        if (p[offset15] > cb)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                  else
                    continue;
                  else
                  continue;
                else if (p[offset9] < c_b)
                  if (p[offset7] < c_b)
                  if (p[offset8] < c_b)
                    if (p[offset6] < c_b)
                    if (p[offset5] < c_b)
                      if (p[offset4] < c_b)
                      if (p[offset3] < c_b)
                        if (p[offset1] < c_b)
                        {}
                        else
                        if (p[offset10] < c_b)
                          {}
                        else
                          continue;
                      else
                        if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          if (p[offset12] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                        if (p[offset12] < c_b)
                          if (p[offset13] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset10] < c_b)
                      if (p[offset11] < c_b)
                        if (p[offset12] < c_b)
                        if (p[offset13] < c_b)
                          if (p[offset14] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    if (p[offset10] < c_b)
                      if (p[offset11] < c_b)
                      if (p[offset12] < c_b)
                        if (p[offset13] < c_b)
                        if (p[offset14] < c_b)
                          if (p[offset15] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                  else
                    continue;
                  else
                  continue;
                else
                  continue;
                else
                if (p[offset9] > cb)
                  if (p[offset10] > cb)
                  if (p[offset11] > cb)
                    if (p[offset8] > cb)
                    if (p[offset12] > cb)
                      if (p[offset13] > cb)
                      if (p[offset14] > cb)
                        if (p[offset15] > cb)
                        {}
                        else
                        if (p[offset6] > cb)
                          if (p[offset7] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset5] > cb)
                        if (p[offset6] > cb)
                          if (p[offset7] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset4] > cb)
                        if (p[offset5] > cb)
                        if (p[offset6] > cb)
                          if (p[offset7] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset3] > cb)
                      if (p[offset4] > cb)
                        if (p[offset5] > cb)
                        if (p[offset6] > cb)
                          if (p[offset7] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    if (p[offset1] > cb)
                      if (p[offset12] > cb)
                      if (p[offset13] > cb)
                        if (p[offset14] > cb)
                        if (p[offset15] > cb)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                  else
                    continue;
                  else
                  continue;
                else if (p[offset9] < c_b)
                  if (p[offset7] < c_b)
                  if (p[offset8] < c_b)
                    if (p[offset10] < c_b)
                    if (p[offset11] < c_b)
                      if (p[offset6] < c_b)
                      if (p[offset5] < c_b)
                        if (p[offset4] < c_b)
                        if (p[offset3] < c_b)
                          {}
                        else
                          if (p[offset12] < c_b)
                          {}
                          else
                          continue;
                        else
                        if (p[offset12] < c_b)
                          if (p[offset13] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset12] < c_b)
                        if (p[offset13] < c_b)
                          if (p[offset14] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset12] < c_b)
                        if (p[offset13] < c_b)
                        if (p[offset14] < c_b)
                          if (p[offset15] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else
                  continue;
                else
                  continue;
              else if (p[offset0] < c_b)
                if (p[offset2] > cb)
                if (p[offset9] > cb)
                  if (p[offset7] > cb)
                  if (p[offset8] > cb)
                    if (p[offset6] > cb)
                    if (p[offset5] > cb)
                      if (p[offset4] > cb)
                      if (p[offset3] > cb)
                        if (p[offset1] > cb)
                        {}
                        else
                        if (p[offset10] > cb)
                          {}
                        else
                          continue;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          if (p[offset12] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset10] > cb)
                        if (p[offset11] > cb)
                        if (p[offset12] > cb)
                          if (p[offset13] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset10] > cb)
                      if (p[offset11] > cb)
                        if (p[offset12] > cb)
                        if (p[offset13] > cb)
                          if (p[offset14] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    if (p[offset10] > cb)
                      if (p[offset11] > cb)
                      if (p[offset12] > cb)
                        if (p[offset13] > cb)
                        if (p[offset14] > cb)
                          if (p[offset15] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                  else
                    continue;
                  else
                  continue;
                else if (p[offset9] < c_b)
                  if (p[offset10] < c_b)
                  if (p[offset11] < c_b)
                    if (p[offset8] < c_b)
                    if (p[offset12] < c_b)
                      if (p[offset13] < c_b)
                      if (p[offset14] < c_b)
                        if (p[offset15] < c_b)
                        {}
                        else
                        if (p[offset6] < c_b)
                          if (p[offset7] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset5] < c_b)
                        if (p[offset6] < c_b)
                          if (p[offset7] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset4] < c_b)
                        if (p[offset5] < c_b)
                        if (p[offset6] < c_b)
                          if (p[offset7] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset3] < c_b)
                      if (p[offset4] < c_b)
                        if (p[offset5] < c_b)
                        if (p[offset6] < c_b)
                          if (p[offset7] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    if (p[offset1] < c_b)
                      if (p[offset12] < c_b)
                      if (p[offset13] < c_b)
                        if (p[offset14] < c_b)
                        if (p[offset15] < c_b)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                  else
                    continue;
                  else
                  continue;
                else
                  continue;
                else if (p[offset2] < c_b)
                if (p[offset4] > cb)
                  if (p[offset11] > cb)
                  if (p[offset7] > cb)
                    if (p[offset8] > cb)
                    if (p[offset9] > cb)
                      if (p[offset10] > cb)
                      if (p[offset6] > cb)
                        if (p[offset5] > cb)
                        if (p[offset3] > cb)
                          {}
                        else
                          if (p[offset12] > cb)
                          {}
                          else
                          continue;
                        else
                        if (p[offset12] > cb)
                          if (p[offset13] > cb)
                          if (p[offset14] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset12] > cb)
                        if (p[offset13] > cb)
                          if (p[offset14] > cb)
                          if (p[offset15] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else if (p[offset11] < c_b)
                  if (p[offset12] < c_b)
                    if (p[offset13] < c_b)
                    if (p[offset10] < c_b)
                      if (p[offset14] < c_b)
                      if (p[offset15] < c_b)
                        if (p[offset1] < c_b)
                        {}
                        else
                        if (p[offset8] < c_b)
                          if (p[offset9] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset7] < c_b)
                          if (p[offset8] < c_b)
                          if (p[offset9] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset5] < c_b)
                        if (p[offset6] < c_b)
                        if (p[offset7] < c_b)
                          if (p[offset8] < c_b)
                          if (p[offset9] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset1] < c_b)
                      if (p[offset3] < c_b)
                        if (p[offset14] < c_b)
                        if (p[offset15] < c_b)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else
                  continue;
                else if (p[offset4] < c_b)
                  if (p[offset5] > cb)
                  if (p[offset12] > cb)
                    if (p[offset7] > cb)
                    if (p[offset8] > cb)
                      if (p[offset9] > cb)
                      if (p[offset10] > cb)
                        if (p[offset11] > cb)
                        if (p[offset13] > cb)
                          if (p[offset6] > cb)
                          {}
                          else
                          if (p[offset14] > cb)
                            if (p[offset15] > cb)
                            {}
                            else
                            continue;
                          else
                            continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else if (p[offset12] < c_b)
                    if (p[offset13] < c_b)
                    if (p[offset14] < c_b)
                      if (p[offset15] < c_b)
                      if (p[offset1] < c_b)
                        if (p[offset3] < c_b)
                        {}
                        else
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset8] < c_b)
                        if (p[offset9] < c_b)
                          if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset6] < c_b)
                        if (p[offset7] < c_b)
                        if (p[offset8] < c_b)
                          if (p[offset9] < c_b)
                          if (p[offset10] < c_b)
                            if (p[offset11] < c_b)
                            {}
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else if (p[offset5] < c_b)
                  if (p[offset7] > cb)
                    if (p[offset14] > cb)
                    if (p[offset8] > cb)
                      if (p[offset9] > cb)
                      if (p[offset10] > cb)
                        if (p[offset11] > cb)
                        if (p[offset12] > cb)
                          if (p[offset13] > cb)
                          if (p[offset6] > cb)
                            {}
                          else
                            if (p[offset15] > cb)
                            {}
                            else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else if (p[offset14] < c_b)
                    if (p[offset15] < c_b)
                      if (p[offset1] < c_b)
                      if (p[offset3] < c_b)
                        if (p[offset6] < c_b)
                        {}
                        else
                        if (p[offset13] < c_b)
                          {}
                        else
                          continue;
                      else
                        if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          if (p[offset12] < c_b)
                          if (p[offset13] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset8] < c_b)
                        if (p[offset9] < c_b)
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          if (p[offset12] < c_b)
                            if (p[offset13] < c_b)
                            {}
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      continue;
                    else
                    continue;
                  else if (p[offset7] < c_b)
                    if (p[offset3] < c_b)
                    if (p[offset1] < c_b)
                      if (p[offset6] < c_b)
                      if (p[offset8] < c_b)
                        {}
                      else
                        if (p[offset15] < c_b)
                        {}
                        else
                        continue;
                      else
                      if (p[offset13] < c_b)
                        if (p[offset14] < c_b)
                        if (p[offset15] < c_b)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset8] < c_b)
                      if (p[offset9] < c_b)
                        if (p[offset10] < c_b)
                        if (p[offset6] < c_b)
                          {}
                        else
                          if (p[offset11] < c_b)
                          if (p[offset12] < c_b)
                            if (p[offset13] < c_b)
                            if (p[offset14] < c_b)
                              if (p[offset15] < c_b)
                              {}
                              else
                              continue;
                            else
                              continue;
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    if (p[offset10] < c_b)
                      if (p[offset11] < c_b)
                      if (p[offset12] < c_b)
                        if (p[offset8] < c_b)
                        if (p[offset9] < c_b)
                          if (p[offset6] < c_b)
                          {}
                          else
                          if (p[offset13] < c_b)
                            if (p[offset14] < c_b)
                            if (p[offset15] < c_b)
                              {}
                            else
                              continue;
                            else
                            continue;
                          else
                            continue;
                        else
                          if (p[offset1] < c_b)
                          if (p[offset13] < c_b)
                            if (p[offset14] < c_b)
                            if (p[offset15] < c_b)
                              {}
                            else
                              continue;
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                        if (p[offset1] < c_b)
                          if (p[offset13] < c_b)
                          if (p[offset14] < c_b)
                            if (p[offset15] < c_b)
                            {}
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                  else
                    if (p[offset14] < c_b)
                    if (p[offset15] < c_b)
                      if (p[offset1] < c_b)
                      if (p[offset3] < c_b)
                        if (p[offset6] < c_b)
                        {}
                        else
                        if (p[offset13] < c_b)
                          {}
                        else
                          continue;
                      else
                        if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          if (p[offset12] < c_b)
                          if (p[offset13] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset8] < c_b)
                        if (p[offset9] < c_b)
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          if (p[offset12] < c_b)
                            if (p[offset13] < c_b)
                            {}
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      continue;
                    else
                    continue;
                  else
                  if (p[offset12] > cb)
                    if (p[offset7] > cb)
                    if (p[offset8] > cb)
                      if (p[offset9] > cb)
                      if (p[offset10] > cb)
                        if (p[offset11] > cb)
                        if (p[offset13] > cb)
                          if (p[offset14] > cb)
                          if (p[offset6] > cb)
                            {}
                          else
                            if (p[offset15] > cb)
                            {}
                            else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else if (p[offset12] < c_b)
                    if (p[offset13] < c_b)
                    if (p[offset14] < c_b)
                      if (p[offset15] < c_b)
                      if (p[offset1] < c_b)
                        if (p[offset3] < c_b)
                        {}
                        else
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset8] < c_b)
                        if (p[offset9] < c_b)
                          if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset6] < c_b)
                        if (p[offset7] < c_b)
                        if (p[offset8] < c_b)
                          if (p[offset9] < c_b)
                          if (p[offset10] < c_b)
                            if (p[offset11] < c_b)
                            {}
                            else
                            continue;
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    continue;
                else
                  if (p[offset11] > cb)
                  if (p[offset7] > cb)
                    if (p[offset8] > cb)
                    if (p[offset9] > cb)
                      if (p[offset10] > cb)
                      if (p[offset12] > cb)
                        if (p[offset13] > cb)
                        if (p[offset6] > cb)
                          if (p[offset5] > cb)
                          {}
                          else
                          if (p[offset14] > cb)
                            {}
                          else
                            continue;
                        else
                          if (p[offset14] > cb)
                          if (p[offset15] > cb)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else if (p[offset11] < c_b)
                  if (p[offset12] < c_b)
                    if (p[offset13] < c_b)
                    if (p[offset10] < c_b)
                      if (p[offset14] < c_b)
                      if (p[offset15] < c_b)
                        if (p[offset1] < c_b)
                        {}
                        else
                        if (p[offset8] < c_b)
                          if (p[offset9] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset6] < c_b)
                        if (p[offset7] < c_b)
                          if (p[offset8] < c_b)
                          if (p[offset9] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset5] < c_b)
                        if (p[offset6] < c_b)
                        if (p[offset7] < c_b)
                          if (p[offset8] < c_b)
                          if (p[offset9] < c_b)
                            {}
                          else
                            continue;
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset1] < c_b)
                      if (p[offset3] < c_b)
                        if (p[offset14] < c_b)
                        if (p[offset15] < c_b)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else
                  continue;
                else
                if (p[offset9] > cb)
                  if (p[offset7] > cb)
                  if (p[offset8] > cb)
                    if (p[offset10] > cb)
                    if (p[offset11] > cb)
                      if (p[offset6] > cb)
                      if (p[offset5] > cb)
                        if (p[offset4] > cb)
                        if (p[offset3] > cb)
                          {}
                        else
                          if (p[offset12] > cb)
                          {}
                          else
                          continue;
                        else
                        if (p[offset12] > cb)
                          if (p[offset13] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset12] > cb)
                        if (p[offset13] > cb)
                          if (p[offset14] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset12] > cb)
                        if (p[offset13] > cb)
                        if (p[offset14] > cb)
                          if (p[offset15] > cb)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      continue;
                    else
                    continue;
                  else
                    continue;
                  else
                  continue;
                else if (p[offset9] < c_b)
                  if (p[offset10] < c_b)
                  if (p[offset11] < c_b)
                    if (p[offset8] < c_b)
                    if (p[offset12] < c_b)
                      if (p[offset13] < c_b)
                      if (p[offset14] < c_b)
                        if (p[offset15] < c_b)
                        {}
                        else
                        if (p[offset6] < c_b)
                          if (p[offset7] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                      else
                        if (p[offset5] < c_b)
                        if (p[offset6] < c_b)
                          if (p[offset7] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset4] < c_b)
                        if (p[offset5] < c_b)
                        if (p[offset6] < c_b)
                          if (p[offset7] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset3] < c_b)
                      if (p[offset4] < c_b)
                        if (p[offset5] < c_b)
                        if (p[offset6] < c_b)
                          if (p[offset7] < c_b)
                          {}
                          else
                          continue;
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    if (p[offset1] < c_b)
                      if (p[offset12] < c_b)
                      if (p[offset13] < c_b)
                        if (p[offset14] < c_b)
                        if (p[offset15] < c_b)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                  else
                    continue;
                  else
                  continue;
                else
                  continue;
              else
                if (p[offset7] > cb)
                if (p[offset8] > cb)
                  if (p[offset9] > cb)
                  if (p[offset6] > cb)
                    if (p[offset5] > cb)
                    if (p[offset4] > cb)
                      if (p[offset3] > cb)
                      if (p[offset2] > cb)
                        if (p[offset1] > cb)
                        {}
                        else
                        if (p[offset10] > cb)
                          {}
                        else
                          continue;
                      else
                        if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset10] > cb)
                        if (p[offset11] > cb)
                        if (p[offset12] > cb)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset10] > cb)
                      if (p[offset11] > cb)
                        if (p[offset12] > cb)
                        if (p[offset13] > cb)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    if (p[offset10] > cb)
                      if (p[offset11] > cb)
                      if (p[offset12] > cb)
                        if (p[offset13] > cb)
                        if (p[offset14] > cb)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                  else
                    if (p[offset10] > cb)
                    if (p[offset11] > cb)
                      if (p[offset12] > cb)
                      if (p[offset13] > cb)
                        if (p[offset14] > cb)
                        if (p[offset15] > cb)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else
                  continue;
                else
                  continue;
                else if (p[offset7] < c_b)
                if (p[offset8] < c_b)
                  if (p[offset9] < c_b)
                  if (p[offset6] < c_b)
                    if (p[offset5] < c_b)
                    if (p[offset4] < c_b)
                      if (p[offset3] < c_b)
                      if (p[offset2] < c_b)
                        if (p[offset1] < c_b)
                        {}
                        else
                        if (p[offset10] < c_b)
                          {}
                        else
                          continue;
                      else
                        if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                        if (p[offset12] < c_b)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                    else
                      if (p[offset10] < c_b)
                      if (p[offset11] < c_b)
                        if (p[offset12] < c_b)
                        if (p[offset13] < c_b)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                    if (p[offset10] < c_b)
                      if (p[offset11] < c_b)
                      if (p[offset12] < c_b)
                        if (p[offset13] < c_b)
                        if (p[offset14] < c_b)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                  else
                    if (p[offset10] < c_b)
                    if (p[offset11] < c_b)
                      if (p[offset12] < c_b)
                      if (p[offset13] < c_b)
                        if (p[offset14] < c_b)
                        if (p[offset15] < c_b)
                          {}
                        else
                          continue;
                        else
                        continue;
                      else
                        continue;
                      else
                      continue;
                    else
                      continue;
                    else
                    continue;
                  else
                  continue;
                else
                  continue;
                else
                continue;
            }
            if (total == n_expected_corners)   
            {                
              if (n_expected_corners == 0)
              {
                n_expected_corners = 512;
                corners.reserve (n_expected_corners);
              }
              else
              {
                n_expected_corners *= 2;
                corners.reserve (n_expected_corners);
              }
            }
            h.u = float (x);
            h.v = float (y);
            corners.push_back (h);
            total++;
          }
        }
      }

      ///////////////////////////////////////////////////////////////////////////////////
      // Helper method for OastDetector9_16::computeCornerScore
      template <typename T1, typename T2> int
      OastDetector9_16_computeCornerScore (
          const T1* p, 
          double im_bmax,
          double score_threshold,
          int_fast16_t s_offset0,
          int_fast16_t s_offset1,
          int_fast16_t s_offset2,
          int_fast16_t s_offset3,
          int_fast16_t s_offset4,
          int_fast16_t s_offset5,
          int_fast16_t s_offset6,
          int_fast16_t s_offset7,
          int_fast16_t s_offset8,
          int_fast16_t s_offset9,
          int_fast16_t s_offset10,
          int_fast16_t s_offset11,
          int_fast16_t s_offset12,
          int_fast16_t s_offset13,
          int_fast16_t s_offset14,
          int_fast16_t s_offset15)
      {
        T2 bmin = T2 (score_threshold);
        T2 bmax = T2 (im_bmax);
        int b_test = int ((bmax + bmin) / 2);

        int_fast16_t offset0  = s_offset0;
        int_fast16_t offset1  = s_offset1;
        int_fast16_t offset2  = s_offset2;
        int_fast16_t offset3  = s_offset3;
        int_fast16_t offset4  = s_offset4;
        int_fast16_t offset5  = s_offset5;
        int_fast16_t offset6  = s_offset6;
        int_fast16_t offset7  = s_offset7;
        int_fast16_t offset8  = s_offset8;
        int_fast16_t offset9  = s_offset9;
        int_fast16_t offset10 = s_offset10;
        int_fast16_t offset11 = s_offset11;
        int_fast16_t offset12 = s_offset12;
        int_fast16_t offset13 = s_offset13;
        int_fast16_t offset14 = s_offset14;
        int_fast16_t offset15 = s_offset15;

        while (1)
        {
          const T2 cb = *p + T2 (b_test);
          const T2 c_b = *p - T2 (b_test);
          if (p[offset0] > cb)
            if (p[offset2] > cb)
              if (p[offset4] > cb)
                if (p[offset5] > cb)
                  if (p[offset7] > cb)
                    if (p[offset3] > cb)
                      if (p[offset1] > cb)
                        if (p[offset6] > cb)
                          if (p[offset8] > cb)
                            goto is_a_corner;
                          else
                            if (p[offset15] > cb)
                              goto is_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset13] > cb)
                            if (p[offset14] > cb)
                              if (p[offset15] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset8] > cb)
                          if (p[offset9] > cb)
                            if (p[offset10] > cb)
                              if (p[offset6] > cb)
                                goto is_a_corner;
                              else
                                if (p[offset11] > cb)
                                  if (p[offset12] > cb)
                                    if (p[offset13] > cb)
                                      if (p[offset14] > cb)
                                        if (p[offset15] > cb)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          if (p[offset12] > cb)
                            if (p[offset8] > cb)
                              if (p[offset9] > cb)
                                if (p[offset6] > cb)
                                  goto is_a_corner;
                                else
                                  if (p[offset13] > cb)
                                    if (p[offset14] > cb)
                                      if (p[offset15] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                if (p[offset1] > cb)
                                  if (p[offset13] > cb)
                                    if (p[offset14] > cb)
                                      if (p[offset15] > cb)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset1] > cb)
                                if (p[offset13] > cb)
                                  if (p[offset14] > cb)
                                    if (p[offset15] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else if (p[offset7] < c_b)
                    if (p[offset14] > cb)
                      if (p[offset15] > cb)
                        if (p[offset1] > cb)
                          if (p[offset3] > cb)
                            if (p[offset6] > cb)
                              goto is_a_corner;
                            else
                              if (p[offset13] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset10] > cb)
                              if (p[offset11] > cb)
                                if (p[offset12] > cb)
                                  if (p[offset13] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset8] > cb)
                            if (p[offset9] > cb)
                              if (p[offset10] > cb)
                                if (p[offset11] > cb)
                                  if (p[offset12] > cb)
                                    if (p[offset13] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else if (p[offset14] < c_b)
                      if (p[offset8] < c_b)
                        if (p[offset9] < c_b)
                          if (p[offset10] < c_b)
                            if (p[offset11] < c_b)
                              if (p[offset12] < c_b)
                                if (p[offset13] < c_b)
                                  if (p[offset6] < c_b)
                                    goto is_a_corner;
                                  else
                                    if (p[offset15] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    if (p[offset14] > cb)
                      if (p[offset15] > cb)
                        if (p[offset1] > cb)
                          if (p[offset3] > cb)
                            if (p[offset6] > cb)
                              goto is_a_corner;
                            else
                              if (p[offset13] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset10] > cb)
                              if (p[offset11] > cb)
                                if (p[offset12] > cb)
                                  if (p[offset13] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset8] > cb)
                            if (p[offset9] > cb)
                              if (p[offset10] > cb)
                                if (p[offset11] > cb)
                                  if (p[offset12] > cb)
                                    if (p[offset13] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else if (p[offset5] < c_b)
                  if (p[offset12] > cb)
                    if (p[offset13] > cb)
                      if (p[offset14] > cb)
                        if (p[offset15] > cb)
                          if (p[offset1] > cb)
                            if (p[offset3] > cb)
                              goto is_a_corner;
                            else
                              if (p[offset10] > cb)
                                if (p[offset11] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset8] > cb)
                              if (p[offset9] > cb)
                                if (p[offset10] > cb)
                                  if (p[offset11] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset6] > cb)
                            if (p[offset7] > cb)
                              if (p[offset8] > cb)
                                if (p[offset9] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else if (p[offset12] < c_b)
                    if (p[offset7] < c_b)
                      if (p[offset8] < c_b)
                        if (p[offset9] < c_b)
                          if (p[offset10] < c_b)
                            if (p[offset11] < c_b)
                              if (p[offset13] < c_b)
                                if (p[offset6] < c_b)
                                  goto is_a_corner;
                                else
                                  if (p[offset14] < c_b)
                                    if (p[offset15] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  if (p[offset12] > cb)
                    if (p[offset13] > cb)
                      if (p[offset14] > cb)
                        if (p[offset15] > cb)
                          if (p[offset1] > cb)
                            if (p[offset3] > cb)
                              goto is_a_corner;
                            else
                              if (p[offset10] > cb)
                                if (p[offset11] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset8] > cb)
                              if (p[offset9] > cb)
                                if (p[offset10] > cb)
                                  if (p[offset11] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset6] > cb)
                            if (p[offset7] > cb)
                              if (p[offset8] > cb)
                                if (p[offset9] > cb)
                                  if (p[offset10] > cb)
                                    if (p[offset11] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else if (p[offset12] < c_b)
                    if (p[offset7] < c_b)
                      if (p[offset8] < c_b)
                        if (p[offset9] < c_b)
                          if (p[offset10] < c_b)
                            if (p[offset11] < c_b)
                              if (p[offset13] < c_b)
                                if (p[offset14] < c_b)
                                  if (p[offset6] < c_b)
                                    goto is_a_corner;
                                  else
                                    if (p[offset15] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
              else if (p[offset4] < c_b)
                if (p[offset11] > cb)
                  if (p[offset12] > cb)
                    if (p[offset13] > cb)
                      if (p[offset10] > cb)
                        if (p[offset14] > cb)
                          if (p[offset15] > cb)
                            if (p[offset1] > cb)
                              goto is_a_corner;
                            else
                              if (p[offset8] > cb)
                                if (p[offset9] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset7] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset9] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset5] > cb)
                            if (p[offset6] > cb)
                              if (p[offset7] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset9] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset1] > cb)
                          if (p[offset3] > cb)
                            if (p[offset14] > cb)
                              if (p[offset15] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else if (p[offset11] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset8] < c_b)
                      if (p[offset9] < c_b)
                        if (p[offset10] < c_b)
                          if (p[offset6] < c_b)
                            if (p[offset5] < c_b)
                              if (p[offset3] < c_b)
                                goto is_a_corner;
                              else
                                if (p[offset12] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset12] < c_b)
                                if (p[offset13] < c_b)
                                  if (p[offset14] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset12] < c_b)
                              if (p[offset13] < c_b)
                                if (p[offset14] < c_b)
                                  if (p[offset15] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                if (p[offset11] > cb)
                  if (p[offset12] > cb)
                    if (p[offset13] > cb)
                      if (p[offset10] > cb)
                        if (p[offset14] > cb)
                          if (p[offset15] > cb)
                            if (p[offset1] > cb)
                              goto is_a_corner;
                            else
                              if (p[offset8] > cb)
                                if (p[offset9] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset6] > cb)
                              if (p[offset7] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset9] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset5] > cb)
                            if (p[offset6] > cb)
                              if (p[offset7] > cb)
                                if (p[offset8] > cb)
                                  if (p[offset9] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset1] > cb)
                          if (p[offset3] > cb)
                            if (p[offset14] > cb)
                              if (p[offset15] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else if (p[offset11] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset8] < c_b)
                      if (p[offset9] < c_b)
                        if (p[offset10] < c_b)
                          if (p[offset12] < c_b)
                            if (p[offset13] < c_b)
                              if (p[offset6] < c_b)
                                if (p[offset5] < c_b)
                                  goto is_a_corner;
                                else
                                  if (p[offset14] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                if (p[offset14] < c_b)
                                  if (p[offset15] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
            else if (p[offset2] < c_b)
              if (p[offset9] > cb)
                if (p[offset10] > cb)
                  if (p[offset11] > cb)
                    if (p[offset8] > cb)
                      if (p[offset12] > cb)
                        if (p[offset13] > cb)
                          if (p[offset14] > cb)
                            if (p[offset15] > cb)
                              goto is_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset7] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset5] > cb)
                              if (p[offset6] > cb)
                                if (p[offset7] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset4] > cb)
                            if (p[offset5] > cb)
                              if (p[offset6] > cb)
                                if (p[offset7] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset3] > cb)
                          if (p[offset4] > cb)
                            if (p[offset5] > cb)
                              if (p[offset6] > cb)
                                if (p[offset7] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset1] > cb)
                        if (p[offset12] > cb)
                          if (p[offset13] > cb)
                            if (p[offset14] > cb)
                              if (p[offset15] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else if (p[offset9] < c_b)
                if (p[offset7] < c_b)
                  if (p[offset8] < c_b)
                    if (p[offset6] < c_b)
                      if (p[offset5] < c_b)
                        if (p[offset4] < c_b)
                          if (p[offset3] < c_b)
                            if (p[offset1] < c_b)
                              goto is_a_corner;
                            else
                              if (p[offset10] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset10] < c_b)
                              if (p[offset11] < c_b)
                                if (p[offset12] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset10] < c_b)
                            if (p[offset11] < c_b)
                              if (p[offset12] < c_b)
                                if (p[offset13] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                            if (p[offset12] < c_b)
                              if (p[offset13] < c_b)
                                if (p[offset14] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          if (p[offset12] < c_b)
                            if (p[offset13] < c_b)
                              if (p[offset14] < c_b)
                                if (p[offset15] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                goto is_not_a_corner;
            else
              if (p[offset9] > cb)
                if (p[offset10] > cb)
                  if (p[offset11] > cb)
                    if (p[offset8] > cb)
                      if (p[offset12] > cb)
                        if (p[offset13] > cb)
                          if (p[offset14] > cb)
                            if (p[offset15] > cb)
                              goto is_a_corner;
                            else
                              if (p[offset6] > cb)
                                if (p[offset7] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset5] > cb)
                              if (p[offset6] > cb)
                                if (p[offset7] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset4] > cb)
                            if (p[offset5] > cb)
                              if (p[offset6] > cb)
                                if (p[offset7] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset3] > cb)
                          if (p[offset4] > cb)
                            if (p[offset5] > cb)
                              if (p[offset6] > cb)
                                if (p[offset7] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset1] > cb)
                        if (p[offset12] > cb)
                          if (p[offset13] > cb)
                            if (p[offset14] > cb)
                              if (p[offset15] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else if (p[offset9] < c_b)
                if (p[offset7] < c_b)
                  if (p[offset8] < c_b)
                    if (p[offset10] < c_b)
                      if (p[offset11] < c_b)
                        if (p[offset6] < c_b)
                          if (p[offset5] < c_b)
                            if (p[offset4] < c_b)
                              if (p[offset3] < c_b)
                                goto is_a_corner;
                              else
                                if (p[offset12] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset12] < c_b)
                                if (p[offset13] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset12] < c_b)
                              if (p[offset13] < c_b)
                                if (p[offset14] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset12] < c_b)
                            if (p[offset13] < c_b)
                              if (p[offset14] < c_b)
                                if (p[offset15] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                goto is_not_a_corner;
          else if (p[offset0] < c_b)
            if (p[offset2] > cb)
              if (p[offset9] > cb)
                if (p[offset7] > cb)
                  if (p[offset8] > cb)
                    if (p[offset6] > cb)
                      if (p[offset5] > cb)
                        if (p[offset4] > cb)
                          if (p[offset3] > cb)
                            if (p[offset1] > cb)
                              goto is_a_corner;
                            else
                              if (p[offset10] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset10] > cb)
                              if (p[offset11] > cb)
                                if (p[offset12] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset10] > cb)
                            if (p[offset11] > cb)
                              if (p[offset12] > cb)
                                if (p[offset13] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset10] > cb)
                          if (p[offset11] > cb)
                            if (p[offset12] > cb)
                              if (p[offset13] > cb)
                                if (p[offset14] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          if (p[offset12] > cb)
                            if (p[offset13] > cb)
                              if (p[offset14] > cb)
                                if (p[offset15] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else if (p[offset9] < c_b)
                if (p[offset10] < c_b)
                  if (p[offset11] < c_b)
                    if (p[offset8] < c_b)
                      if (p[offset12] < c_b)
                        if (p[offset13] < c_b)
                          if (p[offset14] < c_b)
                            if (p[offset15] < c_b)
                              goto is_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset7] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset5] < c_b)
                              if (p[offset6] < c_b)
                                if (p[offset7] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset4] < c_b)
                            if (p[offset5] < c_b)
                              if (p[offset6] < c_b)
                                if (p[offset7] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset3] < c_b)
                          if (p[offset4] < c_b)
                            if (p[offset5] < c_b)
                              if (p[offset6] < c_b)
                                if (p[offset7] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset1] < c_b)
                        if (p[offset12] < c_b)
                          if (p[offset13] < c_b)
                            if (p[offset14] < c_b)
                              if (p[offset15] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                goto is_not_a_corner;
            else if (p[offset2] < c_b)
              if (p[offset4] > cb)
                if (p[offset11] > cb)
                  if (p[offset7] > cb)
                    if (p[offset8] > cb)
                      if (p[offset9] > cb)
                        if (p[offset10] > cb)
                          if (p[offset6] > cb)
                            if (p[offset5] > cb)
                              if (p[offset3] > cb)
                                goto is_a_corner;
                              else
                                if (p[offset12] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset12] > cb)
                                if (p[offset13] > cb)
                                  if (p[offset14] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset12] > cb)
                              if (p[offset13] > cb)
                                if (p[offset14] > cb)
                                  if (p[offset15] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else if (p[offset11] < c_b)
                  if (p[offset12] < c_b)
                    if (p[offset13] < c_b)
                      if (p[offset10] < c_b)
                        if (p[offset14] < c_b)
                          if (p[offset15] < c_b)
                            if (p[offset1] < c_b)
                              goto is_a_corner;
                            else
                              if (p[offset8] < c_b)
                                if (p[offset9] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              if (p[offset7] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset9] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset5] < c_b)
                            if (p[offset6] < c_b)
                              if (p[offset7] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset9] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset1] < c_b)
                          if (p[offset3] < c_b)
                            if (p[offset14] < c_b)
                              if (p[offset15] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else if (p[offset4] < c_b)
                if (p[offset5] > cb)
                  if (p[offset12] > cb)
                    if (p[offset7] > cb)
                      if (p[offset8] > cb)
                        if (p[offset9] > cb)
                          if (p[offset10] > cb)
                            if (p[offset11] > cb)
                              if (p[offset13] > cb)
                                if (p[offset6] > cb)
                                  goto is_a_corner;
                                else
                                  if (p[offset14] > cb)
                                    if (p[offset15] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else if (p[offset12] < c_b)
                    if (p[offset13] < c_b)
                      if (p[offset14] < c_b)
                        if (p[offset15] < c_b)
                          if (p[offset1] < c_b)
                            if (p[offset3] < c_b)
                              goto is_a_corner;
                            else
                              if (p[offset10] < c_b)
                                if (p[offset11] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset8] < c_b)
                              if (p[offset9] < c_b)
                                if (p[offset10] < c_b)
                                  if (p[offset11] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset6] < c_b)
                            if (p[offset7] < c_b)
                              if (p[offset8] < c_b)
                                if (p[offset9] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else if (p[offset5] < c_b)
                  if (p[offset7] > cb)
                    if (p[offset14] > cb)
                      if (p[offset8] > cb)
                        if (p[offset9] > cb)
                          if (p[offset10] > cb)
                            if (p[offset11] > cb)
                              if (p[offset12] > cb)
                                if (p[offset13] > cb)
                                  if (p[offset6] > cb)
                                    goto is_a_corner;
                                  else
                                    if (p[offset15] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else if (p[offset14] < c_b)
                      if (p[offset15] < c_b)
                        if (p[offset1] < c_b)
                          if (p[offset3] < c_b)
                            if (p[offset6] < c_b)
                              goto is_a_corner;
                            else
                              if (p[offset13] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset10] < c_b)
                              if (p[offset11] < c_b)
                                if (p[offset12] < c_b)
                                  if (p[offset13] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset8] < c_b)
                            if (p[offset9] < c_b)
                              if (p[offset10] < c_b)
                                if (p[offset11] < c_b)
                                  if (p[offset12] < c_b)
                                    if (p[offset13] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else if (p[offset7] < c_b)
                    if (p[offset3] < c_b)
                      if (p[offset1] < c_b)
                        if (p[offset6] < c_b)
                          if (p[offset8] < c_b)
                            goto is_a_corner;
                          else
                            if (p[offset15] < c_b)
                              goto is_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset13] < c_b)
                            if (p[offset14] < c_b)
                              if (p[offset15] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset8] < c_b)
                          if (p[offset9] < c_b)
                            if (p[offset10] < c_b)
                              if (p[offset6] < c_b)
                                goto is_a_corner;
                              else
                                if (p[offset11] < c_b)
                                  if (p[offset12] < c_b)
                                    if (p[offset13] < c_b)
                                      if (p[offset14] < c_b)
                                        if (p[offset15] < c_b)
                                          goto is_a_corner;
                                        else
                                          goto is_not_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          if (p[offset12] < c_b)
                            if (p[offset8] < c_b)
                              if (p[offset9] < c_b)
                                if (p[offset6] < c_b)
                                  goto is_a_corner;
                                else
                                  if (p[offset13] < c_b)
                                    if (p[offset14] < c_b)
                                      if (p[offset15] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                if (p[offset1] < c_b)
                                  if (p[offset13] < c_b)
                                    if (p[offset14] < c_b)
                                      if (p[offset15] < c_b)
                                        goto is_a_corner;
                                      else
                                        goto is_not_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset1] < c_b)
                                if (p[offset13] < c_b)
                                  if (p[offset14] < c_b)
                                    if (p[offset15] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset14] < c_b)
                      if (p[offset15] < c_b)
                        if (p[offset1] < c_b)
                          if (p[offset3] < c_b)
                            if (p[offset6] < c_b)
                              goto is_a_corner;
                            else
                              if (p[offset13] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset10] < c_b)
                              if (p[offset11] < c_b)
                                if (p[offset12] < c_b)
                                  if (p[offset13] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset8] < c_b)
                            if (p[offset9] < c_b)
                              if (p[offset10] < c_b)
                                if (p[offset11] < c_b)
                                  if (p[offset12] < c_b)
                                    if (p[offset13] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  if (p[offset12] > cb)
                    if (p[offset7] > cb)
                      if (p[offset8] > cb)
                        if (p[offset9] > cb)
                          if (p[offset10] > cb)
                            if (p[offset11] > cb)
                              if (p[offset13] > cb)
                                if (p[offset14] > cb)
                                  if (p[offset6] > cb)
                                    goto is_a_corner;
                                  else
                                    if (p[offset15] > cb)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else if (p[offset12] < c_b)
                    if (p[offset13] < c_b)
                      if (p[offset14] < c_b)
                        if (p[offset15] < c_b)
                          if (p[offset1] < c_b)
                            if (p[offset3] < c_b)
                              goto is_a_corner;
                            else
                              if (p[offset10] < c_b)
                                if (p[offset11] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset8] < c_b)
                              if (p[offset9] < c_b)
                                if (p[offset10] < c_b)
                                  if (p[offset11] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset6] < c_b)
                            if (p[offset7] < c_b)
                              if (p[offset8] < c_b)
                                if (p[offset9] < c_b)
                                  if (p[offset10] < c_b)
                                    if (p[offset11] < c_b)
                                      goto is_a_corner;
                                    else
                                      goto is_not_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
              else
                if (p[offset11] > cb)
                  if (p[offset7] > cb)
                    if (p[offset8] > cb)
                      if (p[offset9] > cb)
                        if (p[offset10] > cb)
                          if (p[offset12] > cb)
                            if (p[offset13] > cb)
                              if (p[offset6] > cb)
                                if (p[offset5] > cb)
                                  goto is_a_corner;
                                else
                                  if (p[offset14] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                              else
                                if (p[offset14] > cb)
                                  if (p[offset15] > cb)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else if (p[offset11] < c_b)
                  if (p[offset12] < c_b)
                    if (p[offset13] < c_b)
                      if (p[offset10] < c_b)
                        if (p[offset14] < c_b)
                          if (p[offset15] < c_b)
                            if (p[offset1] < c_b)
                              goto is_a_corner;
                            else
                              if (p[offset8] < c_b)
                                if (p[offset9] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset6] < c_b)
                              if (p[offset7] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset9] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset5] < c_b)
                            if (p[offset6] < c_b)
                              if (p[offset7] < c_b)
                                if (p[offset8] < c_b)
                                  if (p[offset9] < c_b)
                                    goto is_a_corner;
                                  else
                                    goto is_not_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset1] < c_b)
                          if (p[offset3] < c_b)
                            if (p[offset14] < c_b)
                              if (p[offset15] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
            else
              if (p[offset9] > cb)
                if (p[offset7] > cb)
                  if (p[offset8] > cb)
                    if (p[offset10] > cb)
                      if (p[offset11] > cb)
                        if (p[offset6] > cb)
                          if (p[offset5] > cb)
                            if (p[offset4] > cb)
                              if (p[offset3] > cb)
                                goto is_a_corner;
                              else
                                if (p[offset12] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                            else
                              if (p[offset12] > cb)
                                if (p[offset13] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset12] > cb)
                              if (p[offset13] > cb)
                                if (p[offset14] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset12] > cb)
                            if (p[offset13] > cb)
                              if (p[offset14] > cb)
                                if (p[offset15] > cb)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else if (p[offset9] < c_b)
                if (p[offset10] < c_b)
                  if (p[offset11] < c_b)
                    if (p[offset8] < c_b)
                      if (p[offset12] < c_b)
                        if (p[offset13] < c_b)
                          if (p[offset14] < c_b)
                            if (p[offset15] < c_b)
                              goto is_a_corner;
                            else
                              if (p[offset6] < c_b)
                                if (p[offset7] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset5] < c_b)
                              if (p[offset6] < c_b)
                                if (p[offset7] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset4] < c_b)
                            if (p[offset5] < c_b)
                              if (p[offset6] < c_b)
                                if (p[offset7] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset3] < c_b)
                          if (p[offset4] < c_b)
                            if (p[offset5] < c_b)
                              if (p[offset6] < c_b)
                                if (p[offset7] < c_b)
                                  goto is_a_corner;
                                else
                                  goto is_not_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset1] < c_b)
                        if (p[offset12] < c_b)
                          if (p[offset13] < c_b)
                            if (p[offset14] < c_b)
                              if (p[offset15] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                goto is_not_a_corner;
          else
            if (p[offset7] > cb)
              if (p[offset8] > cb)
                if (p[offset9] > cb)
                  if (p[offset6] > cb)
                    if (p[offset5] > cb)
                      if (p[offset4] > cb)
                        if (p[offset3] > cb)
                          if (p[offset2] > cb)
                            if (p[offset1] > cb)
                              goto is_a_corner;
                            else
                              if (p[offset10] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset10] > cb)
                              if (p[offset11] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset10] > cb)
                            if (p[offset11] > cb)
                              if (p[offset12] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset10] > cb)
                          if (p[offset11] > cb)
                            if (p[offset12] > cb)
                              if (p[offset13] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset10] > cb)
                        if (p[offset11] > cb)
                          if (p[offset12] > cb)
                            if (p[offset13] > cb)
                              if (p[offset14] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset10] > cb)
                      if (p[offset11] > cb)
                        if (p[offset12] > cb)
                          if (p[offset13] > cb)
                            if (p[offset14] > cb)
                              if (p[offset15] > cb)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                goto is_not_a_corner;
            else if (p[offset7] < c_b)
              if (p[offset8] < c_b)
                if (p[offset9] < c_b)
                  if (p[offset6] < c_b)
                    if (p[offset5] < c_b)
                      if (p[offset4] < c_b)
                        if (p[offset3] < c_b)
                          if (p[offset2] < c_b)
                            if (p[offset1] < c_b)
                              goto is_a_corner;
                            else
                              if (p[offset10] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                          else
                            if (p[offset10] < c_b)
                              if (p[offset11] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                        else
                          if (p[offset10] < c_b)
                            if (p[offset11] < c_b)
                              if (p[offset12] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                      else
                        if (p[offset10] < c_b)
                          if (p[offset11] < c_b)
                            if (p[offset12] < c_b)
                              if (p[offset13] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                    else
                      if (p[offset10] < c_b)
                        if (p[offset11] < c_b)
                          if (p[offset12] < c_b)
                            if (p[offset13] < c_b)
                              if (p[offset14] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                  else
                    if (p[offset10] < c_b)
                      if (p[offset11] < c_b)
                        if (p[offset12] < c_b)
                          if (p[offset13] < c_b)
                            if (p[offset14] < c_b)
                              if (p[offset15] < c_b)
                                goto is_a_corner;
                              else
                                goto is_not_a_corner;
                            else
                              goto is_not_a_corner;
                          else
                            goto is_not_a_corner;
                        else
                          goto is_not_a_corner;
                      else
                        goto is_not_a_corner;
                    else
                      goto is_not_a_corner;
                else
                  goto is_not_a_corner;
              else
                goto is_not_a_corner;
            else
              goto is_not_a_corner;

          is_a_corner:
            bmin = T2 (b_test);
            goto end;

          is_not_a_corner:
            bmax = T2 (b_test);
            goto end;

          end:

          if (bmin == bmax - 1 || bmin == bmax)
            return (int (bmin));
          b_test = int ((bmin + bmax) / 2);
        }
      }
    } // namespace agast
  } // namespace keypoints
} // namespace pcl

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::OastDetector9_16::initPattern ()
{
  s_offset0_  = static_cast<int_fast16_t> ((-3) + (0)  * width_);
  s_offset1_  = static_cast<int_fast16_t> ((-3) + (-1) * width_);
  s_offset2_  = static_cast<int_fast16_t> ((-2) + (-2) * width_);
  s_offset3_  = static_cast<int_fast16_t> ((-1) + (-3) * width_);
  s_offset4_  = static_cast<int_fast16_t> ((0)  + (-3) * width_);
  s_offset5_  = static_cast<int_fast16_t> ((1)  + (-3) * width_);
  s_offset6_  = static_cast<int_fast16_t> ((2)  + (-2) * width_);
  s_offset7_  = static_cast<int_fast16_t> ((3)  + (-1) * width_);
  s_offset8_  = static_cast<int_fast16_t> ((3)  + (0)  * width_);
  s_offset9_  = static_cast<int_fast16_t> ((3)  + (1)  * width_);
  s_offset10_ = static_cast<int_fast16_t> ((2)  + (2)  * width_);
  s_offset11_ = static_cast<int_fast16_t> ((1)  + (3)  * width_);
  s_offset12_ = static_cast<int_fast16_t> ((0)  + (3)  * width_);
  s_offset13_ = static_cast<int_fast16_t> ((-1) + (3)  * width_);
  s_offset14_ = static_cast<int_fast16_t> ((-2) + (2)  * width_);
  s_offset15_ = static_cast<int_fast16_t> ((-3) + (1)  * width_);
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::OastDetector9_16::detect (const unsigned char* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > & corners) const
{
  return (pcl::keypoints::agast::OastDetector9_16_detect<unsigned char, int> (
        im, int (width_), int (height_), threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_,
        s_offset8_,
        s_offset9_,
        s_offset10_,
        s_offset11_,
        s_offset12_,
        s_offset13_,
        s_offset14_,
        s_offset15_,
        corners));
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::OastDetector9_16::detect (const float* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > & corners) const
{
  return (pcl::keypoints::agast::OastDetector9_16_detect<float, float> (
        im, int (width_), int (height_), threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_,
        s_offset8_,
        s_offset9_,
        s_offset10_,
        s_offset11_,
        s_offset12_,
        s_offset13_,
        s_offset14_,
        s_offset15_,
        corners));
}

/////////////////////////////////////////////////////////////////////////////////////////
int
pcl::keypoints::agast::OastDetector9_16::computeCornerScore (const unsigned char* p) const
{
  return (pcl::keypoints::agast::OastDetector9_16_computeCornerScore<unsigned char, int> (
        p, bmax_, threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_,
        s_offset8_,
        s_offset9_,
        s_offset10_,
        s_offset11_,
        s_offset12_,
        s_offset13_,
        s_offset14_,
        s_offset15_));
}

/////////////////////////////////////////////////////////////////////////////////////////
int
pcl::keypoints::agast::OastDetector9_16::computeCornerScore (const float* p) const
{
  return (pcl::keypoints::agast::OastDetector9_16_computeCornerScore<float, float> (
        p, bmax_, threshold_, 
        s_offset0_, 
        s_offset1_,
        s_offset2_,
        s_offset3_,
        s_offset4_,
        s_offset5_,
        s_offset6_,
        s_offset7_,
        s_offset8_,
        s_offset9_,
        s_offset10_,
        s_offset11_,
        s_offset12_,
        s_offset13_,
        s_offset14_,
        s_offset15_));
}

