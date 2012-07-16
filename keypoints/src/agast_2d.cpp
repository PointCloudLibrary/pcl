/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and threshold_inary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in threshold_inary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may threshold_e used to endorse or promote products derived
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
pcl::keypoints::agast::AbstractAgastDetector::detectKeypoints (
    const std::vector<unsigned char> & intensity_data, 
    pcl::PointCloud<pcl::PointXY> &output)
{
  detect (&(intensity_data[0]), output.points);

  output.height = 1;
  output.width = static_cast<uint32_t> (output.points.size ());
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AbstractAgastDetector::applyNonMaxSuppression (
  const std::vector<unsigned char>& intensity_data, 
  const pcl::PointCloud<pcl::PointXY> &input,
  pcl::PointCloud<pcl::PointXY> &output)
{
  std::vector<int> scores;
  std::vector<int> nmsFlags;

  computeCornerScores (&(intensity_data[0]), input.points, scores);

  const std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > & corners_all = input.points;
  std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > & corners_nms = output.points;

  int currCorner_ind;
  int lastRow = 0, next_lastRow = 0;
  std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >::const_iterator currCorner;
  int lastRowCorner_ind = 0, next_lastRowCorner_ind = 0;
  std::vector<int>::iterator nmsFlags_p;
  std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >::iterator currCorner_nms;
  int j;
  int numCorners_all = int (corners_all.size ());
  int nMaxCorners = int (corners_nms.capacity ());

  currCorner = corners_all.begin ();

  if (numCorners_all > nMaxCorners)
  {
    if (nMaxCorners == 0)
    {
      nMaxCorners = 512 > numCorners_all ? 512 : numCorners_all;
      corners_nms.reserve (nMaxCorners);
      nmsFlags.reserve (nMaxCorners);
    }
    else
    {
      nMaxCorners *= 2;
      if (numCorners_all > nMaxCorners)
        nMaxCorners = numCorners_all;
      corners_nms.reserve (nMaxCorners);
      nmsFlags.reserve (nMaxCorners);
    }
  }
  corners_nms.resize (numCorners_all);
  nmsFlags.resize (numCorners_all);

  nmsFlags_p = nmsFlags.begin ();
  currCorner_nms = corners_nms.begin ();

  //set all flags to MAXIMUM
  for (j = numCorners_all; j > 0; j--)
    *nmsFlags_p++=-1;
  nmsFlags_p = nmsFlags.begin ();

  for (currCorner_ind = 0; currCorner_ind < numCorners_all; currCorner_ind++)
  {
    int t;

    //check above
    if (lastRow + 1 < currCorner->y)
    {
      lastRow=next_lastRow;
      lastRowCorner_ind=next_lastRowCorner_ind;
    }
    if (next_lastRow != currCorner->y)
    {
      next_lastRow = int (currCorner->y);
      next_lastRowCorner_ind = currCorner_ind;
    }
    if (lastRow+1 == currCorner->y)
    {
      //find the corner above the current one
      while ((corners_all[lastRowCorner_ind].x < currCorner->x) && (corners_all[lastRowCorner_ind].y == lastRow))
        lastRowCorner_ind++;

      if ((corners_all[lastRowCorner_ind].x == currCorner->x) && (lastRowCorner_ind!=currCorner_ind))
      {
        int t = lastRowCorner_ind;
        while (nmsFlags[t] != -1) //find the maximum in this block
          t = nmsFlags[t];

        if (scores[currCorner_ind] < scores[t])
        {
          nmsFlags[currCorner_ind] = t;
        }
        else
          nmsFlags[t] = currCorner_ind;
      }
    }

    //check left
    t=currCorner_ind-1;
    if ((currCorner_ind!=0) && (corners_all[t].y == currCorner->y) && (corners_all[t].x+1 == currCorner->x))
    {
      int currCornerMaxAbove_ind = nmsFlags[currCorner_ind];

      while (nmsFlags[t] != -1) //find the maximum in that area
        t=nmsFlags[t];

      if (currCornerMaxAbove_ind == -1) //no maximum above
      {
        if (t != currCorner_ind)
        {
          if (scores[currCorner_ind] < scores[t] )
            nmsFlags[currCorner_ind] = t;
          else
            nmsFlags[t] = currCorner_ind;
        }
      }
      else  //maximum above
      {
        if (t != currCornerMaxAbove_ind)
        {
          if (scores[currCornerMaxAbove_ind] < scores[t])
          {
            nmsFlags[currCornerMaxAbove_ind] = t;
            nmsFlags[currCorner_ind] = t;
          }
          else
          {
            nmsFlags[t] = currCornerMaxAbove_ind;
            nmsFlags[currCorner_ind] = currCornerMaxAbove_ind;
          }
        }
      }
    }

    currCorner++;
  }

  //collecting maximum corners
  corners_nms.resize (0);
  for (currCorner_ind = 0; currCorner_ind < numCorners_all; currCorner_ind++)
  {
    if (*nmsFlags_p++ == -1)
      corners_nms.push_back (corners_all[currCorner_ind]);
  }

  output.height = 1;
  output.width = static_cast<uint32_t> (output.points.size ());
  output.is_dense = input.is_dense;
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AbstractAgastDetector::computeCornerScores (
  const unsigned char* im,
  const std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > &corners_all, 
  std::vector<int> & scores)
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
    scores[n] = computeCornerScore (im + static_cast<size_t> (corners_all[n].y) * width_ + static_cast<size_t> (corners_all[n].x));
}


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
pcl::keypoints::agast::AgastDetector7_12s::detect (const unsigned char* im, std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > & corners_all) const
{
  int total = 0;
  int nExpectedCorners = int (corners_all.capacity ());
  pcl::PointXY h;
  register int x, y;
  register int widthB  = int (width_) - 3; //2, +1 due to faster test x>widthB
  register int heightB = int (height_) - 2;
  register int_fast16_t offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7, offset8, offset9, offset10, offset11;
  register int width;

  corners_all.resize (0);

  offset0  = s_offset0_;
  offset1  = s_offset1_;
  offset2  = s_offset2_;
  offset3  = s_offset3_;
  offset4  = s_offset4_;
  offset5  = s_offset5_;
  offset6  = s_offset6_;
  offset7  = s_offset7_;
  offset8  = s_offset8_;
  offset9  = s_offset9_;
  offset10 = s_offset10_;
  offset11 = s_offset11_;
  width    = int (width_);

  for (y = 2; y < heightB; y++)
  {                    
    x = 1;
    while (1)              
    {                  
homogeneous:
{
      x++;      
      if (x>widthB)  
        break;
      else
      {
        register const unsigned char* const p = im + y*width + x;
        register const int cb = *p + int (threshold_);
        register const int c_b = *p - int (threshold_);
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
      if (x>widthB)  
        break;    
      else
      {
        register const unsigned char* const p = im + y*width + x;
        register const int cb = *p + int (threshold_);
        register const int c_b = *p - int (threshold_);
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
      if (total == nExpectedCorners)
      {
        if (nExpectedCorners==0)
        {
          nExpectedCorners=512;
          corners_all.reserve(nExpectedCorners);
        }
        else
        {
          nExpectedCorners *=2;
          corners_all.reserve(nExpectedCorners);
        }
      }
      h.x = float (x);
      h.y = float (y);
      corners_all.push_back(h);
      total++;            
      goto homogeneous;        
success_structured:
      if (total == nExpectedCorners)
      {
        if (nExpectedCorners==0)
        {
          nExpectedCorners=512;
          corners_all.reserve(nExpectedCorners);
        }
        else
        {
          nExpectedCorners *=2;
          corners_all.reserve(nExpectedCorners);
        }
      }
      h.x = float (x);
      h.y = float (y);
      corners_all.push_back (h);
      total++;            
      goto structured;        
    }                  
  }                    
}

/////////////////////////////////////////////////////////////////////////////////////////
int
pcl::keypoints::agast::AgastDetector7_12s::computeCornerScore (const unsigned char* p) const
{
  int bmin = int (threshold_);
  int bmax = 255;
  int b_test = (bmax + bmin) / 2;

  register int_fast16_t offset0  = s_offset0_;
  register int_fast16_t offset1  = s_offset1_;
  register int_fast16_t offset2  = s_offset2_;
  register int_fast16_t offset3  = s_offset3_;
  register int_fast16_t offset4  = s_offset4_;
  register int_fast16_t offset5  = s_offset5_;
  register int_fast16_t offset6  = s_offset6_;
  register int_fast16_t offset7  = s_offset7_;
  register int_fast16_t offset8  = s_offset8_;
  register int_fast16_t offset9  = s_offset9_;
  register int_fast16_t offset10 = s_offset10_;
  register int_fast16_t offset11 = s_offset11_;

  while (1)
  {
    register const int cb = *p + b_test;
    register const int c_b = *p - b_test;
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
      bmin=b_test;
      goto end;

    is_not_a_corner:
      bmax=b_test;
      goto end;

    end:

    if (bmin == bmax - 1 || bmin == bmax)
      return (bmin);
    b_test = (bmin + bmax) / 2;
  }
}

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
int
pcl::keypoints::agast::AgastDetector5_8::computeCornerScore (const unsigned char* p) const
{
  int bmin = int (threshold_);
  int bmax = 255;
  int b_test = (bmax + bmin) / 2;

  register int_fast16_t offset0 = s_offset0_;
  register int_fast16_t offset1 = s_offset1_;
  register int_fast16_t offset2 = s_offset2_;
  register int_fast16_t offset3 = s_offset3_;
  register int_fast16_t offset4 = s_offset4_;
  register int_fast16_t offset5 = s_offset5_;
  register int_fast16_t offset6 = s_offset6_;
  register int_fast16_t offset7 = s_offset7_;

  while (1)
  {
    register const int cb = *p + b_test;
    register const int c_b = *p - b_test;
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
      bmin=b_test;
      goto end;

    is_not_a_corner:
      bmax=b_test;
      goto end;

    end:

    if (bmin == bmax - 1 || bmin == bmax)
      return (bmin);
    b_test = (bmin + bmax) / 2;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
void
pcl::keypoints::agast::AgastDetector5_8::detect (const unsigned char* im, std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > & corners_all) const
{
  int total = 0;
  int nExpectedCorners = int (corners_all.capacity ());
  pcl::PointXY h;
  register int x, y;
  register int xsizeB = int (width_) - 2;
  register int ysizeB = int (height_) - 1;
  register int_fast16_t offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7;
  register int width;

  corners_all.resize (0);

  offset0 = s_offset0_;
  offset1 = s_offset1_;
  offset2 = s_offset2_;
  offset3 = s_offset3_;
  offset4 = s_offset4_;
  offset5 = s_offset5_;
  offset6 = s_offset6_;
  offset7 = s_offset7_;
  width   = int (width_);

  for (y = 1; y < ysizeB; y++)
  {                    
    x = 0;
    while (1)              
    {                  
homogeneous:
{
      x++;      
      if (x>xsizeB)  
        break;
      else
      {
        register const unsigned char* const p = im + y*width + x;
        register const int cb = *p + int (threshold_);
        register const int c_b = *p - int (threshold_);
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
      if (x>xsizeB)  
        break;
      else
      {
        register const unsigned char* const p = im + y*width + x;
        register const int cb = *p + int (threshold_);
        register const int c_b = *p - int (threshold_);
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
      if (total == nExpectedCorners)   
      {                
        if (nExpectedCorners==0)
        {
          nExpectedCorners=512;
          corners_all.reserve(nExpectedCorners);
        }
        else
        {
          nExpectedCorners *=2;
          corners_all.reserve(nExpectedCorners);
        }
      }
      h.x = float (x);
      h.y = float (y);
      corners_all.push_back (h);
      total++;            
      goto homogeneous;        
success_structured:
      if (total == nExpectedCorners)   
      {                
        if (nExpectedCorners==0)
        {
          nExpectedCorners=512;
          corners_all.reserve(nExpectedCorners);
        }
        else
        {
          nExpectedCorners *=2;
          corners_all.reserve(nExpectedCorners);
        }
      }
      h.x = float (x);
      h.y = float (y);
      corners_all.push_back (h);
      total++;
      goto structured;
    }                  
  }                    
}

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
pcl::keypoints::agast::OastDetector9_16::detect (const unsigned char* im, std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > & corners_all) const
{
  int total = 0;
  int nExpectedCorners = int (corners_all.capacity ());
  pcl::PointXY h;
  register int x, y;
  register int xsizeB = int (width_) - 4;
  register int ysizeB = int (height_) - 3;
  register int_fast16_t offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7, offset8, offset9, offset10, offset11, offset12, offset13, offset14, offset15;
  register int width;

  corners_all.resize (0);

  offset0  = s_offset0_;
  offset1  = s_offset1_;
  offset2  = s_offset2_;
  offset3  = s_offset3_;
  offset4  = s_offset4_;
  offset5  = s_offset5_;
  offset6  = s_offset6_;
  offset7  = s_offset7_;
  offset8  = s_offset8_;
  offset9  = s_offset9_;
  offset10 = s_offset10_;
  offset11 = s_offset11_;
  offset12 = s_offset12_;
  offset13 = s_offset13_;
  offset14 = s_offset14_;
  offset15 = s_offset15_;
  width    = int (width_);

  for (y = 3; y < ysizeB; y++)      
  {                    
    x = 2;
    while (1)
    {
      x++;
      if (x > xsizeB)
        break;
      else
      {
        register const unsigned char* const p = im + y*width + x;
        register const int cb = *p + int (threshold_);
        register const int c_b = *p - int (threshold_);
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
      if (total == nExpectedCorners)   
      {                
        if (nExpectedCorners==0)
        {
          nExpectedCorners = 512;
          corners_all.reserve (nExpectedCorners);
        }
        else
        {
          nExpectedCorners *= 2;
          corners_all.reserve (nExpectedCorners);
        }
      }
      h.x = float (x);
      h.y = float (y);
      corners_all.push_back (h);
      total++;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
int
pcl::keypoints::agast::OastDetector9_16::computeCornerScore (const unsigned char* p) const
{
  int bmin = int (threshold_);
  int bmax = 255;
  int b_test = (bmax + bmin) / 2;

  register int_fast16_t offset0  = s_offset0_;
  register int_fast16_t offset1  = s_offset1_;
  register int_fast16_t offset2  = s_offset2_;
  register int_fast16_t offset3  = s_offset3_;
  register int_fast16_t offset4  = s_offset4_;
  register int_fast16_t offset5  = s_offset5_;
  register int_fast16_t offset6  = s_offset6_;
  register int_fast16_t offset7  = s_offset7_;
  register int_fast16_t offset8  = s_offset8_;
  register int_fast16_t offset9  = s_offset9_;
  register int_fast16_t offset10 = s_offset10_;
  register int_fast16_t offset11 = s_offset11_;
  register int_fast16_t offset12 = s_offset12_;
  register int_fast16_t offset13 = s_offset13_;
  register int_fast16_t offset14 = s_offset14_;
  register int_fast16_t offset15 = s_offset15_;

  while (1)
  {
    register const int cb = *p + b_test;
    register const int c_b = *p - b_test;
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
      bmin=b_test;
      goto end;

    is_not_a_corner:
      bmax=b_test;
      goto end;

    end:

    if (bmin == bmax - 1 || bmin == bmax)
      return (bmin);
    b_test = (bmin + bmax) / 2;
  }
}


// Instantiations of specific point types

