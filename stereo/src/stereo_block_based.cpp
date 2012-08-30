/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *
 */

/** \brief Block based (or fixed window) Stereo Matching algorithm implementation
  * please see related documentation on stereo/stereo_matching.h
  *
  * \author Federico Tombari (federico.tombari@unibo.it)
  * \ingroup stereo
  */


#include "pcl/stereo/stereo_matching.h"

//////////////////////////////////////////////////////////////////////////////
pcl::BlockBasedStereoMatching::BlockBasedStereoMatching ()
{
  radius_ = 5; //default value
}

//////////////////////////////////////////////////////////////////////////////
void 
pcl::BlockBasedStereoMatching::compute_impl (unsigned char* ref_img, unsigned char* trg_img)
{
  int n = radius_ * 2 + 1;
  int sad_min;
  short int dbest = 0;

  int sad_max = std::numeric_limits<int>::max ();

  int *acc = new int[max_disp_];
  memset (acc, 0, sizeof (int) * max_disp_);

  int **v = new int *[width_];
  for (int d = 0; d < width_; d++)
  {
    v[d] = new int[max_disp_];
    memset (v[d], 0, sizeof (int) * max_disp_);
  }

  //First row
  for (int x = max_disp_ + x_off_; x < max_disp_ + x_off_ + n; x++)
  {
    for (int d = 0; d < max_disp_; d++)
    {
      for (int y = 0; y < n; y++)
        v[x][d] += abs (ref_img[y*width_+x] - trg_img[y*width_+x -d-x_off_]);
      //acc[d] += V[x][d];
    }
  }

  //STAGE 2: other positions
  for (int x = max_disp_ + x_off_ + radius_ + 1; x < width_ - radius_; x++)
  {
    for (int d = 0; d < max_disp_; d++)
    {
      for (int y = 0; y < n; y++)
      {
        v[x+radius_][d] += abs (ref_img[y*width_ + x+radius_] - trg_img[ y*width_ + x+radius_ -d-x_off_]);
      }
      acc[d] = acc[d] + v[x+radius_][d] - v[x-radius_-1][d];
    }//d
  }//x

  //2
  unsigned char *lp, *rp, *lpp, *rpp;
  int ind1 = radius_ + radius_ + max_disp_ + x_off_;

  for (int y = radius_ + 1; y < height_ - radius_; y++)
  {
    //first position
    for (int d = 0; d < max_disp_; d++)
    {
      acc[d] = 0;
      for (int x = max_disp_ + x_off_; x < max_disp_ + x_off_ + n; x++)
      {
        v[x][d] = v[x][d] + abs( ref_img[ (y+radius_)*width_+x] - trg_img[ (y+radius_)*width_+x -d-x_off_] ) - abs( ref_img[ (y-radius_-1)*width_+x] - trg_img[ (y-radius_-1)*width_ +x -d-x_off_] );
        
        acc[d] += v[x][d];
      }
    }

    //all other positions
    lp  = static_cast<unsigned char*> (ref_img) + (y + radius_)    * width_ + ind1;
    rp  = static_cast<unsigned char*> (trg_img) + (y + radius_)    * width_ + ind1 - x_off_;
    lpp = static_cast<unsigned char*> (ref_img) + (y - radius_ -1) * width_ + ind1;
    rpp = static_cast<unsigned char*> (trg_img) + (y - radius_ -1) * width_ + ind1 - x_off_;

    for (int x = max_disp_ + x_off_ + radius_ + 1; x < width_ - radius_; x++)
    {
      sad_min = sad_max;

      lp++;
      rp++;
      lpp++;
      rpp++;

      for (int d = 0; d < max_disp_; d++)
      {
        v[x+radius_][d] = v[x+radius_][d] + abs( *lp - *rp ) - abs( *lpp - *rpp );
        rp--;
        rpp--;
        
        acc[d] = acc[d] + v[x+radius_][d] - v[x-radius_-1][d];

        if (acc[d] < sad_min)
        {
          sad_min = acc[d];
          dbest = static_cast<short int> (d);
        }
      }
      
      rp += max_disp_;
      rpp += max_disp_;

      if (ratio_filter_ > 0)
        dbest = doStereoRatioFilter (acc, dbest, sad_min, ratio_filter_, max_disp_);
      if (peak_filter_ > 0)
        dbest = doStereoPeakFilter (acc, dbest, peak_filter_, max_disp_);

      disp_map_[y * width_ + x] = static_cast<short int> (dbest * 16);

      //subpixel refinement
      if (dbest > 0 && dbest < max_disp_ - 1)
        disp_map_[y*width_+x] = computeStereoSubpixel (dbest, acc[dbest-1], acc[dbest], acc[dbest+1]);

    }//x
  }//y

  for (int d = 0; d < width_; d++)
    delete [] v[d];
  delete [] v;
  delete [] acc;
}

