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

/** \brief Compact Representation of the per-pixel likelihood Stereo Matching algorithm implementation
  * please see related documentation on stereo/stereo_matching.h
  *
  * \author Jilliam Diaz Barros (jilliam@ieee.org)
  * \ingroup stereo
  */


#include "pcl/stereo/stereo_matching.h"
#include "stereo_aux.h"

//////////////////////////////////////////////////////////////////////////////
pcl::CompactRepresentationStereoMatching::CompactRepresentationStereoMatching ()
{
  radius_ = 15;                     //default value

  filter_radius_ = 2;               //default value
  num_disp_candidates_ = 20;        //default value
  thresh_tad_ = 50;                 //default value

  is_soft_thresh_ = false;          //default value

  sampling_method_ = 0;             //default value
  sampling_ratio_ = 1;              //default value

  gamma_s_ = 17.5;                  //default value
  gamma_c_ = 5 ;                    //default value
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::CompactRepresentationStereoMatching::compute_impl (unsigned char* ref_img, unsigned char* trg_img)
{
  //
  int n = radius_ * 2 + 1;
  int m = filter_radius_ * 2 + 1;
  int sampling_ratio_ind=1, sampling_ratio_dep=1;

  if (sampling_method_ == 0)
  {
    sampling_ratio_ind = sampling_ratio_;
  }
  else
  {
    sampling_ratio_dep = sampling_ratio_;
  }

  //Allocation and initialization of prefiltered likelihood function e_1_h(p,d) and matrix with indices of subset of disparity hypothesis o(q,d)
  float *pixel_lhood = new float[height_ * width_* max_disp_];
  memset (pixel_lhood, 0.0, sizeof (float) * height_ * width_ * max_disp_);

  int *acc = new int[max_disp_];
  memset (acc, 0, sizeof (int) * max_disp_);

  int **v = new int *[width_];
  for (int d = 0; d < width_; d++)
  {
    v[d] = new int[max_disp_];
    memset (v[d], 0, sizeof (int) * max_disp_);
  }

  //First row
  for (int x = max_disp_ + x_off_; x < max_disp_ + x_off_ + m; x++)
  {
    for (int d = 0; d < max_disp_; d++)
    {
      for (int y = 0; y < m; y++)
        v[x][d] += thresh_tad_ - abs (ref_img[y*width_+x] - trg_img[y*width_+x -d-x_off_]);
    }
  }

  //First row: other positions
  for (int x = max_disp_ + x_off_ + filter_radius_ + 1; x < width_ - filter_radius_; x++)
  {
    for (int d = 0; d < max_disp_; d++)
    {
      for (int y = 0; y < m; y++)
      {
        v[x+filter_radius_][d] += thresh_tad_ - abs (ref_img[y*width_ + x+filter_radius_] - trg_img[ y*width_ + x+filter_radius_ -d-x_off_]);
      }
    }//d
  }//x

  //2
  unsigned char *lp, *rp, *lpp, *rpp;
  int ind1 = filter_radius_ + filter_radius_ + max_disp_ + x_off_;

  for (int y = filter_radius_ + 1; y < height_ - filter_radius_; y++)
  {
    //First position
    for (int d = 0; d < max_disp_; d++)
    {
      acc[d] = 0;
      for (int x = max_disp_ + x_off_; x < max_disp_ + x_off_ + m; x++)
      {
        v[x][d] += - abs( ref_img[ (y+filter_radius_)*width_+x] - trg_img[ (y+filter_radius_)*width_+x -d-x_off_] ) + abs( ref_img[ (y-filter_radius_-1)*width_+x] - trg_img[ (y-filter_radius_-1)*width_ +x -d-x_off_] );

        acc[d] += v[x][d];
      }
    }

    //All other positions
    lp  = static_cast<unsigned char*> (ref_img) + (y + filter_radius_)    * width_ + ind1;
    rp  = static_cast<unsigned char*> (trg_img) + (y + filter_radius_)    * width_ + ind1 - x_off_;
    lpp = static_cast<unsigned char*> (ref_img) + (y - filter_radius_ -1) * width_ + ind1;
    rpp = static_cast<unsigned char*> (trg_img) + (y - filter_radius_ -1) * width_ + ind1 - x_off_;

    for (int x = max_disp_ + x_off_ + filter_radius_ + 1; x < width_ - filter_radius_; x++)
    {
      std::vector< int > local_max, non_local_max, temp_vector;
      std::vector<size_t> local_max_index, non_max_index, index_cand;

      lp++;
      rp++;
      lpp++;
      rpp++;

      for (int d = 0; d < max_disp_; d++)
      {
        v[x+filter_radius_][d] = v[x+filter_radius_][d] - abs( *lp - *rp ) + abs( *lpp - *rpp );
        rp--;
        rpp--;

        acc[d] = acc[d] + v[x+filter_radius_][d] - v[x-filter_radius_-1][d];

        if ( (x % sampling_ratio_ind == 0) && (y % sampling_ratio_ind == 0) )
        {
          pixel_lhood[d*width_*height_ + y*width_ + x] = acc[d];
        }
        else
        {
          pixel_lhood[d*width_*height_ + y*width_ + x] = 0.0;
        }
      }      

      rp += max_disp_;
      rpp += max_disp_;

      // Spatial sampling
      if ( (x % sampling_ratio_ind == 0) && (y % sampling_ratio_ind == 0) )
      {
        int global_max = acc[0], global_max_index  = 0;
        for (int d = 0; d < max_disp_; d++)
        {
          // Compute local maxima
          if ( ( (d==0) && (acc[d] > acc[d+1]) ) || ( (d==(max_disp_-1)) && (acc[d] > acc[d-1]) ) || ( (d>0 && d<max_disp_-1) && (acc[d] > acc[d-1]) && (acc[d] > acc[d+1]) ) )
          {
            local_max.push_back(acc[d]);
            local_max_index.push_back(d);
          }
          else
          {
            non_local_max.push_back(acc[d]);
            non_max_index.push_back(d);
          }

          // Compute global maxima
          if (acc[d] > global_max)
          {
            global_max = acc[d];
            global_max_index = d;
          }
        }

        //If number of local maxima is greater than num_disp_candidates_, select num_disp_candidates_ disparities candidates with the greatest likelihood
        if (local_max_index.size() >= num_disp_candidates_)
        {
          sort(local_max, local_max, index_cand);
          std::sort (index_cand.begin(), index_cand.begin()+num_disp_candidates_);

          int ind=0;
          for (int d = 0; d < max_disp_; d++)
          {
            if ( (ind < num_disp_candidates_) && ( d == local_max_index[ index_cand[ind] ] ) )
            {
              ind++;

              if (is_soft_thresh_ == true)
              {
                pixel_lhood[d*width_*height_ + y*width_ + x] *= static_cast<float> (exp(-pow(global_max_index - d, 2.0)/num_disp_candidates_));
              }

            }
            else
            {
              pixel_lhood[d*width_*height_ + y*width_ + x] = 0.0;
            }
          }
        }
        else //If number of local maxima is less than num_disp_candidates_, fill with candidates in the per-pixel likelihood function with the greatest values
        {
          int rem_values = num_disp_candidates_ - local_max_index.size();
          sort(non_local_max, temp_vector, index_cand);
          std::sort (index_cand.begin(), index_cand.begin()+rem_values);

          for (int i = 0; i < rem_values; i++)
          {
            local_max_index.push_back( non_max_index[ index_cand[i] ] );
          }

          std::sort (local_max_index.begin(), local_max_index.end());

          int ind=0;
          for (int d = 0; d < max_disp_; d++)
          {
            if ( (ind < num_disp_candidates_) && ( d == local_max_index[ind] ) )
            {
              ind++;

              if (is_soft_thresh_ == true)
              {
                pixel_lhood[d*width_*height_ + y*width_ + x] *= static_cast<float> ( exp( -pow(global_max_index - d, 2.0)/num_disp_candidates_ ) );
              }
            }
            else
            {
              pixel_lhood[d*width_*height_ + y*width_ + x] = 0.0;
            }
          }
        }
      }
    }//x
  }//y

  //Allocation and initialization of agreggated cost E^h(p,d)
  float *agg_cost = new float[max_disp_];
  memset (agg_cost, 0.0, sizeof (float) * max_disp_);

  //Spatial distance initialization
  float **ds = new float *[n];
  for (int j = 0; j < n; j++)
  {
    ds[j] = new float[n];
  }
  for (int i = -radius_; i <= radius_; i++)
  {
    for (int j = -radius_; j <= radius_; j++)
    {
      ds[i+radius_][j+radius_] = static_cast<float> (exp (- sqrt( (i*i)+(j*j) ) / gamma_s_));
    }
  }

  //LUT for color distance weight computation
  float lut[256];
  for (int j = 0; j < 256; j++)
    lut[j] = float (exp (-j / gamma_c_));


  //Allocation and initialization of weight w(p,q)
  float **weight_left = new float *[n];
  for (int j = 0; j < n; j++)
  {
    weight_left[j] = new float[n];
    memset (weight_left[j], 1.0, sizeof (float) * n);
  }

  //Compute weight and perform aggregation cost
  for (int y = radius_ + 1; y < height_ - radius_; y++)
  {
    for (int x = max_disp_ + x_off_ + 1; x < width_ - radius_; x++)
    {
      int ind_sampling_x = ( (x-radius_)%sampling_ratio_ind==0 ) ? 0 : (sampling_ratio_ind-(x-radius_)%sampling_ratio_ind);
      int ind_sampling_y = ( (y-radius_)%sampling_ratio_ind==0 ) ? 0 : (sampling_ratio_ind-(y-radius_)%sampling_ratio_ind);

      for (int i = (radius_%sampling_ratio_dep==0 && (x-radius_)%sampling_ratio_ind==0) ? -radius_ : (-radius_+(radius_%sampling_ratio_dep)+(ind_sampling_x)); i <= radius_; i+=sampling_ratio_dep*sampling_ratio_ind)
      {
        for (int j = (radius_%sampling_ratio_dep==0 && (y-radius_)%sampling_ratio_ind==0) ? -radius_ : (-radius_+(radius_%sampling_ratio_dep)+(ind_sampling_y)); j <= radius_; j+=sampling_ratio_dep*sampling_ratio_ind)
        {
          weight_left[i+radius_][j+radius_] = lut[ abs(ref_img[(y+j)*width_+x+i] - ref_img[y*width_+x]) ] * ds[i+radius_][j+radius_];
        }
      }

      for (int d = 0; d < max_disp_; d++)
      {
      //if (pixel_lhood[d*width_*height_ + (y)*width_ + x] > 0)
      //{
        float num = 0.0;

        for (int i = (radius_%sampling_ratio_dep==0 && (x-radius_)%sampling_ratio_ind==0) ? -radius_ : (-radius_+(radius_%sampling_ratio_dep)+(ind_sampling_x)); i <= radius_; i+=sampling_ratio_dep*sampling_ratio_ind)
        {
          for (int j = (radius_%sampling_ratio_dep==0 && (y-radius_)%sampling_ratio_ind==0) ? -radius_ : (-radius_+(radius_%sampling_ratio_dep)+(ind_sampling_y)); j <= radius_; j+=sampling_ratio_dep*sampling_ratio_ind)
          {
            if (pixel_lhood[d*width_*height_ + (y+j)*width_ + x+i] > 0)
            {
              num += weight_left[i+radius_][j+radius_] * (pixel_lhood[d*width_*height_ + (y+j)*width_ + x+i]);
            }
          }
        }
        agg_cost[d] = num;
      }
      //}

      float max_disp = agg_cost[0];
      short int dbest = 0;
      for (int d = 1; d < max_disp_; d++)
      {
        if (agg_cost[d] > max_disp)
        {
          max_disp = agg_cost[d];
          dbest = static_cast<short int> (d);
        }
      }

      if (ratio_filter_ > 0)
        dbest = doStereoRatioFilter (agg_cost, dbest, max_disp, ratio_filter_, max_disp_);
      if (peak_filter_ > 0)
        dbest = doStereoPeakFilter (agg_cost, dbest, peak_filter_, max_disp_);

      disp_map_[y * width_ + x] = static_cast<short int> (dbest * 16);

      //Subpixel refinement
      if (dbest > 0 && dbest < max_disp_ - 1)
        disp_map_[y*width_+x] = computeStereoSubpixel (dbest, agg_cost[dbest-1], agg_cost[dbest], agg_cost[dbest+1]);

    }
  }

  delete [] pixel_lhood;

  for (int j = 0; j < width_; j++)
  {
    delete [] v[j];
  }

  delete [] acc;
  delete [] agg_cost;

  for (int j = 0; j < n; j++)
  {
    delete [] ds[j];
    delete [] weight_left[j];
  }
}
