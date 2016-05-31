/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id:$
 *
 */

#ifndef PCL_HOUGH_SPACE_3D_IMPL_H_
#define PCL_HOUGH_SPACE_3D_IMPL_H_

#include <pcl/geometry/hough_space_3d.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename VoteT>
pcl::HoughSpace3D<VoteT>::HoughSpace3D (const Eigen::Vector3d &min_coord, const Eigen::Vector3d &bin_size, const Eigen::Vector3d &max_coord)
{
  min_coord_ = min_coord;
  bin_size_ = bin_size;

  for (int i = 0; i < 3; ++i)
  {
    bin_count_[i] = static_cast<int> (ceil ((max_coord[i] - min_coord_[i]) / bin_size_[i]));
  }

  partial_bin_products_[0] = 1;
  for (int i=1; i<=3; ++i)
    partial_bin_products_[i] = bin_count_[i-1]*partial_bin_products_[i-1];

  total_bins_count_ = partial_bin_products_[3];

  hough_space_.clear ();
  hough_space_.resize (total_bins_count_, 0.0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename VoteT> void
  pcl::HoughSpace3D<VoteT>::reset ()
{
  hough_space_.clear ();
  hough_space_.resize (total_bins_count_, 0.0);

  voter_ids_.clear ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename VoteT> int
  pcl::HoughSpace3D<VoteT>::vote (const Eigen::Vector3d &single_vote_coord, VoteT weight, int voter_id)
{
  int index = 0;

  for (int i=0; i<3; ++i)
  {
    int currentBin = static_cast<int> (floor ((single_vote_coord[i] - min_coord_[i])/bin_size_[i]));
    if (currentBin < 0 || currentBin >= bin_count_[i])
    {
      //PCL_ERROR("Current Vote goes out of bounds in the Hough Table!\nDimension: %d, Value inserted: %f, Min value: %f, Max value: %f\n", i, 
      //  single_vote_coord[i], min_coord_[i], min_coord_[i] + bin_size_[i]*bin_count_[i]);
      return -1;
    }

    index += partial_bin_products_[i] * currentBin;
  }

  hough_space_[index] += weight;
  voter_ids_[index].push_back (voter_id);

  return (index);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename VoteT> int
  pcl::HoughSpace3D<VoteT>::voteInt (const Eigen::Vector3d &single_vote_coord, VoteT weight, int voter_id)
{
  int central_bin_index = 0;

  const int n_neigh = 27; // total number of neighbours = 3^nDim = 27

  Eigen::Vector3i central_bin_coord;
  Eigen::Vector3f bin_centroid;
  Eigen::Vector3f central_bin_weight;
  Eigen::Vector3i interp_bin;
  std::vector<float> interp_weight (n_neigh);

  for (int n = 0; n < n_neigh; ++n)
    interp_weight[n] = 1.0;

  for (int d = 0; d < 3; ++d)
  {
    // Compute coordinates of central bin
    central_bin_coord[d] = static_cast<int> (floor ((single_vote_coord[d] - min_coord_[d]) / bin_size_[d]));
    if (central_bin_coord[d] < 0 || central_bin_coord[d] >= bin_count_[d])
    {
      //PCL_ERROR("Current Vote goes out of bounds in the Hough Table!\nDimension: %d, Value inserted: %f, Min value: %f, Max value: %f\n", d,
      //  single_vote_coord[d], min_coord_[d], min_coord_[d] + bin_size_[d]*bin_count_[d]);
      return -1;
    }

    central_bin_index += partial_bin_products_[d] * central_bin_coord[d];

    // Compute coordinates of the centroid of the bin
    bin_centroid[d] = static_cast<float> ((2 * static_cast<double> (central_bin_coord[d]) * bin_size_[d] + bin_size_[d]) / 2.0 );

    // Compute interpolated weight for each coordinate of the central bin
    central_bin_weight[d] = static_cast<float> (1 - (fabs (single_vote_coord[d] - min_coord_[d] - bin_centroid[d]) / bin_size_[d] ) );

    // Compute the neighbor bins where the weight has to be interpolated
    if ((single_vote_coord[d] - min_coord_[d]) < bin_centroid[d])
    {
      interp_bin[d] = central_bin_coord[d] - 1;
    }
    else if ((single_vote_coord[d] - min_coord_[d]) > bin_centroid[d])
    {
      interp_bin[d] = central_bin_coord[d] + 1;
    }
    else
    {
      interp_bin[d] = central_bin_coord[d]; // the vote is precisely in the middle, so along this dimension the vote is given all to the central bin
    }
  }

  // For each neighbor of the central point
  //int counterRealVotes = 0;
  for (int n = 0; n < n_neigh; ++n)
  {
    int final_bin_index = 0;
    int exp = 1;
    int curr_neigh_index = 0;
    bool invalid = false;

    for (int d = 0; d < 3; ++d)
    {
      curr_neigh_index = central_bin_coord[d] + ( n % (exp*3) ) / exp - 1; // (n % 3^(d+1) / 3^d) - 1
      if (curr_neigh_index >= 0 && curr_neigh_index <= bin_count_[d]-1)
      {
        // Each coordinate of the neighbor has to be equal either to one of the central bin or to one of the interpolated bins
        if(curr_neigh_index == interp_bin[d])
        {
          interp_weight[n] *= 1-central_bin_weight[d];
        }
        else if(curr_neigh_index == central_bin_coord[d])
        {
          interp_weight[n] *= central_bin_weight[d];
        }
        else
        {
          invalid = true;
          break;
        }

        final_bin_index += curr_neigh_index * partial_bin_products_[d];
      }
      else
      {
        invalid = true;
        break;
      }
      exp *= 3;
    }

    if (!invalid)
    {
      hough_space_[final_bin_index] += (VoteT)(weight * interp_weight[n]);
      voter_ids_[final_bin_index].push_back (voter_id);
    }
  }

  return (central_bin_index);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename VoteT> VoteT
  pcl::HoughSpace3D<VoteT>::findMaxima (VoteT min_threshold, std::vector<VoteT> &maxima_values, std::vector<std::vector<int> > &maxima_voter_ids)
{
  // If min_threshold between -1 and 0 use it as a percentage of maximum vote
  if (min_threshold < 0)
  {
    VoteT hough_maximum = std::numeric_limits<VoteT>::min ();
    for (int i = 0; i < total_bins_count_; ++i)
    {
      if (hough_space_[i] > hough_maximum)
      {
        hough_maximum = hough_space_[i];
      }
    }

    min_threshold = min_threshold >= -1 ? -min_threshold * hough_maximum : hough_maximum;
  }

  maxima_voter_ids.clear ();
  maxima_values.clear ();

  double indexes[3];

  //int zeros = 0;

  for (int i=0; i < total_bins_count_; ++i)
  {
    //if (hough_space_[i] == 0)
    //{
    //  ++zeros;
    //}
    if (hough_space_[i] < min_threshold)
      continue;

    // Check with neighbors
    bool is_maximum = true;
    int moduled_index = i;

    for (int k = 2; k >= 0; --k){

      moduled_index = moduled_index % partial_bin_products_[k+1];
      indexes[k] = moduled_index / partial_bin_products_[k];

      if (indexes[k] > 0 && hough_space_[i] < hough_space_[i-partial_bin_products_[k]])
      {
        is_maximum = false;
        break;
      }
      if (indexes[k] < bin_count_[k]-1 && hough_space_[i] < hough_space_[i+partial_bin_products_[k]])
      {
        is_maximum = false;
        break;
      }
    }

    if (is_maximum)
    {
      maxima_values.push_back (hough_space_[i]);
      maxima_voter_ids.push_back ( voter_ids_[i] );
    }
  }

  return (min_threshold);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename VoteT> VoteT
  pcl::HoughSpace3D<VoteT>::findMaximum (std::vector<int> &maximum_voter_ids)
{

  //find all displacements
  int nNeighs = (int)pow((float)3, 3);
  std::vector<int> vDispl(nNeighs);
  std::vector<std::vector<int> > vvActiveDim(nNeighs, std::vector<int>(3));
  vDispl[0] = -1;
  vvActiveDim[0][0] = -1;
  vvActiveDim[0][1] = 0;
  vvActiveDim[0][2] = 0;
  vDispl[1] = 0;
  vvActiveDim[1][0] = 0;
  vvActiveDim[1][1] = 0;
  vvActiveDim[1][2] = 0;
  vDispl[2] = +1;
  vvActiveDim[2][0] = 1;
  vvActiveDim[2][1] = 0;
  vvActiveDim[2][2] = 0;
  size_t vectorSize;
  int iNeigh = 3;
  for(int di = 1; di<3; di++)
  {
    vectorSize = iNeigh;
    for(size_t si=0; si<vectorSize; si++)
    {
      vDispl[iNeigh] = -partial_bin_products_[di] + vDispl[si];
      vvActiveDim[iNeigh] = vvActiveDim[si];
      vvActiveDim[iNeigh][di] = -1;
      iNeigh++;

      vDispl[iNeigh] = partial_bin_products_[di] + vDispl[si];
      vvActiveDim[iNeigh] = vvActiveDim[si];
      vvActiveDim[iNeigh][di] = 1;
      iNeigh++;
    }
  }
  vectorSize = vDispl.size();

  VoteT maxSum = 0;


  VoteT *hough_space_ptr = &(hough_space_[0]);
  int *ptrDispl;

  int width = bin_count_[0] - 1;
  int height = bin_count_[1] - 1;
  int depth = bin_count_[2] - 1;
  int maxWidth = 0;
  int maxHeight = 0;
  int maxDepth = 0;

  hough_space_ptr += partial_bin_products_[2];
  for(int de=1; de<depth; de++)
  {
    hough_space_ptr += partial_bin_products_[1];
    for(int he=1; he<height; he++)
    {
      hough_space_ptr++;
      for(int wi=1; wi<width; wi++)
      {
        if(*hough_space_ptr != 0)
        {
          //sum all the 27 neighs
          ptrDispl = &vDispl[0];
          VoteT voteSum = 0;
          for(size_t si=0; si<vectorSize; si++)
          {
            voteSum += *(hough_space_ptr + *ptrDispl);
            ptrDispl++;
          }
          if(voteSum > maxSum)
          {
            maxSum = voteSum;
            maxWidth = wi;
            maxHeight = he;
            maxDepth = de;
          }
        }
        hough_space_ptr++;
      }
      hough_space_ptr++;
    }
    hough_space_ptr += partial_bin_products_[1];
  }

  //consider borders
  int deOut=0;
  std::vector<int>* ptrActiveDim;
  hough_space_ptr = &(hough_space_[0]);
  for(int he=0; he<(height+1); he++)
  {
    for(int wi=0; wi<(width+1); wi++)
    {
      if(*hough_space_ptr != 0)
      {
        //sum all the 27 neighs
        ptrDispl = &vDispl[0];
        ptrActiveDim = &vvActiveDim[0];
        VoteT voteSum = 0;
        for(size_t si=0; si<vectorSize; si++)
        {
          if( (*ptrActiveDim)[2] != -1)
          {
            if( ((*ptrActiveDim)[1] != -1) || (he!=0) )
            {	
              if( ((*ptrActiveDim)[1] != 1) || (he!=height) )
              {
                if( ((*ptrActiveDim)[0] != -1) || (wi!=0) )
                {	
                  if( ((*ptrActiveDim)[0] != 1) || (wi!=width) )
                  {
                    voteSum += *(hough_space_ptr + *ptrDispl);
                  }
                }
              }
            }
          }
          ptrDispl++;
          ptrActiveDim++;
        }
        if(voteSum > maxSum)
        {
          maxSum = voteSum;
          maxWidth = wi;
          maxHeight = he;
          maxDepth = deOut;
        }
      }
      hough_space_ptr++;
    }
  }

  deOut=height;
  hough_space_ptr = &(hough_space_[0]) + depth*(height+1)*(width+1);
  for(int he=0; he<(height+1); he++)
  {
    for(int wi=0; wi<(width+1); wi++)
    {
      if(*hough_space_ptr != 0)
      {
        //sum all the 27 neighs
        ptrDispl = &vDispl[0];
        ptrActiveDim = &vvActiveDim[0];
        VoteT voteSum = 0;
        for(size_t si=0; si<vectorSize; si++)
        {
          if( (*ptrActiveDim)[2] != 1)
          {
            if( ((*ptrActiveDim)[1] != -1) || (he!=0) )
            {	
              if( ((*ptrActiveDim)[1] != 1) || (he!=height) )
              {
                if( ((*ptrActiveDim)[0] != -1) || (wi!=0) )
                {	
                  if( ((*ptrActiveDim)[0] != 1) || (wi!=width) )
                  {
                    voteSum += *(hough_space_ptr + *ptrDispl);
                  }
                }
              }
            }
          }
          ptrDispl++;
          ptrActiveDim++;
        }
        if(voteSum > maxSum)
        {
          maxSum = voteSum;
          maxWidth = wi;
          maxHeight = he;
          maxDepth = deOut;
        }
      }
      hough_space_ptr++;
    }
  }


  int heOut=0;
  hough_space_ptr = &(hough_space_[0]) + (height+1)*(width+1);
  for(int de=1; de<(depth); de++)
  {
    for(int wi=0; wi<(width+1); wi++)
    {
      if(*hough_space_ptr != 0)
      {
        //sum all the 27 neighs
        ptrDispl = &vDispl[0];
        ptrActiveDim = &vvActiveDim[0];
        VoteT voteSum = 0;
        for(size_t si=0; si<vectorSize; si++)
        {
          if( (*ptrActiveDim)[1] != -1)
          {
            if( ((*ptrActiveDim)[0] != -1) || (wi!=0) )
            {	
              if( ((*ptrActiveDim)[0] != 1) || (wi!=width) )
              {
                voteSum += *(hough_space_ptr + *ptrDispl);
              }
            }
          }
          ptrDispl++;
          ptrActiveDim++;
        }
        if(voteSum > maxSum)
        {
          maxSum = voteSum;
          maxWidth = wi;
          maxHeight = heOut;
          maxDepth = de;
        }
      }
      hough_space_ptr++;
    }
    hough_space_ptr += height*(width+1);
  }

  heOut=height;
  hough_space_ptr = &(hough_space_[0]) + (height+1)*(width+1) + (width+1)*height;
  for(int de=1; de<(depth); de++)
  {
    for(int wi=0; wi<(width+1); wi++)
    {
      if(*hough_space_ptr != 0)
      {
        //sum all the 27 neighs
        ptrDispl = &vDispl[0];
        ptrActiveDim = &vvActiveDim[0];
        VoteT voteSum = 0;
        for(size_t si=0; si<vectorSize; si++)
        {
          if( (*ptrActiveDim)[1] != +1)
          {
            if( ((*ptrActiveDim)[0] != -1) || (wi!=0) )
            {	
              if( ((*ptrActiveDim)[0] != 1) || (wi!=width) )
              {
                voteSum += *(hough_space_ptr + *ptrDispl);
              }
            }
          }
          ptrDispl++;
          ptrActiveDim++;
        }
        if(voteSum > maxSum)
        {
          maxSum = voteSum;
          maxWidth = wi;
          maxHeight = heOut;
          maxDepth = de;
        }
      }
      hough_space_ptr++;
    }
    hough_space_ptr += height*(width+1);
  }


  int wiOut=0;
  hough_space_ptr = &(hough_space_[0]) + (height+1)*(width+1) + (width+1);
  for(int de=1; de<depth; de++)
  {
    for(int he=1; he<height; he++)
    {
      if(*hough_space_ptr != 0)
      {
        //sum all the 27 neighs
        ptrDispl = &vDispl[0];
        ptrActiveDim = &vvActiveDim[0];
        VoteT voteSum = 0;
        for(size_t si=0; si<vectorSize; si++)
        {
          if( (*ptrActiveDim)[0] != -1)
          {
            voteSum += *(hough_space_ptr + *ptrDispl);
          }
          ptrDispl++;
          ptrActiveDim++;
        }
        if(voteSum > maxSum)
        {
          maxSum = voteSum;
          maxWidth = wiOut;
          maxHeight = he;
          maxDepth = de;
        }
      }
      hough_space_ptr += (width+1);
    }
    hough_space_ptr += (width+1);
  }

  wiOut=width;
  hough_space_ptr = &(hough_space_[0]) + (height+1)*(width+1) + (width+1) + width;
  for(int de=1; de<depth; de++)
  {
    for(int he=1; he<height; he++)
    {
      if(*hough_space_ptr != 0)
      {
        //sum all the 27 neighs
        ptrDispl = &vDispl[0];
        ptrActiveDim = &vvActiveDim[0];
        VoteT voteSum = 0;
        for(size_t si=0; si<vectorSize; si++)
        {
          if( (*ptrActiveDim)[0] != 1)
          {
            voteSum += *(hough_space_ptr + *ptrDispl);
          }
          ptrDispl++;
          ptrActiveDim++;
        }
        if(voteSum > maxSum)
        {
          maxSum = voteSum;
          maxWidth = wiOut;
          maxHeight = he;
          maxDepth = de;
        }
      }
      hough_space_ptr += (width+1);
    }
    hough_space_ptr += (width+1);
  }


  width++;
  height++;
  depth++;

  int maxIndex = height*width*maxDepth + width*maxHeight + maxWidth;

  //accumulate vote ids for 3x3x3 cube centered at bin maxIndex 
  maximum_voter_ids.clear();

  ptrDispl = &vDispl[0];
  int currIndex;
  for(size_t si=0; si<vectorSize; si++)
  {
    currIndex = maxIndex + *ptrDispl;

    maximum_voter_ids.insert (maximum_voter_ids.end(), voter_ids_[currIndex].begin(), voter_ids_[currIndex].end() );

    ptrDispl++;
  }

  return maxSum;

}


#endif // PCL_HOUGH_SPACE_3D_IMPL_H_
