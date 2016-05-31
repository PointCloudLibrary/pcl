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

#ifndef PCL_HOUGH_SPACE_3D_H_
#define PCL_HOUGH_SPACE_3D_H_


#include <pcl/point_types.h>

namespace pcl
{

  /** \brief HoughSpace3D is a 3D voting space. Cast votes can be interpolated in order to better deal with approximations introduced by bin quantization. A weight can also be associated with each vote. 
  * \author Federico Tombari (original), Tommaso Cavallari (PCL port)
  * \ingroup recognition
  */
  template<typename VoteT = double>
  class HoughSpace3D
  {

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \brief Constructor
      *
      * \param[in] min_coord minimum (x,y,z) coordinates of the Hough space 
      * \param[in] bin_size  size of each bing of the Hough space.
      * \param[in] max_coord maximum (x,y,z) coordinates of the Hough space.
      */
      HoughSpace3D (const Eigen::Vector3d &min_coord, const Eigen::Vector3d &bin_size, const Eigen::Vector3d &max_coord);

    /** \brief Reset all cast votes. */
    void
      reset ();

    /** \brief Casting a vote for a given position in the Hough space.
    * 
    * \param[in] single_vote_coord coordinates of the vote being cast (in absolute coordinates)
    * \param[in] weight weight associated with the vote.
    * \param[in] voter_id the numeric id of the voter. Useful to trace back the voting correspondence, if the vote is returned by findMaxima as part of a maximum of the Hough Space.
    * \return the index of the bin in which the vote has been cast.
    */
    int
      vote (const Eigen::Vector3d &single_vote_coord, VoteT weight, int voter_id);

    /** \brief Vote for a given position in the 3D space. The weight is interpolated between the bin pointed by single_vote_coord and its neighbors.
    * 
    * \param[in] single_vote_coord coordinates of the vote being cast.
    * \param[in] weight weight associated with the vote.
    * \param[in] voter_id the numeric id of the voter. Useful to trace back the voting correspondence, if the vote is returned by findMaxima as a part of a maximum of the Hough Space.
    * \return the index of the bin in which the vote has been cast.
    */
    int
      voteInt (const Eigen::Vector3d &single_vote_coord, VoteT weight, int voter_id);

    /** \brief Find the bins with most votes.
    * 
    * \param[in] min_threshold the minimum number of votes to be included in a bin in order to have its value returned. 
    * If set to a value between -1 and 0 the Hough space maximum_vote is found and the returned values are all the votes greater than -min_threshold * maximum_vote.
    * \param[out] maxima_values the list of Hough Space bin values greater than min_threshold.
    * \param[out] maxima_voter_ids for each value returned, a list of the voter ids who cast a vote in that position. 
    * \return The min_threshold used, either set by the user or found by this method.
    */
    VoteT
      findMaxima (VoteT min_threshold, std::vector<VoteT> & maxima_values, std::vector<std::vector<int> > &maxima_voter_ids);

    /** \brief Find the bin with most votes.
    * 
    * For each bin, compute the sum of the values of the bins in a 3 x 3 x 3 cube centered in the bin so as to find the bin with the largest sum.  
    * The method does not perform any non-maximum suppression.
    * The bin exhibiting the largest sum is the bin with most votes.
    * \param[out] maximum_voter_ids a list of the voter ids who cast a vote in the bin with most votes. 
    * \return the sum of the values of the bins in a 3 x 3 x 3 cube centered in the bin with most votes.
    */
    VoteT
      findMaximum (std::vector<int> &maximum_voter_ids);

  protected:

    /** \brief Minimum coordinate in the Hough Space. */
    Eigen::Vector3d min_coord_;

    /** \brief Size of each bin in the Hough Space. */
    Eigen::Vector3d bin_size_;

    /** \brief Number of bins for each dimension. */
    Eigen::Vector3i bin_count_;

    /** \brief Used to access hough_space_ as if it was a matrix. */
    int partial_bin_products_[4];

    /** \brief Total number of bins in the Hough Space. */
    int total_bins_count_;

    /** \brief The Hough Space. */
    std::vector<VoteT> hough_space_;
    //boost::unordered_map<int, VoteT> hough_space_;

    /** \brief List of voters for each bin. */
    boost::unordered_map<int, std::vector<int> > voter_ids_;
  };
}

//#ifdef PCL_NO_PRECOMPILE
#include <pcl/geometry/impl/hough_space_3d.hpp>
//#endif

#endif // PCL_HOUGH_SPACE_3D_H_
