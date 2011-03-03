/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */
#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_TYPES_H_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_TYPES_H_

#include <limits>
#include <Eigen/Core>

namespace pcl
{
  namespace registration
  {
		/** \brief class representing a match between two descriptors */
    struct Correspondence
    {
      union {
        float data[4];
        struct {
          /** \brief index of the query point */
          int indexQuery;
          /** \brief index of the matching point */
          int indexMatch;
          /** \brief distance between query and matching point (w.r.t. the used feature descriptors) */
          float distance;
        };
      }; EIGEN_ALIGN16;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      /** \brief standard Constructor */
      inline Correspondence()
      : indexQuery (0)
      , indexMatch (-1)
      , distance (std::numeric_limits<float>::max ())
      {
      	data[3] = 1.0f;
      }
      /** \brief Ctor */
      inline Correspondence(int _indexQuery, int _indexMatch, float _distance)
      : indexQuery (indexQuery)
      , indexMatch (indexMatch)
      , distance (distance)
      { 
      	data[3] = 1.0f;
      }
    };
    
    /** \brief overloaded << operator */
    inline std::ostream& operator << (std::ostream& os, const Correspondence& c)
    {
      os << c.indexQuery << " " << c.indexMatch << " " << c.distance;
      return (os);
    }
  }
}

/** \brief calculates the mean and standard deviation of descriptor distances from correspondences
		\param[in] correspondences list of correspondences
		\param[out] mean the mean descriptor distance of correspondences
		\param[out] stddev the standard deviation of descriptor distances.
		\note The sample varaiance is used to determine the standard deviation
*/
inline void
pcl::registration::getCorDistMeanStd(const Correspondences &correspondences, double &mean, double &stddev)
{
  if ( correspondences.size() == 0 )
    return;

  double sum = 0, sq_sum = 0;

  for (size_t i = 0; i < correspondences.size(); ++i)
  {
    sum += correspondences[i].distance;
    sq_sum += correspondences[i].distance * correspondences[i].distance;
  }
  mean = sum / correspondences.size();
  double variance = (double)(sq_sum - sum * sum / correspondences.size()) / (correspondences.size() - 1);
  stddev = sqrt(variance);
}

/** \brief extracts the query indices
		\param[in] correspondences list of correspondences
		\param[out] indices array of extracted indices.
		\note order of indices corresponds to input list of descriptor correspondences
*/
inline void
pcl::registration::getQueryIndices(const Correspondences& correspondences, std::vector<int>& indices)
{
  indices.resize(correspondences.size());
  for ( unsigned int i = 0; i < correspondences.size(); ++i)
    indices[i] = correspondences[i].indexQuery;
}

/** \brief extracts the match indices
		\param[in] correspondences list of correspondences
		\param[out] indices array of extracted indices.
		\note order of indices corresponds to input list of descriptor correspondences
*/
inline void
pcl::registration::getMatchIndices(const Correspondences& correspondences, std::vector<int>& indices)
{
  indices.resize(correspondences.size());
  for ( unsigned int i = 0; i < correspondences.size(); ++i)
    indices[i] = correspondences[i].indexMatch;
}

#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_TYPES_H_ */
