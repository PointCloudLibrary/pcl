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

#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_H_

#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_sorting.h>

namespace pcl
{
  namespace registration
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** @b CorrespondenceRejector represents the base class for correspondence rejection methods
     * \author Dirk Holz
     */
    // template <typename PointSource, typename PointTarget>
    // TODO: maybe templated later
    // (on input and/or target points if these need to be accessible for rejection)
    class CorrespondenceRejector /*: public PCLBase<PointSource> */
    {

    public:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      CorrespondenceRejector() : input_correspondences_() {};

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Provide a pointer to the vector of the input correspondences.
        * \param correspondences the const boost shared pointer to a std::vector of correspondences
        */
      virtual inline void setInputCorrespondences(const CorrespondencesConstPtr &correspondences) { input_correspondences_ = correspondences; };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a pointer to the vector of the input correspondences. */
      inline CorrespondencesConstPtr getInputCorrespondences() { return input_correspondences_; };

      inline void getCorrespondeces(pcl::registration::Correspondences &correspondences)
      {
        // something like initCompute() ?

        if ( !input_correspondences_ || (input_correspondences_->size() == 0) )
          return;

        applyRejection(correspondences);

        // something like deinintCompute() ?
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a pointer to the vector of the input correspondences. */
      virtual inline void getCorrespondences(const pcl::registration::Correspondences& original_correspondences, pcl::registration::Correspondences& remaining_correspondences) = 0;

      static bool compareCorrespondencesDistance(pcl::registration::Correspondence a, pcl::registration::Correspondence b) { return (a.distance < b.distance); }

      inline void getRejectedQueryIndices(const std::vector<pcl::registration::Correspondence> &correspondences, std::vector<int>& indices)
      {
        if ( !input_correspondences_ || input_correspondences_->size() == 0 )
        {
          ROS_WARN ("[pcl::%s::getRejectedQueryIndices] Input correspondences not set (lookup of rejected correspondences _not_ possible).", getClassName ().c_str ());
          return;
        }

        std::vector<int> indices_before, indices_after;
        indices_before.resize(input_correspondences_->size());
        for (unsigned int i = 0; i < input_correspondences_->size(); ++i)
          indices_before[i] = input_correspondences_->at(i).indexQuery;
        indices_after.resize(correspondences.size());
        for (unsigned int i = 0; i < correspondences.size(); ++i)
          indices_after[i] = correspondences.at(i).indexQuery;

        std::vector<int> remaining_indices;
        set_difference (
            indices_before.begin (), indices_before.end (),
            indices_after.begin (),  indices_after.end (),
            inserter (remaining_indices, remaining_indices.begin ()));
        indices =  remaining_indices;
      }

    protected:

      /** \brief The name of the rejection method. */
      std::string rejection_name_;

      /** \brief The input correspondences. */
      CorrespondencesConstPtr input_correspondences_;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get a string representation of the name of this class. */
      inline const std::string& getClassName () const { return (rejection_name_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Abstract rejection method. */
      virtual void applyRejection(Correspondences &correspondences) = 0;
    };

  }
}


#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_H_ */
