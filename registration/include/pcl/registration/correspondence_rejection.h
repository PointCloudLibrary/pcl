/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *
 */

#ifndef PCL_REGISTRATION_CORRESPONDENCE_REJECTION_H_
#define PCL_REGISTRATION_CORRESPONDENCE_REJECTION_H_

#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_sorting.h>
#include <pcl/console/print.h>

namespace pcl
{
  namespace registration
  {
    /** @b CorrespondenceRejector represents the base class for correspondence rejection methods
      * \author Dirk Holz
      * \ingroup registration
      */
    class CorrespondenceRejector
    {
      public:
        /** \brief Empty constructor. */
        CorrespondenceRejector () : rejection_name_ (), input_correspondences_ () {};

        /** \brief Empty destructor. */
        virtual ~CorrespondenceRejector () {}

        /** \brief Provide a pointer to the vector of the input correspondences.
          * \param[in] correspondences the const boost shared pointer to a correspondence vector
          */
        virtual inline void 
        setInputCorrespondences (const CorrespondencesConstPtr &correspondences) 
        { 
          input_correspondences_ = correspondences; 
        };

        /** \brief Get a pointer to the vector of the input correspondences.
          * \return correspondences the const boost shared pointer to a correspondence vector
          */
        inline CorrespondencesConstPtr 
        getInputCorrespondences () { return input_correspondences_; };

        /** \brief Run correspondence rejection
          * \param[out] correspondences Vector of correspondences that have not been rejected.
          */
        inline void 
        getCorrespondences (pcl::Correspondences &correspondences)
        {
          if (!input_correspondences_ || (input_correspondences_->empty ()))
            return;

          applyRejection (correspondences);
        }

        /** \brief Get a list of valid correspondences after rejection from the original set of correspondences.
          * Pure virtual. Compared to \a getCorrespondences this function is
          * stateless, i.e., input correspondences do not need to be provided beforehand,
          * but are directly provided in the function call.
          * \param[in] original_correspondences the set of initial correspondences given
          * \param[out] remaining_correspondences the resultant filtered set of remaining correspondences
          */
        virtual inline void 
        getRemainingCorrespondences (const pcl::Correspondences& original_correspondences, 
                                     pcl::Correspondences& remaining_correspondences) = 0;

        /** \brief Determine the indices of query points of
          * correspondences that have been rejected, i.e., the difference
          * between the input correspondences (set via \a setInputCorrespondences)
          * and the given correspondence vector.
          * \param[in] correspondences Vector of correspondences after rejection
          * \param[out] indices Vector of query point indices of those correspondences
          * that have been rejected.
          */
        inline void 
        getRejectedQueryIndices (const pcl::Correspondences &correspondences, 
                                 std::vector<int>& indices)
        {
          if (!input_correspondences_ || input_correspondences_->empty ())
          {
            PCL_WARN ("[pcl::%s::getRejectedQueryIndices] Input correspondences not set (lookup of rejected correspondences _not_ possible).\n", getClassName ().c_str ());
            return;
          }

          pcl::getRejectedQueryIndices(*input_correspondences_, correspondences, indices);
        }

      protected:

        /** \brief The name of the rejection method. */
        std::string rejection_name_;

        /** \brief The input correspondences. */
        CorrespondencesConstPtr input_correspondences_;

        /** \brief Get a string representation of the name of this class. */
        inline const std::string& 
        getClassName () const { return (rejection_name_); }

        /** \brief Abstract rejection method. */
        virtual void 
        applyRejection (Correspondences &correspondences) = 0;
    };

  }
}

#endif /* PCL_REGISTRATION_CORRESPONDENCE_REJECTION_H_ */

