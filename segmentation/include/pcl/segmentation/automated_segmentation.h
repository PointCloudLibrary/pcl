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
 * $Id: automated_segmentation.h $
 *
 */

#ifndef PCL_SEGMENTATION_AUTOMATED_SEGMENTATION_H_
#define PCL_SEGMENTATION_AUTOMATED_SEGMENTATION_H_

#include <pcl/pcl_base.h>

namespace pcl
{
  /** \brief @b AutomatedSegmentation represents the base class for classes that are about automated segmentation.
    * \details <ul><li>The derived classes may adapt at most 1 input parameter, which should be an intuitive one
    * (like a slider from 0 to 1, indicating overall segmentation aggresiveness).</li>
    * <li>All derived classes have to implement the <tt>applySegmentation ()</tt> method, which modifies the internal
    * <tt>segmentation_probabilities_</tt> array, indicating the likelihood that points are part of the desired
    * segmentation (1 = part of segmentation, 0 = not part of segmentation).</li>
    * <li>The <tt>segment (std::vector<int> &segmentation_indices)</tt> method calls the <tt>applySegmentation ()</tt>
    * method and afterwards passes all point indices that have <tt>segmentation_probabilities_ > 0.5</tt>.</li></ul>
    * \author Frits Florentinus
    * \ingroup segmentation
    */
  template<typename PointT>
  class AutomatedSegmentation : public PCLBase<PointT>
  {
    public:
      /** \brief Empty constructor.*/
      AutomatedSegmentation () :
          segmentation_probabilities_ (new std::vector<float>)
      {
      }

      /** \brief Empty virtual destructor.*/
      virtual
      ~AutomatedSegmentation ()
      {
      }

      /** \brief Set the internal <tt>segmentation_probabilities_</tt> array, indicating the likelihood that points
        * are part of the desired segmentation (1 = part of segmentation, 0 = not part of segmentation).
        * \warning The array size needs to match the size of the input point cloud, otherwise computation generates
        * a new default <tt>segmentation_probabilities_</tt> array, discarding the old one.
        * \note The initial/default values of <tt>segmentation_probabilities_</tt> are 0.5.
        * \param[in] segmentation_probabilities a pointer to the new <tt>segmentation_probabilities_</tt> array.
        */
      inline void
      setSegmentationProbabilities (boost::shared_ptr<std::vector<float> > segmentation_probabilities)
      {
        segmentation_probabilities_ = segmentation_probabilities;
      }

      /** \brief Get the internal <tt>segmentation_probabilities_</tt> array, indicating the likelihood that points
        * are part of the desired segmentation (1 = part of segmentation, 0 = not part of segmentation).
        * \return A pointer to the currently used <tt>segmentation_probabilities_</tt> array.
        */
      inline boost::shared_ptr<std::vector<float> >
      getSegmentationProbabilities ()
      {
        return (segmentation_probabilities_);
      }

      /** \brief Performs segmentation computation and returns the points that are likely part of the segmentation.
        * \details Calls the <tt>applySegmentation ()</tt> method, which modifies the internal <tt>segmentation_probabilities_</tt>
        * array, indicating the likelihood that points are part of the desired segmentation.
        * Afterwards, passes all point indices that have <tt>segmentation_probabilities_ > 0.5</tt>.
        * The intermediate results can be retrieved with the <tt>getSegmentationProbabilities ()</tt> method.
        * \note If the array size of <tt>segmentation_probabilities_</tt> does not match the size of the input point cloud,
        * a new array is generated prior to calling the <tt>applySegmentation ()</tt> method.
        * \param[out] segmentation_indices an array of indices into the input point cloud that are likely part of the segmentation.
        */
      inline void
      segment (std::vector<int> &segmentation_indices)
      {
        if (!initCompute ())
          return;

        // Generate the default segmentation_probabilities_ array
        if (segmentation_probabilities_->size () != input_->points.size ())
        {
          segmentation_probabilities_->resize (input_->points.size ());
          for (int p_it = 0; p_it < static_cast<int> (input_->points.size ()); ++p_it)
            (*segmentation_probabilities_)[p_it] = 0.5f;
        }

        // Call the pure virtual segmentation method
        applySegmentation ();

        // Pass the likely points to segmentation_indices
        // Assumes that the points not indexed through indices_ are never having segmentation_probabilities_ > 0.5f
        segmentation_indices.resize (indices_->size ());
        int i_out_it = 0;
        for (int i_in_it = 0; i_in_it < static_cast<int> (indices_->size ()); ++i_in_it)
          if ((*segmentation_probabilities_)[(*indices_)[i_in_it]] > 0.5f)
            segmentation_indices[i_out_it++] = (*indices_)[i_in_it];
        segmentation_indices.resize (i_out_it);

        deinitCompute ();
      }

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using PCLBase<PointT>::initCompute;
      using PCLBase<PointT>::deinitCompute;

      /** \brief Should modify the <tt>segmentation_probabilities_</tt> array, indicating the likelihood that points
        * are part of the desired segmentation (1 = part of segmentation, 0 = not part of segmentation).
        * \details The reason to use this type of weighting instead of boolean classification is to be able to
        * build more advanced systems that use \b AutomatedSegmentation classes as intermediate steps (e.g. SRAM).
        * For this reason, the <tt>applySegmentation ()</tt> method should ideally modify the <tt>segmentation_probabilities_</tt>
        * relatively; change the output probabilities relative to the input probabilities instead of setting them to fixed values.
        * \note The initial/default values of <tt>segmentation_probabilities_</tt> are 0.5.
        */
      virtual void
      applySegmentation () = 0;

      /** \brief Weighting to each of the points in the cloud, indicating the likelihood that points
        * are part of the desired segmentation (1 = part of segmentation, 0 = not part of segmentation).
        * \details The reason to use this type of weighting instead of boolean classification is to be able to
        * build more advanced systems that use \b AutomatedSegmentation classes as intermediate steps (e.g. SRAM).
        */
      boost::shared_ptr<std::vector<float> > segmentation_probabilities_;
  };
}

#endif  //#ifndef PCL_SEGMENTATION_AUTOMATED_SEGMENTATION_H_

