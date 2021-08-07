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
 * $Id$
 *
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

namespace pcl {
namespace registration {
/** \brief @b ConvergenceCriteria represents an abstract base class for
 * different convergence criteria used in registration loops.
 *
 * This should be used as part of an Iterative Closest Point (ICP)-like
 * method, to verify if the algorithm has reached convergence.
 *
 * Typical convergence criteria that could inherit from this include:
 *
 *  * a maximum number of iterations has been reached
 *  * the transformation (R, t) cannot be further updated (the difference between
 * current and previous is smaller than a threshold)
 *  * the Mean Squared Error (MSE) between the current set of correspondences and the
 * previous one is smaller than some threshold
 *
 * \author Radu B. Rusu
 * \ingroup registration
 */
class PCL_EXPORTS ConvergenceCriteria {
public:
  using Ptr = shared_ptr<ConvergenceCriteria>;
  using ConstPtr = shared_ptr<const ConvergenceCriteria>;

  /** \brief Empty constructor. */
  ConvergenceCriteria() {}

  /** \brief Empty destructor. */
  virtual ~ConvergenceCriteria() {}

  /** \brief Check if convergence has been reached. Pure virtual. */
  virtual bool
  hasConverged() = 0;

  /** \brief Bool operator. */
  operator bool() { return (hasConverged()); }
};
} // namespace registration
} // namespace pcl
