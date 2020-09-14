/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 */

#include <pcl/types.h>
#include <pcl/correspondence.h>
#include <algorithm>
#include <iterator>

//////////////////////////////////////////////////////////////////////////////
void
pcl::getRejectedQueryIndices (const pcl::Correspondences &correspondences_before,
                              const pcl::Correspondences &correspondences_after,
                              Indices& indices,
                              bool presorting_required)
{
  indices.clear();

  const auto nr_correspondences_before = correspondences_before.size ();
  const auto nr_correspondences_after = correspondences_after.size ();

  if (nr_correspondences_before == 0)
    return;
  if (nr_correspondences_after == 0)
  {
    indices.resize(nr_correspondences_before);
    for (std::size_t i = 0; i < nr_correspondences_before; ++i)
      indices[i] = correspondences_before[i].index_query;
    return;
  }

  Indices indices_before (nr_correspondences_before);
  for (std::size_t i = 0; i < nr_correspondences_before; ++i)
    indices_before[i] = correspondences_before[i].index_query;

  Indices indices_after (nr_correspondences_after);
  for (std::size_t i = 0; i < nr_correspondences_after; ++i)
    indices_after[i] = correspondences_after[i].index_query;

  if (presorting_required)
  {
    std::sort (indices_before.begin (), indices_before.end ());
    std::sort (indices_after.begin (), indices_after.end ());
  }

  set_difference (
      indices_before.begin (), indices_before.end (),
      indices_after.begin (),  indices_after.end (),
      inserter (indices, indices.begin ()));
}

namespace pcl
{
  std::ostream&
  operator << (std::ostream& os, const Correspondence& c)
  {
    os << c.index_query << " " << c.index_match << " " << c.distance;
    return (os);
  }
}

