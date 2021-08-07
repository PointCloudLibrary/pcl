/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#ifndef _PCL_GPU_OCTREE_DEVICE_FORMAT_HPP_
#define _PCL_GPU_OCTREE_DEVICE_FORMAT_HPP_

#include <pcl/gpu/containers/device_array.h>

namespace pcl
{
    namespace gpu
    {
        struct NeighborIndices
        {
            DeviceArray<int> data;
            DeviceArray<int> sizes;
            int max_elems;  

            NeighborIndices() {}
            NeighborIndices(int query_number, int max_elements) : max_elems(0)
            {
                create(query_number, max_elements);
            }

            void create(int query_number, int max_elements)
            {
                max_elems = max_elements;
                data.create (query_number * max_elems);

                if (max_elems != 1)
                    sizes.create(query_number);                
            }

            void upload(const std::vector<int>& data, const std::vector<int>& sizes, int max_elements)
            {
                this->data.upload(data);
                this->sizes.upload(sizes);
                max_elems = max_elements;
            }

            bool validate(std::size_t cloud_size) const
            {
                return (sizes.size() == cloud_size) && (cloud_size * max_elems == data.size());
            }

            operator PtrStep<int>() const
            {
                return {(int*)data.ptr(), max_elems * sizeof(int)};
            }            

            std::size_t neighboors_size() const { return data.size()/max_elems; }
        };
    }
}

#endif /* _PCL_GPU_OCTREE_DEVICE_FORMAT_HPP_ */
