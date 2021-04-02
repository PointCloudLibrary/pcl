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

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/point_cloud_handlers.h>

namespace pcl
{
  namespace visualization
  {
    template <typename PointT>
    class PointCloudColorHandlerRGBHack : public PointCloudColorHandler<PointT>
    {
      using PointCloudColorHandler<PointT>::capable_;
      using PointCloudColorHandler<PointT>::cloud_;

      using PointCloudConstPtr = typename PointCloudColorHandler<PointT>::PointCloud::ConstPtr;
      using RgbCloudConstPtr = pcl::PointCloud<RGB>::ConstPtr;

    public:
      using Ptr = shared_ptr<PointCloudColorHandlerRGBHack<PointT> >;
      using ConstPtr = shared_ptr<const PointCloudColorHandlerRGBHack<PointT> >;
      
      PointCloudColorHandlerRGBHack (const PointCloudConstPtr& cloud, const RgbCloudConstPtr& colors) : 
          PointCloudColorHandler<PointT> (cloud), rgb_ (colors)
      {
        capable_ = true;
      }
            
      vtkSmartPointer<vtkDataArray>
      getColor () const override
      {
        if (!capable_)
          return nullptr;
      
        auto scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        scalars->SetNumberOfComponents (3);
        
        vtkIdType nr_points = static_cast<vtkIdType>(cloud_->size ());
        reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
        unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->GetPointer (0);
        
        // Color every point
        if (nr_points != static_cast<vtkIdType>(rgb_->size ()))
          std::fill_n(colors, nr_points * 3, (unsigned char)0xFF);
        else
          for (vtkIdType cp = 0; cp < nr_points; ++cp)
          {
            int idx = cp * 3;
            colors[idx + 0] = (*rgb_)[cp].r;
            colors[idx + 1] = (*rgb_)[cp].g;
            colors[idx + 2] = (*rgb_)[cp].b;
          }
        return scalars;
      }
    
    private:
      std::string getFieldName () const override { return ("rgb"); }    
      inline std::string getName () const override { return ("PointCloudColorHandlerRGBHack"); }
      RgbCloudConstPtr rgb_;    
    };
  }
}
