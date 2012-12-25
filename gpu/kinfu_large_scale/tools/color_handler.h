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

#ifndef COLOR_HANDLER_H_
#define COLOR_HANDLER_H_

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

      typedef typename PointCloudColorHandler<PointT>::PointCloud::ConstPtr PointCloudConstPtr;                            
      typedef typename pcl::PointCloud<RGB>::ConstPtr RgbCloudConstPtr;

    public:
      typedef boost::shared_ptr<PointCloudColorHandlerRGBHack<PointT> > Ptr;
      typedef boost::shared_ptr<const PointCloudColorHandlerRGBHack<PointT> > ConstPtr;
      
      PointCloudColorHandlerRGBHack (const PointCloudConstPtr& cloud, const RgbCloudConstPtr& colors) : 
          PointCloudColorHandler<PointT> (cloud), rgb_ (colors)
      {
        capable_ = true;
      }
            
      virtual bool 
      getColor (vtkSmartPointer<vtkDataArray> &scalars) const
      {
        if (!capable_)
          return (false);
      
        if (!scalars)
          scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        scalars->SetNumberOfComponents (3);
        
        vtkIdType nr_points = (int)cloud_->points.size ();
        reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
        unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->GetPointer (0);
        
        // Color every point
        if (nr_points != (int)rgb_->points.size ())
          std::fill(colors, colors + nr_points * 3, (unsigned char)0xFF);
        else
          for (vtkIdType cp = 0; cp < nr_points; ++cp)
          {
            int idx = cp * 3;
            colors[idx + 0] = rgb_->points[cp].r;
            colors[idx + 1] = rgb_->points[cp].g;
            colors[idx + 2] = rgb_->points[cp].b;
          }
        return (true);
      }
    
    private:
      virtual std::string getFieldName () const { return ("rgb"); }    
      virtual inline std::string getName () const { return ("PointCloudColorHandlerRGBHack"); }
      RgbCloudConstPtr rgb_;    
    };
  }
}


#endif /* COLOR_HANDLER_H_ */
