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
#ifndef PCL_MODELER_POLYMESH_ITEM_H_
#define PCL_MODELER_POLYMESH_ITEM_H_

#include <pcl/apps/modeler/tree_item.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>


namespace pcl
{
  namespace modeler
  {
    class MainWindow;
    class RenderWidget;

    class PolymeshItem : public TreeItem, private pcl::PolygonMesh
    {
      public:
        typedef sensor_msgs::PointCloud2  PointCloud2;
        typedef PointCloud2::Ptr          PointCloud2Ptr;
        typedef PointCloud2::ConstPtr     PointCloud2ConstPtr;

        typedef boost::shared_ptr<PolymeshItem> Ptr;
        typedef boost::shared_ptr<const PolymeshItem> ConstPtr;

        PolymeshItem (MainWindow* main_window, const std::string& id);
        ~PolymeshItem ();

        std::vector<std::string>
        getAvaiableFieldNames() const;

        PointCloud2Ptr
        getCloud()
        {return boost::shared_ptr<PointCloud2>(&cloud, NullDeleter());}
        PointCloud2ConstPtr
        getCloud() const
        {return boost::shared_ptr<const PointCloud2>(&cloud, NullDeleter());}

        std::vector<pcl::Vertices>&
        getPolygons() {return polygons;}
        const std::vector<pcl::Vertices>&
        getPolygons() const {return polygons;}

        void
        updateGeometryItems();

        void
        attachNormalItem();

        void
        setNormalField(pcl::PointCloud<pcl::Normal>::Ptr normals);

        static void
        save(const std::vector<PolymeshItem*>& polymesh_items, const std::string& filename);

        virtual void
        updateOnInserted();

        virtual void
        updateOnAboutToBeRemoved();

      protected:
        virtual void
        prepareContextMenu(QMenu* menu) const;

      private:
        void
        open();

        virtual RenderWidget*
        getParent();

        static bool
        concatenatePointCloud (const PointCloud2& cloud, PointCloud2& cloud_out);

      private:
        std::string     filename_;

        struct NullDeleter
        {
          void operator()(void const *) const
          {}
        };
    };
  }
}

#endif // PCL_MODELER_POLYMESH_ITEM_H_
