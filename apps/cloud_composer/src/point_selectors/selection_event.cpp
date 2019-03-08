/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#include <pcl/apps/cloud_composer/point_selectors/selection_event.h>

#include <pcl/point_types.h>

pcl::cloud_composer::SelectionEvent::~SelectionEvent ()
{
  renderer_->RemoveActor (selected_actor_);
}


void
pcl::cloud_composer::SelectionEvent::findIndicesInItem (CloudItem* cloud_item, pcl::PointIndices::Ptr indices)
{
  // WE DON'T NEED TO DO THIS SEARCH BECAUSE WE HAVE A 1-1 CORRESPONDENCE VTK TO PCL
  // THIS IS ONLY THE CASE FOR CLOUDS WITH NO NANs
 //Go through every point in the selected data set and find the matching index in this cloud
 //pcl::search::KdTree<pcl::PointXYZ>::Ptr search = cloud_item->data (KD_TREE_SEARCH).value <pcl::search::Search<pcl::PointXYZ>::Ptr> ();
 // double dblpts[3];
 // PointXYZ pt;
 // std::vector<int> indices;
 // std::vector<float> distances;
  /*for (int i = 0; i < points->GetNumberOfPoints (); ++i)
  {
    points->GetPoint(i, dblpts);
    pt.x = dblpts[0];
    pt.y = dblpts[1];
    pt.z = dblpts[2];
    search->radiusSearch (pt,0.001,indices,distances);
    if (indices.size () == 1)
    {
      id_list.append (indices[0]);
    }
    //If there are multiple found we need to only assign it to one cloud
    else if (indices.size () > 1)
    {
      int idx = 0;
      while (used_indices.contains (indices[idx]))
        ++idx;
      if (idx < indices.size ())
      {
        used_indices.append (indices[idx]);
        id_list.append (indices[idx]);        
      }
      else
        qCritical () << "More found points in selection than are possible from cloud items!";
    }
      
  }*/
  if (id_selected_data_map_.contains (cloud_item->getId ()))
  {
    vtkPolyData* points_in_item = id_selected_data_map_.value (cloud_item->getId ());
    vtkIdTypeArray* point_ids  = vtkIdTypeArray::SafeDownCast(points_in_item->GetPointData ()->GetArray ("vtkIdFilter_Ids"));
  
    indices->indices.resize (point_ids->GetNumberOfTuples ());
    for(vtkIdType i =0; i < point_ids->GetNumberOfTuples (); ++i)
    {
    //qDebug () << "id="<<point_ids->GetValue (i);
      indices->indices[i] = point_ids->GetValue (i);
    }
    //qDebug () << points_in_item->GetNumberOfPoints () << " selected points in "<<cloud_item->getId ();
  }
  
}
