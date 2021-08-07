#include <pcl/apps/cloud_composer/point_selectors/selection_event.h>

#include <pcl/point_types.h>

pcl::cloud_composer::SelectionEvent::~SelectionEvent ()
{
  renderer_->RemoveActor (selected_actor_);
}


void
pcl::cloud_composer::SelectionEvent::findIndicesInItem (CloudItem* cloud_item, const pcl::PointIndices::Ptr& indices)
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
