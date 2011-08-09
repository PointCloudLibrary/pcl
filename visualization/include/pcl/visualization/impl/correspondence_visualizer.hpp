#include <sstream>

template <typename PointT>
pcl::visualization::CorrespondenceVisualizer<PointT>::
CorrespondenceVisualizer (const std::string & name) : 
  viz_ (name), initialized_ (false), r_ (1.0), g_ (0.0), b_ (0.0)
{}

template <typename PointT>
void pcl::visualization::CorrespondenceVisualizer<PointT>::
showCorrespondences (const typename pcl::PointCloud<PointT>::Ptr & source_points, 
                     const typename pcl::PointCloud<PointT>::Ptr & target_points,
                     const std::vector<int> & correspondences)
{
  // Make sure that the size of source_points matches the size of correspondences
  if (source_points->size () != correspondences.size ())
  {
    PCL_ERROR ("The number of source points and the number of correspondences must match!");
    return;
  }
  
  if (initialized_)
  {
    viz_.updatePointCloud (source_points, "source_points");
    viz_.updatePointCloud (target_points, "target_points");
    removeCorrespondences (viz_, line_ids_);
    addCorrespondences (viz_, source_points, target_points, correspondences, line_ids_);
  }
  else
  {
    viz_.addPointCloud (source_points, "source_points");
    viz_.addPointCloud (target_points, "target_points");
    addCorrespondences (viz_, source_points, target_points, correspondences, line_ids_);
    initialized_ = true;
  }
}

template <typename PointT>
void pcl::visualization::CorrespondenceVisualizer<PointT>::
showCorrespondences (const typename pcl::PointCloud<PointT>::Ptr & source_points, 
                     const typename pcl::PointCloud<PointT>::Ptr & target_points,
                     const std::vector<pcl::registration::Correspondence> & correspondences)
{
  if (initialized_)
  {
    viz_.updatePointCloud (source_points, "source_points");
    viz_.updatePointCloud (target_points, "target_points");
    removeCorrespondences (viz_, line_ids_);
    addCorrespondences (viz_, source_points, target_points, correspondences, line_ids_);
  }
  else
  {
    viz_.addPointCloud (source_points, "source_points");
    viz_.addPointCloud (target_points, "target_points");
    addCorrespondences (viz_, source_points, target_points, correspondences, line_ids_);
    initialized_ = true;
  }
}

template <typename PointT>
void pcl::visualization::CorrespondenceVisualizer<PointT>::
setLineColor (float r, float g, float b)
{
  if ((r < 0.0) || (r > 1.0) ||
      (g < 0.0) || (g > 1.0) ||
      (b < 0.0) || (b > 1.0))
  {
    PCL_ERROR ("Color channel values must be between 0.0 and 1.0!");
    return;
  }
  r_ = r;
  g_ = g;
  b_ = b;
}


template <typename PointT>
void pcl::visualization::CorrespondenceVisualizer<PointT>::
spin ()
{
  viz_.spin ();
}


template <typename PointT>
void pcl::visualization::CorrespondenceVisualizer<PointT>::
spinOnce (int time, bool force_redraw)
{
  viz_.spinOnce (time, force_redraw);
}


template <typename PointT>
void pcl::visualization::CorrespondenceVisualizer<PointT>::
addCorrespondences (pcl::visualization::PCLVisualizer & viz, 
                    const typename pcl::PointCloud<PointT>::Ptr & source_points,
                    const typename pcl::PointCloud<PointT>::Ptr & target_points,
                    const std::vector<int> & correspondences,
                    std::vector<std::string> & line_ids)
{
  // Draw lines between the best corresponding points
  for (size_t i = 0; i < source_points->size (); ++i)
  {
    const PointT & p_src = source_points->points[i];
    const PointT & p_tgt = target_points->points[correspondences[i]];
    
    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;
    line_ids.push_back (ss.str ());
    
    // Draw the line
    viz.addLine (p_src, p_tgt, r_, g_, b_, ss.str ());
  }
}


template <typename PointT>
void pcl::visualization::CorrespondenceVisualizer<PointT>::
addCorrespondences (pcl::visualization::PCLVisualizer & viz, 
                    const typename pcl::PointCloud<PointT>::Ptr & source_points,
                    const typename pcl::PointCloud<PointT>::Ptr & target_points,
                    const std::vector<pcl::registration::Correspondence> & correspondences,
                    std::vector<std::string> & line_ids)
{
  // Draw lines between the best corresponding points
  for (size_t i = 0; i < source_points->size (); ++i)
  {
    const PointT & p_src = source_points->points[correspondences[i].indexQuery];
    const PointT & p_tgt = target_points->points[correspondences[i].indexMatch];
    
    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;
    line_ids.push_back (ss.str ());
    
    // Draw the line
    viz.addLine (p_src, p_tgt, r_, g_, b_, ss.str ());
  }
}


template <typename PointT>
void pcl::visualization::CorrespondenceVisualizer<PointT>::
removeCorrespondences (pcl::visualization::PCLVisualizer & viz, 
                       std::vector<std::string> & line_ids)
{
  for (size_t i = 0; i < line_ids.size (); ++i)
    viz.removeShape (line_ids[i]);
  line_ids.resize (0);
}
