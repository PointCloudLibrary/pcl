#ifndef PCL_CORRESPONDENCE_VISUALIZER_H_
#define PCL_CORRESPONDENCE_VISUALIZER_H_

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_types.h>

#include <string>
#include <vector>

namespace pcl
{
  namespace visualization
  {

    template <typename PointT>
    class CorrespondenceVisualizer
    {
    public:
      CorrespondenceVisualizer (const std::string & name = "");
      
      /** \brief Adds/updates the given point clouds to the screen and draws lines between corresponding points
       * Only one set of points/correspondences will be shown at a time.  Each call to \a showCorrespondences will 
       * replace any existing points. The color of the lines can be specified by calling \a setLineColor().
       * \param source_points The source points
       * \param target_points The target points
       * \param correspondences The mapping from source points to target points. The size of \a correspondences must 
       * match the size of \a source_points, and each element must be an index into \a target_points 
       */
      void
      showCorrespondences (const typename pcl::PointCloud<PointT>::Ptr & source_points, 
                           const typename pcl::PointCloud<PointT>::Ptr & target_points,
                           const std::vector<int> & correspondences);

      /** \brief Adds/updates the given point clouds to the screen and draws lines between corresponding points
       * Only one set of points/correspondences will be shown at a time.  Each call to \a showCorrespondences will 
       * replace any existing points. The color of the lines can be specified by calling \a setLineColor().
       * \param source_points The source points
       * \param target_points The target points
       * \param correspondences The mapping from source points to target points.
       */
      void
      showCorrespondences (const typename pcl::PointCloud<PointT>::Ptr & source_points, 
                           const typename pcl::PointCloud<PointT>::Ptr & target_points,
                           const std::vector<pcl::registration::Correspondence> & correspondences);
      
      /** \brief Determines the color of the correspondence lines.  The new color will take effect on the next call to
       * \a showCorrespondences().
       * \param r, g, b An RGB color (values must be between 0.0 and 1.0)
       */
      void
      setLineColor (float r, float g, float b);
      
      
      /** Updates the screen in a continuous loop. */
      void
      spin ();

      
      /** Updates the screen once. */
      void
      spinOnce (int time = 1, bool force_redraw = false);
      
    private:
      /** \brief Add the specified correspondences to the display. 
       * \param vis The PCLVisualizer that will be updated
       * \param source_points The source points
       * \param target_points The target points
       * \param correspondences The mapping from source points to target points. The size of \a correspondences must
       * match the size of \a source_points, and each element must be an index into \a target_points 
       * \param line_ids A vector of strings into which the IDs of the newly drawn lines will be added
       */
      void
      addCorrespondences (pcl::visualization::PCLVisualizer & viz, 
                          const typename pcl::PointCloud<PointT>::Ptr & source_points,
                          const typename pcl::PointCloud<PointT>::Ptr & target_points,
                          const std::vector<int> & correspondences,
                          std::vector<std::string> & line_ids);

      /** \brief Add the specified correspondences to the display. 
       * \param vis The PCLVisualizer that will be updated
       * \param source_points The source points
       * \param target_points The target points
       * \param correspondences The mapping from source points to target points.
       * \param line_ids A vector of strings into which the IDs of the newly drawn lines will be added
       */
      void
      addCorrespondences (pcl::visualization::PCLVisualizer & viz, 
                          const typename pcl::PointCloud<PointT>::Ptr & source_points,
                          const typename pcl::PointCloud<PointT>::Ptr & target_points,
                          const std::vector<pcl::registration::Correspondence> & correspondences,
                          std::vector<std::string> & line_ids);
      
      /** \brief Remove the specified correspondences from the display. 
       * \param vis The PCLVisualizer that will be updated
       * \param line_ids The ID strings of the lines that must be removed from the display
       */
      void
      removeCorrespondences (pcl::visualization::PCLVisualizer & viz, std::vector<std::string> & line_ids);
      
    private:
      /** \brief The visualizer window */
      pcl::visualization::PCLVisualizer viz_;
      
      /** \brief A boolean value storing whether the visualizer currently contains any points/correspondences.*/
      bool initialized_;
      
      /** \brief  The RGB color used when drawing the correspondence lines.*/
      float r_, g_, b_; 
      
      /** \brief A vector to store the ID strings of each of the correspondence lines.*/
      std::vector<std::string> line_ids_;
    };
  
  }
}

#include <pcl/visualization/impl/correspondence_visualizer.hpp>

#endif // #ifndef PCL_CORRESPONDENCE_VISUALIZER_H_
