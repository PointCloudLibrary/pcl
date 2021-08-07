#pragma once

#include <pcl/visualization/histogram_visualizer.h>

//#include <pcl/visualization/impl/simple_buffer_visualizer.hpp>


namespace pcl
{
  namespace visualization
  {
    /** \brief PCL simple buffer visualizer main class. 
      * \note The idea is to offer a simple visualizer that stores and display the last X values as a curve.
      * \note The class is based on PCLHistogramVisualizer and pcl::VFHSignature308 for display.
      * \note Therefore, the number of values is limited to [2-308].
      * \author Raphael Favier
      * \ingroup visualization
      */
    class PCL_EXPORTS PCLSimpleBufferVisualizer
    {
      public:
        /** \brief PCL simple buffer visualizer visualizer default constructor. */
        PCLSimpleBufferVisualizer ()
        {
          histo_ = new PCLHistogramVisualizer ();
          nb_values_ = 308;
          
          // init values buffer
          initValuesAndVisualization();
        }
        
        /** \brief PCL simple buffer visualizer visualizer constructor. 
          * \param[in] nb_values the number of values stored in the buffer [2 - 308]
          */
        PCLSimpleBufferVisualizer (const int nb_values)
        {
          histo_ = new PCLHistogramVisualizer ();
          nb_values_ = nb_values;

          if(nb_values_ > 308)
          {
            PCL_WARN("Maximum number of values can only be 308 (%d given). Setting back to 308. \n");
            nb_values_ = 308;
          }

          if(nb_values_ <= 1)
          {
            PCL_WARN("Number of values must be at least 2 (%d given). Setting it to default (308). \n");
            nb_values_ = 308;     
          }

          // init values buffer
          initValuesAndVisualization();  
        }

        /** \brief force display of values. 
          * \param[in] time - How long (in ms) should the visualization loop be allowed to run
          */
        void
        displayValues (const int time = 1)
        {
          // load values into cloud
          updateValuesToDisplay();
                
          // check if we need to automatically handle the background color
          if(control_background_color_)
          {
            if(values_.back() < lowest_threshold_)
            {
              histo_->setBackgroundColor(255.0, 140.0, 0.0);
            }
            else
            {
              histo_->setBackgroundColor(255.0, 255.0, 255.0);            
            }
          }
                      
          // add cloud to the visualizer
          histo_->updateFeatureHistogram(cloud_, nb_values_);

          // check if we need to handle the Y scale ourselves
          if (handle_y_scale_)
          {
            histo_->setGlobalYRange(min_, max_);        
          }

          // spin once
          spinOnce(time);  
        }
  
        /** \brief add a new value at the end of the buffer. 
          * \param[in] val the float value to add.
          */
        void
        addValue (const float val)
        {
          // remove front value
          values_.pop_front();

          // push new value in the back
          values_.push_back(val);

          // udapte min_ and max_ values
          if (val > max_)
            max_ = val;
            
          if (val < min_)
            min_ = val;  
        }
    
        /** \brief spinOnce method. 
          * \param[in] time - How long (in ms) should the visualization loop be allowed to run
          */
        void
        spinOnce (const int time = 1)
        {
          histo_->spinOnce(time);  
        }

        /** \brief spin method. */
        void
        spin ()
        {
          histo_->spin();  
        }
    
        /** \brief set background color handling mode.
          * \note The point here is to change the background to orange when the latest value is under a threshold.
          * \param[in] value if true, automatic mode is enabled. Else, background will be white
          * \param[in] threshold value that triggers the background to turn orange if the latest value is lower
          * \note This functionality does not work yet at time of commit (see http://dev.pointclouds.org/issues/829)
          */
        void
        setAutomaticBackgroundColorControl (const bool value = true, const float threshold = 0.0f)
        {
          control_background_color_ = value;

          // if the user sets it back to false, we make sure to reset the bckgrd color to white
          if(value == false)
            histo_->setBackgroundColor(255.0, 255.0, 255.0);

          lowest_threshold_ = threshold; 
        }

        /** \brief set Y scale policy.
          * \note If set to true, the minimal and maximal Y values are kept forever.
          * \note If set to false, the Y scale is automatically adjusted to the current values (default).
          * \param[in] value boolean that enable or disable this policy
          */
        void
        setManuallyManageYScale (const bool value = false)
        {
          handle_y_scale_ = value;  
        }
    
      private:
        /** \brief initialize the buffer that stores the values to zero. 
          * \note The size is set by private member nb_values_ which is in the range [2-308].
          */
        void
        initValuesAndVisualization ()
        {
          cloud_.resize(1);
          
          PCL_WARN("Setting buffer size to %d entries.\n", nb_values_);
          values_.resize(nb_values_);

          // add the cloud to the histogram viewer
          histo_->addFeatureHistogram(cloud_, nb_values_); 

          // init GUI-related variables
          initGUIValues();  
        }
    
        /** \brief pushes the values contained inside the buffer to the cloud used for visualization. */
        void 
        updateValuesToDisplay ()
        {
          for(int i = 0 ; i < nb_values_ ; ++i)
          {
            cloud_[0].histogram[i] = values_[i];
          }  
        }
    
        /** \brief initialize private variables linked to the GUI */
        void
        initGUIValues ()
        {
          control_background_color_ = false;
          lowest_threshold_ = 0.0f;  

          handle_y_scale_ = false;      

          min_ =  -1.0f; // std::numeric_limits<float>::max( );
          max_ =  1.0f; // std::numeric_limits<float>::min( );  
        }
    
        /** \brief visualizer object */
        PCLHistogramVisualizer *histo_;
    
        /** \brief cloud used for visualization */
        PointCloud<VFHSignature308> cloud_;
    
        /** \brief buffer of values */
        std::deque<float> values_;
     
        /** \brief number of values stored in the buffer 
          * \note ([2-308])
          */
        int nb_values_;
    
        /** \brief boolean used to know if we need to change the background color in case of low values. */
        bool control_background_color_;
    
        /** \brief threshold to turn the background orange if latest value is lower. */
        float lowest_threshold_;

        /** \brief boolean used to know if we need to change the background color in case of low values. True means we do it ourselves. */
        bool handle_y_scale_;
    
        /** \brief float tracking the minimal and maximal values ever observed. */
        float min_, max_;
    };    
  }  
}
