/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
#ifndef PCL_VISUALUALIZATION_PCL_PLOTTER_H_
#define	PCL_VISUALUALIZATION_PCL_PLOTTER_H_

#include <iostream>
#include <vector>
#include <utility>
#include <cfloat>

#include <pcl/visualization/common/common.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>

class PCLVisualizerInteractor;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkContextView;
class vtkChartXY;
class vtkColorSeries;

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkChart.h>

namespace pcl
{
  namespace visualization
  {
    /** \brief PCL Plotter main class. Given point correspondences this class
      * can be used to plot the data one against the other and display it on the
      * screen. It also has methods for providing plot for important functions
      * like histogram etc. Important functions of PCLHistogramVisualizer are
      * redefined here so that this single class can take responsibility of all
      * plotting related functionalities.
      *
      * \author Kripasindhu Sarkar
      * \ingroup visualization
      */
    class PCL_EXPORTS PCLPlotter
    {
      public:
	
        /**\brief A representation of polynomial function. i'th element of the vector denotes the coefficient of x^i of the polynomial in variable x. 
         */
        typedef std::vector<double> PolynomialFunction;
        
        /**\brief A representation of rational function, defined as the ratio of two polynomial functions. pair::first denotes the numerator and pair::second denotes the denominator of the Rational function. 
         */
        typedef std::pair<PolynomialFunction, PolynomialFunction> RationalFunction;
        
        /** \brief PCL Plotter constructor.  
          * \param[in] name Name of the window
          */
        PCLPlotter (char const * name = "PCL Plotter");

        /** \brief Destructor. */
        ~PCLPlotter();

        /** \brief Adds a plot with correspondences in the arrays arrayX and arrayY
          * \param[in] array_X X coordinates of point correspondence array
          * \param[in] array_Y Y coordinates of point correspondence array
          * \param[in] size length of the array arrayX and arrayY
          * \param[in] name name of the plot which appears in the legend when toggled on
          * \param[in] type type of the graph plotted. vtkChart::LINE for line plot, vtkChart::BAR for bar plot, and vtkChart::POINTS for a scattered point plot
          * \param[in] color a character array of 4 fields denoting the R,G,B and A component of the color of the plot ranging from 0 to 255. If this argument is not passed (or NULL is passed) the plot is colored based on a color scheme 
          */
        void 
        addPlotData (double const *array_X, 
                     double const *array_Y, 
                     unsigned long size, 
                     char const * name = "Y Axis", 
                     int type  = vtkChart::LINE ,
                     char const *color=NULL);
	
        /** \brief Adds a plot with correspondences in vectors arrayX and arrayY. This is the vector version of the addPlotData function. 
          * \param[in] array_x X coordinates of point correspondence array
          * \param[in] array_y Y coordinates of point correspondence array
          * \param[in] name name of the plot which appears in the legend when toggled on
          * \param[in] type type of the graph plotted. vtkChart::LINE for line plot, vtkChart::BAR for bar plot, and vtkChart::POINTS for a scattered point plot
          * \param[in] color a character array of 4 fields denoting the R,G,B and A component of the color of the plot ranging from 0 to 255. If this argument is not passed (or NULL is passed) the plot is colored based on a color scheme 
         */
        void 
        addPlotData (std::vector<double> const &array_x, 
                     std::vector<double>const &array_y, 
                     char const * name = "Y Axis", 
                     int type = vtkChart::LINE,
                     std::vector<char> const &color = std::vector<char> ());
        
        /** \brief Adds a plot with correspondences in vector of pairs. The the first and second field of the pairs of the vector forms the correspondence.
          * \param plot_data
          * \param[in] name name of the plot which appears in the legend when toggled on
          * \param[in] type type of the graph plotted. vtkChart::LINE for line plot, vtkChart::BAR for bar plot, and vtkChart::POINTS for a scattered point plot
          * \param[in] color a character array of 4 fields denoting the R,G,B and A component of the color of the plot ranging from 0 to 255. If this argument is not passed (or NULL is passed) the plot is colored based on a color scheme 
          */
        void
        addPlotData (std::vector<std::pair<double, double> > const &plot_data, 
                    char const * name = "Y Axis",
                    int type = vtkChart::LINE,
                    std::vector<char> const &color = std::vector<char>());
        
        /** \brief Adds a plot based on the given polynomial function and the range in X axis. 
          * \param[in] p_function A polynomial function which is represented by a vector which stores the coefficients. See description on the  typedef.   
          * \param[in] x_min the left boundary of the range for displaying the plot
          * \param[in] x_max the right boundary of the range for displaying the plot
          * \param[in] name name of the plot which appears in the legend when toggled on
          * \param[in] num_points Number of points plotted to show the graph. More this number, more is the resolution.
          * \param[in] type type of the graph plotted. vtkChart::LINE for line plot, vtkChart::BAR for bar plot, and vtkChart::POINTS for a scattered point plot
          * \param[in] color a character array of 4 fields denoting the R,G,B and A component of the color of the plot ranging from 0 to 255. If this argument is not passed (or NULL is passed) the plot is colored based on a color scheme 
          */
        void
        addPlotData (PolynomialFunction const & p_function,
                     double x_min, double x_max,
                     char const *name = "Y Axis",
                     int num_points = 100,
                     int type = vtkChart::LINE,
                     std::vector<char> const &color = std::vector<char>());
        
        /** \brief Adds a plot based on the given rational function and the range in X axis. 
          * \param[in] r_function A rational function which is represented by the ratio of two polynomial functions. See description on the  typedef for more details.
          * \param[in] x_min the left boundary of the range for displaying the plot
          * \param[in] x_max the right boundary of the range for displaying the plot
          * \param[in] name name of the plot which appears in the legend when toggled on
          * \param[in] num_points Number of points plotted to show the graph. More this number, more is the resolution.
          * \param[in] type type of the graph plotted. vtkChart::LINE for line plot, vtkChart::BAR for bar plot, and vtkChart::POINTS for a scattered point plot
          * \param[in] color a character array of 4 fields denoting the R,G,B and A component of the color of the plot ranging from 0 to 255. If this argument is not passed (or NULL is passed) the plot is colored based on a color scheme 
          */
        void
        addPlotData (RationalFunction const & r_function,
                     double x_min, double x_max,
                     char const *name = "Y Axis",
                     int num_points = 100,
                     int type = vtkChart::LINE,
                     std::vector<char> const &color = std::vector<char>());
        
        /** \brief Adds a plot based on a user defined callback function representing the function to plot
          * \param[in] function a user defined callback function representing the relation y = function(x)
          * \param[in] x_min the left boundary of the range for displaying the plot
          * \param[in] x_max the right boundary of the range for displaying the plot
          * \param[in] name name of the plot which appears in the legend when toggled on
          * \param[in] num_points Number of points plotted to show the graph. More this number, more is the resolution.
          * \param[in] type type of the graph plotted. vtkChart::LINE for line plot, vtkChart::BAR for bar plot, and vtkChart::POINTS for a scattered point plot
          * \param[in] color a character array of 4 fields denoting the R,G,B and A component of the color of the plot ranging from 0 to 255. If this argument is not passed (or NULL is passed) the plot is colored based on a color scheme 
          */
        void
        addPlotData (double (*function)(double),
                     double x_min, double x_max,
                     char const *name = "Y Axis",
                     int num_points = 100,
                     int type = vtkChart::LINE,
                     std::vector<char> const &color = std::vector<char>());
        
        /** \brief Adds a plot based on a space/tab delimited table provided in a file
          * \param[in] filename name of the file containing the table. 1st column represents the values of X-Axis. Rest of the columns represent the corresponding values in Y-Axes. First row of the file is concidered for naming/labling of the plot. The plot-names should not contain any space in between.
          * \param[in] type type of the graph plotted. vtkChart::LINE for line plot, vtkChart::BAR for bar plot, and vtkChart::POINTS for a scattered point plot
          */
        void
        addPlotData (char const * filename,
                     int type = vtkChart::LINE);
                    
        /** \brief Bins the elements in vector data into nbins equally spaced containers and plots the resulted histogram 
          * \param[in] data the raw data 
          * \param[in] nbins the number of bins for the histogram
          * \param[in] name name of this histogram which will appear on legends if toggled on
          * \param[in] color a character array of 4 fields denoting the R,G,B and A component of the color of the plot ranging from 0 to 255. If this argument is not passed (or an empty vector is passed) the histogram is colored based on the current color scheme 
          */
        void
        addHistogramData (std::vector<double> const & data, 
                          int const nbins = 10, 
                          char const * name = "Histogram", 
                          std::vector<char> const &color = std::vector<char>());
        
        //##PCLHistogramVisulizer methods##
        /** \brief Add a histogram feature to screen as a separate window, from a cloud containing a single histogram.
          * \param[in] cloud the PointCloud dataset containing the histogram
          * \param[in] hsize the length of the histogram
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] win_width the width of the window
          * \param[in] win_height the height of the window
          */
        template <typename PointT> bool 
        addFeatureHistogram (const pcl::PointCloud<PointT> &cloud, 
                             int hsize, 
                             const std::string &id = "cloud", int win_width = 640, int win_height = 200);
        
        /** \brief Add a histogram feature to screen as a separate window from a cloud containing a single histogram.
          * \param[in] cloud the PointCloud dataset containing the histogram
          * \param[in] field_name the field name containing the histogram
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] win_width the width of the window
          * \param[in] win_height the height of the window
          */
        bool 
        addFeatureHistogram (const pcl::PCLPointCloud2 &cloud,
                             const std::string &field_name, 
                             const std::string &id = "cloud", int win_width = 640, int win_height = 200);
        
        /** \brief Add a histogram feature to screen as a separate window.
          * \param[in] cloud the PointCloud dataset containing the histogram
          * \param[in] field_name the field name containing the histogram
          * \param[in] index the point index to extract the histogram from
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] win_width the width of the window
          * \param[in] win_height the height of the window 
          */
        template <typename PointT> bool 
        addFeatureHistogram (const pcl::PointCloud<PointT> &cloud, 
                             const std::string &field_name, 
                             const int index,
                             const std::string &id = "cloud", int win_width = 640, int win_height = 200);
        
        /** \brief Add a histogram feature to screen as a separate window.
          * \param[in] cloud the PointCloud dataset containing the histogram
          * \param[in] field_name the field name containing the histogram
          * \param[in] index the point index to extract the histogram from
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] win_width the width of the window
          * \param[in] win_height the height of the window
          */
        bool 
        addFeatureHistogram (const pcl::PCLPointCloud2 &cloud,
                             const std::string &field_name, 
                             const int index,
                             const std::string &id = "cloud", int win_width = 640, int win_height = 200);
        
        /** \brief Draws all the plots added by addPlotData() or addHistogramData() till now */
        void 
        plot ();
        
        /** \brief Spins (runs the event loop) the interactor for spin_time amount of time. The name is confusing and will be probably obsolete in the future release with a single overloaded spin()/display() function.
          *  \param[in] spin_time - How long (in ms) should the visualization loop be allowed to run.
          */
        void 
        spinOnce (const int spin_time = 1);
        
        /** \brief Spins (runs the event loop) the interactor indefinitely. Same as plot() - added to retain the similarity between other existing visualization classes. */
        void 
        spin ();
        
        /** \brief Remove all plots from the window. */
        void
        clearPlots();
        
        /** \brief Set method for the color scheme of the plot. The plots gets autocolored differently based on the color scheme.
          * \param[in] scheme the color scheme. Possible values are vtkColorSeries::SPECTRUM, vtkColorSeries::WARM, vtkColorSeries::COOL, vtkColorSeries::BLUES, vtkColorSeries::WILD_FLOWER, vtkColorSeries::CITRUS
          */       
        void
        setColorScheme (int scheme);
        
        /** \brief get the currently used color scheme
          * \return[out] the currently used color scheme. Values include WARM, COOL, BLUES, WILD_FLOWER, CITRUS, CUSTOM
          */  
        int 
        getColorScheme ();
        
        /** \brief set/get method for the viewport's background color.
          * \param[in] r the red component of the RGB color
          * \param[in] g the green component of the RGB color
          * \param[in] b the blue component of the RGB color
          */
        void 
        setBackgroundColor (const double r, const double g, const double b);
        
        /** \brief set/get method for the viewport's background color.
         * \param [in] color the array containing the 3 component of the RGB color
         */
        void
        setBackgroundColor (const double color[3]);
        
        /** \brief set/get method for the viewport's background color.
         * \return [out] color the array containing the 3 component of the RGB color
         */
        double *
        getBackgroundColor ();
        
        /** \brief Set logical range of the X-Axis in plot coordinates 
          * \param[in] min the left boundary of the range
          * \param[in] max the right boundary of the range
          */
        void 
        setXRange (double min, double max);
        
        /** \brief Set logical range of the Y-Axis in plot coordinates 
          * \param[in] min the left boundary of the range
          * \param[in] max the right boundary of the range
          */
        void
        setYRange (double min, double max);
        
        /** \brief Set the main title of the plot
          * \param[in] title the title to set 
          */
        void 
        setTitle (const char *title);
        
        /** \brief Set the title of the X-Axis
          * \param[in] title the title to set 
          */
        void 
        setXTitle (const char *title);
        
        /** \brief Set the title of the Y-Axis
          * \param[in] title the title to set 
          */
        void 
        setYTitle (const char *title);
        
        /** \brief Shows the legend of the graph
          * \param[in] flag pass flag = true for the display of the legend of the graph
          */
        void 
        setShowLegend (bool flag);
        
        /** \brief set/get method for the window size.
          * \param[in] w the width of the window
          * \param[in] h the height of the window
          */
        void
        setWindowSize (int w, int h);
        
        /** \brief Set the position in screen coordinates.
        * \param[in] x where to move the window to (X)
        * \param[in] y where to move the window to (Y)
        */
        void
        setWindowPosition (int x, int y);
        
        /** \brief Set the visualizer window name.
        * \param[in] name the name of the window
        */
        void
        setWindowName (const std::string &name);
        
        /** \brief set/get method for the window size.
          * \return[in] array containing the width and height of the window
          */
        int*
        getWindowSize ();

        /** \brief Return a pointer to the underlying VTK RenderWindow used. */
        vtkSmartPointer<vtkRenderWindow>
        getRenderWindow ();
        
        /** \brief Set the view's interactor. */
        void
        setViewInteractor (vtkSmartPointer<vtkRenderWindowInteractor> interactor);
        
        /** \brief Initialize and Start the view's interactor. */
        void
        startInteractor ();
        
        /** \brief Render the vtkWindow once. */
        void renderOnce();

        /** \brief Returns true when the user tried to close the window */
        bool
        wasStopped () const;
        
        /** \brief Stop the interaction and close the visualizaton window. */
        void
        close ();
      
      private:
        vtkSmartPointer<vtkContextView> view_;  
        vtkSmartPointer<vtkChartXY> chart_;
        vtkSmartPointer<vtkColorSeries> color_series_;   //for automatic coloring
        
        //extra state variables
        int current_plot_;          //stores the id of the current (most recent) plot, used in automatic coloring and other state change schemes 
        int win_width_, win_height_;
        int win_x_, win_y_; //window position according to screen coordinate
        double bkg_color_[3];
        std::string win_name_;
          
        //####event callback class####
        struct ExitMainLoopTimerCallback : public vtkCommand
        {
          static ExitMainLoopTimerCallback* New ()
          {
            return (new ExitMainLoopTimerCallback);
          }
          virtual void 
          Execute (vtkObject*, unsigned long event_id, void* call_data);

          int right_timer_id;
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
          PCLVisualizerInteractor *interactor;
#else
          vtkRenderWindowInteractor *interactor;
#endif
        };
        
        struct ExitCallback : public vtkCommand
        {
          static ExitCallback* New ()
          {
            return new ExitCallback;
          }
          virtual void 
          Execute (vtkObject*, unsigned long event_id, void*);

          PCLPlotter *plotter;
        };
        
         /** \brief Set to false if the interaction loop is running. */
        bool stopped_;
        
        /** \brief Callback object enabling us to leave the main loop, when a timer fires. */
        vtkSmartPointer<ExitMainLoopTimerCallback> exit_loop_timer_;
        vtkSmartPointer<ExitCallback> exit_callback_;
        
        ////////////////////////////////////IMPORTANT PRIVATE COMPUTING FUNCTIONS////////////////////////////////////////////////////
        /** \brief computes the value of the polynomial function at val
          * \param[in] p_function polynomial function
          * \param[in] value the value at which the function is to be computed
          */
        double 
        compute (PolynomialFunction const & p_function, double val);
        
        /** \brief computes the value of the rational function at val
          * \param[in] r_function the rational function
          * \param[in] value the value at which the function is to be computed
          */
        double 
        compute (RationalFunction const & r_function, double val);
        
        /** \brief bins the elements in vector data into nbins equally spaced containers and returns the histogram form, ie, computes the histogram for 'data'
          * \param[in] data data who's frequency distribution is to be found
          * \param[in] nbins number of bins for the histogram
          * \param[out] histogram vector of pairs containing the histogram. The first field of the pair represent the middle value of the corresponding bin. The second field denotes the frequency of data in that bin.
          * \note NaN values will be ignored!
          */
        void 
        computeHistogram (std::vector<double> const & data, int const nbins, std::vector<std::pair<double, double> > &histogram);
    };
  }
}

#include <pcl/visualization/impl/pcl_plotter.hpp>

#endif	/* PCL_VISUALUALIZATION_PCL_PLOTTER_H_ */

