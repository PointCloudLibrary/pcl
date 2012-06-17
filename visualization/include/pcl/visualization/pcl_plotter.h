 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
  */
#ifndef PCL_VISUALUALIZATION_PCL_PLOTTER_H_
#define	PCL_VISUALUALIZATION_PCL_PLOTTER_H_

#include<iostream>
#include<vector>
#include<utility>

//VTK includes
#include<pcl/visualization/vtk.h>


namespace pcl
{
  namespace visualization
  {
    /** \brief PCL Plotter main class.
      * \author Kripasindhu Sarkar
      * \ingroup visualization
      */
    class PCLPlotter: public vtkChartXY 
    {
      
      public:
	
	/** \brief PCL Plotter constructor.  
         *  \param[in] name Name of the window
         */
	PCLPlotter (char const * name = "PCL Plotter");
	
	/** \brief adds a plot with correspondences in the arrays arrayX and arrayY
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
                    char * name = "Y Axis", 
                    int type  = LINE , 
                    char const *color=NULL);
	
	/** \brief adds a plot with correspondences in vectors arrayX and arrayY. This is the vector version of the addPlotData function. Parameters mean same as before
          */
	void 
	addPlotData (std::vector<double> const &array_X, 
                    std::vector<double>const &array_Y, 
                    char * name = "Y Axis", 
                    int type = LINE,
                    std::vector<char> const &color = std::vector<char>());
	
        /** \brief adds a plot with correspondences in vector of pairs. The the first and second field of the pairs of the vector forms the correspondence. Rest parameters mean same as before
          */
        void
        addPlotData (std::vector<std::pair<double, double> > const &plot_data, 
                    char * name = "Y Axis",
                    int type = LINE,
                    std::vector<char> const &color = std::vector<char>());
        
        /** \brief bins the elements in vector data into nbins equally spaced containers and plots the resulted histogram 
          * \param[in] data the raw data 
          * \param[in] nbins the number of bins for the histogram
	  * \param[in] name name of this histogram which will appear on legends if toggled on
	  * \param[in] color a character array of 4 fields denoting the R,G,B and A component of the color of the plot ranging from 0 to 255. If this argument is not passed (or an empty vector is passed) the histogram is colored based on the current color scheme 
          */
        void
        addHistogramData (std::vector<double> const & data, 
                          int const nbins = 10, 
                          char * name = "Histogram", 
                          std::vector<char> const &color = std::vector<char>());
	
        /** \brief draws all the plots added by addPlotData() or addHistogramData() till now 
        */
	void 
	plot ();
	
        
        /** \brief set method for the color scheme of the plot. The plots gets autocolored differently based on the color scheme.
          * \param[in] scheme the color scheme. Possible values are vtkColorSeries::WARM, vtkColorSeries::COOL, vtkColorSeries::BLUES, vtkColorSeries::WILD_FLOWER, vtkColorSeries::CITRUS
          */       
        void
        setColorScheme(int scheme);
        
        /** \brief get the currently used color scheme
          * \return[out] the currently used color scheme. Values include WARM, COOL, BLUES, WILD_FLOWER, CITRUS, CUSTOM
          */  
        int getColorScheme();
        
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
        
        
        /** \brief set/get method for the window size.
          * \param[in] w the width of the window
          * \param[in] h the height of the window
          */
        void
        setWindowSize (int w, int h);
        
        /** \brief set/get method for the window size.
          * \return[in] array containing the width and height of the window
          */
        int *
        getWindowSize ();
        
        
      private:
	vtkSmartPointer<vtkContextView> view_;  
	//vtkSmartPointer<vtkChartXY> chart_;
        vtkSmartPointer<vtkColorSeries> color_series_;   //for automatic coloring
        
        //extra state variables
        int current_plot_;          //stores the id of the current (most recent) plot, used in automatic coloring and other state change schemes 
        int win_width_, win_height_;
        double bkg_color_[3];
        
        
        /** \brief bins the elements in vector data into nbins equally spaced containers and returns the histogram form, ie, computes the histogram for 'data'
          * \param[in] data data who's frequency distribution is to be found
          * \param[in] nbins number of bins for the histogram
          * \param[out] histogram vector of pairs containing the histogram. The first field of the pair represent the middle value of the corresponding bin. The second field denotes the frequency of data in that bin.
          */
        void 
        computeHistogram (std::vector<double> const & data, int const nbins, std::vector<std::pair<double, double> > &histogram);

    };
  }
}
#endif	/* PCL_VISUALUALIZATION_PCL_PLOTTER_H_ */

