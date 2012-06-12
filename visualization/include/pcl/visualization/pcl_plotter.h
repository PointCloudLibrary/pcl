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

//VTK includes
#include<pcl/visualization/vtk.h>


namespace pcl
{
  namespace visualization
  {
    /** \brief PCL Visualizer main class.
      * \author Kripasindhu Sarkar
      * \ingroup visualization
      */
    class PCLPlotter: public vtkChartXY 
    {
      public:
	
	/** \brief PCL Visualizer constructor.  */
	PCLPlotter();
	
	/** \brief adds a plot with correspondences in the float arrays arrayX and arrayY
          * \param[in] array_X X coordinates of point correspondence array
          * \param[in] array_Y Y coordinates of point correspondence array
	  * \param[in] size length of the array arrayX and arrayY
	  * \param[in] color a character array of 4 fields denoting the R,G,B and A component of the color of the plot ranging from 0 to 255
          */
	void 
	addPlotData(float const *array_X, float const *array_Y, int size, char const *color=NULL);
	
	/** \brief adds a plot with correspondences in vectors arrayX and arrayY
          * \param[in] array_X X coordinates of point correspondence vector
          * \param[in] array_Y Y coordinates of point correspondence vector
	  * \param[in] color a character vector of 4 fields denoting the R,G,B and A component of the color of the plot ranging from 0 to 255
          */
	void 
	addPlotData(std::vector<float> const &array_X, std::vector<float>const &array_Y,std::vector<char> const &color= std::vector<char>());
	
	/** \brief draws all the plots added by addPlotData() till now */
	void 
	plot();
	
      private:
	  vtkSmartPointer<vtkContextView> view_;  
	  //vtkSmartPointer<vtkChartXY> chart_;

    };
  }
}
#endif	/* PCL_VISUALUALIZATION_PCL_PLOTTER_H_ */

