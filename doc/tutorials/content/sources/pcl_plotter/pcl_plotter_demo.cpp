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


/* \author Kripasindhu Sarkar */

#include<pcl/visualization/pcl_plotter.h>

#include<iostream>
#include<vector>
#include<utility>
#include<math.h>  //for abs()

using namespace std;
using namespace pcl::visualization;

void
generateData (double *ax, double *acos, double *asin, int numPoints)
{
  double inc = 7.5 / (numPoints - 1);
  for (int i = 0; i < numPoints; ++i)
  {
    ax[i] = i*inc;
    acos[i] = cos (i * inc);
    asin[i] = sin (i * inc);
  }
}

//.....................callback functions defining Y= f(X)....................
double
step (double val)
{
  if (val > 0)
    return (double) (int) val;
  else
    return (double) ((int) val - 1);
}

double
identity (double val)
{
  return val;
}
//............................................................................

int
main (int argc, char * argv [])
{
  //defining a plotter
  PCLPlotter *plotter = new PCLPlotter ("My Plotter");

  //setting some properties
  plotter->setShowLegend (true);

  //generating point correspondances
  int numPoints = 69;
  double ax[100], acos[100], asin[100];
  generateData (ax, acos, asin, numPoints);

  //adding plot data
  plotter->addPlotData (ax, acos, numPoints, "cos");
  plotter->addPlotData (ax, asin, numPoints, "sin");

  //display for 2 seconds
  plotter->spinOnce (3000);
  plotter->clearPlots ();
  
  
  //...................plotting implicit functions and custom callbacks....................

  //make a fixed axis
  plotter->setYRange (-10, 10);

  //defining polynomials
  vector<double> func1 (1, 0);
  func1[0] = 1; //y = 1
  vector<double> func2 (3, 0);
  func2[2] = 1; //y = x^2

  plotter->addPlotData (std::make_pair (func1, func2), -10, 10, "y = 1/x^2", 100, vtkChart::POINTS);
  plotter->spinOnce (2000);

  plotter->addPlotData (func2, -10, 10, "y = x^2");
  plotter->spinOnce (2000);

  //callbacks
  plotter->addPlotData (identity, -10, 10, "identity");
  plotter->spinOnce (2000);

  plotter->addPlotData (abs, -10, 10, "abs");
  plotter->spinOnce (2000);

  plotter->addPlotData (step, -10, 10, "step", 100, vtkChart::POINTS);
  plotter->spinOnce (2000);

  plotter->clearPlots ();

  //........................A simple animation..............................
  vector<double> fsq (3, 0);
  fsq[2] = -100; //y = x^2
  while (!plotter->wasStopped ())
  {
    if (fsq[2] == 100) fsq[2] = -100;
    fsq[2]++;
    char str[50];
    sprintf (str, "y = %dx^2", (int) fsq[2]);
    plotter->addPlotData (fsq, -10, 10, str);

    plotter->spinOnce (100);
    plotter->clearPlots ();
  }

  return 1;
}

