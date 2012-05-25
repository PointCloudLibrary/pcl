/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: edge.hpp nsomani $
 *
 */
#include "../convolution_2d.h"
#include <vector>
#ifndef PCL_2D_EDGE_IMPL_HPP
#define PCL_2D_EDGE_IMPL_HPP


void
pcl::pcl_2d::edge::sobelXY  (ImageType &Gx, ImageType &Gy, ImageType &input)
{
	ImageType kernelX;
	kernelX.resize(3);kernelX[0].resize(3);kernelX[1].resize(3);kernelX[2].resize(3);
	kernelX[0][0] = -1; kernelX[0][1] = 0; kernelX[0][2] = 1;
	kernelX[1][0] = -2; kernelX[1][1] = 0; kernelX[1][2] = 2;
	kernelX[2][0] = -1; kernelX[2][1] = 0; kernelX[2][2] = 1;
	
	conv_2d->conv(Gx, kernelX, input);

	ImageType kernelY;
	kernelY.resize(3);kernelY[0].resize(3);kernelY[1].resize(3);kernelY[2].resize(3);
	kernelY[0][0] = 1; kernelY[0][1] = 2; kernelY[0][2] = 1;
	kernelY[1][0] = 0; kernelY[1][1] = 0; kernelY[1][2] = 0;
	kernelY[2][0] = -1; kernelY[2][1] = -2; kernelY[2][2] = -1;		
	conv_2d->conv(Gy, kernelY, input);
}
void
pcl::pcl_2d::edge::sobelGThet  (ImageType &G, ImageType &thet, ImageType &input)
{
	ImageType Gx;
	ImageType Gy;
	sobelXY(Gx, Gy, input);
	G.resize(input.size());
	thet.resize(input.size());
	for(int i = 0;i < input.size();i++)
	{
		G[i].resize(input[i].size());
		thet[i].resize(input[i].size());
		for(int j = 0;j < input[i].size();j++)
		{
			G[i][j] = sqrt(Gx[i][j]*Gx[i][j]+Gy[i][j]*Gy[i][j]);
			thet[i][j] = atan2(Gy[i][j],Gx[i][j]);
		}
	}
}

void
pcl::pcl_2d::edge::prewittXY  (ImageType &Gx, ImageType &Gy, ImageType &input)
{
	ImageType kernelX;
	kernelX.resize(3);kernelX[0].resize(3);kernelX[1].resize(3);kernelX[2].resize(3);
	kernelX[0][0] = -1; kernelX[0][1] = 0; kernelX[0][2] = 1;
	kernelX[1][0] = -1; kernelX[1][1] = 0; kernelX[1][2] = 1;
	kernelX[2][0] = -1; kernelX[2][1] = 0; kernelX[2][2] = 1;	
	conv_2d->conv(Gx, kernelX, input);
	
	ImageType kernelY;
	kernelY.resize(3);kernelY[0].resize(3);kernelY[1].resize(3);kernelY[2].resize(3);
	kernelY[0][0] = 1; kernelY[0][1] = 1; kernelY[0][2] = 1;
	kernelY[1][0] = 0; kernelY[1][1] = 0; kernelY[1][2] = 0;
	kernelY[2][0] = -1; kernelY[2][1] = -1; kernelY[2][2] = -1;		
	conv_2d->conv(Gy, kernelY, input);
}
void
pcl::pcl_2d::edge::prewittGThet  (ImageType &G, ImageType &thet, ImageType &input)
{
	ImageType Gx;
	ImageType Gy;
	prewittXY(Gx, Gy, input);
	G.resize(input.size());
	thet.resize(input.size());
	for(int i = 0;i < input.size();i++)
	{
		G[i].resize(input[i].size());
		thet[i].resize(input[i].size());
		for(int j = 0;j < input[i].size();j++)
		{
			G[i][j] = sqrt(Gx[i][j]*Gx[i][j]+Gy[i][j]*Gy[i][j]);
			thet[i][j] = atan2(Gy[i][j],Gx[i][j]);
		}
	}
}

void
pcl::pcl_2d::edge::robertsXY  (ImageType &Gx, ImageType &Gy, ImageType &input)
{
	ImageType kernelX;
	kernelX.resize(2);kernelX[0].resize(2);kernelX[1].resize(2);
	kernelX[0][0] = 1; kernelX[0][1] = 0;
	kernelX[1][0] = 0; kernelX[1][1] = -1;
	conv_2d->conv(Gx, kernelX, input);
	
	ImageType kernelY;
	kernelY.resize(2);kernelY[0].resize(2);kernelY[1].resize(2);
	kernelY[0][0] = 0; kernelY[0][1] = 1;
	kernelY[1][0] = -1; kernelY[1][1] = 0;	
	conv_2d->conv(Gy, kernelY, input);
}
void
pcl::pcl_2d::edge::robertsGThet  (ImageType &G, ImageType &thet, ImageType &input){
	ImageType Gx;
	ImageType Gy;
	robertsXY(Gx, Gy, input);
	G.resize(input.size());
	thet.resize(input.size());
	for(int i = 0;i < input.size();i++)
	{
		G[i].resize(input[i].size());
		thet[i].resize(input[i].size());
		for(int j = 0;j < input[i].size();j++)
		{
			G[i][j] = sqrt(Gx[i][j]*Gx[i][j]+Gy[i][j]*Gy[i][j]);
			thet[i][j] = atan2(Gy[i][j],Gx[i][j]);
		}
	}
}
void
pcl::pcl_2d::edge::cannyTraceEdge  (int rowOffset,int colOffset,int row,int col, float theta, float tLow, float tHigh, ImageType &G, ImageType &thet){
	int newRow = row+rowOffset;
	int newCol = col+colOffset;	
	if(newRow > 0 && newRow < G.size() && newCol > 0 && newCol < G[0].size())
	{
		if(G[newRow][newCol] < tLow || G[newRow][newCol] > tHigh)
			return;
		G[newRow][newCol] = tHigh+1;			
		switch((int)thet[newRow][newCol])
		{
			case 0:
				cannyTraceEdge(0,1,newRow,newCol,0,tLow, tHigh, G, thet);
				cannyTraceEdge(0,-1,newRow,newCol,0,tLow, tHigh, G, thet);
				break;
			case 45:
				cannyTraceEdge(-1,1,newRow,newCol,45,tLow, tHigh, G, thet);
				cannyTraceEdge(1,-1,newRow,newCol,45,tLow, tHigh, G, thet);
				break;
			case 90:
				cannyTraceEdge(-1,0,newRow,newCol,90,tLow, tHigh, G, thet);
				cannyTraceEdge(1,0,newRow,newCol,90,tLow, tHigh, G, thet);
				break;
			case 135:
				cannyTraceEdge(-1,-1,newRow,newCol,135,tLow, tHigh, G, thet);
				cannyTraceEdge(1,1,newRow,newCol,135,tLow, tHigh, G, thet);
				break;
		}		
	}
}
void
pcl::pcl_2d::edge::canny  (ImageType &output, ImageType &input)
{
	float tHigh = 50;
	float tLow = 20;
	/*noise reduction using gaussian blurring*/
	ImageType gaussian_kernel;
	conv_2d->gaussian(5, 1.0, gaussian_kernel);
	conv_2d->conv(output, gaussian_kernel, input);
	
	/*edge detection usign Sobel*/
	ImageType G;
	ImageType thet;
	sobelGThet(G, thet, output);
	
	/*edge discretization*/
	float angle;
	for(int i = 0;i < thet.size();i++)
	{
		for(int j = 0;j < thet.size();j++)
		{
			angle = (thet[i][j]/3.14)*180;	
			if (((angle < 22.5) && (angle > -22.5)) || (angle > 157.5) || (angle < -157.5))
				thet[i][j] = 0;
			else if (((angle > 22.5) && (angle < 67.5)) || ((angle < -112.5) && (angle > -157.5)))
				thet[i][j] = 45;
			else if (((angle > 67.5) && (angle < 112.5)) || ((angle < -67.5) && (angle > -112.5)))
				thet[i][j] = 90;
			else if (((angle > 112.5) && (angle < 157.5)) || ((angle < -22.5) && (angle > -67.5)))
				thet[i][j] = 135;
		}
	}
	
	/*tHigh and non-maximal supression*/
	for(int i = 1;i < thet.size()-1;i++)
	{
		for(int j = 1;j < thet.size()-1;j++)
		{
			if(G[i][j] < tHigh)
				continue;
			switch((int)thet[i][j])
			{
				case 0:
					if(G[i][j] > G[i-1][j] && G[i][j] > G[i+1][j]);						
					else
						G[i][j] = 0;
					break;
				case 45:
					if(G[i][j] > G[i+1][j+1] && G[i][j] > G[i-1][j-1]);						
					else
						G[i][j] = 0;
					break;
				case 90:
					if(G[i][j] > G[i][j-1] && G[i][j] > G[i][j+1]);						
					else
						G[i][j] = 0;
					break;
				case 135:
					if(G[i][j] > G[i+1][j-1] && G[i][j] > G[i-1][j+1]);						
					else
						G[i][j] = 0;
					break;
			}				
		}
	}
	
	/*edge tracing*/
	for(int i = 0;i < thet.size();i++)
	{
		for(int j = 0;j < thet.size();j++)
		{
			if(G[i][j] < tHigh)
				continue;
			switch((int)thet[i][j])
			{
				case 0:
					cannyTraceEdge(0,1,i,j,0,tLow, tHigh, G, thet);
					cannyTraceEdge(0,-1,i,j,0,tLow, tHigh, G, thet);
					break;
				case 45:
					cannyTraceEdge(-1,1,i,j,45,tLow, tHigh, G, thet);
					cannyTraceEdge(1,-1,i,j,45,tLow, tHigh, G, thet);
					break;
				case 90:
					cannyTraceEdge(-1,0,i,j,90,tLow, tHigh, G, thet);
					cannyTraceEdge(1,0,i,j,90,tLow, tHigh, G, thet);
					break;
				case 135:
					cannyTraceEdge(-1,-1,i,j,135,tLow, tHigh, G, thet);
					cannyTraceEdge(1,1,i,j,135,tLow, tHigh, G, thet);
					break;
			}		
		}
	}
	
	/*final thresholding*/
	output.resize(G.size());
	for(int i = 0;i < G.size();i++)
	{
		output[i].resize(G[i].size());
		for(int j = 0;j < G[i].size();j++)
		{
			if(G[i][j] < tHigh)
				output[i][j] = 0;
			else
				output[i][j] = 255;
		}
	}
}

void
pcl::pcl_2d::edge::LoGKernel  (ImageType &kernel, const int dim, const float sigma)
{
	float sum = 0;
	float temp = 0;
	kernel.resize(dim);
	for(int i = 0;i < dim;i++)
	{
		kernel[i].resize(dim);
		for(int j = 0;j < dim;j++)
		{
			temp = (((i-dim/2)*(i-dim/2)+(j-dim/2)*(j-dim/2))/(2*sigma*sigma));
			kernel[i][j] = (1-temp)*exp(-temp);
			sum += kernel[i][j];
		}
	}
	for(int i = 0;i < dim;i++)
	{
		for(int j = 0;j < dim;j++)
		{
			kernel[i][j] /= sum;
		}
	}
}
void
pcl::pcl_2d::edge::LoG  (ImageType &output, const int dim, const float sigma, ImageType &input)
{
	ImageType kernel;
	LoGKernel(kernel, dim, sigma);
	conv_2d->conv(output, kernel, input);
}				
void
pcl::pcl_2d::edge::LoG  (ImageType &output, ImageType &input)
{
	ImageType kernel;
	LoGKernel(kernel, 9, 1.4);
	conv_2d->conv(output, kernel, input);
}
#endif
