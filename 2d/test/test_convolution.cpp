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
 * $Id: test_convolution.cpp nsomani $
 *
 */

#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkPNGReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageData.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkImageImport.h>

#include "../include/pcl/2d/convolution_2d.h"
#include "../include/pcl/2d/edge.h"
#include <vector>
#include <vtkImageFlip.h>
#include <vtkPNGWriter.h>
#include <vtkUnsignedCharArray.h>

void flipAndWritePng(const std::string &file_name, vtkSmartPointer<vtkImageImport>& importer)
{
	vtkSmartPointer<vtkImageFlip> flipYFilter = vtkSmartPointer<vtkImageFlip>::New();
	flipYFilter->SetFilteredAxis(1); // flip y axis
	flipYFilter->SetInputConnection(importer->GetOutputPort());
	flipYFilter->Update();

	vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName(file_name.c_str());
	writer->SetInputConnection(flipYFilter->GetOutputPort());
	writer->Write();
}

void saveMonoPNGFile (const std::string &file_name, const unsigned char *mono_image, int width, int height)
{
  vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New ();
  importer->SetNumberOfScalarComponents (1);
  importer->SetDataScalarTypeToUnsignedChar ();
  importer->SetWholeExtent (0, width - 1, 0, height - 1, 0, 0);
  importer->SetDataExtentToWholeExtent ();

  void* data = const_cast<void*> (reinterpret_cast<const void*> (mono_image));
  importer->SetImportVoidPointer (data, 1);
  importer->Update ();

  flipAndWritePng(file_name, importer);
}

void saveFloatPNGFile (const std::string &file_name, const float *mono_image, int width, int height)
{
  vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New ();
  importer->SetNumberOfScalarComponents (1);
  importer->SetDataScalarTypeToUnsignedChar ();
  importer->SetWholeExtent (0, width - 1, 0, height - 1, 0, 0);
  importer->SetDataExtentToWholeExtent ();

  void* data = const_cast<void*> (reinterpret_cast<const void*> (mono_image));
  importer->SetImportVoidPointer (data, 1);
  importer->Update ();

  flipAndWritePng(file_name, importer);
}


void get_data_from_reader(vtkPNGReader *reader, vector<vector<vector<float> > > &image){

	vtkSmartPointer<vtkImageData> data = vtkSmartPointer<vtkImageData>::New();
	data = reader->GetOutput();
	int dim[3];
	data->GetDimensions(dim);
	printf("%d %d %d \n", dim[0], dim[1], dim[2]);
	vtkUnsignedCharArray *float_data = vtkUnsignedCharArray::SafeDownCast(data->GetPointData()->GetScalars());
	vtkIdType tupleIndex = 0;
	unsigned char tuple[] = {0, 0, 0, 0};
	for(int i = 0;i < dim[0];i++){
		for(int j = 0;j < dim[1];j++){
			unsigned char r=0, g=0, b=0, a=0;
			float_data->GetTupleValue(tupleIndex++, tuple);
			printf("%d\n", float_data->GetNumberOfComponents());
			printf("%d %d %d\n", tuple[0], tuple[1], tuple[2]);
			switch(float_data->GetNumberOfComponents())
			{
			case 1:
				r = g = b = tuple[0];
				a = 255;
				break;
			case 2:
				r = g = b = tuple[0];
				a = tuple[1];
				break;
			case 3:
				r = tuple[0];
				g = tuple[1];
				b = tuple[2];
				a = 255;
				break;
			case 4:
				r = tuple[0];
				g = tuple[1];
				b = tuple[2];
				a = tuple[3];
				break;
			}
			printf("%d\n", tupleIndex);
			image[i][j][0] = r;
			image[i][j][1] = g;
			image[i][j][2] = b;
			image[i][j][3] = a;

		}
	}
}

int main (int argc, char** argv)
{
	FILE *f;
	f = fopen("image.txt", "w");
	vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
	reader->SetFileName("lena-grayscale.png");
	reader->Update();

	// Visualize
//	vtkSmartPointer<vtkImageViewer2> imageViewer =
//			vtkSmartPointer<vtkImageViewer2>::New();
//	imageViewer->SetInputConnection(reader->GetOutputPort());
//	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//			vtkSmartPointer<vtkRenderWindowInteractor>::New();
//	imageViewer->SetupInteractor(renderWindowInteractor);
//	imageViewer->Render();
//	imageViewer->GetRenderer()->ResetCamera();
//	imageViewer->Render();

//	renderWindowInteractor->Start();
	vector<vector<vector<float> > > image;
	//get_data_from_reader(reader, image);

	vtkSmartPointer<vtkImageData> data = vtkSmartPointer<vtkImageData>::New();
	data = reader->GetOutput();
	int dim[3];
	data->GetDimensions(dim);
	printf("%d %d %d \n", dim[0], dim[1], dim[2]);


	vtkUnsignedCharArray *float_data = vtkUnsignedCharArray::SafeDownCast(data->GetPointData()->GetScalars());
	vtkIdType tupleIndex = 0;
	unsigned char tuple[] = {0, 0, 0, 0};
	image.resize(dim[0]);
	for(int i = 0;i < dim[0];i++){
		image[i].resize(dim[1]);
		for(int j = 0;j < dim[1];j++){
			image[i][j].resize(4);
			unsigned char r=0, g=0, b=0, a=0;
			float_data->GetTupleValue(tupleIndex++, tuple);
//			printf("%d\n", tupleIndex);
//			printf("%d %d %d\n", tuple[0], tuple[1], tuple[2]);
			switch(float_data->GetNumberOfComponents())
			{
			case 1:
				r = g = b = tuple[0];
				a = 255;
				break;
			case 2:
				r = g = b = tuple[0];
				a = tuple[1];
				break;
			case 3:
				r = tuple[0];
				g = tuple[1];
				b = tuple[2];
				a = 255;
				break;
			case 4:
				r = tuple[0];
				g = tuple[1];
				b = tuple[2];
				a = tuple[3];
				break;
			}
			image[i][j][0] = r;
			image[i][j][1] = g;
			image[i][j][2] = b;
			image[i][j][3] = a;
			fprintf(f, "%d\n", r);
		}
	}

	vector<vector<float> > kernel;
	kernel.resize(5);
	for(int i = 0;i < 5;i++){
		kernel[i].resize(5);
		for(int j = 0;j < 5;j++)
			kernel[i][j] = 1.0/(5*5);
	}

	vector<vector<float> > output;
	vector<vector<unsigned char> > outputPNG;
	vector<vector<float> > input;

	input.resize(image.size());
	output.resize(image.size());
	for(int i = 0;i < image.size();i++){
		input[i].resize(image[i].size());
		output[i].resize(image[i].size());
		for(int j = 0;j < image[i].size();j++){
			input[i][j] = image[i][j][0];
		}
	}
//	pcl::pcl_2d::convolution_2d *conv2d = new pcl::pcl_2d::convolution_2d();
//
//	conv2d->conv(output, kernel, input,2);

	pcl::pcl_2d::edge *e = new pcl::pcl_2d::edge();

	vector<vector<float> > G;
	vector<vector<float> > thet;

	//e->sobelXY(G, thet, input);
	e->canny(output, input);
	outputPNG.resize(output.size());
	for(int i = 0;i < output.size();i++){
		outputPNG[i].resize(output[i].size());
		for(int j = 0;j < output[i].size();j++){
			outputPNG[i][j] = output[i][j];
		}
	}
	saveMonoPNGFile("output.png", &outputPNG[0][0], outputPNG.size(), outputPNG[0].size());

	outputPNG.resize(input.size());
	for(int i = 0;i < input.size();i++){
		outputPNG[i].resize(input[i].size());
		for(int j = 0;j < input[i].size();j++){
			outputPNG[i][j] = input[i][j];
			//printf("%f\t", output[i][j]);
		}
	}
	saveMonoPNGFile("input.png", &outputPNG[0][0], outputPNG.size(), outputPNG[0].size());

	fclose(f);
}

