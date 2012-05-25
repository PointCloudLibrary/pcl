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
#include <pcl/io/png_io.h>


unsigned char* getUcharArray(vector<vector<float> > &data){
	unsigned char *image_char_array = (unsigned char*)malloc(sizeof(unsigned char)*data.size()*data[0].size());
	for(int i = 0;i < data.size();i++){
		for(int j = 0;j < data[i].size();j++){
			image_char_array[i*data[0].size()+j] = data[i][j];
		}
	}
	return image_char_array;
}

void saveGrayPNG(char *fname, vector<vector<float> > &data){
	unsigned char *image_char_array = getUcharArray(data);
	pcl::io::saveMonoPNGFile(fname, image_char_array, data[0].size(), data.size());
	free(image_char_array);
}

void visualizeGrayImage(vector<vector<float> > &data){

	unsigned char *image_char_array = getUcharArray(data);

	vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New ();
	importer->SetNumberOfScalarComponents (1);
	importer->SetDataScalarTypeToUnsignedChar ();
	importer->SetWholeExtent (0, data[0].size() - 1, 0, data.size() - 1, 0, 0);
	importer->SetDataExtentToWholeExtent ();
	void* image_data = const_cast<void*> (reinterpret_cast<const void*> (image_char_array));
	importer->SetImportVoidPointer (image_data, 1);
	importer->Update ();

	vtkSmartPointer<vtkImageViewer2> imageViewer =
			vtkSmartPointer<vtkImageViewer2>::New();
	imageViewer->SetInputConnection(importer->GetOutputPort());
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
			vtkSmartPointer<vtkRenderWindowInteractor>::New();
	imageViewer->SetupInteractor(renderWindowInteractor);
	imageViewer->Render();
	imageViewer->GetRenderer()->ResetCamera();
	imageViewer->Render();

	renderWindowInteractor->Start();
	free(image_char_array);
}
void readPNGVector(char *fname, vector<vector<vector<float> > > &image){

	vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
	reader->SetFileName(fname);
	reader->Update();

	vtkSmartPointer<vtkImageData> data = vtkSmartPointer<vtkImageData>::New();
	data = reader->GetOutput();
	int dim[3];
	data->GetDimensions(dim);

	vtkUnsignedCharArray *image_data = vtkUnsignedCharArray::SafeDownCast(data->GetPointData()->GetScalars());
	vtkIdType tupleIndex = 0;
	unsigned char tuple[] = {0, 0, 0, 0};

	image.resize(dim[0]);
	for(int i = 0;i < dim[0];i++){
		image[i].resize(dim[1]);
		for(int j = 0;j < dim[1];j++){
			image[i][j].resize(4);
			float r=0, g=0, b=0, a=0;
			image_data->GetTupleValue(tupleIndex++, tuple);
			switch(image_data->GetNumberOfComponents())
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
//			fprintf(f, "%d\n", r);
		}
	}
}

int main (int argc, char** argv)
{
	FILE *f;
	f = fopen("image.txt", "w");
	vector<vector<vector<float> > > image;
	readPNGVector("lena-grayscale.png", image);

	vector<vector<float> > kernel;
//	kernel.resize(5);
//	for(int i = 0;i < 5;i++){
//		kernel[i].resize(5);
//		for(int j = 0;j < 5;j++)
//			kernel[i][j] = 1.0/(5*5);
//	}

	vector<vector<float> > output;
	vector<vector<float> > input;
	input.resize(image.size());
	for(int i = 0;i < image.size();i++){
		input[i].resize(image[i].size());
		for(int j = 0;j < image[i].size();j++){
			input[i][j] = image[i][j][0];
		}
	}
	saveGrayPNG("input.png", input);
	visualizeGrayImage(input);
	pcl::pcl_2d::convolution_2d *conv2d = new pcl::pcl_2d::convolution_2d();
	conv2d->gaussian(51, 2, kernel);
	conv2d->conv(output, kernel, input,1);

	vector<vector<float> > G;
	vector<vector<float> > thet;
	pcl::pcl_2d::edge *e = new pcl::pcl_2d::edge();
	e->prewittGThet(G, thet, input);
	saveGrayPNG("prewitt_g.png", G);
	saveGrayPNG("prewitt_thet.png", thet);
	e->sobelGThet(G, thet, input);
	saveGrayPNG("sobel_g.png", G);
	saveGrayPNG("sobel_thet.png", thet);
	e->robertsGThet(G, thet, input);
	saveGrayPNG("roberts_g.png", G);
	saveGrayPNG("roberts_thet.png", thet);
	e->LoG(output, input);
	saveGrayPNG("log.png", output);
	e->canny(output, input);
	saveGrayPNG("canny.png", output);

	fclose(f);
}
