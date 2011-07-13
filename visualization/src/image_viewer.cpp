#include <pcl/visualization/image_viewer.h>
#include <vtkImageImport.h>
#include <vtkImageViewer.h>
#include <vtkXRenderWindowInteractor.h>
#include <vtk-5.4/vtkRenderWindowInteractor.h>
#include <vtk-5.4/vtkImageViewer.h>
#include <vtk-5.4/vtkImageViewer2.h>

pcl::visualization::ImageViewer::ImageViewer (const std::string& window_title)
  : Window (window_title)
  , image_viewer_ (vtkImageViewer2::New ())
{
  memset (dummy_, 0, 48);
  showRGBImage (dummy_, 16, 16);
  //setWindowTitle (window_title);
  //image_viewer_->SetupInteractor (interactor_);
  image_viewer_->SetRenderWindow (window_);
}

void pcl::visualization::ImageViewer::showRGBImage (const unsigned char* rgb_data, unsigned width, unsigned height)
{
  vtkImageImport* importer = vtkImageImport::New();
  importer->SetNumberOfScalarComponents (3);
  importer->SetWholeExtent (0, width - 1, 0, height - 1, 0, 0);
  importer->SetDataScalarTypeToUnsignedChar();
  importer->SetDataExtentToWholeExtent();
  
  void* data = const_cast<void*> ((const void*)rgb_data);
  importer->SetImportVoidPointer (data, 1);
  
  image_viewer_->SetInputConnection( importer->GetOutputPort());
  image_viewer_->SetColorLevel(127.5);
  image_viewer_->SetColorWindow(255);
  image_viewer_->Render();
}
