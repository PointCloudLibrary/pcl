#ifndef __IMAGE_VISUALIZER_H__
#define	__IMAGE_VISUALIZER_H__

#include "window.h"
#include <vtkImageViewer2.h>
#include <vtkInteractorStyle.h>

namespace pcl
{
  namespace PCL_EXPORTS visualization
  {
    class ImageViewer : public Window
    {
      public:
        ImageViewer (const std::string& window_title = "");
        
        void showRGBImage (const unsigned char* data, unsigned width, unsigned height);
      protected:
        vtkSmartPointer<vtkImageViewer2> image_viewer_;
        unsigned char dummy_[48];
    };
  }
}

#endif	/* __IMAGE_VISUALIZER_H__ */

