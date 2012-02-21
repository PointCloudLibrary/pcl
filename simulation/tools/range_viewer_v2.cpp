#include <GL/glew.h>
#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <cfloat>
// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/window.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
//#include <vtkPolyDataReader.h>
//#include <vtkProp.h>
#include <boost/algorithm/string.hpp>
#include <string>

#ifdef _WIN32
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
#endif
#include <GL/gl.h>

#include "pcl/simulation/camera.h"
#include "pcl/simulation/model.h"
#include "pcl/simulation/scene.h"
#include "pcl/simulation/range_likelihood.h"

#include <pcl/io/vtk_lib_io.h>


using namespace std;
using namespace pcl;

uint16_t t_gamma[2048];

// global for now:
Camera::Ptr camera_;

namespace pcl
{
  namespace simulation
  {
    class OpenGLProp : public vtkProp {
      public:
        static OpenGLProp*
        New ();
	
        virtual int
        RenderOpaqueGeometry (vtkViewport * viewport);
	
	Scene::Ptr scene_;
	
	RangeLikelihood::Ptr range_likelihood_;
	
	
	pcl::visualization::ImageViewer iviewer_;
	pcl::visualization::ImageViewer iviewer_color_;
	
	void display_depth_image(const float* depth_buffer);
	int load_PolygonMesh_model(string &polygon_file);

    };
    
    
    int
    OpenGLProp::RenderOpaqueGeometry(vtkViewport * viewport)
    {      
      //scene_->draw();
      
      //glViewport(range_likelihood_->width(), 0, range_likelihood_->width(), range_likelihood_->height());
  

      float* reference = new float[range_likelihood_->getRowHeight() * range_likelihood_->getColWidth()];
      const float* depth_buffer = range_likelihood_->getDepthBuffer();
      // Copy one image from our last as a reference.
      for (int i=0, n=0; i<range_likelihood_->getRowHeight(); ++i) {
	for (int j=0; j<range_likelihood_->getColWidth(); ++j) {
	  reference[n++] = depth_buffer[i*range_likelihood_->getWidth() + j];
	}
      }

      std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
      std::vector<float> scores;
      int n = range_likelihood_->getRows()*range_likelihood_->getCols();
      for (int i=0; i<n; ++i) {
	Camera camera(*camera_);

	//camera.set(10.0, 0.0, 0.0,
	//	   0.0, 0.0, 0.0); //hard coded settings
	//camera.set_pitch(0);
	
	
	//camera.move(10.0,i*0.02,10.0);
	camera.move(0.0,i*0.02,0.0);
	poses.push_back(camera.pose());

	Eigen::Isometry3d this_pose;
	this_pose = camera.pose();
	      cout << "current pose: " << this_pose.translation().x()
	      << ", " << this_pose.translation().y()
	      << ", " << this_pose.translation().z() << "\n";

      }
      
      
      float* depth_field =NULL;
      bool do_depth_field =false;
      range_likelihood_->computeLikelihoods(reference, poses, scores,depth_field,do_depth_field);
    //  range_likelihood_->compute_likelihoods(reference, poses, scores);
      delete [] reference;
      delete [] depth_field;

      std::cout << "score: ";
      for (size_t i=0; i<scores.size(); ++i) {
	std::cout << " " << scores[i];
      }
      std::cout << std::endl;
      
      iviewer_.setWindowTitle ("Color Image");
      
      iviewer_.showFloatImage 	( 	range_likelihood_->getDepthBuffer(),
 		640,480,0,1,true);
      iviewer_.setWindowTitle ("Color Image");		
		
      iviewer_color_.showRGBImage(range_likelihood_->getColorBuffer(),640,480);
      
      
//       // Draw the depth image
//       //  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//       //  glColorMask(true, true, true, true);
//       glDisable(GL_DEPTH_TEST);
//       glViewport(0, 0, range_likelihood_->width(), range_likelihood_->height());
//       //glViewport(0, 0, range_likelihood_->width(), range_likelihood_->height());
// 
//       glMatrixMode(GL_PROJECTION);
//       glLoadIdentity();
//       glMatrixMode(GL_MODELVIEW);
//       glLoadIdentity();
// 
//       //glRasterPos2i(-1,-1);
//       glDrawPixels(range_likelihood_->width(), range_likelihood_->height(), GL_LUMINANCE, GL_FLOAT, range_likelihood_->depth_buffer());
	
       display_depth_image(range_likelihood_->getDepthBuffer());
//       //glutSwapBuffers();      
//       //cout << "after adding triangles\n";
       
      return 1;
    }
    

    void  OpenGLProp::display_depth_image(const float* depth_buffer)
    {
      int npixels = range_likelihood_->getWidth() * range_likelihood_->getHeight();
      uint8_t* depth_img = new uint8_t[npixels* 3];
      for (int i=0; i<npixels; i++) {
		float zn = 0.7;
		float zf = 20.0;
		float d = depth_buffer[i];
		float z = -zf*zn/((zf-zn)*(d - zf/(zf-zn)));
		float b = 0.075;
		float f = 580.0;
		uint16_t kd = static_cast<uint16_t>(1090 - b*f/z*8);
		if (kd < 0) kd = 0;
		else if (kd>2047) kd = 2047;

		int pval = t_gamma[kd];
		int lb = pval & 0xff;
		switch (pval>>8) {
		    case 0:
			depth_img[3*i+0] = 255;
			depth_img[3*i+1] = 255-lb;
			depth_img[3*i+2] = 255-lb;
			break;
		    case 1:
			depth_img[3*i+0] = 255;
			depth_img[3*i+1] = lb;
			depth_img[3*i+2] = 0;
			break;
		    case 2:
			depth_img[3*i+0] = 255-lb;
			depth_img[3*i+1] = 255;
			depth_img[3*i+2] = 0;
			break;
		    case 3:
			depth_img[3*i+0] = 0;
			depth_img[3*i+1] = 255;
			depth_img[3*i+2] = lb;
			break;
		    case 4:
			depth_img[3*i+0] = 0;
			depth_img[3*i+1] = 255-lb;
			depth_img[3*i+2] = 255;
			break;
		    case 5:
			depth_img[3*i+0] = 0;
			depth_img[3*i+1] = 0;
			depth_img[3*i+2] = 255-lb;
			break;
		    default:
			depth_img[3*i+0] = 0;
			depth_img[3*i+1] = 0;
			depth_img[3*i+2] = 0;
			break;

		}

      }
      delete [] depth_img;
      
      // do something here...
    }    


    int
    OpenGLProp::load_PolygonMesh_model(string &polygon_file)
    {
      pcl::PolygonMesh mesh;	// (new pcl::PolygonMesh);
      //pcl::io::loadPolygonFile("/home/mfallon/data/models/dalet/Darlek_modified_works.obj",mesh);
      pcl::io::loadPolygonFile(polygon_file.c_str(),mesh);
      pcl::PolygonMesh::Ptr cloud (new pcl::PolygonMesh(mesh));
      
      // Not sure if PolygonMesh assumes triangles if to
      // TODO: Ask a developer
      PolygonMeshModel::Ptr model = PolygonMeshModel::Ptr(new PolygonMeshModel(GL_POLYGON,cloud));
      scene_->add(model);  
      
      std::cout << "Just read " << polygon_file << std::endl;
      std::cout << mesh.polygons.size() << " polygons and " 
		<< mesh.cloud.data.size() << " triangles\n";
      
      return 1;
    }

    vtkStandardNewMacro(OpenGLProp);

    class RangeViewer : public pcl::visualization::Window
    {
      public:
        RangeViewer (const std::string& window_name);
        virtual ~RangeViewer ();

        boost::signals2::connection
        registerMouseCallback (boost::function<void (const pcl::visualization::MouseEvent&)> event)
        { return pcl::visualization::Window::registerMouseCallback (event); }

        boost::signals2::connection
        registerKeyboardCallback (boost::function<void (const pcl::visualization::KeyboardEvent&)> event)
        { return pcl::visualization::Window::registerKeyboardCallback (event); }

      private:
        vtkRenderer* renderer_;
        OpenGLProp* prop_;
    };

    RangeViewer::RangeViewer (const std::string& window_name) : pcl::visualization::Window(window_name)
    {
      prop_ = OpenGLProp::New();
      
      //prop_
      camera_ = Camera::Ptr(new Camera());
      
      prop_->scene_ = Scene::Ptr(new Scene());
      prop_->range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, 480, 640, prop_->scene_, 0));
      //  range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(10, 10, 96, 96, scene_));
      //range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, 480, 640, scene_));
      
      // Actually corresponds to default parameters:
      prop_->range_likelihood_->setCameraIntrinsicsParameters(640,480,576.09757860,
	    576.09757860, 321.06398107,  242.97676897);

//      string map_file = "/home/mfallon/data/models/stata_models/data/d2_ply_models/stata_03.ply";
      string map_file = "/home/hordurj/data/3dmodels/stata_03.ply";
      cout << "About to read: " << map_file.c_str() << endl;
      prop_->load_PolygonMesh_model(map_file);
//    load_PolygonMesh_model(map_file);

      // 
      camera_->set(-5.0, 0.0, 1.0, 0.0, 0.0, 0.0);
      
      prop_->iviewer_.setWindowTitle ("Color Image");
      prop_->iviewer_color_.setWindowTitle ("Depth Image");

//      win_->SetSize (500, 500);
      //win_->SetSize (1000, 1000);
      //win_->SetSize (640, 480);
      win_->SetSize (640, 480);

      renderer_ = vtkRenderer::New();
      renderer_->AddActor( prop_ );
    //  renderer_->SetBackground( 0.1, 0.2, 0.4 );
      renderer_->SetViewport(0.0, 0.0, 1.0, 1.0);

      win_->AddRenderer( renderer_ );
    }

    RangeViewer::~RangeViewer ()
    {
      renderer_->Delete();
      prop_->Delete();
    }
  }
}

class RangeVisualization
{
  public:
    RangeVisualization ();

    void
    spin ();

    void
    on_keyboard (const pcl::visualization::KeyboardEvent& event);

    void
    on_mouse (const pcl::visualization::MouseEvent& event);

  private:
    boost::shared_ptr<pcl::simulation::RangeViewer> viewer_;
};

RangeVisualization::RangeVisualization ()
{
  
  viewer_.reset (new pcl::simulation::RangeViewer ("Range Simulation"));
  viewer_->registerMouseCallback (boost::bind (&RangeVisualization::on_mouse, this, _1));
  viewer_->registerKeyboardCallback (boost::bind (&RangeVisualization::on_keyboard, this, _1));
}

void
RangeVisualization::on_keyboard (const pcl::visualization::KeyboardEvent& event)
{
//  cout << "on_keyboard\n";
  double speed = 0.1;
  if (event.getKeyCode()){
    cout << "the key \'" << event.getKeyCode() << "\' (" << (int)event.getKeyCode() << ") was";
  }else{
    string KeySym = event.getKeySym();
    if (KeySym.compare("Up") == 0){
      camera_->move(speed,0,0);
      cout << "u";
    }else if (KeySym.compare("Down") == 0){
      camera_->move(-speed,0,0);
      cout << "d";
    }else if (KeySym.compare("Left") == 0){
      camera_->move(0,-speed,0);
      cout << "l";
    }else if (KeySym.compare("Right") == 0){
      camera_->move(0,speed,0);
      cout << "r";
    }else if (KeySym.compare("Prior") == 0){
      camera_->move(0,0,speed);
      cout << "r";
    }else if (KeySym.compare("Next") == 0){
      camera_->move(0,0,-speed);
      cout << "r";
    }

    //cout << "the special key code \'" << event.getKeyCode() << "\' was";
    //cout << "the special key \'" << event.getKeySym() << "\' was";
  }
   
   /*
  if (event.keyDown())
    cout << " pressed" << endl;
  else
    cout << " released" << endl;  
     */ 
      
  
}

void
RangeVisualization::on_mouse (const pcl::visualization::MouseEvent& event)
{
}

void
RangeVisualization::spin ()
{
  viewer_->spin ();
}

int
main (int argc, char** argv)
{
 
  int i;
  for (i=0; i<2048; i++) {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }  
  
  RangeVisualization range_viz;
  range_viz.spin ();

  return 0;
}

