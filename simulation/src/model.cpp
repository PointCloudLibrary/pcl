#include "pcl/simulation/model.hpp"

namespace pcl
{

/* RWX support disabled due to libbot depencency
RWXModel::RWXModel(const std::string & filename) :
    filename_(filename),
    rwx_model_(NULL),
    display_list_ready_(false)
{
  
  rwx_model_ = bot_rwx_model_create(filename_.c_str());

}

RWXModel::~RWXModel()
{
  if (rwx_model_)
      bot_rwx_model_destroy(rwx_model_);
}

void RWXModel::draw()
{
  glEnable(GL_DEPTH_TEST);
  if (!display_list_ready_) {
    rwx_dl_ = glGenLists (1);
    glNewList (rwx_dl_, GL_COMPILE);
    bot_rwx_model_gl_draw(rwx_model_);
    glEndList ();
    display_list_ready_ = true;
  } else {
    glCallList (rwx_dl_);
  } 
}
*/

PointCloudModel::PointCloudModel(GLenum mode, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc) : mode_(mode)
{
//   std::cout << "Loaded "
//       << pc->width * pc->height
//       << " data points. "
//       << std::endl;

  nvertices_ = pc->points.size();
  vertices_ = new float[3*nvertices_];
  colors_ = new float[4*nvertices_];

  for (size_t i = 0; i < pc->points.size (); ++i)
  {
    unsigned char* rgba_ptr = (unsigned char*)&pc->points[i].rgba;

    /*
    std::cout << "    " << pc->points[i].x
                << " "    << pc->points[i].y
                << " "    << pc->points[i].z
                << " "    << static_cast<int>(*rgba_ptr)
                << " "    << static_cast<int>(*rgba_ptr+1)
                << " "    << static_cast<int>(*rgba_ptr+2)
                << " "    << static_cast<int>(*rgba_ptr+3)
                << std::endl;
    */
    vertices_[3*i + 0] = pc->points[i].x;
    vertices_[3*i + 1] = pc->points[i].y;
    vertices_[3*i + 2] = pc->points[i].z;


    int rgba_one = *reinterpret_cast<int*>(&pc->points[i].rgba);
    colors_[4*i + 3] =((float) ((rgba_one >> 24) & 0xff))/255.0;
    colors_[4*i + 2] =((float) ((rgba_one >> 16) & 0xff))/255.0;
    colors_[4*i + 1] =((float) ((rgba_one >> 8) & 0xff))/255.0;
    colors_[4*i + 0] =((float) (rgba_one & 0xff) )/255.0;
/*
    colors_[4*i + 0] = static_cast<int>(*rgba_ptr)/255.0;
    colors_[4*i + 1] = static_cast<int>(*rgba_ptr+1)/255.0;
    colors_[4*i + 2] = static_cast<int>(*rgba_ptr+2)/255.0;
    colors_[4*i + 3] = static_cast<int>(*rgba_ptr+3)/255.0;
    */
  }
}

PointCloudModel::~PointCloudModel()
{
  delete vertices_;
  delete colors_;
}

void PointCloudModel::draw()
{
  glEnable(GL_DEPTH_TEST);

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  glVertexPointer(3, GL_FLOAT, 0, vertices_);
  glColorPointer(4, GL_FLOAT, 0, colors_);

  glDrawArrays(mode_, 0, nvertices_);

  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}
}

