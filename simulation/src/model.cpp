#include "pcl/simulation/model.hpp"

namespace pcl
{

// Create a PolygonMeshModel by converting the PolygonMesh to our format
// TODO: support color. I had a look but it seems that PointClouds that
//       get as far as here only have x,y,z fields but no rgb[a] fields
//       this might be because the vtk file reader doesnt read any color
//       info from what im providing as input - but i dont know much about
//       these file types to be sure. mfallon
PolygonMeshModel::PolygonMeshModel(GLenum mode, pcl::PolygonMesh::Ptr plg ) : mode_(mode)
{
  pcl::PointCloud<pcl::PointXYZ> newcloud;  
  pcl::fromROSMsg(plg->cloud, newcloud);
  Eigen::Vector4f tmp;
  for(size_t i=0; i< plg->polygons.size (); i++){ // each triangle/polygon
    pcl::Vertices apoly_in = plg->polygons[i];
    SinglePoly apoly;
    apoly.nvertices_ =apoly_in.vertices.size ();
    apoly.vertices_ = new float[3*apoly_in.vertices.size ()];
    apoly.colors_ = new float[4*apoly_in.vertices.size ()]; 

    for(size_t j=0; j< apoly_in.vertices.size (); j++){ // each point
      uint32_t pt = apoly_in.vertices[j];
      tmp = newcloud.points[pt].getVector4fMap();
      
      // x,y,z
      apoly.vertices_[3*j + 0] = (float) tmp(0);
      apoly.vertices_[3*j + 1] = (float) tmp(1);
      apoly.vertices_[3*j + 2] = (float) tmp(2);  
      
      // Color: currently using red in place of true color
      // TODO: figure out how to read in color obj/vtk/ply files
      apoly.colors_[4*j + 3] =(float) 0.0/255.0; // transparancy? 
      apoly.colors_[4*j + 2] =(float) 0.0/255.0; // Blue
      apoly.colors_[4*j + 1] =(float) 0.0/255.0; // Green
      apoly.colors_[4*j + 0] =(float) 255.0/255.0;  // Red  
    }
    polygons.push_back(apoly);
  }
}

PolygonMeshModel::~PolygonMeshModel(){
  // TODO: memory management!
  
}

void PolygonMeshModel::draw()
{
  // This might be a little quicker than drawing using individual polygons
  // TODO: test by how much
  glEnable(GL_DEPTH_TEST);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  for (size_t i=0; i < polygons.size(); i++){
    glVertexPointer(3, GL_FLOAT, 0, polygons[i].vertices_);
    glColorPointer(4, GL_FLOAT, 0, polygons[i].colors_);
    glDrawArrays(mode_, 0, polygons[i].nvertices_);
  }
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

  
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

PointCloudModel::PointCloudModel(GLenum mode, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) : mode_(mode)
{
  nvertices_ = pc->points.size();
  vertices_ = new float[3*nvertices_];
  colors_ = new float[4*nvertices_];

  for (size_t i = 0; i < pc->points.size (); ++i)
  {
    vertices_[3*i + 0] = pc->points[i].x;
    vertices_[3*i + 1] = pc->points[i].y;
    vertices_[3*i + 2] = pc->points[i].z;

    int rgba_one = *reinterpret_cast<int*>(&pc->points[i].rgba);
    colors_[4*i + 3] =((float) ((rgba_one >> 24) & 0xff))/255.0;
    colors_[4*i + 2] =((float) ((rgba_one >> 16) & 0xff))/255.0;
    colors_[4*i + 1] =((float) ((rgba_one >> 8) & 0xff))/255.0;
    colors_[4*i + 0] =((float) (rgba_one & 0xff) )/255.0;    
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