#ifndef PCL_MODEL_HPP_
#define PCL_MODEL_HPP_

#if defined(_WIN32) && !defined(APIENTRY) && !defined(__CYGWIN__)
# define WIN32_LEAN_AND_MEAN 1
# include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>

#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//RWX support disabled due to libbot depencency
//#include <bot_vis/bot_vis.h>

// Polygons:
#include <pcl/io/vtk_lib_io.h>


namespace pcl
{

typedef struct _SinglePoly {
  float* vertices_;
  float* colors_;
  GLenum mode_;
  size_t nvertices_;
} SinglePoly;

  
class Model
{
public:
  virtual void draw() = 0;

  typedef boost::shared_ptr<Model> Ptr;
  typedef boost::shared_ptr<const Model> ConstPtr;
};

/*
 * RWX support disabled due to libbot depencency

class RWXModel : public Model
{
public:
  RWXModel(const std::string & filename);
  virtual ~RWXModel();
  virtual void draw();

  typedef boost::shared_ptr<RWXModel> Ptr;
  typedef boost::shared_ptr<const RWXModel> ConstPtr;
private:
  std::string filename_;
  BotRwxModel* rwx_model_;
  bool display_list_ready_;
  GLuint rwx_dl_;
};
*/

class PolygonMeshModel : public Model
{
public:
  PolygonMeshModel(GLenum mode, pcl::PolygonMesh::Ptr plg);
  virtual ~PolygonMeshModel();
  virtual void draw();
  
  typedef boost::shared_ptr<PolygonMeshModel> Ptr;
  typedef boost::shared_ptr<const PolygonMeshModel> ConstPtr;
private:
  std::vector<SinglePoly> polygons;
  
  /*
    GL_POINTS;
    GL_LINE_STRIP;
    GL_LINE_LOOP;
    GL_LINES;
    GL_TRIANGLE_STRIP;
    GL_TRIANGLE_FAN;
    GL_TRIANGLES;
    GL_QUAD_STRIP;
    GL_QUADS;
    GL_POLYGON;
  */
  GLenum mode_;
};  



class PointCloudModel : public Model
{
public:
  PointCloudModel(GLenum mode, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc);
  virtual ~PointCloudModel();
  virtual void draw();

  typedef boost::shared_ptr<PointCloudModel> Ptr;
  typedef boost::shared_ptr<const PointCloudModel> ConstPtr;
private:
  float* vertices_;
  float* colors_;
  

  /*
    GL_POINTS;
    GL_LINE_STRIP;
    GL_LINE_LOOP;
    GL_LINES;
    GL_TRIANGLE_STRIP;
    GL_TRIANGLE_FAN;
    GL_TRIANGLES;
    GL_QUAD_STRIP;
    GL_QUADS;
    GL_POLYGON;
  */
  GLenum mode_;
  size_t nvertices_;
};

}

#endif /* PCL_MODEL_HPP_ */
