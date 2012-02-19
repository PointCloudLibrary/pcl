#ifndef PCL_MODEL_HPP_
#define PCL_MODEL_HPP_

#if defined(_WIN32) && !defined(APIENTRY) && !defined(__CYGWIN__)
# define WIN32_LEAN_AND_MEAN 1
# include <windows.h>
#endif
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <boost/shared_ptr.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl/PolygonMesh.h"

//RWX support disabled due to libbot depencency
//#include <bot_vis/bot_vis.h>

namespace pcl
{
namespace simulation
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

class PCL_EXPORTS PolygonMeshModel : public Model
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

class PCL_EXPORTS PointCloudModel : public Model
{
public:
  PointCloudModel(GLenum mode, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);
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

/**
 * Renders a single quad providing position (-1,-1,0) - (1,1,0) and
 * texture coordinates (0,0) - (1,1) to each vertex.
 * Coordinates are (lower left) - (upper right).
 * Position is set as vertex attribute 0 and the texture coordinate
 * as vertex attribute 1.
 */
class Quad
{
public:
  /**
   * Setup the vbo for the quad.
   */
  Quad ();

  /**
   * Release any resources.
   */

  ~Quad ();

  /**
   * Render the quad.
   */
  void
  render ();

private:
  GLuint quad_vbo_;
};

} // namespace - simulation
} // namespace - pcl

#endif /* PCL_SIMULATION_MODEL_HPP_ */
