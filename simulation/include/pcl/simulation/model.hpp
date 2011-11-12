#ifndef MRG_MODEL_HPP_
#define MRG_MODEL_HPP_

#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <GL/gl.h>
#include <GL/glu.h>
//RWX support disabled due to libbot depencency
//#include <bot_vis/bot_vis.h>

namespace pcl
{

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

#endif /* MRG_MODEL_HPP_ */
