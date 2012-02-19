#include "pcl/simulation/model.h"

namespace pcl
{

namespace simulation
{

// Create a PolygonMeshModel by converting the PolygonMesh to our format
PolygonMeshModel::PolygonMeshModel (GLenum mode, pcl::PolygonMesh::Ptr plg ) : mode_(mode)
{
  bool found_rgb=false;
  for (size_t i=0; i<plg->cloud.fields.size() ;i++)
    if (plg->cloud.fields[i].name.compare("rgb") == 0)
      found_rgb =true;
  
  if (found_rgb)
  {
    pcl::PointCloud<pcl::PointXYZRGB> newcloud;  
    pcl::fromROSMsg(plg->cloud, newcloud);
    Eigen::Vector4f tmp;
    for(size_t i=0; i< plg->polygons.size (); i++)
    { // each triangle/polygon
      pcl::Vertices apoly_in = plg->polygons[i];
      SinglePoly apoly;
      apoly.nvertices_ =apoly_in.vertices.size ();
      apoly.vertices_ = new float[3*apoly_in.vertices.size ()];
      apoly.colors_ = new float[4*apoly_in.vertices.size ()]; 

      for(size_t j=0; j< apoly_in.vertices.size (); j++)
      { // each point
	uint32_t pt = apoly_in.vertices[j];
	tmp = newcloud.points[pt].getVector4fMap();
	// x,y,z
	apoly.vertices_[3*j + 0] = (float) tmp(0);
	apoly.vertices_[3*j + 1] = (float) tmp(1);
	apoly.vertices_[3*j + 2] = (float) tmp(2);  
	// r,g,b: input is ints 0->255, opengl wants floats 0->1
	apoly.colors_[4*j + 0] =(float) newcloud.points[pt].r/255.0; // Red  
	apoly.colors_[4*j + 1] =(float) newcloud.points[pt].g/255.0; // Green
	apoly.colors_[4*j + 2] =(float) newcloud.points[pt].b/255.0; // Blue
	apoly.colors_[4*j + 3] =(float) 1.0; // transparancy? unnecessary?
      }
      polygons.push_back (apoly);
    }
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ> newcloud;  
    pcl::fromROSMsg(plg->cloud, newcloud);
    Eigen::Vector4f tmp;
    for(size_t i=0; i< plg->polygons.size (); i++)
    { // each triangle/polygon
      pcl::Vertices apoly_in = plg->polygons[i];
      SinglePoly apoly;
      apoly.nvertices_ =apoly_in.vertices.size ();
      apoly.vertices_ = new float[3*apoly_in.vertices.size ()];
      apoly.colors_ = new float[4*apoly_in.vertices.size ()]; 

      for(size_t j=0; j< apoly_in.vertices.size (); j++)
      { // each point
	uint32_t pt = apoly_in.vertices[j];
	tmp = newcloud.points[pt].getVector4fMap();
	// x,y,z
	apoly.vertices_[3*j + 0] = (float) tmp(0);
	apoly.vertices_[3*j + 1] = (float) tmp(1);
	apoly.vertices_[3*j + 2] = (float) tmp(2);  
	// r,g,b: input is ints 0->255, opengl wants floats 0->1
	apoly.colors_[4*j + 0] =(float) 255/255.0; // Red  
	apoly.colors_[4*j + 1] =(float) 0.0/255.0; // Green
	apoly.colors_[4*j + 2] =(float) 0.0/255.0; // Blue
	apoly.colors_[4*j + 3] =(float) 1.0; // transparancy? 
      }
      polygons.push_back (apoly);
    }
  }
}

PolygonMeshModel::~PolygonMeshModel ()
{
  // TODO: memory management!
}

void
PolygonMeshModel::draw ()
{
  // This might be a little quicker than drawing using individual polygons
  // TODO: test by how much
  glEnable(GL_DEPTH_TEST);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  for (size_t i=0; i < polygons.size(); i++)
  {
    glVertexPointer(3, GL_FLOAT, 0, polygons[i].vertices_);
    glColorPointer(4, GL_FLOAT, 0, polygons[i].colors_);
    glDrawArrays(mode_, 0, polygons[i].nvertices_);
  }
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

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

    // TODO: is this necessary or correct if we are using rgb and not rgba?
    int rgba_one = *reinterpret_cast<int*>(&pc->points[i].rgba);
    colors_[4*i + 3] =((float) ((rgba_one >> 24) & 0xff))/255.0;
    //
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

void
PointCloudModel::draw()
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

Quad::Quad()
{
  // vertex pos: xyz , texture coord: uv
  const static float vertices[20] = {-1.0, -1.0, 0.0, 0.0, 0.0,
                                     -1.0,  1.0, 0.0, 0.0, 1.0,
                                      1.0,  1.0, 0.0, 1.0, 1.0,
                                      1.0, -1.0, 0.0, 1.0, 0.0 };

  glGenBuffers(1, &quad_vbo_);
  glBindBuffer(GL_ARRAY_BUFFER, quad_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

Quad::~Quad()
{
  glDeleteBuffers(1, &quad_vbo_);
}

void
Quad::render()
{
  glBindBuffer(GL_ARRAY_BUFFER, quad_vbo_);
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*5, 0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(float)*5, (const GLvoid*)12);

  glDrawArrays(GL_QUADS, 0, 4);

  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

} // namespace - simulation
} // namespace - pcl
