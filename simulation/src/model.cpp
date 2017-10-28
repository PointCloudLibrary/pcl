#include <pcl/simulation/model.h>
using namespace pcl::simulation;

pcl::simulation::TriangleMeshModel::TriangleMeshModel (pcl::PolygonMesh::Ptr plg)
{
  Vertices vertices;
  Indices indices;

  bool found_rgb = false;
  for (size_t i=0; i < plg->cloud.fields.size () ; ++i)
    if (plg->cloud.fields[i].name.compare ("rgb") == 0)
      found_rgb = true;

  if (found_rgb)
  {
    pcl::PointCloud<pcl::PointXYZRGB> newcloud;
    pcl::fromPCLPointCloud2 (plg->cloud, newcloud);

    PCL_DEBUG("RGB Triangle mesh: ");
    PCL_DEBUG("Mesh polygons: %ld", plg->polygons.size ());
    PCL_DEBUG("Mesh points: %ld", newcloud.points.size ());

    Eigen::Vector4f tmp;
    for(size_t i=0; i< plg->polygons.size (); ++i)
    { // each triangle/polygon
      pcl::Vertices apoly_in = plg->polygons[i];
      for(size_t j = 0; j < apoly_in.vertices.size (); ++j)
      { // each point
        uint32_t pt = apoly_in.vertices[j];
        tmp = newcloud.points[pt].getVector4fMap();
        vertices.push_back (Vertex (Eigen::Vector3f (tmp (0), tmp (1), tmp (2)),
                                    Eigen::Vector3f (newcloud.points[pt].r/255.0f,
                                                     newcloud.points[pt].g/255.0f,
                                                     newcloud.points[pt].b/255.0f)));
        indices.push_back (indices.size ());
      }
    }
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ> newcloud;
    pcl::fromPCLPointCloud2 (plg->cloud, newcloud);
    Eigen::Vector4f tmp;
    for(size_t i=0; i< plg->polygons.size (); i++)
    { // each triangle/polygon
      pcl::Vertices apoly_in = plg->polygons[i];
      for(size_t j=0; j< apoly_in.vertices.size (); j++)
      { // each point
        uint32_t pt = apoly_in.vertices[j];
        tmp = newcloud.points[pt].getVector4fMap();
        vertices.push_back (Vertex (Eigen::Vector3f (tmp (0), tmp (1), tmp (2)),
                                    Eigen::Vector3f (1.0, 1.0, 1.0)));
        indices.push_back (indices.size ());
      }
    }
  }

  PCL_DEBUG("Vertices: %ld", vertices.size ());
  PCL_DEBUG("Indices: %ld", indices.size ());

  glGenBuffers (1, &vbo_);
  glBindBuffer (GL_ARRAY_BUFFER, vbo_);
  glBufferData (GL_ARRAY_BUFFER, vertices.size () * sizeof (vertices[0]), &(vertices[0]), GL_STATIC_DRAW);
  glBindBuffer (GL_ARRAY_BUFFER, 0);

  glGenBuffers (1, &ibo_);
  glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, ibo_);
  glBufferData (GL_ELEMENT_ARRAY_BUFFER, indices.size () * sizeof (indices[0]), &(indices[0]), GL_STATIC_DRAW);
  glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, 0);

  if (indices.size () > std::numeric_limits<GLuint>::max ())
    PCL_THROW_EXCEPTION(PCLException, "Too many vertices");

  size_ = static_cast<GLuint>(indices.size ());
}

void
pcl::simulation::TriangleMeshModel::draw ()
{
  glEnable (GL_DEPTH_TEST);

  glEnableClientState (GL_VERTEX_ARRAY);
  glEnableClientState (GL_COLOR_ARRAY);
  //glEnableClientState(GL_NORMAL_ARRAY);
  glBindBuffer (GL_ARRAY_BUFFER, vbo_);
  glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, ibo_);

  glVertexPointer (3, GL_FLOAT, sizeof (Vertex), 0);
  glColorPointer (3, GL_FLOAT, sizeof (Vertex), reinterpret_cast<GLvoid*> (12));

  // glNormalPointer(GL_FLOAT, sizeof(Vertex), (GLvoid*)((char*)&(vertices_[0].norm)-(char*)&(vertices_[0].pos)));
  //  glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, 0);
  glDrawElements (GL_TRIANGLES, size_, GL_UNSIGNED_INT, 0);
  glDisableClientState (GL_VERTEX_ARRAY);
  glDisableClientState (GL_COLOR_ARRAY);
  //glDisableClientState(GL_NORMAL_ARRAY);
  glVertexPointer (3, GL_FLOAT, 0, 0);
  glColorPointer (3, GL_FLOAT, 0, 0);
  //glNormalPointer(GL_FLOAT, 0, 0);

  glBindBuffer (GL_ARRAY_BUFFER, 0);
  glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, 0);
}

pcl::simulation::TriangleMeshModel::~TriangleMeshModel ()
{
  if (glIsBuffer (vbo_) == GL_TRUE)
    glDeleteBuffers (1, &vbo_);

  if (glIsBuffer (ibo_) == GL_TRUE)
    glDeleteBuffers (1, &ibo_);
}

// Create a PolygonMeshModel by converting the PolygonMesh to our format
pcl::simulation::PolygonMeshModel::PolygonMeshModel (GLenum mode, pcl::PolygonMesh::Ptr plg) : mode_ (mode)
{
  bool found_rgb=false;
  for (size_t i=0; i<plg->cloud.fields.size () ;i++)
    if (plg->cloud.fields[i].name.compare ("rgb") == 0 || plg->cloud.fields[i].name.compare ("rgba") == 0)
      found_rgb = true;
  
  if (found_rgb)
  {
    pcl::PointCloud<pcl::PointXYZRGB> newcloud;  
    pcl::fromPCLPointCloud2 (plg->cloud, newcloud);
    Eigen::Vector4f tmp;
    for(size_t i = 0; i< plg->polygons.size (); i++)
    { // each triangle/polygon
      pcl::Vertices apoly_in = plg->polygons[i];
      SinglePoly apoly;
      apoly.nvertices_ = apoly_in.vertices.size ();
      apoly.vertices_ = new float[3*apoly_in.vertices.size ()];
      apoly.colors_ = new float[4*apoly_in.vertices.size ()]; 

      for(size_t j=0; j< apoly_in.vertices.size (); j++)
      { // each point
	uint32_t pt = apoly_in.vertices[j];
	tmp = newcloud.points[pt].getVector4fMap ();
	// x,y,z
	apoly.vertices_[3*j + 0] = tmp (0);
	apoly.vertices_[3*j + 1] = tmp (1);
	apoly.vertices_[3*j + 2] = tmp (2);
	// r,g,b: input is ints 0->255, opengl wants floats 0->1
	apoly.colors_[4*j + 0] = newcloud.points[pt].r/255.0f; // Red
	apoly.colors_[4*j + 1] = newcloud.points[pt].g/255.0f; // Green
	apoly.colors_[4*j + 2] = newcloud.points[pt].b/255.0f; // Blue
	apoly.colors_[4*j + 3] = 1.0f; // transparancy? unnecessary?
      }
      polygons.push_back (apoly);
    }
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ> newcloud;  
    pcl::fromPCLPointCloud2 (plg->cloud, newcloud);
    Eigen::Vector4f tmp;
    for(size_t i=0; i< plg->polygons.size (); i++)
    { // each triangle/polygon
      pcl::Vertices apoly_in = plg->polygons[i];
      SinglePoly apoly;
      apoly.nvertices_ = apoly_in.vertices.size ();
      apoly.vertices_ = new float [3*apoly_in.vertices.size ()];
      apoly.colors_ = new float [4*apoly_in.vertices.size ()];

      for(size_t j=0; j< apoly_in.vertices.size (); j++)
      { // each point
	uint32_t pt = apoly_in.vertices[j];
	tmp = newcloud.points[pt].getVector4fMap ();
	// x,y,z
	apoly.vertices_[3*j + 0] = tmp (0);
	apoly.vertices_[3*j + 1] = tmp (1);
	apoly.vertices_[3*j + 2] = tmp (2);
	// r,g,b: input is ints 0->255, opengl wants floats 0->1
	apoly.colors_[4*j + 0] = 1.0f; // Red
	apoly.colors_[4*j + 1] = 0.0f; // Green
	apoly.colors_[4*j + 2] = 0.0f; // Blue
	apoly.colors_[4*j + 3] = 1.0;
      }
      polygons.push_back (apoly);
    }
  }
}

pcl::simulation::PolygonMeshModel::~PolygonMeshModel ()
{
  // TODO: memory management!
  for (size_t i = 0; i < polygons.size (); ++i)
  {
    delete [] polygons[i].vertices_;
    delete [] polygons[i].colors_;
  }
}

void
pcl::simulation::PolygonMeshModel::draw ()
{
  // This might be a little quicker than drawing using individual polygons
  // TODO: test by how much
  glEnable (GL_DEPTH_TEST);
  glEnableClientState (GL_VERTEX_ARRAY);
  glEnableClientState (GL_COLOR_ARRAY);

  for (size_t i = 0; i < polygons.size (); i++)
  {
    glVertexPointer (3, GL_FLOAT, 0, polygons[i].vertices_);
    glColorPointer (4, GL_FLOAT, 0, polygons[i].colors_);
    glDrawArrays (mode_, 0, polygons[i].nvertices_);
  }
  glDisableClientState (GL_COLOR_ARRAY);
  glDisableClientState (GL_VERTEX_ARRAY);
}

pcl::simulation::PointCloudModel::PointCloudModel (GLenum mode, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) : mode_ (mode)
{
  nvertices_ = pc->points.size ();
  vertices_ = new float[3*nvertices_];
  colors_ = new float[4*nvertices_];

  for (size_t i = 0; i < pc->points.size (); ++i)
  {
    vertices_[3*i + 0] = pc->points[i].x;
    vertices_[3*i + 1] = pc->points[i].y;
    vertices_[3*i + 2] = pc->points[i].z;

    colors_[4*i + 0] = pc->points[i].r / 255.0f;
    colors_[4*i + 1] = pc->points[i].g / 255.0f;
    colors_[4*i + 2] = pc->points[i].b / 255.0f;
    colors_[4*i + 3] = 1.0;
  }
}

pcl::simulation::PointCloudModel::~PointCloudModel ()
{
  delete vertices_;
  delete colors_;
}

void
pcl::simulation::PointCloudModel::draw ()
{
  glEnable (GL_DEPTH_TEST);

  glEnableClientState (GL_VERTEX_ARRAY);
  glEnableClientState (GL_COLOR_ARRAY);

  float att[3] = {0.0f, 0.25f, 0.0f};
  glPointParameterf(GL_POINT_SIZE_MIN, 1.0f);
  glPointParameterf(GL_POINT_SIZE_MAX, 500.0f); 
  glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION, att);
  glEnable(GL_POINT_SPRITE);

  glVertexPointer (3, GL_FLOAT, 0, vertices_);
  glColorPointer (4, GL_FLOAT, 0, colors_);

  glDrawArrays (mode_, 0, nvertices_);

  glDisableClientState (GL_COLOR_ARRAY);
  glDisableClientState (GL_VERTEX_ARRAY);

  glDisable(GL_POINT_SPRITE);
}

pcl::simulation::Quad::Quad ()
{
  // vertex pos: xyz , texture coord: uv
  const static float vertices[20] = {-1.0, -1.0, 0.0, 0.0, 0.0,
                                     -1.0,  1.0, 0.0, 0.0, 1.0,
                                      1.0,  1.0, 0.0, 1.0, 1.0,
                                      1.0, -1.0, 0.0, 1.0, 0.0 };

  glGenBuffers (1, &quad_vbo_);
  glBindBuffer (GL_ARRAY_BUFFER, quad_vbo_);
  glBufferData (GL_ARRAY_BUFFER, sizeof (vertices), vertices, GL_STATIC_DRAW);
  glBindBuffer (GL_ARRAY_BUFFER, 0);
}

pcl::simulation::Quad::~Quad ()
{
  glDeleteBuffers (1, &quad_vbo_);
}

void
pcl::simulation::Quad::render ()
{
  glBindBuffer (GL_ARRAY_BUFFER, quad_vbo_);
  glEnableVertexAttribArray (0);
  glEnableVertexAttribArray (1);
  glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, sizeof (float)*5, 0);
  glVertexAttribPointer (1, 2, GL_FLOAT, GL_FALSE, sizeof (float)*5, (const GLvoid*)12);

  glDrawArrays (GL_QUADS, 0, 4);

  glDisableVertexAttribArray (1);
  glDisableVertexAttribArray (0);
  glBindBuffer (GL_ARRAY_BUFFER, 0);
}

pcl::simulation::TexturedQuad::TexturedQuad (int width, int height) : width_ (width), height_ (height)
{
  program_ = gllib::Program::loadProgramFromFile ("single_texture.vert", "single_texture.frag");
  program_->use ();
  Eigen::Matrix<float, 4, 4> MVP;
  MVP.setIdentity();
  program_->setUniform ("MVP", MVP);
  glUseProgram (0);

  glGenTextures (1, &texture_);
  glBindTexture (GL_TEXTURE_2D, texture_);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
  glBindTexture (GL_TEXTURE_2D, 0);
}

pcl::simulation::TexturedQuad::~TexturedQuad ()
{
  glDeleteTextures (1, &texture_);
}

void
pcl::simulation::TexturedQuad::setTexture (const uint8_t* data)
{
  glBindTexture (GL_TEXTURE_2D, texture_);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
  glBindTexture (GL_TEXTURE_2D, 0);
}

void
pcl::simulation::TexturedQuad::render ()
{
  program_->use ();
  glActiveTexture (GL_TEXTURE0);
  glBindTexture (GL_TEXTURE_2D, texture_);
  quad_.render ();
  glBindTexture (GL_TEXTURE_2D, 0);
  glUseProgram (0);
}
