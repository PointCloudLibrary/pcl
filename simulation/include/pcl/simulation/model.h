#ifndef PCL_MODEL_HPP_
#define PCL_MODEL_HPP_

#if defined(_WIN32) && !defined(APIENTRY) && !defined(__CYGWIN__)
# define WIN32_LEAN_AND_MEAN 1
# include <windows.h>
#endif

#include <GL/glew.h>

#include <pcl/pcl_config.h>
#ifdef OPENGL_IS_A_FRAMEWORK
# include <OpenGL/gl.h>
# include <OpenGL/glu.h>
#else
# include <GL/gl.h>
# include <GL/glu.h>
#endif

#include <boost/shared_ptr.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/simulation/glsl_shader.h>

namespace pcl
{
  namespace simulation
  {
    typedef struct _SinglePoly
    {
      float* vertices_;
      float* colors_;
      GLenum mode_;
      GLuint nvertices_;
    } SinglePoly;

    struct Vertex
    {
      Vertex () {}
      //Vertex(Eigen::Vector3f pos, Eigen::Vector3f norm) : pos(pos), norm(norm) {}
      Vertex (Eigen::Vector3f pos, Eigen::Vector3f rgb) : pos (pos), rgb (rgb) {}
      Eigen::Vector3f pos;
      Eigen::Vector3f rgb;
      //Eigen::Vector3f norm;
      //Eigen::Vector2f tex;
    };

    struct Face
    {
      // Index int to the index list
      unsigned int index_offset;
      // Number of vertices on face
      unsigned int count;
      // Normal of face
      Eigen::Vector3f norm;
    };

    typedef std::vector<Vertex> Vertices;
    typedef std::vector<size_t> Indices;

    class Model
    {
      public:
        virtual void draw () = 0;

        typedef boost::shared_ptr<Model> Ptr;
        typedef boost::shared_ptr<const Model> ConstPtr;
    };

    class PCL_EXPORTS TriangleMeshModel : public Model
    {
      public:
        typedef boost::shared_ptr<TriangleMeshModel> Ptr;
        typedef boost::shared_ptr<const TriangleMeshModel> ConstPtr;

        TriangleMeshModel (pcl::PolygonMesh::Ptr plg);

        virtual
        ~TriangleMeshModel ();

        virtual void
        draw ();

      private:
        GLuint vbo_;
        GLuint ibo_;
        GLuint size_;
        //Vertices vertices_;
        //Indices indices_;
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
        typedef boost::shared_ptr<PointCloudModel> Ptr;
        typedef boost::shared_ptr<const PointCloudModel> ConstPtr;

        PointCloudModel (GLenum mode, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);

        virtual
        ~PointCloudModel ();

        virtual void
        draw();

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
    class PCL_EXPORTS Quad
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

    class PCL_EXPORTS TexturedQuad
    {
      public:
        typedef boost::shared_ptr<TexturedQuad> Ptr;
        typedef boost::shared_ptr<const TexturedQuad> ConstPtr;

        TexturedQuad (int width, int height);
        ~TexturedQuad ();

        void
        setTexture (const uint8_t* data);

        void
        render ();
  
      private:
        int width_;
        int height_;
        Quad quad_;
        GLuint texture_;
        gllib::Program::Ptr program_;
    };
  } // namespace - simulation
} // namespace - pcl

#endif /* PCL_SIMULATION_MODEL_HPP_ */
