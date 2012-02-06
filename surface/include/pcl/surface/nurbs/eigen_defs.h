#ifndef _NURBS_DEFS_H_
#define _NURBS_DEFS_H_

#undef Success
#include <Eigen/StdVector>
#include <list>
#include <vector>

namespace pcl
{
  namespace nurbs
  {

    typedef Eigen::Vector2f vec2f;
    typedef Eigen::Vector3f vec3f;
    typedef Eigen::Vector4f vec4f;
    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > vector_vec2f;
    typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > vector_vec3f;
    typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > vector_vec4f;
    typedef std::list<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > list_vec2f;
    typedef std::list<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > list_vec3f;
    typedef std::list<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > list_vec4f;

    typedef Eigen::Vector2d vec2d;
    typedef Eigen::Vector3d vec3d;
    typedef Eigen::Vector4d vec4d;
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vector_vec2d;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vector_vec3d;
    typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > vector_vec4d;
    typedef std::list<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > list_vec2d;
    typedef std::list<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > list_vec3d;
    typedef std::list<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > list_vec4d;

    typedef vec2d vec2;
    typedef vec3d vec3;
    typedef vec4d vec4;
    typedef vector_vec2d vector_vec2;
    typedef vector_vec3d vector_vec3;
    typedef vector_vec4d vector_vec4;
    typedef list_vec2d list_vec2;
    typedef list_vec3d list_vec3;
    typedef list_vec4d list_vec4;

  }
}

#endif
