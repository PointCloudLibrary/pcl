#ifndef PCL_GEOMETRY_POLYGON_OPERATIONS_HPP_
#define PCL_GEOMETRY_POLYGON_OPERATIONS_HPP_
#include <pcl/geometry/polygon_operations.h>

template<typename PointT> void
pcl::approximatePolygon (const PlanarPolygon<PointT>& polygon, PlanarPolygon<PointT>& approx_polygon, float threshold, bool refine, bool closed)
{
  const Eigen::Vector4f& coefficients = polygon.getCoefficients ();
  const std::vector<PointT, Eigen::aligned_allocator<PointT> >& contour = polygon.getContour ();
  
  Eigen::Vector3f rotation_axis = Eigen::Vector3f(coefficients[1], -coefficients[0], 0).normalized ();
  float rotation_angle = acos(coefficients [2]);
  Eigen::Affine3f transformation = Eigen::Translation3f (0, 0, coefficients [3]) * Eigen::AngleAxisf (rotation_angle, rotation_axis);

  std::vector<PointT> polygon2D (contour.size ());
  std::cout << "plane params:\n" << coefficients << std::endl;
  for (unsigned pIdx = 0; pIdx < polygon2D.size (); ++pIdx)
  {
    polygon2D [pIdx].getVector3fMap () = transformation * contour [pIdx].getVector3fMap ();
    //std::cout << contour [pIdx] << " -> " << polygon2D [pIdx] << std::endl;
  }
  //return;
  std::vector<PointT> approx_polygon2D;
  approximatePolygon2D (polygon2D, approx_polygon2D, threshold, refine, closed);
  
  std::vector<PointT, Eigen::aligned_allocator<PointT> >& approx_contour = approx_polygon.getContour ();
  approx_contour.resize (approx_polygon2D.size ());
  
  Eigen::Affine3f inv_transformation = transformation.inverse ();
  for (unsigned pIdx = 0; pIdx < approx_polygon2D.size (); ++pIdx)
  {
    //std::cout << "polygon point " << pIdx << " :: " << approx_polygon2D [pIdx] << std::endl;
    approx_contour [pIdx].getVector3fMap () = inv_transformation * approx_polygon2D [pIdx].getVector3fMap ();
  }
}

template <typename PointT> void
pcl::approximatePolygon2D (const std::vector<PointT>& polygon, std::vector<PointT>& approx_polygon, float threshold, bool refine, bool closed)
{
  approx_polygon.clear ();
  if (polygon.size () < 3)
    return;
  
  std::vector<std::pair<unsigned, unsigned> > intervals;
  std::pair<unsigned,unsigned> interval (0, 0);
  
  float max_distance = .0f;
  for (unsigned idx = 1; idx < polygon.size (); ++idx)
  {
    float distance = (polygon [0].x - polygon [idx].x) * (polygon [0].x - polygon [idx].x) + 
                     (polygon [0].y - polygon [idx].y) * (polygon [0].y - polygon [idx].y) ;
    
    if (distance > max_distance)
    {
      max_distance = distance;
      interval.second = idx;
    }
  }
  
  for (unsigned idx = 1; idx < polygon.size (); ++idx)
  {
    float distance = (polygon [interval.second].x - polygon [idx].x) * (polygon [interval.second].x - polygon [idx].x) + 
                     (polygon [interval.second].y - polygon [idx].y) * (polygon [interval.second].y - polygon [idx].y) ;
    
    if (distance > max_distance)
    {
      max_distance = distance;
      interval.first = idx;
    }
  }
  
  if (max_distance < threshold * threshold)
    return;
  
  intervals.push_back (interval);
  if (closed)
  {
    std::swap (interval.first, interval.second);
    intervals.push_back (interval);
  }
  
  std::vector<unsigned> result;
  // recursively refine
  while (!intervals.empty ())
  {
    std::pair<unsigned, unsigned>& currentInterval = intervals.back();
    float line_x = polygon [currentInterval.first].y - polygon [currentInterval.second].y;
    float line_y = polygon [currentInterval.second].x - polygon [currentInterval.first].x;
    float line_d = polygon [currentInterval.first].x * polygon [currentInterval.second].y - polygon [currentInterval.first].y * polygon [currentInterval.second].x;
    
    float linelen = 1.0f / sqrt (line_x * line_x + line_y * line_y);
    
    line_x *= linelen;
    line_y *= linelen;
    line_d *= linelen;
    
    float max_distance = 0.0;
    unsigned first_index = currentInterval.first + 1;
    unsigned max_index = 0;

    // => 0-crossing
    if( currentInterval.first > currentInterval.second )
    {
      for( unsigned idx = first_index; idx < polygon.size(); idx++ )
      {
        float distance = fabs( line_x * polygon[idx].x + line_y * polygon[idx].y + line_d );
        if( distance > max_distance )
        {
          max_distance = distance;
          max_index  = idx;
        }
      }
      first_index = 0;
    }

    for( int idx = first_index; idx < currentInterval.second; idx++ )
    {
      float distance = fabs( line_x * polygon[idx].x + line_y * polygon[idx].y + line_d );
      if( distance > max_distance )
      {
        max_distance = distance;
        max_index  = idx;
      }
    }

    if( max_distance > threshold )
    {
//      std::cout << currentInterval.first << " - " << currentInterval.second << " -> " 
//                << currentInterval.first << " - " << max_index << " + "
//                << max_index << " - " << currentInterval.second << std::endl;
      std::pair<unsigned, unsigned> interval (max_index, currentInterval.second);
      currentInterval.second = max_index;
      intervals.push_back (interval);
    }
    else
    {
      result.push_back (currentInterval.second);
      intervals.pop_back ();
    }
  }
  
  if (refine)
  {
    std::vector<Eigen::Vector3f> lines (result.size ());
    std::reverse (result.begin (), result.end ());
    for (unsigned rIdx = 0; rIdx < result.size (); ++rIdx)
    {
      unsigned nIdx = rIdx + 1;
      if (nIdx == result.size ())
        nIdx = 0;
      
      Eigen::Matrix3f covar = Eigen::Matrix3f::Zero ();
      unsigned pIdx = result[rIdx];
      if (pIdx > result[nIdx])
      {
        for (; pIdx < polygon.size (); ++pIdx)
        {
          covar.coeffRef (0) += polygon [pIdx].x * polygon [pIdx].x;
          covar.coeffRef (1) += polygon [pIdx].x * polygon [pIdx].y;
          covar.coeffRef (2) += polygon [pIdx].x;
          covar.coeffRef (4) += polygon [pIdx].y * polygon [pIdx].y;
          covar.coeffRef (5) += polygon [pIdx].y;
          covar.coeffRef (8) += 1.0;
        }
        pIdx = 0;
      }
      
      for (; pIdx < result[nIdx]; ++pIdx)
      {
        covar.coeffRef (0) += polygon [pIdx].x * polygon [pIdx].x;
        covar.coeffRef (1) += polygon [pIdx].x * polygon [pIdx].y;
        covar.coeffRef (2) += polygon [pIdx].x;
        covar.coeffRef (4) += polygon [pIdx].y * polygon [pIdx].y;
        covar.coeffRef (5) += polygon [pIdx].y;
        covar.coeffRef (8) += 1.0;
      }
      
      covar.coeffRef (3) = covar.coeff (1);
      covar.coeffRef (6) = covar.coeff (2);
      covar.coeffRef (7) = covar.coeff (5);

      float eval;
      eigen33 (covar, eval, lines [rIdx]);
      
      // need normalized later to find almost parallel lines
      lines [rIdx] /= sqrt (lines [rIdx][0] * lines [rIdx][0] + lines [rIdx][1] * lines [rIdx][1]);
    }
    
    approx_polygon.resize (result.size ());
    const float angle_threshold_ = 0.966f; // = cos (165deg)
    for (unsigned rIdx = 0; rIdx < lines.size (); ++rIdx)
    {
      unsigned nIdx = rIdx + 1;
      if (nIdx == result.size ())
        nIdx = 0;      
      
      // if almost parallel
      if (lines [rIdx][0] * lines [nIdx][0] + lines [rIdx][1] * lines [nIdx][1] > angle_threshold_)
      {
        approx_polygon[nIdx] = polygon [result[nIdx]];
      }
      else
      {
        Eigen::Vector3f vertex = lines [rIdx].cross (lines [nIdx]);
        vertex /= vertex [2];
        vertex [2] = 0.0;
        approx_polygon[nIdx].getVector3fMap () = vertex;
      }
    }
  }
  else
  {
    // we have a new polygon in results, but inverted (clockwise <-> counter-clockwise)
    approx_polygon.reserve (result.size ());
    for (std::vector<unsigned>::reverse_iterator it = result.rbegin (); it != result.rend (); ++it)
      approx_polygon.push_back (polygon [*it]);
  }
}
#endif // PCL_GEOMETRY_POLYGON_OPERATIONS_HPP_