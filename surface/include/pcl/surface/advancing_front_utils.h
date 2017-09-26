#ifndef PCL_SURFACE_ADVANCING_FRONT_UTILS_H_
#define PCL_SURFACE_ADVANCING_FRONT_UTILS_H_

#include <pcl/surface/advancing_front_point_type.h>
#include <pcl/point_types.h>

namespace pcl
{
  namespace afront
  {
    struct DistPoint2LineResults
    {
      Eigen::Vector3f points[3]; /**< \brief The points. Note: Seg = (points[1] - points[0]) */
      Eigen::Vector3f p;         /**< \brief The closest point on line segment. */
      float mu;                  /**< \brief The corresponding line paramentric value resulting in the minimum distance. */
      double d;                  /**< \brief The distance between point and line segments */
    };

    /**
      * \brief Get the distance between a point and line
      * \param[in] p1 Origin point of the line
      * \param[in] p2 Terminating point of the line
      * \param[in] p The point for calulating the distance to the line
      * \return DistPoint2LineResults
      */
    DistPoint2LineResults
    distPoint2Line (const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p)
    {
      DistPoint2LineResults results;

      results.points[0] = p1;
      results.points[1] = p2;
      results.points[2] = p;

      Eigen::Vector3f v = results.points[1] - results.points[0];
      Eigen::Vector3f w = p - results.points[0];

      double c1 = w.dot (v);
      double c2 = v.dot (v);

      if (c1 <= 0)
        results.mu = 0.0;
      else if (c2 <= c1)
        results.mu = 1.0;
      else
        results.mu = c1 / c2;

      results.p = results.points[0] + results.mu * v;
      results.d = (p - results.p).norm ();

      return results;
    }

    struct DistLine2LineResults
    {
      Eigen::Vector3f points[4]; /**< \brief The line segment points. Note: Seg1 = (points[1] - points[0]) and Seg2 = (points[3] - points[2]) */
      Eigen::Vector3f p[2];      /**< \brief The closest point on each line segment. Note: p[0] is the closet point on Seg1. */
      float mu[2];               /**< \brief The corresponding line paramentric value resulting in the minimum distance. */
      double d;                  /**< \brief The distance between both line segments */
      bool parallel;             /**< \brief Indicate that both lines are parallel. */
    };

    /**
      * \brief Distance between two line segments
      *
      * Follow the link for more details http://paulbourke.net/geometry/pointlineplane/
      * \param[in] p1 Origin point of the first line
      * \param[in] p2 Terminating point of the first line
      * \param[in] p3 Origin point of the second line
      * \param[in] p4 Terminating point of the second line
      * \return DistLine2LineResults
      */
    DistLine2LineResults
    distLine2Line (const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, const Eigen::Vector3f &p4)
    {
      DistLine2LineResults result;
      result.points[0] = p1;
      result.points[1] = p2;
      result.points[2] = p3;
      result.points[3] = p4;

      double d1321 = (p1 - p3).dot ((p2 - p1));
      double d2121 = (p2 - p1).dot ((p2 - p1));
      double d4321 = (p4 - p3).dot ((p2 - p1));
      double d1343 = (p1 - p3).dot ((p4 - p3));
      double d4343 = (p4 - p3).dot ((p4 - p3));

      double denom = d2121 * d4343 - d4321 * d4321;
      if (denom < 1.0e-8) // The lines are parallel
      {
        DistPoint2LineResults temp, dist;
        result.parallel = true;

        dist = distPoint2Line (p1, p2, p3);
        result.mu[0] = dist.mu;

        temp = distPoint2Line (p1, p2, p4);
        if (temp.mu < dist.mu)
          result.mu[0] = temp.mu;

        result.p[0] = p1 + result.mu[0] * (p2 - p1);
        temp = distPoint2Line (p3, p4, result.p[0]);

        result.mu[1] = temp.mu;
        result.p[1] = p3 + result.mu[1] * (p4 - p3);
      }
      else
      {
        result.mu[0] = (d1343 * d4321 - d1321 * d4343) / denom;
        result.mu[1] = (d1343 + result.mu[0] * d4321) / d4343;

        if (result.mu[0] > 1.0)
          result.mu[0] = 1.0;
        else if (result.mu[0] < 0.0)
          result.mu[0] = 0.0;

        if (result.mu[1] > 1.0)
          result.mu[1] = 1.0;
        else if (result.mu[1] < 0.0)
          result.mu[1] = 0.0;

        result.p[0] = p1 + result.mu[0] * (p2 - p1);
        result.p[1] = p3 + result.mu[1] * (p4 - p3);
      }

      result.d = (result.p[1] - result.p[0]).norm ();
      return result;
    }

    struct IntersectionLine2PlaneResults
    {
      Eigen::Vector3f points[2]; /**< \brief The points defining the line. */
      Eigen::Vector3f w;         /**< \brief The vector defining the line direction. */
      Eigen::Vector3f origin;    /**< \brief The origin of the plane. */
      Eigen::Vector3f u;         /**< \brief The vector defining the planes relative u directions. */
      Eigen::Vector3f v;         /**< \brief The vector defining the planes relative v directions. */
      Eigen::Vector3f p;         /**< \brief The point of intersection p = origin + x * u + y * v  || p = points[0] + w * lw */
      float mw;                  /**< \brief The parametric coeff defining the intersection location on the line. */
      float mu;                  /**< \brief The parametric plane u coeff of intersetion. */
      float mv;                  /**< \brief The parametric plane v coeff of intersetion. */
      bool parallel;             /**< \brief Indicate whether the line is parallel to the plane. */
    };

    /**
      * \brief Find the intersection between a plane and line
      * \param[in] p1     The origin point of the line
      * \param[in] p2     The terminating point of the line
      * \param[in] origin The origin of the plane
      * \param[in] u      The vector defining the u direction for the plane
      * \param[in] v      The vector defining the v direction for the plane
      * \return IntersectionLine2PlaneResults
      */
    IntersectionLine2PlaneResults
    intersectionLine2Plane (const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &origin, const Eigen::Vector3f &u, const Eigen::Vector3f &v)
    {
      IntersectionLine2PlaneResults results;
      results.points[0] = p1;
      results.points[1] = p2;
      results.w = p2 - p1;
      results.parallel = false;

      results.origin = origin;
      results.u = u;
      results.v = v;
      Eigen::Vector3f normal = u.cross (v).normalized ();

      if (std::abs (normal.dot (results.w.normalized ())) < 1.0e-8)
      {
        results.parallel = true;
      }
      else
      {
        Eigen::Matrix3f m;
        m << u, v, -results.w;

        Eigen::Vector3f t = p1 - origin;
        Eigen::Vector3f muvw = m.lu ().solve (t);
        results.mu = muvw[0];
        results.mv = muvw[1];
        results.mw = muvw[2];

        results.p = results.points[0] + results.mw * results.w;
      }
      return results;
    }

  } // namespace afront

} // namespace pcl
#endif // PCL_SURFACE_ADVANCING_FRONT_UTILS_H_
