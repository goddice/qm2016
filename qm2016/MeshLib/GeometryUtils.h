#ifndef GEOMETRYUTILS_H
#define GEOMETRYUTILS_H

#include "Point.h"

#include <memory>

#include "Front.h"
#include "HalfEdge.h"

class HalfEdge;
namespace GeometryUtils {

     Point arbitrary_inside_point(std::shared_ptr<Front> front);

     double angle(std::shared_ptr<HalfEdge> prev, std::shared_ptr<HalfEdge> next);
     double angle(const Point& p, const Point& o, const Point& q);

     inline double rad_to_deg(double rad) {
          return rad * 57.295779513224;
     }

     //return 0: non-intersected
     //       1: intersected
     //       2: overlapped
     int is_intersected(const Point& p0, const Point& p1, const Point& q0, const Point &q1, Point& I );


     double compute_angle(std::shared_ptr<HalfEdge> inedge, std::shared_ptr<HalfEdge> outedge);

     double length(std::shared_ptr<HalfEdge> he);


     Point rotate(std::shared_ptr<HalfEdge> he, double angle, double length = 1.0);
}

#endif /* GEOMETRYUTILS_H */
