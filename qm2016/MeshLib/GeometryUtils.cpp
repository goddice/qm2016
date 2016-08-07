#include "GeometryUtils.h"
#include "IO.h"

namespace GeometryUtils {

        Point arbitrary_inside_point(std::shared_ptr<Front> front) {
                CHECK(front->halfedge_size() > 1);
                auto v0 = front->halfedge(0)->source();
                auto v1 = front->halfedge(1)->target();
                CHECK(v0 != v1);
                return (v0->point() + v1->point()) / 2.0;
        }


        double Cross(const Point& p, const Point& q) {
                return p[0] * q[1] - p[1] * q[0];
        }

        bool isZero(const Point& p) {
                return fabs(p.norm()) < 1e-6;
        }

        bool isZero(double v) {
                return fabs(v) < 1e-6;
        }

        int is_intersected(const Point& p, const Point& p2, const Point& q, const Point &q2, Point &I)
        {

                auto r = p2 - p;
                auto s = q2 - q;
                auto rxs = Cross(r, s);
                auto qpxr = (q - p) ^ r;

                // If r x s = 0 and (q - p) x r = 0, then the two lines are collinear.
                if (isZero(rxs) && isZero(qpxr))
                {
                        // 1. If either  0 <= (q - p) * r <= r * r or 0 <= (p - q) * s <= * s
                        // then the two lines are overlapping,
                        //if (considerCollinearOverlapAsIntersect)
                        if ((0 <= (q - p)*r && (q - p)*r <= r*r) || (0 <= (p - q)*s && (p - q)*s <= s*s))
                                return 2;

                        // 2. If neither 0 <= (q - p) * r = r * r nor 0 <= (p - q) * s <= s * s
                        // then the two lines are collinear but disjoint.
                        // No need to implement this expression, as it follows from the expression above.
                        return 0;
                }

                // 3. If r x s = 0 and (q - p) x r != 0, then the two lines are parallel and non-intersecting.
                if (isZero(rxs) && !isZero(qpxr))
                        return 0;

                // t = (q - p) x s / (r x s)
                auto t = Cross((q - p), (s))/rxs;

                // u = (q - p) x r / (r x s)

                auto u = Cross((q - p), (r))/rxs;

                // 4. If r x s != 0 and 0 <= t <= 1 and 0 <= u <= 1
                // the two line segments meet at the point p + t r = q + u s.
                if (!isZero(rxs) && (0 <= t && t <= 1) && (0 <= u && u <= 1))
                {
                        // We can calculate the intersection point using either t or u.
                        I = p + r * t;

                        // An intersection was found.
                        return 1;
                }

                // 5. Otherwise, the two line segments are not parallel but do not intersect.
                return 0;
        }


        // struct point_t rotate_around_from(const struct paving_halfedge_t* edge, double angle) {
        //         struct point_t p;
        //         struct point_t to = edge->to->point;

        //         struct point_t from = edge->from->point;
        //         p.x = (to.x - from.x) * cos(angle) - (to.y - from.y) * sin(angle) ;
        //         p.y = (to.y - from.y) * cos(angle) + (to.x - from.x) * sin(angle) ;

        //         //struct point_t ptmp = normalize(&p);
        //         struct point_t r =  add(&p, &from);

        //         return r;
        // }




        //return the angle from halfedge (op) rotate to (oq) in CCW direction
        double angle(const Point& p, const Point& o, const Point& q) {
                Point v1 = p - o;
                Point v2 = q - o;
                double ang = atan2(v2[1], v2[0]) - atan2(v1[1], v1[0]);
                //        LOG_D("ang: %f", GeometryUtils::rad_to_deg(ang));
                return ang < 0 ? ang + 2 * M_PI : ang;
        }

//return the angle from halfedge prev rotate to next in CCW direction.
        double angle(std::shared_ptr<HalfEdge> prev, std::shared_ptr<HalfEdge> next) {
                CHECK(prev->target() == next->source());
                Point v1 = prev->target()->point() - prev->source()->point();
                Point v2 = next->target()->point() - next->source()->point();
                double ang = atan2(v2[1], v2[0]) - atan2(v1[1], v1[0]);
                return ang < 0 ? ang + 2 * M_PI : ang;

        }

        double compute_angle(std::shared_ptr<HalfEdge> inedge, std::shared_ptr<HalfEdge> outedge)
        {

                double theta = angle(inedge, outedge);
                //struct point_t verify_pt = rotate_around_from(outedge, theta);
                // double delta = distance(&verify_pt, &inedge->from->point) - fabs(length(outedge) - length(inedge));
                // if (delta < 1e-3) {
                //         return theta;
                // } else {
                //         // printf("adjust: \n");
                //         return 2 * M_PI - theta;
                // }

                return theta;

        }



        double length(std::shared_ptr<HalfEdge> he) {
                return (he->vertex(1)->point() - he->vertex(0)->point()).norm();
        }


        Point rotate(std::shared_ptr<HalfEdge> he, double angle, double length) {

                double vec_x = he->target()->point()[0] - he->source()->point()[0];
                double vec_y = he->target()->point()[1] - he->source()->point()[1];

                double vec_len = sqrt(vec_x * vec_x + vec_y * vec_y);

                double dir_x = vec_x / vec_len;
                double dir_y = vec_y / vec_len;

                double new_x = (dir_x * cos(angle) - dir_y * sin(angle)) * length + he->source()->point()[0];
                double new_y = (dir_x * sin(angle) + dir_y * cos(angle)) * length + he->source()->point()[1];

                if (fabs(new_x) < 1e-12 ) {
                        new_x = 0;
                }

                if (fabs(new_y) < 1e-12 ) {
                        new_y = 0;
                }

                return Point(new_x, new_y, 0);
        }


}
