#ifndef MESHUTILS_H
#define MESHUTILS_H

#include <cmath>
#include <vector>
#include <memory>

#include "Front.h"
#include "GeometryUtils.h"

class HalfEdge;

template <typename V, template<typename> typename P>
class Mesh;

template <typename V> class Polygon;

namespace MeshUtils {


    std::vector<std::shared_ptr<Front>> connect_front(const std::vector<std::shared_ptr<HalfEdge>> &edges);

    template <typename V, template<typename>typename P>
        void remove_dup_vertices(std::shared_ptr<Mesh<V, P>> mesh);

    template <typename V, template<typename>typename P>
        void remove_unused_vertices(std::shared_ptr<Mesh<V, P>> mesh);

    template <typename V, template<typename> typename P>
        void reorder(std::shared_ptr<Mesh<V, P>> mesh);

    template<typename V, template<typename>typename P>
        int number_of_connected_components(std::vector<std::shared_ptr<P<V>>> polygons);

    template<typename V>
    std::shared_ptr<Polygon<V>> merge(std::vector<std::shared_ptr<Polygon<V>>> polygons);

    template<typename V>
        std::shared_ptr<Polygon<V>> simplify_boundary(std::shared_ptr<Polygon<V>> polygon);


    template<typename V, template<typename> typename P>
    std::vector<std::shared_ptr<HalfEdge>> generate_halfedges(std::shared_ptr<P<V>> polygon);

    bool is_loop(const std::vector<std::shared_ptr<HalfEdge>> & halfedges);

    bool reorder_loop(std::vector<std::shared_ptr<HalfEdge>> & halfedges);



    Point compute_normal(const Point& from, const Point& to);
}




#include "Log.h"
#include "HalfEdge.h"
#include "Mesh.h"
#include "Polygon.h"
#include "Vertex.h"
#include "IO.h"
#include <map>
#include <set>
#include <queue>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

namespace MeshUtils {
    template <typename V, template<typename> typename P>
    int number_of_connected_components(std::vector<std::shared_ptr<P<V>>> polygons) {
        std::unordered_map<std::shared_ptr<V>, std::vector<std::shared_ptr<P<V>>>> vertex_to_polygon_map;

        for (auto p : polygons) {

            for (typename P<V>::VertexIterator viter(p); !viter.end(); ++viter) {
                auto v = *viter;
                vertex_to_polygon_map[v].push_back(p);
            }
        }

        std::vector< std::set<std::shared_ptr<P<V>>> > cc_clusters;
        std::unordered_set<std::shared_ptr<P<V>>> visited;

        for (auto p : polygons) {
            if (visited.find(p) != visited.end()) {
                continue;
            }

            std::set<std::shared_ptr<P<V>>> cc_polys;
            std::queue<std::shared_ptr<P<V>>> queue;

            queue.push(p);

            while (!queue.empty()) {
                auto current_polygon = queue.front();
                queue.pop();
                visited.insert(current_polygon);
                cc_polys.insert(current_polygon);


                for (typename P<V>::VertexIterator viter(current_polygon); !viter.end(); ++viter) {
                    auto v = *viter;

                    auto pvec = vertex_to_polygon_map[v];

                    for (auto pp : pvec) {
                        if (visited.find(pp) == visited.end()) {
                            queue.push(pp);
                            visited.insert(pp);
                        }
                    }

                }
            }


            cc_clusters.push_back(cc_polys);

        }


        return cc_clusters.size();
    }



template <typename V, template<typename> typename P>
    std::vector< std::set<std::shared_ptr<P<V>>>> connected_components(std::vector<std::shared_ptr<P<V>>> polygons) {
        std::unordered_map<std::shared_ptr<V>, std::vector<std::shared_ptr<P<V>>>> vertex_to_polygon_map;

        for (auto p : polygons) {
            for (typename P<V>::VertexIterator viter(p); !viter.end(); ++viter) {
                auto v = *viter;
                vertex_to_polygon_map[v].push_back(p);
            }
        }

        std::vector< std::set<std::shared_ptr<P<V>>> > cc_clusters;
        std::unordered_set<std::shared_ptr<P<V>>> visited;

        for (auto p : polygons) {
            if (visited.find(p) != visited.end()) {
                continue;
            }

            std::set<std::shared_ptr<P<V>>> cc_polys;
            std::queue<std::shared_ptr<P<V>>> queue;

            queue.push(p);

            while (!queue.empty()) {
                auto current_polygon = queue.front();
                queue.pop();
                visited.insert(current_polygon);
                cc_polys.insert(current_polygon);

                for (typename P<V>::VertexIterator viter(current_polygon); !viter.end(); ++viter) {
                    auto v = *viter;

                    auto pvec = vertex_to_polygon_map[v];

                    for (auto pp : pvec) {
                        if (visited.find(pp) == visited.end()) {
                            queue.push(pp);
                            visited.insert(pp);
                        }
                    }

                }
            }


            cc_clusters.push_back(cc_polys);

        }


        return cc_clusters;
    }


template<typename V, template<typename> typename P>
std::shared_ptr<P<V>> merge(std::vector<std::shared_ptr<P<V>>> polygons) {
        auto merged_polygon = std::make_shared<P<V>>();

        if (number_of_connected_components(polygons) != 1) {
            LOG_W ("not in 1 connected component! #cc: %d", number_of_connected_components(polygons));
            //return merged_polygon;
        }

        std::map<std::pair<std::shared_ptr<V>, std::shared_ptr<V>>, std::shared_ptr<HalfEdge>> vpair_he_map;

        for (auto p : polygons) {
            auto halfedges = MeshUtils::generate_halfedges<V, P>(p);

            for (auto he : halfedges) {
                auto vp = std::make_pair(he->target(), he->source());

                if ((he->source()->id() == 98 && he->target()->id() == 99) ||
                    (he->source()->id() == 99 && he->target()->id() == 98 ) ){
                    LOG_D("** 98 -- 99: %d", p->id());

                }

                if (vpair_he_map.find(vp) != vpair_he_map.end()) {
                    vpair_he_map.erase(vp);
                } else {
                    //find the twin half edge
                    std::swap(vp.first, vp.second);
                    if (vpair_he_map.find(vp) != vpair_he_map.end()) {

                        for (auto dp : polygons) {
                            LOG_D("%d", dp->id());
                        }
                            DIE("%s exist!", IO::to_string(he).c_str());
                    }
                    vpair_he_map[vp] = he;
                }

            }
        }



        std::vector<std::shared_ptr<HalfEdge>> boundary_halfedges;
        for (auto kv : vpair_he_map) {
            auto he = kv.second;
            boundary_halfedges.push_back(he);
        }


        reorder_loop(boundary_halfedges);

        for (auto he : boundary_halfedges) {
            merged_polygon->add_vertex(he->source());
        }

        return merged_polygon;
    }


template<typename V, template<typename> typename P>
std::vector<std::shared_ptr<HalfEdge>> generate_halfedges(std::shared_ptr<P<V>> polygon) {

    std::vector<std::shared_ptr<HalfEdge>> halfedges;

    for (typename P<V>::VertexIterator viter(polygon); !viter.end(); ++viter) {
        auto pit = viter;
        ++pit;
        if (pit.end()) {
            pit.rewind();
        }

        auto he = std::make_shared<HalfEdge>(*viter, *pit);
        halfedges.push_back(he);
    }

    return halfedges;
}


inline double area(const Point &a, const Point& b, const Point& c) {

    return ((b - a)^(c -a)).norm() / 2.0;

}

template<typename V, template<typename> typename P>
    bool point_location(std::shared_ptr<Mesh<V, P>> mesh, const Point& point, double (&bc)[3]) {
    for (typename Mesh<V,P>::PolygonIterator piter(mesh); !piter.end(); ++piter) {
        auto p = *piter;
        Point pts[3] = {p->vertex(0)->point(), p->vertex(1)->point(), p->vertex(2)->point()};

        double area_tri = area(pts[0], pts[1], pts[2]);

        double cbc[3] = {
            area(point, pts[1], pts[2]) / area_tri,
            area(pts[0], point, pts[2]) / area_tri,
            area(pts[0], pts[1], point) / area_tri,
        };

        if (fabs (cbc[0] + cbc[1] + cbc[2] - 1.0) > 1e-8) {
            continue;
        }

        if (cbc[0] < 0 || cbc[1] < 0 || cbc[2] < 0) {
            continue;
        }

        bc[0] = cbc[0], bc[1] = cbc[1], bc[2] = cbc[2];
        return true;
    }
    return false;
}


template<typename V, template<typename> typename P>
    void bounding_box(std::shared_ptr<Mesh<V,P>> mesh, Point& min_point, Point& max_point) {

    min_point = Point(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());

    max_point = Point(std::numeric_limits<float>::lowest(),std::numeric_limits<float>::lowest(),std::numeric_limits<float>::lowest());

//    std::cout << "min: " << min_point[0] << " " << min_point[1] << " " << min_point[2] << "\n";
    //  std::cout << "max: " << max_point[0] << " " << max_point[1] << " " << max_point[2] << "\n";


    for (typename Mesh<V,P>::VertexIterator viter(mesh); !viter.end(); ++viter) {
        auto v = *viter;

        auto p = v->point();
        for (int i = 0; i < 3; ++i) {
            if (p[i] < min_point[i]) {
                min_point[i] = p[i];
            }
            if (p[i] > max_point[i]) {
                max_point[i] = p[i];
            }
        }

    }
}


template<typename V>
    std::shared_ptr<Polygon<V>> simplify_boundary(std::shared_ptr<Polygon<V>> polygon) {

    auto halfedges = generate_halfedges(polygon);

    auto simp_polygon = std::make_shared<Polygon<V>>();

    int he_size = halfedges.size();
    for (int i = 0; i < he_size; ++i) {

        auto prev = halfedges[i];
        auto next = halfedges[(i+1) % he_size];


        double ang = GeometryUtils::angle(prev, next);
        //std::cout << "cos(ang) :" << cos(ang) << " fabs(cos(ang) - 1) = " << fabs(cos(ang) - 1) << "\n";
        if (fabs(cos(ang) - 1) > 1e-6) {
            //    std::cout << "push " << prev->target()->id() << "\n";
            simp_polygon->add_vertex(prev->target());
        }
    }

    return simp_polygon;
}

 void sampling(std::shared_ptr<Mesh<Vertex, Polygon>> mesh, double sample_rate);






}

#endif /* MESHUTILS_H */
