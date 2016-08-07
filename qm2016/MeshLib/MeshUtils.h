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

 void sampling(std::shared_ptr<Mesh<Vertex, Polygon>> mesh, double sample_rate);






}

#endif /* MESHUTILS_H */
