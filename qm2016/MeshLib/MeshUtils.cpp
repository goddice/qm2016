#include "MeshUtils.h"

namespace MeshUtils {



    //TODO: remove dup vertices
    template <typename V, template<typename> typename P>
    void remove_dup_vertices(std::shared_ptr<Mesh<V, P>> mesh) {

    }


    template <typename V, template<typename> typename P>
    void remove_unused_vertices(std::shared_ptr<Mesh<V, P>> mesh) {
        std::unordered_map<std::shared_ptr<V>, std::vector<std::shared_ptr<P<V>>>> vertex_polygon_map;

        for (typename Mesh<V, P>::PolygonIterator piter(mesh); !piter.end(); ++piter) {
            auto polygon = *piter;
            for (typename Polygon<V>::VertexIterator viter(polygon); !viter.end(); ++viter) {
                vertex_polygon_map[*viter].push_back(polygon);
            }
        }


        std::vector<std::shared_ptr<V>> deleted_vertices;
        for (typename Mesh<V, P>::VertexIterator viter(mesh); !viter.end(); ++viter) {
            auto v = *viter;
            if (vertex_polygon_map.find(v) == vertex_polygon_map.end()) {
                deleted_vertices.push_back(v);
            }
        }

        for (auto v : deleted_vertices) {
            mesh->delete_vertex(v);
        }
    }


    //TODO: reorder the mesh

    template <typename V, template<typename> typename P>
    void reorder(std::shared_ptr<Mesh<V, P>> mesh) {

    }





    bool is_loop(const std::vector<std::shared_ptr<HalfEdge>> & halfedges) {
        std::unordered_map<std::shared_ptr<HalfEdge>, bool> visited;

        std::unordered_map<std::shared_ptr<Vertex>, std::shared_ptr<HalfEdge>> source_map;

        //construct the source map: source -> he
        //union and find
        for (int i = 0; i < halfedges.size(); ++i) {
            auto he = halfedges[i];
            if (source_map.find(he->source()) != source_map.end()) {
                LOG_W("%s conflicts with %s", IO::to_string(he).c_str(),
                      IO::to_string(source_map[he->source()]).c_str()
                    );
                return false;
            }
            source_map[he->source()] = he;
        }


        std::vector<std::shared_ptr<HalfEdge>> hes;
        for (int i = 0; i < halfedges.size(); ++i) {

            auto he = halfedges[i];
            auto start_he = he;

            if (visited.find(start_he) != visited.end()) {
                continue;
            }


            do {
                CHECK(visited.find(he) == visited.end());
                visited[he] = true;
                hes.push_back(he);

                he = source_map[he->target()];
            } while (he && he != start_he);

            if (!he) {
                break;
            }

        }

        return hes.size() == halfedges.size();

    }


    bool reorder_loop(std::vector<std::shared_ptr<HalfEdge>> & halfedges) {

        std::unordered_map<std::shared_ptr<HalfEdge>, bool> visited;

        std::unordered_map<std::shared_ptr<Vertex>, std::shared_ptr<HalfEdge>> source_map;

        //construct the source map: source -> he
        //union and find
        for (int i = 0; i < halfedges.size(); ++i) {
            auto he = halfedges[i];
            if (source_map.find(he->source()) != source_map.end()) {
                LOG_W("%s conflicts with %s", IO::to_string(he).c_str(),
                      IO::to_string(source_map[he->source()]).c_str()
                    );
                return false;
            }
            source_map[he->source()] = he;
        }


        std::vector<std::shared_ptr<HalfEdge>> hes;
        for (int i = 0; i < halfedges.size(); ++i) {

            auto he = halfedges[i];
            auto start_he = he;

            if (visited.find(start_he) != visited.end()) {
                continue;
            }

            do {
                CHECK(visited.find(he) == visited.end());
                visited[he] = true;
                hes.push_back(he);
                he = source_map[he->target()];
            } while (he && he != start_he);

            if (!he) {
                break;
            }
        }

        if (hes.size() == halfedges.size()) {
            halfedges = hes;
            return true;
        }

        return false;

    }



 void sampling(std::shared_ptr<Mesh<Vertex, Polygon>> mesh, double sample_rate) {
    std::map<std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Vertex>>, std::vector<std::shared_ptr<Vertex>>> vertex_vertex_interior_vertices_map;


    for (Mesh<Vertex, Polygon>::PolygonIterator piter(mesh); !piter.end(); ++piter)  {

        auto p = *piter;
        auto he_vec = generate_halfedges(p);

        for (auto he : he_vec) {

            auto s = he->source();
            auto t = he->target();


            auto vpair = std::make_pair(t, s);

            if (vertex_vertex_interior_vertices_map.find(vpair) != vertex_vertex_interior_vertices_map.end()) {
//we have got it!
                auto& vec = vertex_vertex_interior_vertices_map[vpair];
                //from rbegin();

                auto existing_v = s;
                for (int i = vec.size() - 1; i >= 0; --i) {
                    p->add_vertex_after(existing_v, vec[i]);
                    existing_v = vec[i];
                }

            } else {

                double length = (s->point() - t->point()).norm();
                int steps = length / sample_rate;

                std::vector<std::shared_ptr<Vertex>> vertex_vec;

                for (int i = 1; i < steps; ++i) {
                    auto pt = t->point() * i / (double)steps + s->point() * (1.0 - i / (double)steps);
                    auto new_v = std::make_shared<Vertex>(mesh->next_vertex_id(), pt);
                    mesh->add_vertex(new_v);
                    vertex_vec.push_back(new_v);

                }

                std::swap(vpair.first, vpair.second);
                vertex_vertex_interior_vertices_map[vpair] = vertex_vec;

//add to polygon

                auto existing_v = s;
                for (int i = 0; i < vertex_vec.size(); ++i) {
                    p->add_vertex_after(existing_v, vertex_vec[i]);
                    existing_v = vertex_vec[i];
                }

            }
        }

    }


}



    std::vector<std::shared_ptr<Front>> connect_front(const std::vector<std::shared_ptr<HalfEdge>> &edges) {

        std::unordered_map<std::shared_ptr<HalfEdge>, bool> visited;

        std::unordered_map<std::shared_ptr<Vertex>, std::shared_ptr<HalfEdge>> source_map;

        std::vector<std::shared_ptr<Front>> fronts;

        //construct the source map: source -> he
        //union and find
        for (int i = 0; i < edges.size(); ++i) {
            auto he = edges[i];
            CHECK(source_map.find(he->source()) == source_map.end());
            source_map[he->source()] = he;
            auto v0 = std::static_pointer_cast<Front::Vertex_on_Front>(he->source());
            auto v1 = std::static_pointer_cast<Front::Vertex_on_Front>(he->target());
            v0->set_out_edge(he);
            v1->set_in_edge(he);
        }

        for (int i = 0; i < edges.size(); ++i) {

            auto he = edges[i];
            auto start_he = he;

            if (visited.find(start_he) != visited.end()) {
                continue;
            }

            auto front = std::make_shared<Front>();
            do {
                CHECK(visited.find(he) == visited.end());
                visited[he] = true;
                front->add_halfedge(he);
                he = source_map[he->target()];
            } while (he && he != start_he);

            fronts.push_back(front);
        }

        return fronts;
    }

    Point compute_normal(const Point& from, const Point& to)
    {
        auto dir = to - from;
        auto norm0 = Point(dir[1], -dir[0], 0);
        auto norm = norm0 / norm0.norm();
        return norm;
    }

}
