#ifndef IO_H
#define IO_H

#include <vector>
#include <memory>

#include "Point.h"
#include "Front.h"

class Vertex;
class HalfEdge;

template<typename V> class Polygon;


template <typename V, template<typename> typename P> class Mesh;

namespace IO {

    //load/save fronts
    //TODO: shall I return rvalue for efficiency?

    std::vector<std::shared_ptr<Front>> load_fronts(const char *filename);

    std::vector<std::shared_ptr<HalfEdge>> load_cm(const char *filename);


    void save_fronts(const std::vector<std::shared_ptr<Front>>& fronts, const char *filename);


    void save_front_as_poly(const std::vector<std::shared_ptr<Front>>& fronts, const char *filename);

    //output functions

    std::string to_string(const Point& point);
    std::string to_string(std::shared_ptr<Vertex> vertex);
    std::string to_string(std::shared_ptr<HalfEdge> halfedge);


    std::string to_string(std::shared_ptr<Front> front);

    //load/save mesh
    template<typename V, template <typename> typename P>
        std::shared_ptr<Mesh<V, P>> load_mesh(const char *filename);


    template <typename V, template<typename> typename P>
        void save_mesh_as_m(std::shared_ptr<Mesh<V, P>> mesh, const char *filename);


    template<typename V, template<typename> typename P>
    void save_polygons_as_m(const std::vector<std::shared_ptr<P<V>>>& polygons, const char *filename);

    template <typename V, template<typename>typename P>
        void save_mesh_as_vtk(std::shared_ptr<Mesh<V, P>> mesh, const char *filename);
}




#include "Log.h"
#include "Polygon.h"
#include "Mesh.h"
#include "Point.h"
#include "Front.h"
#include "Vertex.h"
#include "HalfEdge.h"
#include "MeshUtils.h"
#include "GeometryUtils.h"
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>


namespace IO {







    //Load mesh
    template <typename V, template<typename>typename P>
        std::shared_ptr<Mesh<V, P>> load_mesh(const char *filename) {
        std::ifstream input(filename);

        CHECK(input);

        auto mesh = std::make_shared<Mesh<V, P>>();

        while (input.good()) {
            std::string line;
            getline(input, line);
            if (line.empty()) {
                continue;
            }

            std::stringstream ss(line);

            std::string title;
            ss >> title;

            if (title == "Vertex") {
                int vid = -1;
                ss >> vid;

                CHECK(vid > 0);


                Point pos;
                ss >> pos[0] >> pos[1] >> pos[2];

                auto v = std::make_shared<V>(vid, pos);

                CHECK(!mesh->has_vertex_id(vid));
                mesh->add_vertex(v);

            } else if (title == "Face") {

                int fid = -1;
                ss >> fid;

                CHECK(fid > 0);

                auto polygon = std::make_shared<P<V>>();
                polygon->id() = fid;
                while (ss.good()) {
                    int vid = -1;
                    ss >> vid;
                    if (vid < 0) {
                        continue;
                    }

                    auto v = mesh->vertex(vid);
                    if (!v) {
                        LOG_D("%d not found, line: %s", vid, line.c_str());
                        DIE("");
                    }
                    //CHECK(v);
                    polygon->add_vertex(v);
                }
                mesh->add_polygon(polygon);
            }

        }


        input.close();

        return mesh;
    }



    template<typename V, template<typename>typename P>
    void save_mesh_as_vtk(std::shared_ptr<Mesh<V, P>> mesh, const char *filename) {

        std::ofstream output(filename);


        const char *HEADER = "# vtk DataFile Version 2.0\n " \
            "Mesquite Mesh original_mesh .\n " \
            "ASCII\n" \
            "DATASET UNSTRUCTURED_GRID\n";


        output << HEADER;

        output << "POINTS " << mesh->vertex_size() << " float\n";



        output.close();

    }


        template <typename V, template<typename>typename P>
            void save_mesh_as_m(std::shared_ptr<Mesh<V, P>> mesh, const char *filename) {

        std::ofstream output(filename);


        for (typename Mesh<V, P>::VertexIterator viter(mesh); !viter.end(); ++viter) {
            auto v = *viter;
            output << "Vertex " << v->id() << " " << v->point()[0] << " " << v->point()[1] << " " << v->point()[2] << "\n";
        }

        for (typename Mesh<V, P>::PolygonIterator piter(mesh); !piter.end(); ++piter) {
            auto p = *piter;
            output << "Face " << p->id();
            for (typename Polygon<V>::VertexIterator vpiter(p); !vpiter.end(); ++vpiter) {
                auto v = *vpiter;
                output << " " << v->id();
            }
            output << "\n";
        }

        output.close();

    }


        template<typename V, template<typename> typename P>
            void save_polygons_as_m(const std::vector<std::shared_ptr<P<V>>>& polygons, const char *filename) {


            std::map<int, std::shared_ptr<V>> vertices;

            for (auto p : polygons) {
                for (typename P<V>::VertexIterator viter(p); !viter.end(); ++viter) {
                    auto v = *viter;
                    vertices[v->id()] = v;
                }
            }


            std::ofstream output(filename);
            for (auto kv : vertices) {
                auto v = kv.second;
                output << "Vertex " << v->id() << " " << v->point()[0] << " " << v->point()[1] << " " << v->point()[2] << "\n";

            }


            for( auto p : polygons) {
                output << "Face " << p->id();
                for (typename P<V>::VertexIterator vpiter(p); !vpiter.end(); ++vpiter) {
                    auto v = *vpiter;
                    output << " " << v->id();
                }
                output << "\n";
            }

            output.close();


        }





        template<typename V>
            std::string to_string(std::shared_ptr<Polygon<V>> polygon) {
            //return "to_be_implemented";
            std::string result;
            for (typename Polygon<V>::VertexIterator vit(polygon); !vit.end(); ++vit) {
                result += to_string(*vit) + "\n";
            }
            return result;
        }

}

#endif /* IO_H */
