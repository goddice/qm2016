#ifdef LOG_LEVEL
#undef LOG_LEVEL
#endif

#define LOG_LEVEL LOG_DEBUG


#include "IO.h"

namespace IO {



    std::vector<std::shared_ptr<HalfEdge>> load_cm(const char *filename) {

        std::ifstream input(filename);

        CHECK(input);

        std::map<int, std::shared_ptr<Vertex>> vertices;
        std::vector<std::shared_ptr<HalfEdge>> edges;


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

                auto v = std::make_shared<Vertex>(vid, pos);

                CHECK(vertices.find(vid) == vertices.end());
                vertices[vid] = v;
            } else if (title == "Edge") {

                int vid0 = -1, vid1 = -1;
                ss >> vid0 >> vid1;

                CHECK((vid0 > 0) && (vid1 > 0));

                auto e = std::make_shared<HalfEdge>(vertices[vid0], vertices[vid1]);

                edges.push_back(e);
            }

        }

        input.close();

        return edges;
    }

    std::string to_string(const Point& point) {
        std::stringstream ss;
        ss << "(" << point[0] << "," << point[1] << "," << point[2] << ")";
        return ss.str();
    }


    std::string to_string(std::shared_ptr<Vertex> vertex) {
        std::stringstream ss;
        ss << "V: " << vertex->id() << " " << to_string(vertex->point());
        return ss.str();
    }

    std::string to_string(std::shared_ptr<HalfEdge> halfedge) {
        std::stringstream ss;
        ss << "E: " << halfedge->source()->id() << " -- " << halfedge->target()->id() << "  (" <<  IO::to_string(halfedge->source()) << " -- " << IO::to_string(halfedge->target()) << ")";
        return ss.str();
    }




    void save_polygon_as_cm(std::shared_ptr<Polygon<Vertex>> polygon, const char *filename) {

        std::ofstream output (filename);

        std::map<std::shared_ptr<Vertex>, int> prev_id_map;

        int vid = 1;
        for (Polygon<Vertex>::VertexIterator viter(polygon); !viter.end(); ++viter) {
            auto v = *viter;
            prev_id_map[v] = v->id();
            v->id() = vid++;

            output << "Vertex " << v->id() << " " << v->point()[0] << " " << v->point()[1] << " " << v->point()[2] << "\n";
        }


        auto hes = MeshUtils::generate_halfedges(polygon);
        for (auto he : hes) {
            output << "Edge " << he->source()->id() << " " << he->target()->id() << "\n";
        }

        for (auto kv : prev_id_map) {
            kv.first->id() = kv.second;
        }

        output.close();

    }



    std::vector<std::shared_ptr<Front>> load_fronts(const char *filename) {

        std::vector<std::shared_ptr<Front>> fronts;

        std::ifstream input(filename);

        CHECK(input);

        std::map<int, std::shared_ptr<Front::Vertex_on_Front>> vertices;
        std::vector<std::shared_ptr<HalfEdge>> edges;

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

                auto v = std::make_shared<Front::Vertex_on_Front>(vid, pos);

                CHECK(vertices.find(vid) == vertices.end());
                vertices[vid] = v;
            } else if (title == "Edge") {

                int vid0 = -1, vid1 = -1;
                ss >> vid0 >> vid1;

                CHECK((vid0 > 0) && (vid1 > 0));

                auto e = std::make_shared<HalfEdge>(vertices[vid0], vertices[vid1]);
                edges.push_back(e);
            }
        }

        input.close();

        return MeshUtils::connect_front(edges);

    }

    void save_front_as_poly(const std::vector<std::shared_ptr<Front>>& fronts, const char *filename) {

        //count the vertices
        int num_vertices = 0;
        int num_hes = 0;
        int num_holes = 0;
        std::unordered_map<int, bool> vid_recorder;

        for (auto & front : fronts) {
            //std::shared_ptr<Vertex> v;

            // TODO: why use macro fail??? Bug here.
            for (Front::VertexIterator viter(front); !viter.end(); ++viter) {
                auto v = *viter;
                CHECK(vid_recorder.find(v->id()) == vid_recorder.end());
                vid_recorder[v->id()] = true;
                ++num_vertices;
            }

            for (Front::HalfEdgeIterator heiter(front); !heiter.end(); ++heiter) {
                ++num_hes;
            }

            if (front->is_inner_boundary()) {
                ++num_holes;
            }
        }




        std::ofstream output(filename);


        output << num_vertices << " 2 0 0\n";

        for (auto & front : fronts) {
            for (Front::VertexIterator viter(front); !viter.end(); ++viter) {
                auto v = *viter;
                output << v->id() << " " << v->point()[0] << " " << v->point()[1] << "\n";
            }
        }

        output << "#segments\n";
        output << num_hes << " 0\n";
        int i = 1;
        for (auto & front : fronts) {
            for (Front::HalfEdgeIterator heiter(front); !heiter.end(); ++heiter) {
                auto he = *heiter;
                output << i++ << " " << he->vertex(0)->id() << " " << he->vertex(1)->id() << "\n";
            }
        }

        output << "#holes\n";
        output << num_holes << "\n";

        i = 1;
        for (auto & front : fronts) {
            if (!front->is_inner_boundary()) {
                continue;
            }

            auto pt = GeometryUtils::arbitrary_inside_point(front);

            output << i++ << " " << pt[0] << " " << pt[1] << "\n";
        }


        output.close();

    }


    std::string to_string(std::shared_ptr<Front> front) {
        std::string result;
        for (Front::HalfEdgeIterator heit(front); !heit.end(); ++heit) {
            result += to_string(*heit) + "\n";
        }
        return result;
    }


}
