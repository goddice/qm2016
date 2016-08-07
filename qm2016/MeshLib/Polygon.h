#ifndef POLYGON_H
#define POLYGON_H

#include "Mesh_Interface.h"
#include "HalfEdge.h"
#include "Log.h"
#include <memory>
#include <vector>


template <typename V>
class Polygon : public IPolygon{
public:
    Polygon() {}

    Polygon(const std::vector<std::shared_ptr<HalfEdge>>& halfedges) {
        CHECK(halfedges[0]->source() == halfedges[halfedges.size() - 1]->target());
        for (auto he : halfedges) {
            vertices_.push_back(he->source());
        }
    }

    ~Polygon() {}

    void add_vertex(std::shared_ptr<V> v) {
        vertices_.push_back(v);
    }

    bool add_vertex_after(std::shared_ptr<V> existing_v, std::shared_ptr<V> v) {
        for (int i = 0; i < vertices_.size(); ++i) {
            if (vertices_[i] == existing_v) {
                vertices_.insert(vertices_.begin() + i + 1, v);
                return true;
            }
        }
        return false;

    }

    bool delete_vertex(std::shared_ptr<V> v) {
        //will degenerate..
        if (vertices_.size() < 4) {
            return false;
        }

        vertices_.erase(std::remove(vertices_.begin(), vertices_.end(), 8), vertices_.end());

    }


    int id() const { return id_; }
    int& id() { return id_; }

    std::shared_ptr<Vertex> vertex(int id) {
        CHECK((id >=0 ) && (id < vertices_.size()));
        return vertices_[id];
    }

    //Iterators
    class VertexIterator {
    public:
        VertexIterator(std::shared_ptr<Polygon> polygon) {
            iter_ = polygon->vertices_.begin();
            end_iter_ = polygon->vertices_.end();
            begin_iter_ = polygon->vertices_.begin();
        }

        ~VertexIterator() {}

        bool end() const {
            return iter_ == end_iter_;
        }

        VertexIterator& operator++() {
            ++iter_;
            return *this;
        }

        VertexIterator& operator++(int) {
            ++iter_;
            return *this;
        }

        std::shared_ptr<V> operator*() {
            return *iter_;
        }

        void rewind() {
            iter_ = begin_iter_;
        }


    private:
        typename std::vector< std::shared_ptr<V>>::iterator iter_, end_iter_, begin_iter_;
    };




private:
    int id_;
    std::vector<std::shared_ptr<V>> vertices_;
};


#endif /* POLYGON_H */
