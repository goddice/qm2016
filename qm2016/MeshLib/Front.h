#ifndef FRONT_H
#define FRONT_H

#include <vector>
#include <memory>

#include "Vertex.h"
#include "HalfEdge.h"
#include "Log.h"


class Front {
public:


    Front();
    ~Front();

    class Vertex_on_Front : public Vertex {
    public:
        Vertex_on_Front(int id, const Point& pos);
        ~Vertex_on_Front();


        double compute_angle();

        void set_in_edge(std::shared_ptr<HalfEdge> he);

        std::shared_ptr<HalfEdge> get_in_edge();

        void set_out_edge(std::shared_ptr<HalfEdge> he);

        std::shared_ptr<HalfEdge> get_out_edge();

    private:
        double angle_;
        std::shared_ptr<HalfEdge> inedge_;
        std::shared_ptr<HalfEdge> outedge_;
    };

    class VertexIterator {
    public:
        VertexIterator(std::shared_ptr<Front> front) {
            iter = front->halfedges_.begin();
            end_iter = front->halfedges_.end();
        }

        ~VertexIterator() {}

        bool end() {
            return iter == end_iter;
        }

        void operator++ () { ++iter; }

        std::shared_ptr<Vertex_on_Front> operator*() {
            return std::static_pointer_cast<Vertex_on_Front>((*iter)->source());
        }
    private:
        std::vector<std::shared_ptr<HalfEdge>>::iterator iter, end_iter;
    };

    class HalfEdgeIterator {
    public:
        HalfEdgeIterator(std::shared_ptr<Front> front) {

            begin_iter = front->halfedges_.begin();
            iter = begin_iter;
            end_iter = front->halfedges_.end();
        }

        ~HalfEdgeIterator() {}

        bool end() const {
            return iter == end_iter;
        }
        void operator++ () { ++iter; }

        void rewind() {
            iter = begin_iter;
        }

        std::shared_ptr<HalfEdge> operator*() {
            return (*iter);
        }

    private:
        std::vector<std::shared_ptr<HalfEdge>>::iterator iter, begin_iter, end_iter;

    };


    class HalfEdgeIterator_He {
    public:
        HalfEdgeIterator_He(std::shared_ptr<HalfEdge> start_he)
            : start_he_(start_he)
        {
            auto v0 = std::static_pointer_cast<Front::Vertex_on_Front>(start_he_->target());
            he_ = v0->get_out_edge();
        }

        ~HalfEdgeIterator_He() {}

        bool end() const {
            return he_ == start_he_;
        }

        void operator++() {
            auto v0 = std::static_pointer_cast<Front::Vertex_on_Front>(he_->target());
            he_ = v0->get_out_edge();
        }

        std::shared_ptr<HalfEdge> operator*() {
            return he_;
        }

    private:
        std::shared_ptr<HalfEdge> start_he_, he_;

    };

    void add_halfedge(std::shared_ptr<HalfEdge> he) {
        halfedges_.push_back(he);
    }

    int halfedge_size() const {
        return halfedges_.size();
    }

    bool& is_inner_boundary() { return is_inner_boundary_; }
    bool is_inner_boundary() const { return is_inner_boundary_; }

    std::shared_ptr<HalfEdge> halfedge(int i) {
        CHECK(i >= 0);
        CHECK(i < halfedges_.size());
        return halfedges_[i];
    }

    std::vector<std::shared_ptr<HalfEdge>> halfedges() { return halfedges_; }
private:
    std::vector<std::shared_ptr<HalfEdge>> halfedges_;
    bool is_inner_boundary_;
};


#endif /* FRONT_H */
