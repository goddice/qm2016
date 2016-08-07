#ifndef HALFEDGE_H
#define HALFEDGE_H

#include <memory>

class Vertex;

class HalfEdge {
public:
    HalfEdge(std::shared_ptr<Vertex> src, std::shared_ptr<Vertex> trg)  {
        vertices_[0] = src;
        vertices_[1] = trg;
    }

    ~HalfEdge() {}

    //TODO: can we change the return type as auto?
    std::shared_ptr<Vertex> source() const { return vertices_[0]; }
    std::shared_ptr<Vertex>& source()       { return vertices_[0]; }

    std::shared_ptr<Vertex>  target() const { return vertices_[1]; }
    std::shared_ptr<Vertex>& target()       { return vertices_[1]; }

    std::shared_ptr<Vertex>   vertex(int i) const { return vertices_[i]; }
    std::shared_ptr<Vertex>&  vertex(int i)       { return vertices_[i]; }

private:
    std::shared_ptr<Vertex> vertices_[2];
};


#endif /* HALFEDGE_H */
