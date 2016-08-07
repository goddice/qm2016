#ifndef EDGE_H
#define EDGE_H

#include <memory>

class Vertex;



class Edge {
public:
    Edge(std::shared_ptr<Vertex> v0, std::shared_ptr<Vertex> v1);
    ~Edge();
    double length() const;

    std::shared_ptr<Vertex> vertex(int i);
private:
    std::shared_ptr<Vertex> vertices_[2];
};


#endif /* EDGE_H */
