#include "Edge.h"
#include "Vertex.h"
#include "Log.h"

Edge::Edge(std::shared_ptr<Vertex> v0, std::shared_ptr<Vertex> v1) {
    vertices_[0] = v0;
    vertices_[1] = v1;
}

Edge::~Edge() {

}

double Edge::length() const {
    return (vertices_[0]->point() - vertices_[1]->point()).norm();
}

std::shared_ptr<Vertex> Edge::vertex(int i) {
    CHECK((i == 0) || (i == 1));
    return vertices_[i];
}
