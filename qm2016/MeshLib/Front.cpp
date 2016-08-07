#include "Front.h"
#include "GeometryUtils.h"

/////
/// Front::Vertex_on_Front
/////
Front::Front() : is_inner_boundary_(false) {}
Front::~Front() {}


Front::Vertex_on_Front::Vertex_on_Front(int id, const Point& pos)
    : Vertex(id, pos),
      angle_(0),
      inedge_(nullptr),
      outedge_(nullptr)
{

}

double Front::Vertex_on_Front::compute_angle() {
  CHECK(inedge_);
  CHECK(outedge_);
  auto p = outedge_->target()->point();
  auto o = inedge_->target()->point();
  auto q = inedge_->source()->point();
  double angle = GeometryUtils::angle(p, o, q);
  return angle;
}


Front::Vertex_on_Front::~Vertex_on_Front() {
}



void Front::Vertex_on_Front::set_in_edge(std::shared_ptr<HalfEdge> he) {
    inedge_ = he;
}

std::shared_ptr<HalfEdge> Front::Vertex_on_Front::get_in_edge() {
    return inedge_;
}

void Front::Vertex_on_Front::set_out_edge(std::shared_ptr<HalfEdge> he) {
    outedge_ = he;
}

std::shared_ptr<HalfEdge> Front::Vertex_on_Front::get_out_edge() {
    return outedge_;
}


/////
/// End of Front::Vertex_on_Front
/////
