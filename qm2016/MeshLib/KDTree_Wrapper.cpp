#include "KDTree_Wrapper.h"
#include "IO.h"
#include <iterator>


KDTree_Wrapper::KDTree_Wrapper() {


}


KDTree_Wrapper::~KDTree_Wrapper() {

}


void KDTree_Wrapper::insert(const Point & p) {

    auto v = std::make_shared<Vertex>(-1,p);
    kdtreeNode node(v);

    vertex_kdtreenode_map_[v] = node;
    kdtree_.insert(node);
    is_dirty_ = true;
}

void KDTree_Wrapper::insert(std::shared_ptr<Vertex> v) {
    kdtreeNode node(v);
    vertex_kdtreenode_map_[v] = node;
    kdtree_.insert(node);
    is_dirty_ = true;
}


void KDTree_Wrapper::remove(std::shared_ptr<Vertex> v) {
     auto ret = vertex_kdtreenode_map_.find(v);

    if (ret == vertex_kdtreenode_map_.end()) {
        LOG_W("The vertex (%s) is not existed in the kdtree!", IO::to_string(v).c_str());
        return;
    }

    kdtreeNode node = ret->second;
    kdtree_.erase(node);
    is_dirty_ = true;

}

void KDTree_Wrapper::remove(const Point& p) {
    auto v = std::make_shared<Vertex>(-1,p);
    kdtreeNode node(v);
    kdtree_.erase(node);
    is_dirty_ = true;
}



std::vector<std::shared_ptr<Vertex>> KDTree_Wrapper::find_nearest(const Point& p, double limit) {
    if (is_dirty_) {
        kdtree_.optimize();
    }

    std::vector<std::shared_ptr<Vertex>> result;
    auto v = std::make_shared<Vertex>(-1,p);
    kdtreeNode refNode(v);
    std::vector<kdtreeNode> howClose;
    kdtree_.find_within_range(refNode, limit, std::back_insert_iterator<std::vector<kdtreeNode>>(howClose));

    for (auto n : howClose) {
        result.push_back(n.vertex);
    }
    return result;

}
