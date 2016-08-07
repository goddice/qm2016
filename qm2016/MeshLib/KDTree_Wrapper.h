#ifndef KDTREE_WRAPPER_H
#define KDTREE_WRAPPER_H

#include "Point.h"

#include "3rdparty/kdtree++/kdtree.hpp"

#include <memory>
#include <map>
#include <cmath>
#include <algorithm>
#include "Vertex.h"

class Vertex;

struct kdtreeNode
{
    typedef double value_type;

    std::shared_ptr<Vertex> vertex;
    kdtreeNode() : vertex(nullptr) {

    }

    kdtreeNode(std::shared_ptr<Vertex> v)
        : vertex(v) {}

    value_type operator[](size_t n) const
        {
            return vertex->point()[n];
        }

    double distance( const kdtreeNode &node)
        {
            double x = vertex->point()[0] - node.vertex->point()[0];
            double y = vertex->point()[1] - node.vertex->point()[1];
            double z = vertex->point()[2] - node.vertex->point()[2];

// this is not correct   return sqrt( x*x+y*y+z*z);

// this is what kdtree checks with find_within_range()
// the "manhattan distance" from the search point.
// effectively, distance is the maximum distance in any one dimension.
            return std::max(fabs(x),std::max(fabs(y),fabs(z)));
        }

};



class KDTree_Wrapper {
public:
    KDTree_Wrapper();
    ~KDTree_Wrapper();


    void insert(const Point & p);
    void insert(std::shared_ptr<Vertex> v);

    void remove(std::shared_ptr<Vertex> v);
    void remove(const Point& p);


    std::vector<std::shared_ptr<Vertex>> find_nearest(const Point& point, double limit=1e-5);

private:

    typedef KDTree::KDTree<3, kdtreeNode> tree_type;

    tree_type kdtree_;

    bool is_dirty_;
    std::map<std::shared_ptr<Vertex>, kdtreeNode> vertex_kdtreenode_map_;
};

#endif/* KDTREE_WRAPPER_H */
