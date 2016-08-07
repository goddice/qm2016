#ifndef MESHEXT_H
#define MESHEXT_H


#include <memory>
#include <typeinfo>
#include <typeindex>
#include <unordered_map>

#include "Log.h"
#include "Polygon.h"
#include "Mesh.h"
#include "MeshUtils.h"

struct MeshBase;


template<typename V, template <typename> typename P>
class VertexPolygonExtension : public MeshExtData {
public:
        void build(std::shared_ptr<Mesh<V, P>> mesh)  {
        for (typename Mesh<V,P>::PolygonIterator piter(mesh); !piter.end(); ++piter) {
            auto p = *piter;
            for (typename P<V>::VertexIterator viter(p); !viter.end(); ++viter) {
                auto v = *viter;
                vertex_polygon_map[v].push_back(p);
            }

        }
    }

    struct VertexPolygonIterator {
        VertexPolygonIterator(std::shared_ptr<VertexPolygonExtension> extension,
                              std::shared_ptr<V> vertex)
            {
                auto it = extension->vertex_polygon_map.find(vertex);
                CHECK(it != extension->vertex_polygon_map.end());

                iter = it->second.begin();
                end_iter = it->second.end();
            }


        std::shared_ptr<P<V>> operator*() {
            return *iter;
        }

        void operator++() {
            ++iter;
        }

        bool end() const {
            return iter == end_iter;
        }

        typename std::vector<std::shared_ptr<P<V>>>::iterator iter, end_iter;
    };

    std::unordered_map<std::shared_ptr<V>, std::vector<std::shared_ptr<P<V>>>> vertex_polygon_map;

};


template <typename V, template<typename> typename P>
class PolygonHalfEdgeExtension : public MeshExtData {
public:
        void build(std::shared_ptr<Mesh<V, P>> mesh)  {
        for (typename Mesh<V, P>::PolygonIterator piter(mesh); !piter.end(); ++piter) {
            auto p = *piter;
            auto he_list = MeshUtils::generate_halfedges<V>(p);
            polygon_halfedge_map[p] = he_list;
        }
    }


    struct PolygonHalfEdgeIterator {
        PolygonHalfEdgeIterator(std::shared_ptr<PolygonHalfEdgeExtension> extension,
                              std::shared_ptr<P<V>> polygon)
            {
                auto it = extension->polygon_halfedge_map.find(polygon);
                CHECK(it != extension->polygon_halfedge_map.end());

                iter = it->second.begin();
                end_iter = it->second.end();
            }


        std::shared_ptr<HalfEdge> operator*() {
            return *iter;
        }

        void operator++() {
            ++iter;
        }

        bool end() const {
            return iter == end_iter;
        }

        std::vector<std::shared_ptr<HalfEdge>>::iterator iter, end_iter;
    };




    std::unordered_map<std::shared_ptr<P<V>>, std::vector<std::shared_ptr<HalfEdge>>> polygon_halfedge_map;
};

template<typename V, template <typename> typename P>
    class PolygonPolygonExtension : public MeshExtData {

public:

    void build(std::shared_ptr<Mesh<V, P>> mesh) {
//build halfedges
        std::map<std::pair<std::shared_ptr<V>, std::shared_ptr<V>>, std::vector<std::shared_ptr<P<V>>>> vv_p_map;

        for(typename Mesh<V,P>::PolygonIterator piter(mesh); !piter.end(); ++piter) {
            auto polygon = *piter;
            auto he_list = MeshUtils::generate_halfedges<V>(polygon);

            for (auto he : he_list) {
//check if exist
                auto it = vv_p_map.find(std::make_pair(he->source(), he->target()));

                if (it == vv_p_map.end()) {
                    it = vv_p_map.find(std::make_pair(he->target(), he->source()));
                }

                if (it != vv_p_map.end()) {
                    auto poly_vec = it->second;
                    for (auto pp : poly_vec) {
                        polygon_polygon_map[pp].push_back(polygon);
                        polygon_polygon_map[polygon].push_back(pp);
                    }
                    it->second.push_back(polygon);
                } else {
                    vv_p_map[std::make_pair(he->source(), he->target())].push_back(polygon);
                }

            }
        }
    }


    struct PolygonPolygonIterator {
        PolygonPolygonIterator(std::shared_ptr<PolygonPolygonExtension> extension,
                               std::shared_ptr<P<V>> polygon)
            {
                auto it = extension->polygon_polygon_map.find(polygon);
                CHECK(it != extension->polygon_polygon_map.end());
                iter = it->second.begin();
                end_iter = it->second.end();
            }

        std::shared_ptr<P<V>> operator*() {
            return *iter;
        }

        void operator++() {
            ++iter;
        }

        bool end() const {
            return iter == end_iter;
        }

        typename std::vector<std::shared_ptr<P<V>>>::iterator iter, end_iter;
    };


    std::unordered_map<std::shared_ptr<P<V>>, std::vector<std::shared_ptr<P<V>>>> polygon_polygon_map;
};

template<typename V, template <typename> typename P>
    class InteriorHalfEdgeExtension : public MeshExtData {
public:
    void build(std::shared_ptr<Mesh<V, P>> mesh)  {

        std::set<std::pair<std::shared_ptr<V>, std::shared_ptr<V>>> vvset;

        for(typename Mesh<V,P>::PolygonIterator piter(mesh); !piter.end(); ++piter) {
            auto polygon = *piter;
            auto he_list = MeshUtils::generate_halfedges<V>(polygon);

            for (auto he : he_list) {

                auto vv_pair = std::make_pair(he->target(), he->source());
                auto it = vvset.find(vv_pair);

                if (it !=  vvset.end()) {
                    interior_halfedges.push_back(he);
                } else {
                    std::swap(vv_pair.first, vv_pair.second);
                    vvset.insert(vv_pair);
                }
            }
        }
    }



    struct InteriorHalfEdgeIterator {
        InteriorHalfEdgeIterator(std::shared_ptr<InteriorHalfEdgeExtension> extension)
            {
                iter = extension->interior_halfedges.begin();
                end_iter = extension->interior_halfedges.end();
            }

        std::shared_ptr<HalfEdge> operator*() {
            return *iter;
        }

        void operator++() {
            ++iter;
        }

        bool end() const {
            return iter == end_iter;
        }

        typename std::vector<std::shared_ptr<HalfEdge>>::iterator iter, end_iter;
    };



    std::vector<std::shared_ptr<HalfEdge>> interior_halfedges;

};


template<typename V, template <typename> typename P>
    class BoundaryHalfEdgeExtension : public MeshExtData {
public:
    void build(std::shared_ptr<Mesh<V, P>> mesh)  {

        std::set<std::pair<std::shared_ptr<V>, std::shared_ptr<V>>> vvset;

        std::map<std::pair<std::shared_ptr<V>, std::shared_ptr<V>>, std::shared_ptr<HalfEdge>> vvhemap;

        for(typename Mesh<V,P>::PolygonIterator piter(mesh); !piter.end(); ++piter) {
            auto polygon = *piter;
            auto he_list = MeshUtils::generate_halfedges<V>(polygon);

            for (auto he : he_list) {

                auto vv_pair = std::make_pair(he->target(), he->source());
                auto it = vvset.find(vv_pair);

                if (it !=  vvset.end()) {
//                    boundary_halfedges.push_back(he);
                    vvset.erase(it);
                    vvhemap.erase(vv_pair);
                } else {
                    std::swap(vv_pair.first, vv_pair.second);
                    vvset.insert(vv_pair);
                    vvhemap[vv_pair] = he;
                }
            }
        }


        for (auto &kv : vvhemap) {
            boundary_halfedges.push_back(kv.second);
        }

    }

    struct BoundaryHalfEdgeIterator {
        BoundaryHalfEdgeIterator(std::shared_ptr<BoundaryHalfEdgeExtension> extension) {
            iter = extension->boundary_halfedges.begin();
            end_iter = extension->boundary_halfedges.end();
        }

        std::shared_ptr<HalfEdge> operator*() {
            return *iter;
        }

        void operator++() {
            ++iter;
        }

        bool end() const {
            return iter == end_iter;
        }

        typename std::vector<std::shared_ptr<HalfEdge>>::iterator iter, end_iter;
    };

    std::vector<std::shared_ptr<HalfEdge>> boundary_halfedges;

};

#endif /* MESHEXT_H */
