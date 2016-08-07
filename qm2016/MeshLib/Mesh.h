#ifndef MESH_H
#define MESH_H

#ifdef LOG_LEVEL
#undef LOG_LEVEL
#endif

#define LOG_LEVEL LOG_DEBUG

#include <set>
#include <map>
#include <vector>
#include <memory>
#include <unordered_map>
#include <typeinfo>
#include <typeindex>

#include "Vertex.h"
#include "Polygon.h"
#include "Log.h"
#include "IO.h"

#include "Mesh_Interface.h"
#include "KDTree_Wrapper.h"

struct MeshExtData {};

template<typename V, template<typename> typename P>
    class Mesh : public IMesh, public std::enable_shared_from_this<Mesh<V, P>> {
public:
    Mesh() {
        kdtree_ = std::make_shared<KDTree_Wrapper>();
    }
    ~Mesh() {}

    Mesh(std::vector<std::shared_ptr<P<V>>> polygons) {
        kdtree_ = std::make_shared<KDTree_Wrapper>();

        std::set<std::shared_ptr<V>> vset;
        for (auto p : polygons) {
            for (typename P<V>::VertexIterator viter(p); !viter.end(); ++viter ) {
                auto v = *viter;
                vset.insert(v);
            }
        }

        for (auto v : vset) {
            add_vertex(v);
        }

        for (auto p : polygons) {
            add_polygon(p);
        }

    }

    std::shared_ptr<V> is_existing(const Point &pt) {
        auto ret =  kdtree_->find_nearest(pt);
        if (ret.empty()) {
            return nullptr;
        }
        return std::static_pointer_cast<V>(ret[0]);
    }

    void add_vertex(std::shared_ptr<V> v) {
        vertices_[v->id()] = v;

        //add the vertex into the kdtree

        auto ret = is_existing(v->point());

        if (!ret) {
            kdtree_->insert(v);
        } else {
            LOG_E("The vertex (%s) going to be added into the mesh is close to an existing one (%s)", IO::to_string(v).c_str(), IO::to_string(ret).c_str());
            exit(-1);
        }

    }


    std::shared_ptr<V> add_new_vertex(const Point& pt) {

        auto ret = is_existing(pt);
        if (ret) {
            LOG_W("The point (%s) going to be added into the mesh is close to an existing one (%s)", IO::to_string(pt).c_str(), IO::to_string(ret).c_str());
            return ret;
        }

        auto v = std::make_shared<V>(next_vertex_id(), pt);
        add_vertex(v);

        return v;
    }

    std::shared_ptr<V> add_new_vertex(std::shared_ptr<V> v) {
        auto ret = is_existing(v->point());
        if (ret) {
            LOG_W("The point (%s) going to be added into the mesh is close to an existing one (%s)", IO::to_string(v).c_str(), IO::to_string(ret).c_str());
            return ret;
        }


        v->id() = next_vertex_id();
        add_vertex(v);
        return v;
    }



    std::shared_ptr<V> find_nearest_vertex(const Point& pt, double range) {

        auto ret = kdtree_->find_nearest(pt, range);
        if (ret.empty()) {
            return nullptr;
        }

        return ret[0];
    }


    void delete_vertex(std::shared_ptr<V> v) {
        CHECK(has_vertex_id(v->id()));
        vertices_.erase(v->id());
        kdtree_->remove(v);
    }

    bool has_vertex_id(int vid) const {
        return vertices_.find(vid) != vertices_.end();
    }


    //TODO: do we need to assign the pid??
    void add_polygon(std::shared_ptr<P<V>> p) {
        //TODO: check if the vertices are in the mesh
        polygons_[p->id()] = p;
    }



    std::shared_ptr<V> vertex(int id) {
        if(vertices_.find(id) == vertices_.end()) {
            LOG_E("V: %d not found!", id);

            return nullptr;
        }
        return vertices_[id];
    }

    std::shared_ptr<P<V>> polygon(int id) {
        CHECK(polygons_.find(id) != polygons_.end());
        return polygons_[id];
    }



    //Iterators
    class VertexIterator {
    public:
        VertexIterator(std::shared_ptr<Mesh> mesh) {
            iter_ = mesh->vertices_.begin();
            end_iter_ = mesh->vertices_.end();
        }

        ~VertexIterator() {}

        bool end() const {
            return iter_ == end_iter_;
        }

        void operator++() {
            ++iter_;
        }

        std::shared_ptr<V> operator*() {
            return (*iter_).second;
        }

    private:
        typename std::map<int, std::shared_ptr<V>>::iterator iter_, end_iter_;
    };

    class PolygonIterator {
    public:
        PolygonIterator(std::shared_ptr<Mesh> mesh) {
            iter_ = mesh->polygons_.begin();
            end_iter_ = mesh->polygons_.end();
        }

        ~PolygonIterator() {}

        bool end() const {
            return iter_ == end_iter_;
        }

        void operator++() {
            ++iter_;
        }

        std::shared_ptr<P<V>> operator*() {
            return (*iter_).second;
        }

    private:
        typename std::map<int, std::shared_ptr<P<V>>>::iterator iter_, end_iter_;
    };

    int vertex_size() const { return vertices_.size(); }
    int polygon_size() const { return polygons_.size(); }


    int next_vertex_id() const {
        if (vertices_.empty()) {
            return 1;
        }
        return vertices_.rbegin()->first + 1;
    }


    int next_polygon_id() const {
        if (polygons_.empty()) {
            return 1;
        }
        return polygons_.rbegin()->first + 1;
    }

    template< template<typename, template<typename> typename> typename...E>
        void init_extension() {
        InitMeshExtension<E<V,P>..., _my_centinel_type>();
        LOG_D("init ext!");
    }

    template <template <typename,template<typename> typename> typename U>
        std::shared_ptr<U<V,P>> get_extension() {
        LOG_D("get_extension: %s", typeid(U<V,P>).name());
        auto it = ext_data_map_.find(typeid(U<V,P>));
        CHECK(it != ext_data_map_.end());
        return std::static_pointer_cast<U<V,P>>(it->second);
    }

protected:
    std::map<int, std::shared_ptr<V>> vertices_;
    std::map<int, std::shared_ptr<P<V>>> polygons_;


    std::shared_ptr<KDTree_Wrapper> kdtree_;

    //extensions

    struct _my_centinel_type {};
    template<typename U, typename ...Rest>
        typename std::enable_if<!std::is_same<U,_my_centinel_type>::value , void>::type InitMeshExtension()
    {

        if (ext_data_map_.find(typeid(U)) == ext_data_map_.end()) {
            auto ext = std::make_shared<U>();
            ext->build(this->shared_from_this());
            ext_data_map_[typeid(U)] = ext;
            LOG_D("typeid(U)! %s", typeid(U).name());
        }

        InitMeshExtension<Rest...>();
    }

    template<typename U>
        typename std::enable_if<std::is_same<U,_my_centinel_type>::value , void>::type InitMeshExtension(){

    }
    std::unordered_map<std::type_index, std::shared_ptr<MeshExtData>> ext_data_map_;

};

#endif /* MESH_H */
