#ifndef HexVertex_h__
#define HexVertex_h__

#include "Point.h"
#include <string>
#include <vector>

#include "Mesh_Interface.h"


class Vertex : public IVertex{
public:
	Vertex(int id, const Point& pos);
	~Vertex();
	int id() const { return m_id; }
	int& id() { return m_id; }

	Point& point() { return m_point; }
	const Point& point() const { return m_point; }

private:
	int m_id;
	Point m_point;
};

#endif // HexVertex_h__
//test2
