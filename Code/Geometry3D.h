#ifndef _H_GEOMETRY_3D_
#define _H_GEOMETRY_3D_

#include <vector>
#include <utility>
//#include <cfloat>
#include "math_config.h"
// #include "matrices.h"
#include <ostream>

typedef math::vec3 Point;

typedef struct Line {
	Point start;
	Point end;

	inline Line() {}
	inline Line(const Point& s, const Point& e) :
		start(s), end(e) { }
} Line;

typedef struct Ray {
	Point origin;
	math::vec3 direction;

	inline Ray() : direction(0.0f, 0.0f, 1.0f) {}
	inline Ray(const Point& o, const math::vec3& d) :
		origin(o), direction(d) {
		NormalizeDirection();
	}
	inline void NormalizeDirection() {
		math::normalized(direction);
	}
} Ray;

typedef struct Sphere {
	Point position;
	math::real radius;

	inline Sphere() : radius(1.0f) { }
	inline Sphere(const Point& p, math::real r) :
		position(p), radius(r) { }
} Sphere;

typedef struct AABB {
	Point position;
	math::vec3 size; // HALF SIZE!

	inline AABB() : size(1, 1, 1) { }
	inline AABB(const Point& p, const math::vec3& s) :
		position(p), size(s) { }
} AABB;

typedef struct OBB {
	Point position;
	math::vec3 size; // HALF SIZE!
	math::mat3 orientation;

	inline OBB() : size(1, 1, 1) { }
	inline OBB(const Point& p, const math::vec3& s) :
		position(p), size(s) { }
	inline OBB(const Point& p, const math::vec3& s, const math::mat3& o) :
		position(p), size(s), orientation(o) { }
} OBB;

typedef struct Plane {
	math::vec3 normal;
	math::real distance;

	inline Plane() : normal(1, 0, 0) { }
	inline Plane(const math::vec3& n, math::real d) :
		normal(n), distance(d) { }
} Plane;


typedef struct Triangle {
	union {
		// Utiliser un tableau de float directement
		float values[9];
		// Structure nommée pour les points
		struct {
			float ax, ay, az;
			float bx, by, bz;
			float cx, cy, cz;
		} components;
	};

	inline Triangle() { }
	inline Triangle(const Point& _p1, const Point& _p2, const Point& _p3) {
		components.ax = _p1.x; components.ay = _p1.y; components.az = _p1.z;
		components.bx = _p2.x; components.by = _p2.y; components.bz = _p2.z;
		components.cx = _p3.x; components.cy = _p3.y; components.cz = _p3.z;
	}

	// Accesseurs pour maintenir la compatibilité
	Point GetA() const { return Point(components.ax, components.ay, components.az); }
	Point GetB() const { return Point(components.bx, components.by, components.bz); }
	Point GetC() const { return Point(components.cx, components.cy, components.cz); }
	void SetA(const Point& p) { components.ax = p.x; components.ay = p.y; components.az = p.z; }
	void SetB(const Point& p) { components.bx = p.x; components.by = p.y; components.bz = p.z; }
	void SetC(const Point& p) { components.cx = p.x; components.cy = p.y; components.cz = p.z; }

	Point GetPoint(int index) const {
		switch(index) {
			case 0: return GetA();
			case 1: return GetB();
			case 2: return GetC();
			default: return Point();
		}
	}

	void SetPoint(int index, const Point& p) {
		switch(index) {
			case 0: SetA(p); break;
			case 1: SetB(p); break;
			case 2: SetC(p); break;
		}
	}
} Triangle;

typedef struct BVHNode {
	AABB bounds;
	BVHNode* children;
	int numTriangles;
	int* triangles;

	BVHNode() : children(0), numTriangles(0), triangles(0) {}
} BVHNode;

typedef struct Mesh {
	int numTriangles;
	union {
		Triangle* triangles;
		Point* vertices;
		float* values;
	};
	BVHNode* accelerator;

	Mesh() : numTriangles(0), values(0), accelerator(0) {}
} Mesh;

class Model {
protected:
	Mesh* content;
	AABB bounds;
public:
	math::vec3 position;
	math::vec3 rotation;
	bool flag;
	Model* parent;

	inline Model() : parent(0), content(0), flag(false) { }
	inline Mesh* GetMesh() const {
		return content;
	}
	inline AABB GetBounds() const {
		return bounds;
	}

	void SetContent(Mesh* mesh);
};

typedef struct Interval {
	math::real min;
	math::real max;
} Interval;

typedef struct Frustum {
	union {
		struct {
			math::real values[24];  // 4 floats per plane (normal + distance) * 6 planes
		};
		Plane planes[6];
	};

	inline Frustum() { }

	// Accesseurs pour maintenir la compatibilité
	Plane& Top() { return planes[0]; }
	Plane& Bottom() { return planes[1]; }
	Plane& Left() { return planes[2]; }
	Plane& Right() { return planes[3]; }
	Plane& Near() { return planes[4]; }
	Plane& Far() { return planes[5]; }

	const Plane& Top() const { return planes[0]; }
	const Plane& Bottom() const { return planes[1]; }
	const Plane& Left() const { return planes[2]; }
	const Plane& Right() const { return planes[3]; }
	const Plane& Near() const { return planes[4]; }
	const Plane& Far() const { return planes[5]; }
} Frustum;

typedef struct RaycastResult {
	math::vec3 point;
	math::vec3 normal;
	math::real t;
	bool hit;
} RaycastResult;

void ResetRaycastResult(RaycastResult* outResult);

Point Intersection(Plane p1, Plane p2, Plane p3);
void GetCorners(const Frustum& f, math::vec3* outCorners);

typedef math::vec3 Point3D;
typedef Line Line3D;
typedef Ray Ray3D;
typedef AABB Rectangle3D;
typedef Interval Interval3D;

std::ostream& operator<<(std::ostream& os, const Line& shape);
std::ostream& operator<<(std::ostream& os, const Ray& shape);
std::ostream& operator<<(std::ostream& os, const Sphere& shape);
std::ostream& operator<<(std::ostream& os, const AABB& shape);
std::ostream& operator<<(std::ostream& os, const OBB& shape);
std::ostream& operator<<(std::ostream& os, const Plane& shape);
std::ostream& operator<<(std::ostream& os, const Triangle& shape);

math::real Length(const Line& line);
math::real LengthSq(const Line& line);
Ray FromPoints(const Point& from, const Point& to);
math::vec3 GetMin(const AABB& aabb);
math::vec3 GetMax(const AABB& aabb);
AABB FromMinMax(const math::vec3& min, const math::vec3& max);
math::real PlaneEquation(const Point& point, const Plane& plane);
math::real PlaneEquation(const Plane& plane, const Point& point);

bool PointInSphere(const Point& point, const Sphere& sphere);
bool PointInAABB(const Point& point, const AABB& aabb);
bool PointInOBB(const Point& point, const OBB& obb);
bool PointOnPlane(const Point& point, const Plane& plane);
bool PointOnLine(const Point& point, const Line& line);
bool PointOnRay(const Point& point, const Ray& ray);

bool PointInPlane(const Point& point, const Plane& plane);
bool PointInLine(const Point& point, const Line& line);
bool PointInRay(const Point& point, const Ray& ray);
bool ContainsPoint(const Sphere& sphere, const Point& point);
bool ContainsPoint(const Point& point, const Sphere& sphere);
bool ContainsPoint(const AABB& aabb, const Point& point);
bool ContainsPoint(const Point& point, const AABB& aabb);
bool ContainsPoint(const Point& point, const OBB& obb);
bool ContainsPoint(const OBB& obb, const Point& point);
bool ContainsPoint(const Point& point, const Plane& plane);
bool ContainsPoint(const Plane& plane, const Point& point);
bool ContainsPoint(const Point& point, const Line& line);
bool ContainsPoint(const Line& line, const Point& point);
bool ContainsPoint(const Point& point, const Ray& ray);
bool ContainsPoint(const Ray& ray, const Point& point);

Point ClosestPoint(const Sphere& sphere, const Point& point);
Point ClosestPoint(const AABB& aabb, const Point& point);
Point ClosestPoint(const OBB& obb, const Point& point);
Point ClosestPoint(const Plane& plane, const Point& point);
Point ClosestPoint(const Line& line, const Point& point);
Point ClosestPoint(const Ray& ray, const Point& point);

Point ClosestPoint(const Point& point, const Sphere& sphere);
Point ClosestPoint(const Point& point, const AABB& aabb);
Point ClosestPoint(const Point& point, const OBB& obb);
Point ClosestPoint(const Point& point, const Plane& plane);
Point ClosestPoint(const Point& point, const Line& line);
Point ClosestPoint(const Point& point, const Ray& ray);
Point ClosestPoint(const Point& p, const Triangle& t);

Interval GetInterval(const AABB& aabb, const math::vec3& axis);
Interval GetInterval(const OBB& obb, const math::vec3& axis);
Interval GetInterval(const Triangle& triangle, const math::vec3& axis);

bool OverlapOnAxis(const AABB& aabb, const OBB& obb, const math::vec3& axis);
bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, const math::vec3& axis);
bool OverlapOnAxis(const AABB& aabb, const Triangle& triangle, const math::vec3& axis);
bool OverlapOnAxis(const OBB& obb, const Triangle& triangle, const math::vec3& axis);
bool OverlapOnAxis(const Triangle& t1, const Triangle& t2, const math::vec3& axis);

bool SphereSphere(const Sphere& s1, const Sphere& s2);
bool SphereAABB(const Sphere& sphere, const AABB& aabb);
bool SphereOBB(const Sphere& sphere, const OBB& obb);
bool SpherePlane(const Sphere& sphere, const Plane& plane);
bool AABBAABB(const AABB& aabb1, const AABB& aabb2);
bool AABBOBB(const AABB& aabb, const OBB& obb);
bool AABBPlane(const AABB& aabb, const Plane& plane);
bool OBBOBB(const OBB& obb1, const OBB& obb2);
bool OBBPlane(const OBB& obb, const Plane& plane);
bool PlanePlane(const Plane& plane1, const Plane& plane2);

#define AABBSphere(aabb, sphere) \
	SphereAABB(Sphere, AABB)
#define OBBSphere(obb, sphere) \
	SphereOBB(sphere, obb)
#define PlaneSphere(plane, sphere) \
	SpherePlane(sphere, plane)
#define OBBAABB(obb, aabb) \
	AABBOBB(aabb, obb)
#define PlaneAABB(plane, aabb) \
	AABBPlane(aabb, plane)
#define PlaneOBB(plane, obb) \
	OBBPlane(obb, plane)

bool Raycast(const Sphere& sphere, const Ray& ray, RaycastResult* outResult);
bool Raycast(const AABB& aabb, const Ray& ray, RaycastResult* outResult);
bool Raycast(const OBB& obb, const Ray& ray, RaycastResult* outResult);
bool Raycast(const Plane& plane, const Ray& ray, RaycastResult* outResult);
bool Raycast(const Triangle& triangle, const Ray& ray, RaycastResult* outResult);

bool Linetest(const Sphere& sphere, const Line& line);
bool Linetest(const AABB& aabb, const Line& line);
bool Linetest(const OBB& obb, const Line& line);
bool Linetest(const Plane& plane, const Line& line);
bool Linetest(const Triangle& triangle, const Line& line);

bool Raycast(const Ray& ray, const Sphere& sphere, RaycastResult* outResult);
bool Raycast(const Ray& ray, const AABB& aabb, RaycastResult* outResult);
bool Raycast(const Ray& ray, const OBB& obb, RaycastResult* outResult);
bool Raycast(const Ray& ray, const Plane& plane, RaycastResult* outResult);
bool Linetest(const Line& line, const Sphere& sphere);
bool Linetest(const Line& line, const AABB& aabb);
bool Linetest(const Line& line, const OBB& obb);
bool Linetest(const Line& line, const Plane& plane);

math::vec3 BarycentricOptimized(const Point& p, const Triangle& t);
math::vec3 Centroid(const Triangle& t);

bool PointInTriangle(const Point& p, const Triangle& t);
Plane FromTriangle(const Triangle& t);
Point ClosestPoint(const Triangle& t, const Point& p);
bool TriangleSphere(const Triangle& t, const Sphere& s);
bool TriangleAABB(const Triangle& t, const AABB& a);
bool TriangleOBB(const Triangle& t, const OBB& o);
bool TriangleTriangle(const Triangle& t1, const Triangle& t2);
bool TriangleTriangleRobust(const Triangle& t1, const Triangle& t2);
bool TrianglePlane(const Triangle& t, const Plane& p);

#define SphereTriangle(s, t) \
	TriangleSphere(t, s)
#define AABBTriangle(a, t) \
	TriangleAABB(t, a)
#define OBBTriangle(o, t) \
	TriangleOBB(t, o)
#define PlaneTriangle(p, t) \
	TrianglePlane(t, p)

// A - Edge 0, Point 0
// B - Edge 0, Point 1
// C - Edge 1, Point 0
// D - Edge 1, Point 1
math::vec3 SatCrossEdge(const math::vec3& a, const math::vec3& b, const math::vec3& c, const math::vec3& d);
math::vec3 Barycentric(const Point& p, const Triangle& t);

void AccelerateMesh(Mesh& mesh);
void SplitBVHNode(BVHNode* node, const Mesh& model, int depth);
void FreeBVHNode(BVHNode* node);

bool Linetest(const Mesh& mesh, const Line& line);
bool MeshSphere(const Mesh& mesh, const Sphere& sphere);
bool MeshAABB(const Mesh& mesh, const AABB& aabb);
bool MeshOBB(const Mesh& mesh, const OBB& obb);
bool MeshPlane(const Mesh& mesh, const Plane& plane);
bool MeshTriangle(const Mesh& mesh, const Triangle& triangle);
math::real MeshRay(const Mesh& mesh, const Ray& ray);
math::real Raycast(const Mesh& mesh, const Ray& ray);
math::real Raycast(const Model& mesh, const Ray& ray);

math::mat4 GetWorldMatrix(const Model& model);
OBB GetOBB(const Model& model);

math::real ModelRay(const Model& model, const Ray& ray);
bool Linetest(const Model& model, const Line& line);
bool ModelSphere(const Model& model, const Sphere& sphere);
bool ModelAABB(const Model& model, const AABB& aabb);
bool ModelOBB(const Model& model, const OBB& obb);
bool ModelPlane(const Model& model, const Plane& plane);
bool ModelTriangle(const Model& model, const Triangle& triangle);

float Classify(const AABB& aabb, const Plane& plane);
float Classify(const OBB& obb, const Plane& plane);

bool Intersects(const Frustum& f, const Point& p);
bool Intersects(const Frustum& f, const Sphere& s);
bool Intersects(const Frustum& f, const AABB& aabb);
bool Intersects(const Frustum& f, const OBB& obb);

math::vec3 Unproject(const math::vec3& viewportPoint, const math::vec2& viewportOrigin, const math::vec2& viewportSize, const math::mat4& view, const math::mat4& projection);
Ray GetPickRay(const math::vec2& viewportPoint, const math::vec2& viewportOrigin, const math::vec2& viewportSize, const math::mat4& view, const math::mat4& projection);

// Chapter 15

typedef struct CollisionManifold {
	bool colliding;
	math::vec3 normal;
	math::real depth;
	std::vector<math::vec3> contacts;
};
void ResetCollisionManifold(CollisionManifold* result);

std::vector<Point> GetVertices(const OBB& obb);
std::vector<Line> GetEdges(const OBB& obb);
std::vector<Plane> GetPlanes(const OBB& obb);
bool ClipToPlane(const Plane& plane, const Line& line, Point* outPoint);
std::vector<Point> ClipEdgesToOBB(const std::vector<Line>& edges, const OBB& obb);
math::real PenetrationDepth(const OBB& o1, const OBB& o2, const math::vec3& axis, bool* outShouldFlip);

CollisionManifold FindCollisionFeatures(const Sphere& A, const Sphere& B);
CollisionManifold FindCollisionFeatures(const OBB& A, const Sphere& B);
CollisionManifold FindCollisionFeatures(const OBB& A, const OBB& B);

#endif