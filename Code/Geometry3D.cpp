#include "Geometry3D.h"
#include <cmath>
#include <cmath>
#include <list>

#define CMP(x, y) \
	(fabsf(x - y) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

math::real Length(const Line& line) {
	return math::length(line.start - line.end);
}

math::real LengthSq(const Line& line) {
	return math::lengthSq(line.start - line.end);
}

Ray FromPoints(const Point& from, const Point& to) {
	return Ray(
		from,
		math::normalized(to - from)
	);
}

math::vec3 GetMin(const AABB& aabb) {
	math::vec3 p1 = aabb.position + aabb.size;
	math::vec3 p2 = aabb.position - aabb.size;

	return math::vec3(fminf(p1.x, p2.x), fminf(p1.y, p2.y), fminf(p1.z, p2.z));
}
math::vec3 GetMax(const AABB& aabb) {
	math::vec3 p1 = aabb.position + aabb.size;
	math::vec3 p2 = aabb.position - aabb.size;

	return math::vec3(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y), fmaxf(p1.z, p2.z));
}

AABB FromMinMax(const math::vec3& min, const math::vec3& max) {
	return AABB((min + max) * 0.5f, (max - min) * 0.5f);
}

math::real PlaneEquation(const Point& point, const Plane& plane) {
	return math::dot(point, plane.normal) - plane.distance;
}

math::real PlaneEquation(const Plane& plane, const Point& point) {
	return math::dot(point, plane.normal) - plane.distance;
}

std::ostream& operator<<(std::ostream& os, const Line& shape) {
	os << "start: (" << shape.start.x << ", " << shape.start.y << ", " << shape.start.z << "), end: (";
	os << shape.end.x << ", " << shape.end.y << ", " << shape.end.z << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Ray& shape) {
	os << "origin: (" << shape.origin.x << ", " << shape.origin.y << ", " << shape.origin.z << "), ";
	os << "direction: (" << shape.direction.x << ", " << shape.direction.y << ", " << shape.direction.z << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Sphere& shape) {
	os << "position:" << shape.position.x << ", " << shape.position.y << ", " << shape.position.z << "), ";
	os << "radius: " << shape.radius;
	return os;
}

std::ostream& operator<<(std::ostream& os, const AABB& shape) {
	math::vec3 min = GetMin(shape);
	math::vec3 max = GetMax(shape);
	os << "min: (" << min.x << ", " << min.y << ", " << min.z << "), ";
	os << "max: (" << max.x << ", " << max.y << ", " << max.z << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Plane& shape) {
	os << "normal: (" << shape.normal.x << ", " << shape.normal.y << ", " << shape.normal.z << "), ";
	os << "distance: " << shape.distance;
	return os;
}

std::ostream& operator<<(std::ostream& os, const Triangle& shape) {
	os << "a: (" << shape.GetA().x << ", " << shape.GetA().y << ", " << shape.GetA().z << "), ";
	os << "b: (" << shape.GetB().x << ", " << shape.GetB().y << ", " << shape.GetB().z << "), ";
	os << "c: (" << shape.GetC().x << ", " << shape.GetC().y << ", " << shape.GetC().z << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const OBB& shape) {
	os << "position: (" << shape.position.x << ", " << shape.position.y << ", " << shape.position.z << "), ";
	os << "size: (" << shape.size.x << ", " << shape.size.y << ", " << shape.size.z << "), ";
	os << "x basis: (" << shape.orientation[0].x << ", " << shape.orientation[0].y << ", " << shape.orientation[0].z << "), ";
	os << "y basis: (" << shape.orientation[1].x << ", " << shape.orientation[1].y << ", " << shape.orientation[1].z << "), ";
	os << "z basis: (" << shape.orientation[2].x << ", " << shape.orientation[2].y << ", " << shape.orientation[2].z << ")";
	return os;
}

bool PointInSphere(const Point& point, const Sphere& sphere) {
	return math::lengthSq(point - sphere.position) < sphere.radius * sphere.radius;
}

bool PointOnPlane(const Point& point, const Plane& plane) {
	// This should probably use an epsilon!
	//return math::dot(point, plane.normal) - plane.distance == 0.0f;

	return CMP(math::dot(point, plane.normal) - plane.distance, 0.0f);
}

bool PointInAABB(const Point& point, const AABB& aabb) {
	Point min = GetMin(aabb);
	Point max = GetMax(aabb);

	if (point.x < min.x || point.y < min.y || point.z < min.z) {
		return false;
	}
	if (point.x > max.x || point.y > max.y || point.z > max.z) {
		return false;
	}

	return true;
}

bool PointInOBB(const Point& point, const OBB& obb) {
	math::vec3 dir = point - obb.position;

	for (int i = 0; i < 3; ++i) {
		// On peut accéder directement à la colonne i de la matrice
		math::vec3 axis = obb.orientation[i];  // GLM permet l'accès direct aux colonnes
		math::real distance = math::dot(dir, axis);

		// Accès direct au composant i du vecteur size
		if (std::abs(distance) > obb.size[i]) {
			return false;
		}
	}

	return true;
}

Point ClosestPoint(const Sphere& sphere, const Point& point) {
	math::vec3 sphereToPoint = point - sphere.position;
	math::normalized(sphereToPoint);
	sphereToPoint = sphereToPoint * sphere.radius;
	return sphereToPoint + sphere.position;
}

Point ClosestPoint(const AABB& aabb, const Point& point) {
	Point result = point;
	Point min = GetMin(aabb);
	Point max = GetMax(aabb);

	result.x = (result.x < min.x) ? min.x : result.x;
	result.y = (result.y < min.x) ? min.y : result.y;
	result.z = (result.z < min.x) ? min.z : result.z;

	result.x = (result.x > max.x) ? max.x : result.x;
	result.y = (result.y > max.x) ? max.y : result.y;
	result.z = (result.z > max.x) ? max.z : result.z;

	return result;
}

Point ClosestPoint(const OBB& obb, const Point& point) {
	Point result = obb.position;
	math::vec3 dir = point - obb.position;

	for (int i = 0; i < 3; ++i) {
		math::vec3 axis = obb.orientation[i];
		math::real distance = math::dot(dir, axis);

		if (distance > obb.size[i]) {
			distance = obb.size[i];
		}
		if (distance < -obb.size[i]) {
			distance = -obb.size[i];
		}

		result += axis * distance;
	}

	return result;
}
Point ClosestPoint(const Plane& plane, const Point& point) {
	// This works assuming plane.Normal is math::normalized, which it should be
	math::real distance = math::dot(plane.normal, point) - plane.distance;
	// If the plane normal wasn't math::normalized, we'd need this:
	// distance = distance / math::dot(plane.Normal, plane.Normal);

	return point - plane.normal * distance;
}

bool PointOnLine(const Point& point, const Line& line) {
	Point closest = ClosestPoint(line, point);
	math::real distanceSq = math::lengthSq(closest - point);
	return CMP(distanceSq, 0.0f);
}

Point ClosestPoint(const Line& line, const Point& point) {
	math::vec3 lVec = line.end - line.start; // Line Vector
	// math::project "point" onto the "Line Vector", computing:
	// closest(t) = start + t * (end - start)
	// T is how far along the line the projected point is
	math::real t = math::dot(point - line.start, lVec) / math::dot(lVec, lVec);
	// Clamp t to the 0 to 1 range
	t = fmaxf(t, 0.0f);
	t = fminf(t, 1.0f);
	// Return projected position of t
	return line.start + lVec * t;
}

bool PointOnRay(const Point& point, const Ray& ray) {
	if (point == ray.origin) {
		return true;
	}

	math::vec3 norm = point - ray.origin;
	math::normalized(norm);
	math::real diff = math::dot(norm, ray.direction); // Direction is math::normalized
	// If BOTH vectors point in the same direction, diff should be 1
	return CMP(diff, 1.0f);
}

Point ClosestPoint(const Ray& ray, const Point& point) {
	// math::project point onto ray,
	math::real t = math::dot(point - ray.origin, ray.direction);
	// Not needed if direction is math::normalized!
	// t /= math::dot(ray.direction, ray.direction);

	// We only want to clamp t in the positive direction.
	// The ray extends infinatley in this direction!
	t = fmaxf(t, 0.0f);

	// Compute the projected position from the clamped t
	// Notice we multiply r.Normal by t, not AB.
	// This is becuase we want the ray in the direction
	// of the normal, which technically the line segment is
	// but this is much more explicit and easy to read.
	return Point(ray.origin + ray.direction * t);
}

bool PointInPlane(const Point& point, const Plane& plane) {
	return PointOnPlane(point, plane);
}
bool PointInLine(const Point& point, const Line& line) {
	return PointOnLine(point, line);
}
bool PointInRay(const Point& point, const Ray& ray) {
	return PointOnRay(point, ray);
}
bool ContainsPoint(const Sphere& sphere, const Point& point) {
	return PointInSphere(point, sphere);
}
bool ContainsPoint(const Point& point, const Sphere& sphere) {
	return PointInSphere(point, sphere);
}
bool ContainsPoint(const AABB& aabb, const Point& point) {
	return PointInAABB(point, aabb);
}
bool ContainsPoint(const Point& point, const AABB& aabb) {
	return PointInAABB(point, aabb);
}
bool ContainsPoint(const Point& point, const OBB& obb) {
	return PointInOBB(point, obb);
}
bool ContainsPoint(const OBB& obb, const Point& point) {
	return PointInOBB(point, obb);
}
bool ContainsPoint(const Point& point, const Plane& plane) {
	return PointOnPlane(point, plane);
}
bool ContainsPoint(const Plane& plane, const Point& point) {
	return PointOnPlane(point, plane);
}
bool ContainsPoint(const Point& point, const Line& line) {
	return PointOnLine(point, line);
}
bool ContainsPoint(const Line& line, const Point& point) {
	return PointOnLine(point, line);
}
bool ContainsPoint(const Point& point, const Ray& ray) {
	return PointOnRay(point, ray);
}
bool ContainsPoint(const Ray& ray, const Point& point) {
	return PointOnRay(point, ray);
}
Point ClosestPoint(const Point& point, const Sphere& sphere) {
	return ClosestPoint(sphere, point);
}
Point ClosestPoint(const Point& point, const AABB& aabb) {
	return ClosestPoint(aabb, point);
}
Point ClosestPoint(const Point& point, const OBB& obb) {
	return ClosestPoint(obb, point);
}
Point ClosestPoint(const Point& point, const Plane& plane) {
	return ClosestPoint(plane, point);
}
Point ClosestPoint(const Point& point, const Line& line) {
	return ClosestPoint(line, point);
}
Point ClosestPoint(const Point& point, const Ray& ray) {
	return ClosestPoint(ray, point);
}
Point ClosestPoint(const Point& p, const Triangle& t) {
	return ClosestPoint(t, p);
}

bool SphereSphere(const Sphere& s1, const Sphere& s2) {
	math::real radiiSum = s1.radius + s2.radius;
	math::real sqDistance = math::lengthSq(s1.position - s2.position);
	return sqDistance < radiiSum * radiiSum;
}

bool SphereAABB(const Sphere& sphere, const AABB& aabb) {
	Point closestPoint = ClosestPoint(aabb, sphere.position);
	math::real distSq = math::lengthSq(sphere.position - closestPoint);
	math::real radiusSq = sphere.radius * sphere.radius;
	return distSq < radiusSq;
}

bool SphereOBB(const Sphere& sphere, const OBB& obb) {
	Point closestPoint = ClosestPoint(obb, sphere.position);
	math::real distSq = math::lengthSq(sphere.position - closestPoint);
	math::real radiusSq = sphere.radius * sphere.radius;
	return distSq < radiusSq;
}

bool SpherePlane(const Sphere& sphere, const Plane& plane) {
	Point closestPoint = ClosestPoint(plane, sphere.position);
	math::real distSq = math::lengthSq(sphere.position - closestPoint);
	math::real radiusSq = sphere.radius * sphere.radius;
	return distSq < radiusSq;
}

bool AABBAABB(const AABB& aabb1, const AABB& aabb2) {
	Point aMin = GetMin(aabb1);
	Point aMax = GetMax(aabb1);
	Point bMin = GetMin(aabb2);
	Point bMax = GetMax(aabb2);

	return	(aMin.x <= bMax.x && aMax.x >= bMin.x) &&
			(aMin.y <= bMax.y && aMax.y >= bMin.y) &&
			(aMin.z <= bMax.z && aMax.z >= bMin.z);
}

bool AABBOBB(const AABB& aabb, const OBB& obb) {

	math::vec3 test[15] = {
		// AABB axes (unit vectors)
		math::vec3(1, 0, 0),  // X axis
		math::vec3(0, 1, 0),  // Y axis
		math::vec3(0, 0, 1),  // Z axis
		// OBB axes (colonnes de la matrice d'orientation)
		obb.orientation[0],
		obb.orientation[1],
		obb.orientation[2]
};

	for (int i = 0; i < 3; ++i) { // Fill out rest of axis
		test[6 + i * 3 + 0] = math::cross(test[i], test[0]);
		test[6 + i * 3 + 1] = math::cross(test[i], test[1]);
		test[6 + i * 3 + 2] = math::cross(test[i], test[2]);
	}

	for (int i = 0; i < 15; ++i) {
		if (!OverlapOnAxis(aabb, obb, test[i])) {
			return false; // Seperating axis found
		}
	}

	return true; // Seperating axis not found
}

bool OverlapOnAxis(const AABB& aabb, const OBB& obb, const math::vec3& axis) {
	Interval a = GetInterval(aabb, axis);
	Interval b = GetInterval(obb, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, const math::vec3& axis) {
	Interval a = GetInterval(obb1, axis);
	Interval b = GetInterval(obb1, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

bool OverlapOnAxis(const AABB& aabb, const Triangle& triangle, const math::vec3& axis) {
	Interval a = GetInterval(aabb, axis);
	Interval b = GetInterval(triangle, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

bool OverlapOnAxis(const OBB& obb, const Triangle& triangle, const math::vec3& axis) {
	Interval a = GetInterval(obb, axis);
	Interval b = GetInterval(triangle, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

bool OverlapOnAxis(const Triangle& t1, const Triangle& t2, const math::vec3& axis) {
	Interval a = GetInterval(t1, axis);
	Interval b = GetInterval(t2, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

Interval GetInterval(const Triangle& triangle, const math::vec3& axis) {
	Interval result;

	result.min = math::dot(axis, triangle.GetPoint(0));
	result.max = result.min;
	for (int i = 1; i < 3; ++i) {
		math::real value = math::dot(axis, triangle.GetPoint(i));
		result.min = fminf(result.min, value);
		result.max = fmaxf(result.max, value);
	}

	return result;
}

Interval GetInterval(const OBB& obb, const math::vec3& axis) {
	math::vec3 vertex[8];

	math::vec3 C = obb.position;	// OBB Center
	math::vec3 E = obb.size;		// OBB Extents
	const math::vec3 A[] = {        // OBB Axis
		obb.orientation[0],         // X axis
		obb.orientation[1],         // Y axis
		obb.orientation[2]          // Z axis
 };

	vertex[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	vertex[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	Interval result;
	result.min = result.max = math::dot(axis, vertex[0]);

	for (int i = 1; i < 8; ++i) {
		math::real projection = math::dot(axis, vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

Interval GetInterval(const AABB& aabb, const math::vec3& axis) {
	math::vec3 i = GetMin(aabb);
	math::vec3 a = GetMax(aabb);

	math::vec3 vertex[8] = {
		math::vec3(i.x, a.y, a.z),
		math::vec3(i.x, a.y, i.z),
		math::vec3(i.x, i.y, a.z),
		math::vec3(i.x, i.y, i.z),
		math::vec3(a.x, a.y, a.z),
		math::vec3(a.x, a.y, i.z),
		math::vec3(a.x, i.y, a.z),
		math::vec3(a.x, i.y, i.z)
	};

	Interval result;
	result.min = result.max = math::dot(axis, vertex[0]);

	for (int i = 1; i < 8; ++i) {
		math::real projection = math::dot(axis, vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

bool AABBPlane(const AABB& aabb, const Plane& plane) {
	// math::project the half extents of the AABB onto the plane normal
	math::real pLen =aabb.size.x * fabsf(plane.normal.x) +
				aabb.size.y * fabsf(plane.normal.y) +
				aabb.size.z * fabsf(plane.normal.z);
	// Find the distance from the center of the AABB to the plane
	math::real dist = math::dot(plane.normal, aabb.position) - plane.distance;
	// Intersection occurs if the distance falls within the projected side
	return fabsf(dist) <= pLen;
}

bool OBBOBB(const OBB& obb1, const OBB& obb2) {
	math::vec3 test[15] = {
	// Axes de obb1
	obb1.orientation[0],  // Colonne 0 (axe X)
	obb1.orientation[1],  // Colonne 1 (axe Y)
	obb1.orientation[2],  // Colonne 2 (axe Z)
	// Axes de obb2
	obb2.orientation[0],  // Colonne 0 (axe X)
	obb2.orientation[1],  // Colonne 1 (axe Y)
	obb2.orientation[2]   // Colonne 2 (axe Z)
};

	for (int i = 0; i < 3; ++i) { // Fill out rest of axis
		test[6 + i * 3 + 0] = math::cross(test[i], test[0]);
		test[6 + i * 3 + 1] = math::cross(test[i], test[1]);
		test[6 + i * 3 + 2] = math::cross(test[i], test[2]);
	}

	for (int i = 0; i < 15; ++i) {
		if (!OverlapOnAxis(obb1, obb2, test[i])) {
			return false; // Seperating axis found
		}
	}

	return true; // Seperating axis not found
}

bool OBBPlane(const OBB& obb, const Plane& plane) {
	// Local variables for readability only
	// Récupérer les trois axes d'orientation de l'OBB (colonnes de la matrice)
	const math::vec3 rot[] = {
		obb.orientation[0],  // Axe local X
		obb.orientation[1],  // Axe local Y
		obb.orientation[2]   // Axe local Z
};
	math::vec3 normal = plane.normal;

	// math::project the half extents of the AABB onto the plane normal
	math::real pLen =obb.size.x * fabsf(math::dot(normal, rot[0])) +
				obb.size.y * fabsf(math::dot(normal, rot[1])) +
				obb.size.z * fabsf(math::dot(normal, rot[2]));
	// Find the distance from the center of the OBB to the plane
	math::real dist = math::dot(plane.normal, obb.position) - plane.distance;
	// Intersection occurs if the distance falls within the projected side
	return fabsf(dist) <= pLen;
}

bool PlanePlane(const Plane& plane1, const Plane& plane2) {
	// Compute direction of intersection line
	math::vec3 d = math::cross(plane1.normal, plane2.normal);

	// Check the length of the direction line
	// if the length is 0, no intersection happened
	return !(CMP(math::dot(d, d), 0));

	// We could have used the math::dot product here, instead of the math::cross product
}

bool Raycast(const Sphere& sphere, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);

	math::vec3 e = sphere.position - ray.origin;
	math::real rSq = sphere.radius * sphere.radius;

	math::real eSq = math::lengthSq(e);
	math::real a = math::dot(e, ray.direction); // ray.direction is assumed to be math::normalized
	math::real bSq = /*sqrtf(*/eSq - (a * a)/*)*/;
	math::real f = sqrt(fabsf((rSq)- /*(b * b)*/bSq));

	// Assume normal intersection!
	math::real t = a - f;

	// No collision has happened
	if (rSq - (eSq - a * a) < 0.0f) {
		return false;
	}
	// Ray starts inside the sphere
	else if (eSq < rSq) {
		// Just reverse direction
		t = a + f;
	}
	if (outResult != 0) {
		outResult->t = t;
		outResult->hit = true;
		outResult->point = ray.origin + ray.direction * t;
		outResult->normal = math::normalized(outResult->point - sphere.position);
	}
	return true;
}

bool Raycast(const OBB& obb, const Ray& ray, RaycastResult* outResult) {
   ResetRaycastResult(outResult);

   math::vec3 p = obb.position - ray.origin;

   // Axes de l'OBB
   const math::vec3& X = obb.orientation[0];
   const math::vec3& Y = obb.orientation[1];
   const math::vec3& Z = obb.orientation[2];

   // Projections du rayon et du vecteur p sur les axes
   math::vec3 f(
       math::dot(X, ray.direction),
       math::dot(Y, ray.direction),
       math::dot(Z, ray.direction)
   );

   math::vec3 e(
       math::dot(X, p),
       math::dot(Y, p),
       math::dot(Z, p)
   );

#if 1
   math::real t[6] = { 0, 0, 0, 0, 0, 0 };
   for (int i = 0; i < 3; ++i) {
       if (CMP(f[i], 0)) {
           if (-e[i] - obb.size[i] > 0 || -e[i] + obb.size[i] < 0) {
               return false;
           }
           f[i] = 0.00001f; // Avoid div by 0!
       }

       t[i * 2 + 0] = (e[i] + obb.size[i]) / f[i]; // tmin[x, y, z]
       t[i * 2 + 1] = (e[i] - obb.size[i]) / f[i]; // tmax[x, y, z]
   }

   math::real tmin = fmaxf(fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), fminf(t[4], t[5]));
   math::real tmax = fminf(fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])), fmaxf(t[4], t[5]));
#else
   // The above loop simplifies the below if statements
   if (CMP(f.x, 0)) {
       if (-e.x - obb.size.x > 0 || -e.x + obb.size.x < 0) {
           return false;
       }
       f.x = 0.00001f; // Avoid div by 0!
   }
   else if (CMP(f.y, 0)) {
       if (-e.y - obb.size.y > 0 || -e.y + obb.size.y < 0) {
           return false;
       }
       f.y = 0.00001f; // Avoid div by 0!
   }
   else if (CMP(f.z, 0)) {
       if (-e.z - obb.size.z > 0 || -e.z + obb.size.z < 0) {
           return false;
       }
       f.z = 0.00001f; // Avoid div by 0!
   }

   math::real t1 = (e.x + obb.size.x) / f.x;
   math::real t2 = (e.x - obb.size.x) / f.x;
   math::real t3 = (e.y + obb.size.y) / f.y;
   math::real t4 = (e.y - obb.size.y) / f.y;
   math::real t5 = (e.z + obb.size.z) / f.z;
   math::real t6 = (e.z - obb.size.z) / f.z;

   math::real tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
   math::real tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));
#endif

   // if tmax < 0, ray is intersecting AABB
   // but entire AABB is behind its origin
   if (tmax < 0) {
       return false;
   }

   // if tmin > tmax, ray doesn't intersect AABB
   if (tmin > tmax) {
       return false;
   }

   // If tmin is < 0, tmax is closer
   math::real t_result = tmin;
   if (tmin < 0.0f) {
       t_result = tmax;
   }

   if (outResult != nullptr) {
       outResult->hit = true;
       outResult->t = t_result;
       outResult->point = ray.origin + ray.direction * t_result;

       math::vec3 normals[] = {
           X,            // +x
           X * -1.0f,    // -x
           Y,            // +y
           Y * -1.0f,    // -y
           Z,            // +z
           Z * -1.0f     // -z
       };

       for (int i = 0; i < 6; ++i) {
           if (CMP(t_result, t[i])) {
               outResult->normal = math::normalized(normals[i]);
           }
       }
   }
   return true;
}

void ResetRaycastResult(RaycastResult* outResult) {
	if (outResult != 0) {
		outResult->t = -1;
		outResult->hit = false;
		outResult->normal = math::vec3(0, 0, 1);
		outResult->point = math::vec3(0, 0, 0);
	}
}

bool Raycast(const AABB& aabb, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);

	math::vec3 min = GetMin(aabb);
	math::vec3 max = GetMax(aabb);

	// Any component of direction could be 0!
	// Address this by using a small number, close to
	// 0 in case any of directions components are 0
	math::real t1 = (min.x - ray.origin.x) / (CMP(ray.direction.x, 0.0f) ? 0.00001f : ray.direction.x);
	math::real t2 = (max.x - ray.origin.x) / (CMP(ray.direction.x, 0.0f) ? 0.00001f : ray.direction.x);
	math::real t3 = (min.y - ray.origin.y) / (CMP(ray.direction.y, 0.0f) ? 0.00001f : ray.direction.y);
	math::real t4 = (max.y - ray.origin.y) / (CMP(ray.direction.y, 0.0f) ? 0.00001f : ray.direction.y);
	math::real t5 = (min.z - ray.origin.z) / (CMP(ray.direction.z, 0.0f) ? 0.00001f : ray.direction.z);
	math::real t6 = (max.z - ray.origin.z) / (CMP(ray.direction.z, 0.0f) ? 0.00001f : ray.direction.z);

	math::real tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
	math::real tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));

	// if tmax < 0, ray is intersecting AABB
	// but entire AABB is behing it's origin
	if (tmax < 0) {
		return false;
	}

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax) {
		return false;
	}

	math::real t_result = tmin;

	// If tmin is < 0, tmax is closer
	if (tmin < 0.0f) {
		t_result = tmax;
	}

	if (outResult != 0) {
		outResult->t = t_result;
		outResult->hit = true;
		outResult->point = ray.origin + ray.direction * t_result;

		math::vec3 normals[] = {
			math::vec3(-1, 0, 0),
			math::vec3(1, 0, 0),
			math::vec3(0, -1, 0),
			math::vec3(0, 1, 0),
			math::vec3(0, 0, -1),
			math::vec3(0, 0, 1)
		};
		math::real t[] = { t1, t2, t3, t4, t5, t6 };

		for (int i = 0; i < 6; ++i) {
			if (CMP(t_result, t[i])) {
				outResult->normal = normals[i];
			}
		}
	}

	return true;
}

bool Raycast(const Plane& plane, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);

	math::real nd = math::dot(ray.direction, plane.normal);
	math::real pn = math::dot(ray.origin, plane.normal);

	// nd must be negative, and not 0
	// if nd is positive, the ray and plane normals
	// point in the same direction. No intersection.
	if (nd >= 0.0f) {
		return false;
	}

	math::real t = (plane.distance - pn) / nd;

	// t must be positive
	if (t >= 0.0f) {
		if (outResult != 0) {
			outResult->t = t;
			outResult->hit = true;
			outResult->point = ray.origin + ray.direction * t;
			outResult->normal = math::normalized(plane.normal);
		}
		return true;
	}

	return false;
}

bool Linetest(const Sphere& sphere, const Line& line) {
	Point closest = ClosestPoint(line, sphere.position);
	math::real distSq = math::lengthSq(sphere.position - closest);
	return distSq <= (sphere.radius * sphere.radius);
}

bool Linetest(const Plane& plane, const Line& line) {
	math::vec3 ab = line.end - line.start;

	math::real nA = math::dot(plane.normal, line.start);
	math::real nAB = math::dot(plane.normal, ab);

	if (CMP(nAB, 0)) {
		return false;
	}

	math::real t = (plane.distance - nA) / nAB;
	return t >= 0.0f && t <= 1.0f;
}

bool Linetest(const AABB& aabb, const Line& line) {
	Ray ray;
	ray.origin = line.start;
	ray.direction = math::normalized(line.end - line.start);
	RaycastResult raycast;
	if (!Raycast(aabb, ray, &raycast)) {
		return false;
	}
	math::real t = raycast.t;

	return t >= 0 && t * t <= LengthSq(line);
}

bool Linetest(const OBB& obb, const Line& line) {
	if (math::lengthSq(line.end - line.start) < 0.0000001f) {
		return PointInOBB(line.start, obb);
	}
	Ray ray;
	ray.origin = line.start;
	ray.direction = math::normalized(line.end - line.start);
	RaycastResult result;
	if (!Raycast(obb, ray, &result)) {
		return false;
	}
	math::real t = result.t;

	return t >= 0 && t * t <= LengthSq(line);
}

bool Raycast(const Ray& ray, const Sphere& sphere, RaycastResult* outResult) {
	return Raycast(sphere, ray, outResult);
}

bool Raycast(const Ray& ray, const AABB& aabb, RaycastResult* outResult) {
	return Raycast(aabb, ray, outResult);
}

bool Raycast(const Ray& ray, const OBB& obb, RaycastResult* outResult) {
	return Raycast(obb, ray, outResult);
}

bool Raycast(const Ray& ray, const Plane& plane, RaycastResult* outResult) {
	return Raycast(plane, ray, outResult);
}

bool Linetest(const Line& line, const Sphere& sphere) {
	return Linetest(sphere, line);
}

bool Linetest(const Line& line, const AABB& aabb) {
	return Linetest(aabb, line);
}

bool Linetest(const Line& line, const OBB& obb) {
	return Linetest(obb, line);
}

bool Linetest(const Line& line, const Plane& plane) {
	return Linetest(plane, line);
}

math::vec3 Centroid(const Triangle& t) {
	Point a = t.GetA();
	Point b = t.GetB();
	Point c = t.GetC();

	math::vec3 result;
	result.x = a.x + b.x + c.x;
	result.y = a.y + b.y + c.y;
	result.z = a.z + b.z + c.z;
	result = result * (1.0f / 3.0f);
	return result;
}

bool PointInTriangle(const Point& p, const Triangle& t) {
	// Move the triangle so that the point is  
	// now at the origin of the triangle
	math::vec3 a = t.GetA() - p;
	math::vec3 b = t.GetB() - p;
	math::vec3 c = t.GetC() - p;

	// The point should be moved too, so they are both
	// relative, but because we don't use p in the
	// equation anymore, we don't need it!
	// p -= p; // This would just equal the zero vector!

	math::vec3 normPBC = math::cross(b, c); // Normal of PBC (u)
	math::vec3 normPCA = math::cross(c, a); // Normal of PCA (v)
	math::vec3 normPAB = math::cross(a, b); // Normal of PAB (w)

	// Test to see if the normals are facing 
	// the same direction, return false if not
	if (math::dot(normPBC, normPCA) < 0.0f) {
		return false;
	}
	else if (math::dot(normPBC, normPAB) < 0.0f) {
		return false;
	}

	// All normals facing the same way, return true
	return true;
}

math::vec3 BarycentricOptimized(const Point& p, const Triangle& t) {
	math::vec3 v0 = t.GetB() - t.GetA();
	math::vec3 v1 = t.GetC() - t.GetA();
	math::vec3 v2 = p - t.GetA();

	math::real d00 = math::dot(v0, v0);
	math::real d01 = math::dot(v0, v1);
	math::real d11 = math::dot(v1, v1);
	math::real d20 = math::dot(v2, v0);
	math::real d21 = math::dot(v2, v1);
	math::real denom = d00 * d11 - d01 * d01;

	if (CMP(denom, 0.0f)) {
		return math::vec3();
	}

	math::vec3 result;
	result.y = (d11 * d20 - d01 * d21) / denom;
	result.z = (d00 * d21 - d01 * d20) / denom;
	result.x = 1.0f - result.y - result.z;

	return result;
}

math::vec3 Barycentric(const Point& p, const Triangle& t) {
	math::vec3 ap = p - t.GetA();
	math::vec3 bp = p - t.GetB();
	math::vec3 cp = p - t.GetC();

	math::vec3 ab = t.GetB() - t.GetA();
	math::vec3 ac = t.GetC() - t.GetA();
	math::vec3 bc = t.GetC() - t.GetB();
	math::vec3 cb = t.GetB() - t.GetC();
	math::vec3 ca = t.GetA() - t.GetC();

	math::vec3 v = ab - math::project(ab, cb);
	math::real a = 1.0f - (math::dot(v, ap) / math::dot(v, ab));

	v = bc - math::project(bc, ac);
	math::real b = 1.0f - (math::dot(v, bp) / math::dot(v, bc));

	v = ca - math::project(ca, ab);
	math::real c = 1.0f - (math::dot(v, cp) / math::dot(v, ca));

	return math::vec3(a, b, c);
}

Plane FromTriangle(const Triangle& t) {
	Plane result;
	result.normal = math::normalized(math::cross(t.GetB() - t.GetA(), t.GetC() - t.GetA()));
	result.distance = math::dot(result.normal, t.GetA());
	return result;
}

Point ClosestPoint(const Triangle& t, const Point& p) {
	Plane plane = FromTriangle(t);
	Point closest = ClosestPoint(plane, p);

	// Closest point was inside triangle
	if (PointInTriangle(closest, t)) {
		return closest;
	}

	Point c1 = ClosestPoint(Line(t.GetA(), t.GetB()), closest); // Line AB
	Point c2 = ClosestPoint(Line(t.GetB(), t.GetC()), closest); // Line BC
	Point c3 = ClosestPoint(Line(t.GetC(), t.GetA()), closest); // Line CA

	math::real magSq1 = math::lengthSq(closest - c1);
	math::real magSq2 = math::lengthSq(closest - c2);
	math::real magSq3 = math::lengthSq(closest - c3);

	if (magSq1 < magSq2 && magSq1 < magSq3) {
		return c1;
	}
	else if (magSq2 < magSq1 && magSq2 < magSq3) {
		return c2;
	}

	return c3;
}

bool TriangleSphere(const Triangle& t, const Sphere& s) {
	Point closest = ClosestPoint(t, s.position);
	math::real magSq = math::lengthSq(closest - s.position);
	return magSq <= s.radius * s.radius;
}

bool TriangleAABB(const Triangle& t, const AABB& a) {
	// Compute the edge vectors of the triangle  (ABC)
	math::vec3 f0 = t.GetC() - t.GetA();
	math::vec3 f1 = t.GetC() - t.GetB();
	math::vec3 f2 = t.GetA() - t.GetC();

	// Compute the face normals of the AABB
	math::vec3 u0(1.0f, 0.0f, 0.0f);
	math::vec3 u1(0.0f, 1.0f, 0.0f);
	math::vec3 u2(0.0f, 0.0f, 1.0f);

	math::vec3 test[13] = {
		// 3 Normals of AABB
		u0, // AABB Axis 1
		u1, // AABB Axis 2
		u2, // AABB Axis 3
		// 1 Normal of the Triangle
		math::cross(f0, f1),
		// 9 Axis, math::cross products of all edges
		math::cross(u0, f0),
		math::cross(u0, f1),
		math::cross(u0, f2),
		math::cross(u1, f0),
		math::cross(u1, f1),
		math::cross(u1, f2),
		math::cross(u2, f0),
		math::cross(u2, f1),
		math::cross(u2, f2)
	};

	for (int i = 0; i < 13; ++i) {
		if (!OverlapOnAxis(a, t, test[i])) {
			return false; // Seperating axis found
		}
	}

	return true; // Seperating axis not found
}

bool TriangleOBB(const Triangle& t, const OBB& o) {
	// Compute the edge vectors of the triangle (ABC)
	math::vec3 f0 = t.GetB() - t.GetA();
	math::vec3 f1 = t.GetC() - t.GetB();
	math::vec3 f2 = t.GetA() - t.GetC();

	// Compute the face normals of the AABB
	const math::vec3 u0 = o.orientation[0];  // GLM gère bien le const
	const math::vec3 u1 = o.orientation[1];
	const math::vec3 u2 = o.orientation[2];

	math::vec3 test[13] = {
		// 3 Normals of AABB
		u0, // AABB Axis 1
		u1, // AABB Axis 2
		u2, // AABB Axis 3
		// 1 Normal of the Triangle
		math::cross(f0, f1),
		// 9 Axis, cross products of all edges
		math::cross(u0, f0),
		math::cross(u0, f1),
		math::cross(u0, f2),
		math::cross(u1, f0),
		math::cross(u1, f1),
		math::cross(u1, f2),
		math::cross(u2, f0),
		math::cross(u2, f1),
		math::cross(u2, f2)
};

	for (int i = 0; i < 13; ++i) {
		if (!OverlapOnAxis(o, t, test[i])) {
			return false; // Separating axis found
		}
	}

	return true; // Separating axis not found
}
bool TriangleTriangle(const Triangle& t1, const Triangle& t2) {
#if 0
	math::vec3 axisToTest[] = {
		// Triangle 1, Normal
		SatCrossEdge(t1.a, t1.b, t1.b, t1.c),
		// Triangle 2, Normal
		SatCrossEdge(t2.a, t2.b, t2.b, t2.c),

		// math::cross Product of edges
		SatCrossEdge(t2.a, t2.b, t1.a, t1.b),
		SatCrossEdge(t2.a, t2.b, t1.b, t1.c),
		SatCrossEdge(t2.a, t2.b, t1.c, t1.a),

		SatCrossEdge(t2.b, t2.c, t1.a, t1.b),
		SatCrossEdge(t2.b, t2.c, t1.b, t1.c),
		SatCrossEdge(t2.b, t2.c, t1.c, t1.a),

		SatCrossEdge(t2.c, t2.a, t1.a, t1.b),
		SatCrossEdge(t2.c, t2.a, t1.b, t1.c),
		SatCrossEdge(t2.c, t2.a, t1.c, t1.a),
	};
#else 
	math::vec3 t1_f0 = t1.GetB() - t1.GetA(); // Edge 0
	math::vec3 t1_f1 = t1.GetC() - t1.GetB(); // Edge 1
	math::vec3 t1_f2 = t1.GetA() - t1.GetC(); // Edge 2

	math::vec3 t2_f0 = t2.GetB() - t2.GetA(); // Edge 0
	math::vec3 t2_f1 = t2.GetC() - t2.GetB(); // Edge 1
	math::vec3 t2_f2 = t2.GetA() - t2.GetC(); // Edge 2

	math::vec3 axisToTest[] = {
		// Triangle 1, Normal
		math::cross(t1_f0, t1_f1),
		// Triangle 2, Normal
		math::cross(t2_f0, t2_f1),

		// math::cross Product of edges
		math::cross(t2_f0, t1_f0),
		math::cross(t2_f0, t1_f1),
		math::cross(t2_f0, t1_f2),

		math::cross(t2_f1, t1_f0),
		math::cross(t2_f1, t1_f1),
		math::cross(t2_f1, t1_f2),

		math::cross(t2_f2, t1_f0),
		math::cross(t2_f2, t1_f1),
		math::cross(t2_f2, t1_f2),
	};
#endif

	for (int i = 0; i < 11; ++i) {
		if (!OverlapOnAxis(t1, t2, axisToTest[i])) {
			return false; // Seperating axis found
		}
	}

	return true; // Seperating axis not found
}

bool TriangleTriangleRobust(const Triangle& t1, const Triangle& t2) {
	math::vec3 axisToTest[] = {
		// Triangle 1, Normal
		SatCrossEdge(t1.GetA(), t1.GetB(), t1.GetB(), t1.GetC()),
		// Triangle 2, Normal
		SatCrossEdge(t2.GetA(), t2.GetB(), t2.GetB(), t2.GetC()),

		// math::cross Product of edges
		SatCrossEdge(t2.GetA(), t2.GetB(), t1.GetA(), t1.GetB()),
		SatCrossEdge(t2.GetA(), t2.GetB(), t1.GetB(), t1.GetC()),
		SatCrossEdge(t2.GetA(), t2.GetB(), t1.GetC(), t1.GetA()),

		SatCrossEdge(t2.GetB(), t2.GetC(), t1.GetA(), t1.GetB()),
		SatCrossEdge(t2.GetB(), t2.GetC(), t1.GetB(), t1.GetC()),
		SatCrossEdge(t2.GetB(), t2.GetC(), t1.GetC(), t1.GetA()),

		SatCrossEdge(t2.GetC(), t2.GetA(), t1.GetA(), t1.GetB()),
		SatCrossEdge(t2.GetC(), t2.GetA(), t1.GetB(), t1.GetC()),
		SatCrossEdge(t2.GetC(), t2.GetA(), t1.GetC(), t1.GetA()),
	};

	for (int i = 0; i < 11; ++i) {
		if (!OverlapOnAxis(t1, t2, axisToTest[i])) {
			if (!CMP(math::lengthSq(axisToTest[i]), 0)) {
				return false; // Seperating axis found
			}
		}
	}

	return true; // Seperating axis not found
}

math::vec3 SatCrossEdge(const math::vec3& a, const math::vec3& b, const math::vec3& c, const math::vec3& d) {
	math::vec3 ab = b - a;
	math::vec3 cd = d - c;

	math::vec3 result = math::cross(ab, cd);
	if (!CMP(math::lengthSq(result), 0)) { // Is ab and cd parallel?
		return result; // Not parallel!
	}
	else { // ab and cd are parallel
		// Get an axis perpendicular to AB
		math::vec3 axis = math::cross(ab, c - a);
		result = math::cross(ab, axis);
		if (!CMP(math::lengthSq(result), 0)) { // Still parallel?
			return result; // Not parallel
		}
	}
	// New axis being tested is parallel too.
	// This means that a, b, c and d are on a line
	// Nothing we can do!
	return math::vec3();
}

Point debugRaycastResult;

bool Raycast(const Triangle& triangle, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);
	Plane plane = FromTriangle(triangle);

	RaycastResult planeResult;
	if (!Raycast(plane, ray, &planeResult)) {
		return false;
	}
	math::real t = planeResult.t;

	Point result = ray.origin + ray.direction * t;
	
	math::vec3 barycentric = Barycentric(result, triangle);
	if (barycentric.x >= 0.0f && barycentric.x <= 1.0f &&
		barycentric.y >= 0.0f && barycentric.y <= 1.0f &&
		barycentric.z >= 0.0f && barycentric.z <= 1.0f) {

		if (outResult != 0) {
			outResult->t = t;
			outResult->hit = true;
			outResult->point = ray.origin + ray.direction * t;
			outResult->normal = plane.normal;
		}

		return true;
	}

	return false;
}

bool Linetest(const Triangle& triangle, const Line& line) {
	Ray ray;
	ray.origin = line.start;
	ray.direction = math::normalized(line.end - line.start);
	RaycastResult raycast;
	if (!Raycast(triangle, ray, &raycast)) {
		return false;
	}
	math::real t = raycast.t;

	return t >= 0 && t * t <= LengthSq(line);
}

void AccelerateMesh(Mesh& mesh) {
	if (mesh.accelerator != 0) {
		return;
	}

	math::vec3 min = mesh.vertices[0];
	math::vec3 max = mesh.vertices[0];

	for (int i = 1; i < mesh.numTriangles * 3; ++i) {
		min.x = fminf(mesh.vertices[i].x, min.x);
		min.y = fminf(mesh.vertices[i].y, min.y);
		min.z = fminf(mesh.vertices[i].z, min.z);
	
		max.x = fmaxf(mesh.vertices[i].x, max.x);
		max.y = fmaxf(mesh.vertices[i].y, max.y);
		max.z = fmaxf(mesh.vertices[i].z, max.z);
	}

	mesh.accelerator = new BVHNode();
	mesh.accelerator->bounds = FromMinMax(min, max);
	mesh.accelerator->children = 0;
	mesh.accelerator->numTriangles = mesh.numTriangles;
	mesh.accelerator->triangles = new int[mesh.numTriangles];
	for (int i = 0; i < mesh.numTriangles; ++i) {
		mesh.accelerator->triangles[i] = i;
	}

	SplitBVHNode(mesh.accelerator, mesh, 3);
}

void SplitBVHNode(BVHNode* node, const Mesh& model, int depth) {
	if (depth-- <= 0) { // Decrements depth
		return;
	}

	if (node->children == 0) {
		// Only split if this node contains triangles
		if (node->numTriangles > 0) {
			node->children = new BVHNode[8];

			math::vec3 c = node->bounds.position;
			math::vec3 e = node->bounds.size *0.5f;

			node->children[0].bounds = AABB(c + math::vec3(-e.x, +e.y, -e.z), e);
			node->children[1].bounds = AABB(c + math::vec3(+e.x, +e.y, -e.z), e);
			node->children[2].bounds = AABB(c + math::vec3(-e.x, +e.y, +e.z), e);
			node->children[3].bounds = AABB(c + math::vec3(+e.x, +e.y, +e.z), e);
			node->children[4].bounds = AABB(c + math::vec3(-e.x, -e.y, -e.z), e);
			node->children[5].bounds = AABB(c + math::vec3(+e.x, -e.y, -e.z), e);
			node->children[6].bounds = AABB(c + math::vec3(-e.x, -e.y, +e.z), e);
			node->children[7].bounds = AABB(c + math::vec3(+e.x, -e.y, +e.z), e);

		}
	}

	// If this node was just split
	if (node->children != 0 && node->numTriangles > 0) {
		for (int i = 0; i < 8; ++i) { // For each child
			// Count how many triangles each child will contain
			node->children[i].numTriangles = 0;
			for (int j = 0; j < node->numTriangles; ++j) {
				Triangle t = model.triangles[node->triangles[j]];
				if (TriangleAABB(t, node->children[i].bounds)) {
					node->children[i].numTriangles += 1;
				}
			}
			if (node->children[i].numTriangles == 0) {
				continue;
			}
			node->children[i].triangles = new int[node->children[i].numTriangles];
			int index = 0; // Add the triangles in the new child arrau
			for (int j = 0; j < node->numTriangles; ++j) {
				Triangle t = model.triangles[node->triangles[j]];
				if (TriangleAABB(t, node->children[i].bounds)) {
					node->children[i].triangles[index++] = node->triangles[j];
				}
			}
		}

		node->numTriangles = 0;
		delete[] node->triangles;
		node->triangles = 0;

		// Recurse
		for (int i = 0; i < 8; ++i) {
			SplitBVHNode(&node->children[i], model, depth);
		}
	}
}

void FreeBVHNode(BVHNode* node) {
	if (node->children != 0) {
		for (int i = 0; i < 8; ++i) {
			FreeBVHNode(&node->children[i]);
		}
		delete[] node->children;
		node->children = 0;
	}

	if (node->numTriangles != 0 || node->triangles != 0) {
		delete[] node->triangles;
		node->triangles = 0;
		node->numTriangles = 0;
	}
}

bool MeshAABB(const Mesh& mesh, const AABB& aabb) {
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (TriangleAABB(mesh.triangles[i], aabb)) {
				return true;
			}
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					// Triangle indices in BVHNode index the mesh
					if (TriangleAABB(mesh.triangles[iterator->triangles[i]], aabb)) {
						return true;
					}
				}
			}

			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (AABBAABB(iterator->children[i].bounds, aabb)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool Linetest(const Mesh& mesh, const Line& line) {
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (Linetest(mesh.triangles[i], line)) {
				return true;
			}
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					// Triangle indices in BVHNode index the mesh
					if (Linetest(mesh.triangles[iterator->triangles[i]], line)) {
						return true;
					}
				}
			}

			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (Linetest(iterator->children[i].bounds, line)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshSphere(const Mesh& mesh, const Sphere& sphere) {
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (TriangleSphere(mesh.triangles[i], sphere)) {
				return true;
			}
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					// Triangle indices in BVHNode index the mesh
					if (TriangleSphere(mesh.triangles[iterator->triangles[i]], sphere)) {
						return true;
					}
				}
			}

			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (SphereAABB(sphere, iterator->children[i].bounds)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshOBB(const Mesh& mesh, const OBB& obb) {
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (TriangleOBB(mesh.triangles[i], obb)) {
				return true;
			}
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					// Triangle indices in BVHNode index the mesh
					if (TriangleOBB(mesh.triangles[iterator->triangles[i]], obb)) {
						return true;
					}
				}
			}

			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (AABBOBB(iterator->children[i].bounds, obb)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshPlane(const Mesh& mesh, const Plane& plane) {
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (TrianglePlane(mesh.triangles[i], plane)) {
				return true;
			}
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					// Triangle indices in BVHNode index the mesh
					if (TrianglePlane(mesh.triangles[iterator->triangles[i]], plane)) {
						return true;
					}
				}
			}

			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (AABBPlane(iterator->children[i].bounds, plane)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshTriangle(const Mesh& mesh, const Triangle& triangle) {
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (TriangleTriangle(mesh.triangles[i], triangle)) {
				return true;
			}
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					// Triangle indices in BVHNode index the mesh
					if (TriangleTriangle(mesh.triangles[iterator->triangles[i]], triangle)) {
						return true;
					}
				}
			}

			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (TriangleAABB(triangle, iterator->children[i].bounds)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

math::real Raycast(const Mesh& mesh, const Ray& ray) {
	return MeshRay(mesh, ray);
}

math::real Raycast(const Model& mesh, const Ray& ray) {
	return ModelRay(mesh, ray);
}

math::real MeshRay(const Mesh& mesh, const Ray& ray) {
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			RaycastResult raycast;
			Raycast(mesh.triangles[i], ray, &raycast);
			math::real result = raycast.t;
			if (result >= 0) {
				return result;
			}
		}
	}
	else {
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					// Triangle indices in BVHNode index the mesh
					RaycastResult raycast;
					Raycast(mesh.triangles[iterator->triangles[i]], ray, &raycast);
					math::real r = raycast.t;
					if (r >= 0) {
						return r;
					}
				}
			}

			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					RaycastResult raycast;
					Raycast(iterator->children[i].bounds, ray, &raycast);
					if (raycast.t >= 0) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return -1;
}

bool TrianglePlane(const Triangle& t, const Plane& p) {
	math::real side1 = PlaneEquation(t.GetC(), p);
	math::real side2 = PlaneEquation(t.GetB(), p);
	math::real side3 = PlaneEquation(t.GetC(), p);

	// On Plane
	if (CMP(side1, 0) && CMP(side2, 0) && CMP(side3, 0)) {
		return true;
	}

	// Triangle in front of plane
	if (side1 > 0 && side2 > 0 && side3 > 0) {
		return false;
	}

	// Triangle behind plane
	if (side1 < 0 && side2 < 0 && side3 < 0) {
		return false;
	}

	return true; // Intersection
}

void Model::SetContent(Mesh* mesh) {
	content = mesh;
	if (content != 0) {
		math::vec3 min = mesh->vertices[0];
		math::vec3 max = mesh->vertices[0];

		for (int i = 1; i < mesh->numTriangles * 3; ++i) {
			min.x = fminf(mesh->vertices[i].x, min.x);
			min.y = fminf(mesh->vertices[i].y, min.y);
			min.z = fminf(mesh->vertices[i].z, min.z);

			max.x = fmaxf(mesh->vertices[i].x, max.x);
			max.y = fmaxf(mesh->vertices[i].y, max.y);
			max.z = fmaxf(mesh->vertices[i].z, max.z);
		}
		bounds = FromMinMax(min, max);
	}
}

math::mat4 GetWorldMatrix(const Model& model) {
	math::mat4 translation = math::translation(model.position);
	math::mat4 rotation = math::rotation(model.rotation.x, model.rotation.y, model.rotation.z);
	math::mat4 localMat = /* Scale * */ rotation * translation;
	
	math::mat4 parentMat;
	if (model.parent != 0) {
		parentMat = GetWorldMatrix(*model.parent);
	}

	return localMat * parentMat;
}

OBB GetOBB(const Model& model) {
	math::mat4 world = GetWorldMatrix(model);
	AABB aabb = model.GetBounds();
	OBB obb;

	obb.size = aabb.size;
	obb.position = math::multiplyPoint(aabb.position, world);
	obb.orientation = math::cut(world, 3, 3);

	return obb;
}

math::real ModelRay(const Model& model, const Ray& ray) {
	math::mat4 world = GetWorldMatrix(model);
	math::mat4 inv = math::inverse(world);
	Ray local;
	local.origin =math::multiplyPoint(ray.origin, inv);
	local.direction = math::multiplyVector(ray.direction, inv);
	local.NormalizeDirection();
	if (model.GetMesh() != 0) {
		return MeshRay(*(model.GetMesh()), local);
	}
	return -1;
}

bool Linetest(const Model& model, const Line& line) {
	math::mat4 world = GetWorldMatrix(model);
	math::mat4 inv = math::inverse(world);
	Line local;
	local.start = math::multiplyPoint(line.start, inv);
	local.end = math::multiplyPoint(line.end, inv);
	if (model.GetMesh() != 0) {
		return Linetest(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelSphere(const Model& model, const Sphere& sphere) {
	math::mat4 world = GetWorldMatrix(model);
	math::mat4 inv = math::inverse(world);
	Sphere local;
	local.position = math::multiplyPoint(sphere.position, inv);
	if (model.GetMesh() != 0) {
		return MeshSphere(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelAABB(const Model& model, const AABB& aabb) {
	math::mat4 world = GetWorldMatrix(model);
	math::mat4 inv = math::inverse(world);
	OBB local;
	local.size = aabb.size;
	local.position = math::multiplyPoint(aabb.position, inv);
	local.orientation = math::cut(inv, 3, 3);
	if (model.GetMesh() != 0) {
		return MeshOBB(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelOBB(const Model& model, const OBB& obb) {
	math::mat4 world = GetWorldMatrix(model);
	math::mat4 inv = math::inverse(world);
	OBB local;
	local.size = obb.size;
	local.position = math::multiplyPoint(obb.position, inv);
	local.orientation = obb.orientation * math::cut(inv, 3, 3);
	if (model.GetMesh() != 0) {
		return MeshOBB(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelPlane(const Model& model, const Plane& plane) {
	math::mat4 world = GetWorldMatrix(model);
	math::mat4 inv = math::inverse(world);
	Plane local;
	local.normal = math::multiplyVector(plane.normal, inv);
	local.distance = plane.distance;
	if (model.GetMesh() != 0) {
		return MeshPlane(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelTriangle(const Model& model, const Triangle& triangle) {
	math::mat4 world = GetWorldMatrix(model);
	math::mat4 inv = math::inverse(world);
	Triangle local;
	local.GetA() = math::multiplyPoint(triangle.GetA(), inv);
	local.GetB() = math::multiplyPoint(triangle.GetB(), inv);
	local.GetC() = math::multiplyPoint(triangle.GetC(), inv);
	if (model.GetMesh() != 0) {
		return MeshTriangle(*(model.GetMesh()), local);
	}
	return false;
}

Point Intersection(Plane p1, Plane p2, Plane p3) {
	/*return ((math::cross(p2.normal, p3.normal) * -p1.distance) +
		(math::cross(p3.normal, p1.normal) * -p2.distance) +
		(math::cross(p1.normal, p2.normal) * -p3.distance)) /
		(math::dot(p1.normal, math::cross(p2.normal, p3.normal)));*/
		
#if 1
	math::mat3 D(
		p1.normal.x, p2.normal.x, p3.normal.x,
		p1.normal.y, p2.normal.y, p3.normal.y,
		p1.normal.z, p2.normal.z, p3.normal.z
	);
	math::vec3 A(-p1.distance, -p2.distance, -p3.distance);

	math::mat3 Dx = D, Dy = D, Dz = D;
	Dx[0][0] = A.x; Dx[0][1] = A.y; Dx[0][2] = A.z;
	// Remplacer la deuxième ligne de Dy
	Dy[1][0] = A.x; Dy[1][1] = A.y; Dy[1][2] = A.z;
	// Remplacer la troisième ligne de Dz
	Dz[2][0] = A.x; Dz[2][1] = A.y; Dz[2][2] = A.z;

	math::real detD = math::determinant(D);

	if (math::equal(detD, 0)) {
		return Point();
	}

	math::real detDx = math::determinant(Dx);
	math::real detDy = math::determinant(Dy);
	math::real detDz = math::determinant(Dz);

	return Point(detDx / detD, detDy / detD, detDz / detD);
#else 
	math::vec3 m1(p1.normal.x, p2.normal.x, p3.normal.x);
	math::vec3 m2(p1.normal.y, p2.normal.y, p3.normal.y);
	math::vec3 m3(p1.normal.z, p2.normal.z, p3.normal.z);
	math::vec3 d(-p1.distance, -p2.distance, -p3.distance);
	
	math::vec3 u = math::cross(m2, m3);
	math::vec3 v = math::cross(m1, d);
	math::real denom = math::dot(m1, u);

	if (math::equal(denom, 0.0f)) {
		return Point();
	}

	Point result;
	result.x = math::dot(d, u) / denom;
	result.y = math::dot(m3, v) / denom;
	result.z = -math::dot(m2, v) / denom;
	return result;
#endif
}

void GetCorners(const Frustum& f, math::vec3* outCorners) {
	outCorners[0] = Intersection(f.Near(), f.Top(),    f.Left());
	outCorners[1] = Intersection(f.Near(), f.Top(),    f.Right());
	outCorners[2] = Intersection(f.Near(), f.Bottom(), f.Left());
	outCorners[3] = Intersection(f.Near(), f.Bottom(), f.Right());
	outCorners[4] = Intersection(f.Far(),  f.Top(),    f.Left());
	outCorners[5] = Intersection(f.Far(),  f.Top(),    f.Right());
	outCorners[6] = Intersection(f.Far(),  f.Bottom(), f.Left());
	outCorners[7] = Intersection(f.Far(),  f.Bottom(), f.Right());
}

bool Intersects(const Frustum& f, const Point& p) {
	for (int i = 0; i < 6; ++i) {
		math::vec3 normal = f.planes[i].normal;
		math::real dist = f.planes[i].distance;
		math::real side = math::dot(p, normal) + dist;
		if (side < 0.0f) {
			return false;
		}
	}

	return true;
}

bool Intersects(const Frustum& f, const Sphere& s) {
	for (int i = 0; i < 6; ++i) {
		math::vec3 normal = f.planes[i].normal;
		math::real dist = f.planes[i].distance;
		math::real side = math::dot(s.position, normal) + dist;
		if (side < -s.radius) {
			return false;
		}
	}

	return true;
}

math::real Classify(const AABB& aabb, const Plane& plane) {
	// maximum extent in direction of plane normal 
	math::real r = fabsf(aabb.size.x * plane.normal.x)
		+ fabsf(aabb.size.y * plane.normal.y)
		+ fabsf(aabb.size.z * plane.normal.z);

	// signed distance between box center and plane
	//math::real d = plane.Test(mCenter);
	math::real d = math::dot(plane.normal, aabb.position) + plane.distance;

	// return signed distance
	if (fabsf(d) < r) {
		return 0.0f;
	}
	else if (d < 0.0f) {
		return d + r;
	}
	return d - r;
}

math::real Classify(const OBB& obb, const Plane& plane) {
	math::vec3 normal = math::multiplyVector(plane.normal, obb.orientation);

	// maximum extent in direction of plane normal 
	math::real r = fabsf(obb.size.x * normal.x)
		+ fabsf(obb.size.y * normal.y)
		+ fabsf(obb.size.z * normal.z);

	// signed distance between box center and plane
	//math::real d = plane.Test(mCenter);
	math::real d = math::dot(plane.normal, obb.position) + plane.distance;

	// return signed distance
	if (fabsf(d) < r) {
		return 0.0f;
	}
	else if (d < 0.0f) {
		return d + r;
	}
	return d - r;
}

bool Intersects(const Frustum& f, const OBB& obb) {
	for (int i = 0; i < 6; ++i) {
		math::real side = Classify(obb, f.planes[i]);
		if (side < 0) {
			return false;
		}
	}
	return true;
}

bool Intersects(const Frustum& f, const AABB& aabb) {
	for (int i = 0; i < 6; ++i) {
		math::real side = Classify(aabb, f.planes[i]);
		if (side < 0) {
			return false;
		}
	}
	return true;
}

math::vec3 Unproject(const math::vec3& viewportPoint, const math::vec2& viewportOrigin, const math::vec2& viewportSize, const math::mat4& view, const math::mat4& projection) {
	// Step 1, math::normalize the input vector to the view port
	math::real normalized[4] = {
		(viewportPoint.x - viewportOrigin.x) / viewportSize.x,
		(viewportPoint.y - viewportOrigin.y) / viewportSize.y,
		viewportPoint.z,
		1.0f
	};

	// Step 2, Translate into NDC space
	math::real ndcSpace[4] = {
		normalized[0], normalized[1],
		normalized[2], normalized[3]
	};
	// X Range: -1 to 1
	ndcSpace[0] = ndcSpace[0] * 2.0f - 1.0f;
	// Y Range: -1 to 1, our Y axis is flipped!
	ndcSpace[1] = 1.0f - ndcSpace[1] * 2.0f;
	// Z Range: 0 to 1
	if (ndcSpace[2] < 0.0f) {
		ndcSpace[2] = 0.0f;
	}
	if (ndcSpace[2] > 1.0f) {
		ndcSpace[2] = 1.0f;
	}

	// Step 3, NDC to Eye Space
	math::mat4 invProjection = math::inverse(projection);
	math::real eyeSpace[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	// eyeSpace = MultiplyPoint(ndcSpace, invmath::projection);
	math::multiply(eyeSpace, ndcSpace, 1, 4, glm::value_ptr(invProjection), 4, 4);

	// Step 4, Eye Space to World Space
	math::mat4 invView = math::inverse(view);
	math::real worldSpace[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	// worldSpace = MultiplyPoint(eyeSpace, invView);
	math::multiply(worldSpace, eyeSpace, 1, 4, glm::value_ptr(invView), 4, 4);

	// Step 5, Undo perspective divide!
	if (!math::equal(worldSpace[3], 0.0f)) {
		worldSpace[0] /= worldSpace[3];
		worldSpace[1] /= worldSpace[3];
		worldSpace[2] /= worldSpace[3];
	}

	// Return the resulting world space point
	return math::vec3(worldSpace[0], worldSpace[1], worldSpace[2]);
}

Ray GetPickRay(const math::vec2& viewportPoint, const math::vec2& viewportOrigin, const math::vec2& viewportSize, const math::mat4& view, const math::mat4& projection) {
	math::vec3 nearPoint(viewportPoint.x, viewportPoint.y, 0.0f);
	math::vec3 farPoint(viewportPoint.x, viewportPoint.y, 1.0f);

	math::vec3 pNear = Unproject(nearPoint, viewportOrigin, viewportSize, view, projection);
	math::vec3 pFar = Unproject(farPoint, viewportOrigin, viewportSize, view, projection);

	math::vec3 normal = math::normalized(pFar - pNear);
	math::vec3 origin = pNear;

	return Ray(origin, normal);
}

// Chapter 15

void ResetCollisionManifold(CollisionManifold* result) {
	if (result != 0) {
		result->colliding = false;
		result->normal = math::vec3(0, 0, 1);
		result->depth = FLT_MAX;
		if (result->contacts.size() > 0) {
			result->contacts.clear();
		}
	}
}

std::vector<Point> GetVertices(const OBB& obb) {
	std::vector<math::vec3> v(8);  // Directement avec la taille

	math::vec3 C = obb.position;  // OBB Center
	math::vec3 E = obb.size;      // OBB Extents
	const math::vec3 A[] = {      // OBB Axis
		obb.orientation[0],
		obb.orientation[1],
		obb.orientation[2]
};

	// Même formule mais plus claire
	v[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	v[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	v[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	v[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	v[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	v[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	v[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	v[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	return v;
}

std::vector<Line> GetEdges(const OBB& obb) {
	std::vector<Line> result;
	result.reserve(12);
	std::vector<Point> v = GetVertices(obb);

	int index[][2] = { // Indices of edges
		{ 6, 1 },{ 6, 3 },{ 6, 4 },{ 2, 7 },{ 2, 5 },{ 2, 0 },
		{ 0, 1 },{ 0, 3 },{ 7, 1 },{ 7, 4 },{ 4, 5 },{ 5, 3 }
	};

	for (int j = 0; j < 12; ++j) {
		result.push_back(Line(
			v[index[j][0]], v[index[j][1]]
		));
	}

	return result;
}

std::vector<Plane> GetPlanes(const OBB& obb) {
	math::vec3 c = obb.position;	// OBB Center
	math::vec3 e = obb.size;		// OBB Extents
	math::vec3 a[] = {			// OBB Axis
		obb.orientation[0],
		obb.orientation[1],
		obb.orientation[2]
	};

	std::vector<Plane> result;
	result.resize(6);

	result[0] = Plane(a[0]        ,  math::dot(a[0], (c + a[0] * e.x)));
	result[1] = Plane(a[0] * -1.0f, -math::dot(a[0], (c - a[0] * e.x)));
	result[2] = Plane(a[1]        ,  math::dot(a[1], (c + a[1] * e.y)));
	result[3] = Plane(a[1] * -1.0f, -math::dot(a[1], (c - a[1] * e.y)));
	result[4] = Plane(a[2]        ,  math::dot(a[2], (c + a[2] * e.z)));
	result[5] = Plane(a[2] * -1.0f, -math::dot(a[2], (c - a[2] * e.z)));

	return result;
}


bool ClipToPlane(const Plane& plane, const Line& line, Point* outPoint) {
	math::vec3 ab = line.end - line.start;

	math::real nA = math::dot(plane.normal, line.start);
	math::real nAB = math::dot(plane.normal, ab);

	if (CMP(nAB, 0)) {
		return false;
	}

	math::real t = (plane.distance - nA) / nAB;
	if (t >= 0.0f && t <= 1.0f) {
		if (outPoint != 0) {
			*outPoint = line.start + ab * t;
		}
		return true;
	}

	return false;
}

std::vector<Point> ClipEdgesToOBB(const std::vector<Line>& edges, const OBB& obb) {
	std::vector<Point> result;
	result.reserve(edges.size() * 3);
	Point intersection;

	std::vector<Plane> planes = GetPlanes(obb);

	for (int i = 0; i < planes.size(); ++i) {
		for (int j = 0; j < edges.size(); ++j) {
			if (ClipToPlane(planes[i], edges[j], &intersection)) {
				if (PointInOBB(intersection, obb)) {
					result.push_back(intersection);
				}
			}
		}
	}

	return result;
}

math::real PenetrationDepth(const OBB& o1, const OBB& o2, const math::vec3& axis, bool* outShouldFlip) {
	Interval i1 = GetInterval(o1, math::normalized(axis));
	Interval i2 = GetInterval(o2, math::normalized(axis));

	if (!((i2.min <= i1.max) && (i1.min <= i2.max))) {
		return 0.0f; // No penerattion
	}

	math::real len1 = i1.max - i1.min;
	math::real len2 = i2.max - i2.min;
	math::real min = fminf(i1.min, i2.min);
	math::real max = fmaxf(i1.max, i2.max);
	math::real length = max - min;

	if (outShouldFlip != 0) {
		*outShouldFlip = (i2.min < i1.min);
	}

	return (len1 + len2) - length;
}

CollisionManifold FindCollisionFeatures(const OBB& A, const OBB& B) {
	CollisionManifold result; // Will return result of intersection!
	ResetCollisionManifold(&result);

	Sphere s1(A.position, math::length(A.size));
	Sphere s2(B.position, math::length(B.size));

	if (!SphereSphere(s1, s2)) {
		return result;
	}

	// const math::real* o1 = A.orientation.asArray;
	// const math::real* o2 = B.orientation.asArray;

	math::vec3 test[15] = {
		// Premiers 3 axes de A
		A.orientation[0],
		A.orientation[1],
		A.orientation[2],
		// Premiers 3 axes de B
		B.orientation[0],
		B.orientation[1],
		B.orientation[2]
};

	for (int i = 0; i < 3; ++i) { // Fill out rest of axis
		test[6 + i * 3 + 0] = math::cross(test[i], test[0]);
		test[6 + i * 3 + 1] = math::cross(test[i], test[1]);
		test[6 + i * 3 + 2] = math::cross(test[i], test[2]);
	}

	math::vec3* hitNormal = 0;
	bool shouldFlip;

	for (int i = 0; i < 15; ++i) {
		if (test[i].x < 0.000001f) test[i].x = 0.0f;
		if (test[i].y < 0.000001f) test[i].y = 0.0f;
		if (test[i].z < 0.000001f) test[i].z = 0.0f;
		if (math::lengthSq(test[i])< 0.001f) {
			continue;
		}

		math::real depth = PenetrationDepth(A, B, test[i], &shouldFlip);
		if (depth <= 0.0f) {
			return result;
		}
		else if (depth < result.depth) {
			if (shouldFlip) {
				test[i] = test[i] * -1.0f;
			}
			result.depth = depth;
			hitNormal = &test[i];
		}
	}

	if (hitNormal == 0) {
		return result;
	}
	math::vec3 axis = math::normalized(*hitNormal);

	std::vector<Point> c1 = ClipEdgesToOBB(GetEdges(B), A);
	std::vector<Point> c2 = ClipEdgesToOBB(GetEdges(A), B);
	result.contacts.reserve(c1.size() + c2.size());
	result.contacts.insert(result.contacts.end(), c1.begin(), c1.end());
	result.contacts.insert(result.contacts.end(), c2.begin(), c2.end());

	Interval i = GetInterval(A, axis);
	math::real distance = (i.max - i.min)* 0.5f - result.depth * 0.5f;
	math::vec3 pointOnPlane = A.position + axis * distance;
	
	for (int i = result.contacts.size() - 1; i >= 0; --i) {
		math::vec3 contact = result.contacts[i];
		result.contacts[i] = contact + (axis * math::dot(axis, pointOnPlane - contact));
		
		// This bit is in the "There is more" section of the book
		for (int j = result.contacts.size() - 1; j > i; --j) {
			if (math::lengthSq(result.contacts[j] - result.contacts[i]) < 0.0001f) {
				result.contacts.erase(result.contacts.begin() + j);
				break;
			}
		}
	}

	result.colliding = true;
	result.normal = axis;

	return result;
}

CollisionManifold FindCollisionFeatures(const Sphere& A, const Sphere& B) {
	CollisionManifold result; // Will return result of intersection!
	ResetCollisionManifold(&result);

	math::real r = A.radius + B.radius;
	math::vec3 d = B.position - A.position;

	if (math::lengthSq(d) - r * r > 0 || math::lengthSq(d) == 0.0f) {
		return result;
	}
	math::normalized(d);

	result.colliding = true;
	result.normal = d;
	result.depth = fabsf(math::length(d) - r) * 0.5f;
	
	// dtp - Distance to intersection point
	math::real dtp = A.radius - result.depth;
	Point contact = A.position + d * dtp;
	
	result.contacts.push_back(contact);

	return result;
}

CollisionManifold FindCollisionFeatures(const OBB& A, const Sphere& B) {
	CollisionManifold result; // Will return result of intersection!
	ResetCollisionManifold(&result);

	Point closestPoint = ClosestPoint(A, B.position);

	math::real distanceSq = math::lengthSq(closestPoint - B.position);
	if (distanceSq > B.radius * B.radius) {
		return result;
	}

	math::vec3 normal;
	if (CMP(distanceSq, 0.0f)) {
		if (CMP(math::lengthSq(closestPoint - A.position), 0.0f)) {
			return result;

		}
		// Closest point is at the center of the sphere
		normal = math::normalized(closestPoint - A.position);
	}
	else {
		normal = math::normalized(B.position - closestPoint);
	}

	Point outsidePoint = B.position - normal * B.radius;

	math::real distance = math::length(closestPoint - outsidePoint);

	result.colliding = true;
	result.contacts.push_back(closestPoint + (outsidePoint - closestPoint) * 0.5f);
	result.normal = normal;
	result.depth = distance * 0.5f;

	return result;
}
