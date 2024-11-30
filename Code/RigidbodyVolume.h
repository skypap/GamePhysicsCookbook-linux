#ifndef _H_MASS_RIGIDBODY_
#define _H_MASS_RIGIDBODY_

#include "Rigidbody.h"

#define GRAVITY_CONST math::vec3(0.0f, -9.82f, 0.0f)

class RigidbodyVolume : public Rigidbody {
public:
	math::vec3 position;
	math::vec3 velocity;

#ifndef LINEAR_ONLY
	math::vec3 orientation;
	math::vec3 angVel;
#endif

	math::vec3 forces; // sumForces
#ifndef LINEAR_ONLY
	math::vec3 torques; // Sum torques
#endif

				  //vec3 inertia;
	float mass;
	float cor; // Coefficient of restitution
#ifdef DYNAMIC_FRICTION
	float staticFriction;
	float dynamicFriction;
#else
	float friction;
#endif

	OBB box;
	Sphere sphere;
public:

	inline RigidbodyVolume() :
		cor(0.5f), mass(1.0f),
#ifdef DYNAMIC_FRICTION
		staticFriction(0.5f),
		dynamicFriction(0.3f)
#else
		friction(0.6f)
#endif
		{
		type = RIGIDBODY_TYPE_BASE;
	}

	inline RigidbodyVolume(int bodyType) :
		cor(0.5f), mass(1.0f),
#ifdef DYNAMIC_FRICTION
		staticFriction(0.5f),
		dynamicFriction(0.3f)
#else
		friction(0.6f)
#endif
		{
			type = bodyType;
	}

	virtual ~RigidbodyVolume() { }

	virtual void Render();
	virtual void Update(float dt); // Update Position

	float InvMass();
#ifndef LINEAR_ONLY
	math::mat4 InvTensor();
#endif

	virtual void ApplyForces();
	void SynchCollisionVolumes();

	virtual void AddLinearImpulse(const math::vec3& impulse);
#ifndef LINEAR_ONLY
	virtual void AddRotationalImpulse(const math::vec3& point, const math::vec3& impulse);
#endif
};

CollisionManifold FindCollisionFeatures(RigidbodyVolume& ra, RigidbodyVolume& rb);
void ApplyImpulse(RigidbodyVolume& A, RigidbodyVolume& B, const CollisionManifold& M, int c);

#endif