#ifndef _H_PARTICLE_
#define _H_PARTICLE_

#include "Rigidbody.h";

// The particles will either use euler or verlet integration
// If EULER_INTEGRATION is defined, euler integration will
// be used. Otherwise, verlet integration is the default

// If ACCURATE_EULER_INTEGRATION is defined, a slightly more
// accurate integration model is used. This should help 
// keep the simulation stable over long periods of time

#define EULER_INTEGRATION
//#define ACCURATE_EULER_INTEGRATION

class Particle : public Rigidbody {
	math::vec3 position;
	math::vec3 oldPosition;
	math::vec3 forces;

	math::vec3 gravity;
	float friction;
	float bounce;

#ifdef EULER_INTEGRATION
	math::vec3 velocity;
#endif
	float mass;
public:
	Particle();

	void Update(float deltaTime);
	void Render();
	void ApplyForces();
	void SolveConstraints(const std::vector<OBB>& constraints);

	void SetPosition(const math::vec3& pos);
	math::vec3 GetPosition();

	void SetBounce(float b);
	float GetBounce();

	void AddImpulse(const math::vec3& impulse);
	float InvMass();
	void SetMass(float m);
	math::vec3 GetVelocity();
	void SetFriction(float f);
};

#endif
