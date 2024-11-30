#include "Spring.h"

#include <cmath>

Spring::Spring(float _k, float _b, float len) {
	k = _k;
	b = _b;
	restingLength = len;
}

void Spring::SetParticles(Particle* _p1, Particle* _p2) {
	p1 = _p1;
	p2 = _p2;
}

Particle* Spring::GetP1() {
	return p1;
}

Particle* Spring::GetP2() {
	return p2;
}

void Spring::SetConstants(float _k, float _b) {
	k = _k;
	b = _b;
}

void Spring::ApplyForce(float dt) {
	math::vec3 relPos = p2->GetPosition() - p1->GetPosition();
	math::vec3 relVel = p2->GetVelocity() - p1->GetVelocity();

	// Prevent underflow
	for (int i = 0; i < 3; ++i) {
		relPos[i] = (fabs(relPos[i]) < 0.0000001f) ? 0.0f : relPos[i];
		relVel[i] = (fabsf(relVel[i]) < 0.0000001f) ? 0.0f : relVel[i];
	}

	float x = math::length(relPos) - restingLength;
	float v = math::length(relVel);

	float F = (-k * x) + (-b * v);

	math::vec3 impulse = math::normalized(relPos) * F;
	p1->AddImpulse(impulse * p1->InvMass());
	p2->AddImpulse(impulse*  -1.0f * p2->InvMass());
}