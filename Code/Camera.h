#ifndef _H_CAMERA_
#define _H_CAMERA_

#include "math_config.h"
#include "Geometry3D.h"
#include "TimingUtils.h"
class Camera {
protected:
	float m_nFov;
	float m_nAspect;
	float m_nNear;
	float m_nFar;
	float m_nWidth;
	float m_nHeight;

	math::mat4 m_matWorld; // World Transform
	// View Transform = Inverse(World Transform)
	math::mat4 m_matProj;
	int m_nProjectionMode; // 0 - Perspective, 1 - Ortho, 2 - User
public:
	Camera();
	// Default copy constructor / assignment operator will do!
	inline virtual ~Camera() { }

	math::mat4 GetWorldMatrix();
	math::mat4 GetViewMatrix(); // Inverse of world!
	math::mat4 GetProjectionMatrix();

	float GetAspect();
	bool IsOrthographic();
	bool IsPerspective();

	bool IsOrthoNormal();
	void OrthoNormalize();

	void Resize(int width, int height);
	
	void Perspective(float fov, float aspect, float zNear, float zFar);
	void Orthographic(float width, float height, float zNear, float zFar);
	
	void SetProjection(const math::mat4& projection);
	void SetWorld(const math::mat4& view);

	Frustum GetFrustum();
};

Camera CreatePerspective(float fieldOfView, float aspectRatio, float nearPlane, float farPlane);
Camera CreateOrthographic(float width, float height, float nearPlane, float farPlane);

class OrbitCamera : public Camera {
protected:
	math::vec3 target;
	math::vec2 panSpeed;

	float zoomDistance;
	math::vec2 zoomDistanceLimit; // x = min, y = max;
	float zoomSpeed;

	math::vec2 rotationSpeed;
	math::vec2 yRotationLimit; // x = min, y = max
	math::vec2 currentRotation;
public:
	OrbitCamera();
	inline virtual ~OrbitCamera() { }

	void Rotate(const math::vec2& deltaRot, float deltaTime);
	void Zoom(float deltaZoom, float deltaTime);
	void Pan(const math::vec2& delataPan, float deltaTime);

	void Update(float dt);
	float ClampAngle(float angle, float min, float max);

	void SetTarget(const math::vec3& newTarget);
	void SetZoom(float zoom);
	void SetRotation(const math::vec2& rotation);
	void PrintDebug();
};

#endif 
