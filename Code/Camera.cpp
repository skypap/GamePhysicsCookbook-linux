#include "Camera.h"
#include "GLWindow.h"
#include "Compare.h"

#include <iostream>

Camera::Camera() {
	m_nFov = 60.0f;
	m_nAspect = 1.3f;
	m_nNear = 0.01f;
	m_nFar = 1000.0f;
	m_nWidth = 1.0;
	m_nHeight = 1.0f;

	m_matWorld = math::mat4();
	m_matProj =  math::perspective(m_nFov, m_nAspect, m_nNear, m_nFar);
	m_nProjectionMode = 0;
}

math::mat4 Camera::GetWorldMatrix() {
	return m_matWorld;
	/*mat3 r = Rotation3x3(rotation.x, rotation.y, rotation.z);

	return mat4(
		r._11, r._12, r._13, 0.0f,
		r._21, r._22, r._23, 0.0f,
		r._31, r._32, r._33, 0.0f,
		Dot(position, vec3(r._11, r._12, r._13)),
		Dot(position, vec3(r._21, r._22, r._23)),
		Dot(position, vec3(r._31, r._32, r._33)),
		1.0f
	);*/
}

bool Camera::IsOrthoNormal() {
	math::vec3 right = math::vec3(m_matWorld[0][0], m_matWorld[0][1], m_matWorld[0][2]);
	math::vec3 up = math::vec3(m_matWorld[1][0], m_matWorld[1][1], m_matWorld[1][2]);
	math::vec3 forward = math::vec3(m_matWorld[2][0], m_matWorld[2][1], m_matWorld[2][2]);

	if (!CMP(math::dot(right, right), 1.0f)) {
		return false; // X axis is not normal
	}

	if (!CMP(math::dot(up, up), 1.0f)) {
		return false; // Y axis is not normal
	}

	if (!CMP(math::dot(forward, forward), 1.0f)) {
		return false; // Z axis is not normal
	}

	if (!CMP(math::dot(forward, up), 0.0f)) {
		return false; // Not perpendicular
	}

	if (!CMP(math::dot(forward, right), 0.0f)) {
		return false; // Not perpendicular
	}

	if (!CMP(math::dot(right, up), 0.0f)) {
		return false; // Not perpendicular
	}

	/*mat3 orientation = Cut(m_matWorld, 3, 3);
	mat3 transposed = Transpose(orientation);
	mat3 result = orientation * transposed;
	mat3 identity;
	
	for (int i = 0; i < 9; ++i) {
		if (!CMP(result.asArray[i], identity.asArray[i])) {
			return false;
		}
	}*/
	return true;
}

void Camera::OrthoNormalize() {
	math::vec3 right = math::vec3(m_matWorld[0][0], m_matWorld[0][1], m_matWorld[0][2]);
	math::vec3 up = math::vec3(m_matWorld[1][0], m_matWorld[1][1], m_matWorld[1][2]);
	math::vec3 forward = math::vec3(m_matWorld[2][0], m_matWorld[2][1], m_matWorld[2][2]);

	math::vec3 f = math::normalized(forward);
	math::vec3 r = math::normalized(math::cross(up, f));
	math::vec3 u = math::cross(f, r);

	m_matWorld = math::mat4(
			r.x, r.y, r.z, 0.0f,
			u.x, u.y, u.z, 0.0f,
			f.x, f.y, f.z, 0.0f,
			m_matWorld[3][0], m_matWorld[3][1], m_matWorld[3][2], 1.0f
	);
}

math::mat4 Camera::GetViewMatrix() {
	if (!IsOrthoNormal()) {
		OrthoNormalize();
	}

	math::mat4 inverse = math::transpose(m_matWorld);
	inverse[3][0] = inverse[0][3] = 0.0f;
	inverse[3][1] = inverse[1][3] = 0.0f;
	inverse[3][2] = inverse[2][3] = 0.0f;

	math::vec3 right = math::vec3(m_matWorld[0][0], m_matWorld[0][1], m_matWorld[0][2]);
	math::vec3 up = math::vec3(m_matWorld[1][0], m_matWorld[1][1], m_matWorld[1][2]);
	math::vec3 forward = math::vec3(m_matWorld[2][0], m_matWorld[2][1], m_matWorld[2][2]);
	math::vec3 position = math::vec3(m_matWorld[3][0], m_matWorld[3][1], m_matWorld[3][2]);

	inverse[3][0] = -math::dot(right, position);
	inverse[3][1] = -math::dot(up, position);
	inverse[3][2] = -math::dot(forward, position);

	return inverse;
}

float Camera::GetAspect() {
	return m_nAspect;
}

math::mat4 Camera::GetProjectionMatrix() {
	return m_matProj;
}

void Camera::Resize(int width, int height) {
	m_nAspect = (float)width / (float)height;

	if (m_nProjectionMode == 0) { // Perspective
		m_matProj =  math::perspective(m_nFov, m_nAspect, m_nNear, m_nFar);
	}
	else if (m_nProjectionMode == 1) { // Ortho
		m_nWidth = (float)width;
		m_nHeight = (float)height;

		float halfW = m_nWidth * 0.5f;
		float halfH = m_nHeight * 0.5f;

		m_matProj = math::ortho(-halfW, halfW, halfH, -halfH, m_nNear, m_nFar);
	}
	// m_nProjectionMode == 2
		// User defined
}

bool Camera::IsOrthographic() {
	return m_nProjectionMode == 1;
}

bool Camera::IsPerspective() {
	return m_nProjectionMode == 0;
}

void Camera::Perspective(float fov, float aspect, float zNear, float zFar) {
	m_nFov = fov;
	m_nAspect = aspect;
	m_nNear = zNear;
	m_nFar = zFar;

	m_matProj =  math::perspective(fov, aspect, zNear, zFar);
	m_nProjectionMode = 0;
}

void Camera::Orthographic(float width, float height, float zNear, float zFar) {
	m_nWidth = width;
	m_nHeight = height;
	m_nNear = zNear;
	m_nFar = zFar;

	float halfW = width * 0.5f;
	float halfH = height * 0.5f;

	m_matProj =  math::ortho(-halfW, halfW, halfH, -halfH, zNear, zFar);
	m_nProjectionMode = 1;
}

void Camera::SetProjection(const math::mat4& projection) {
	m_matProj = projection;
	m_nProjectionMode = 2;
}

void Camera::SetWorld(const math::mat4& view) {
	m_matWorld = view;
}

Camera CreatePerspective(float fieldOfView, float aspectRatio, float nearPlane, float farPlane) {
	Camera result;
	result.Perspective(fieldOfView, aspectRatio, nearPlane, farPlane);
	return result;
}

Camera CreateOrthographic(float width, float height, float nearPlane, float farPlane) {
	Camera result;
	result.Orthographic(width, height, nearPlane, farPlane);
	return result;
}

OrbitCamera::OrbitCamera() {
	target =math:: vec3(0, 0, 0);
	zoomDistance = 10.0f;
	zoomSpeed = 200.0f;
	rotationSpeed =math:: vec2(250.0f, 120.0f);
	yRotationLimit =math:: vec2(-20.0f, 80.0f);
	zoomDistanceLimit =math:: vec2(3.0f, 15.0f);
	currentRotation =math:: vec2(0, 0);
	panSpeed =math:: vec2(180.0f, 180.0f);
}

void OrbitCamera::Rotate(const math::vec2& deltaRot, float deltaTime) {
	currentRotation.x += deltaRot.x * rotationSpeed.x * zoomDistance* deltaTime;
	currentRotation.y += deltaRot.y * rotationSpeed.y * zoomDistance * deltaTime;

	currentRotation.x = ClampAngle(currentRotation.x, -360, 360);
	currentRotation.y = ClampAngle(currentRotation.y, yRotationLimit.x, yRotationLimit.y);
}

void OrbitCamera::Zoom(float deltaZoom, float deltaTime) {
	zoomDistance = zoomDistance + deltaZoom  * zoomSpeed * deltaTime;
	if (zoomDistance < zoomDistanceLimit.x) {
		zoomDistance = zoomDistanceLimit.x;
	}
	if (zoomDistance > zoomDistanceLimit.y) {
		zoomDistance = zoomDistanceLimit.y;
	}
}

void OrbitCamera::Pan(const math::vec2& delataPan, float deltaTime) {
	math::vec3 right(m_matWorld[0][0], m_matWorld[0][1], m_matWorld[0][2]);

	// Pan X axis in local space
	target = target - (right * (delataPan.x * panSpeed.x * deltaTime));
	// Pan Y Axis in global space
	target = target + (math::vec3(0, 1, 0) * (delataPan.y * panSpeed.y * deltaTime));

	// Reset zoom to allow infinate zooming after a motion
	// This part of the code is not in the book!
	float midZoom = zoomDistanceLimit.x + (zoomDistanceLimit.y - zoomDistanceLimit.x) * 0.5f;
	zoomDistance = midZoom - zoomDistance;
	math::vec3 rotation = math::vec3(currentRotation.y, currentRotation.x, 0);
	math::mat3 orient = math::rotation3x3(rotation.x, rotation.y, rotation.z);
	math::vec3 dir = math::multiplyVector(math::vec3(0.0, 0.0, -zoomDistance), orient);
	target = target - dir;
	zoomDistance = midZoom;
}

void OrbitCamera::Update(float dt) {
	math::vec3 rotation = math::vec3(currentRotation.y, currentRotation.x, 0);
	math::mat3 orient = math::rotation3x3(rotation.x, rotation.y, rotation.z);
	math::vec3 dir = math::multiplyVector(math::vec3(0.0, 0.0, -zoomDistance), orient);
	math::vec3 position = /*rotation * vec3(0.0, 0.0, -distance)*/dir + target;
	m_matWorld = math::fastInverse(math::lookAt(position, target, math::vec3(0, 1, 0)));
}

float OrbitCamera::ClampAngle(float angle, float min, float max) {
	while (angle < -360) {
		angle += 360;
	}
	while (angle > 360) {
		angle -= 360;
	}
	if (angle < min) {
		angle = min;
	}
	if (angle > max) {
		angle = max;
	}
	return angle;
}

Frustum Camera::GetFrustum() {
	Frustum result;

	math::mat4 vp = GetViewMatrix() * GetProjectionMatrix();

	math::vec3 col1(vp[0][0], vp[1][0], vp[2][0]);//, vp[3][0] };
	math::vec3 col2(vp[0][1], vp[1][1], vp[2][1]);//, vp[3][1] };
	math::vec3 col3(vp[0][2], vp[1][2], vp[2][2]);//, vp[3][2] };
	math::vec3 col4(vp[0][3], vp[1][3], vp[2][3]);//, vp[3][3] };

	// Find plane magnitudes
	result.Left().normal = col4 + col1;
	result.Right().normal = col4 - col1;
	result.Bottom().normal = col4 + col2;
	result.Top().normal = col4 - col2;
	result.Near().normal = /*col4 +*/ col3;
	result.Far().normal = col4 - col3;

	// Find plane distances
	result.Left().distance = vp[3][3] + vp[3][0];
	result.Right().distance = vp[3][3] - vp[3][0];
	result.Bottom().distance = vp[3][3] + vp[3][1];
	result.Top().distance = vp[3][3] - vp[3][1];
	result.Near().distance = /*vp[3][3] +*/ vp[3][2];
	result.Far().distance = vp[3][3] - vp[3][2];

	// Normalize all 6 planes
	for (int i = 0; i < 6; ++i) {
		float mag = 1.0f / math::length(result.planes[i].normal);
		result.planes[i].normal = math::normalized(result.planes[i].normal);
		result.planes[i].distance *= mag;
	}

	return result;
}

void OrbitCamera::PrintDebug() {
	std::cout << "Target: (" << target.x << ", " << target.y << ", " << target.z << ")\n";
	std::cout << "Zoom distance: " << zoomDistance << "\n";
	std::cout << "Rotation: (" << currentRotation.x << ", " << currentRotation.y << ")\n";
}

void OrbitCamera::SetTarget(const math::vec3& newTarget) {
	target = newTarget;
}

void OrbitCamera::SetZoom(float zoom) {
	zoomDistance = zoom;
}

void OrbitCamera::SetRotation(const math::vec2& rotation) {
	currentRotation = rotation;
}