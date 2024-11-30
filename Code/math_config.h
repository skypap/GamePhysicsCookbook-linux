//
// Created by maxime on 11/24/24.
//

#pragma once

#define GLM_FORCE_INLINE
#define GLM_FORCE_EXPLICIT_CTOR
#define GLM_FORCE_XYZW_ONLY
#define GLM_FORCE_LEFT_HANDED
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <ostream>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/projection.hpp>
namespace math {
    // Configuration du type de base
#ifdef USE_DOUBLE_PRECISION
    using real = double;
    using vec2 = glm::dvec2;
    using vec3 = glm::dvec3;
    using vec4 = glm::dvec4;
    using mat2 = glm::dmat2;
    using mat3 = glm::dmat3;
    using mat4 = glm::dmat4;
    constexpr real EPSILON = 1e-12;
#else
    using real = float;
    using vec2 = glm::vec2;
    using vec3 = glm::vec3;
    using vec4 = glm::vec4;
    using mat2 = glm::mat2;
    using mat3 = glm::mat3;
    using mat4 = glm::mat4;
    constexpr real EPSILON = 1e-6f;
#endif

    // Constantes mathématiques
    constexpr real PI = glm::pi<real>();
    constexpr real TWO_PI = PI * real(2);
    constexpr real HALF_PI = PI * real(0.5);

    // Correction des angles (équivalent à CorrectDegrees)
    inline real correctDegrees(real degrees) {
        degrees = std::fmod(degrees, real(360.0));
        return (degrees < real(0.0)) ? degrees + real(360.0) : degrees;
    }

    // Conversions d'angles
    inline real degrees(real radians) { return glm::degrees(radians); }
    inline real radians(real degrees) { return glm::radians(degrees); }

    // Fonctions de matrice
    // inline mat4 perspective(real fov, real aspect, real near, real far) {
        // return glm::perspective(fov, aspect, near, far);
    // }

    // inline mat4 lookAt(const vec3& eye, const vec3& center, const vec3& up) {
        // return glm::lookAt(eye, center, up);
    // }

    // Comparaisons
    inline bool equal(real a, real b, real epsilon = EPSILON) {
        return glm::abs(a - b) <= epsilon;
    }

    // Utilitaires mathématiques
    inline real clamp(real value, real min, real max) {
        return glm::clamp(value, min, max);
    }

    // Opérations sur les vecteurs
    inline real length(const vec2& v) { return glm::length(v); }        // Magnitude
    inline real length(const vec3& v) { return glm::length(v); }

    inline real lengthSq(const vec2& v) { return glm::length2(v); }     // MagnitudeSq
    inline real lengthSq(const vec3& v) { return glm::length2(v); }

    inline real distance(const vec2& p1, const vec2& p2) { return glm::distance(p1, p2); }
    inline real distance(const vec3& p1, const vec3& p2) { return glm::distance(p1, p2); }

    inline real distanceSq(const vec2& p1, const vec2& p2) { return glm::distance2(p1, p2); }
    inline real distanceSq(const vec3& p1, const vec3& p2) { return glm::distance2(p1, p2); }

    inline real dot(const vec2& a, const vec2& b) { return glm::dot(a, b); }
    inline real dot(const vec3& a, const vec3& b) { return glm::dot(a, b); }

    inline vec3 cross(const vec3& a, const vec3& b) { return glm::cross(a, b); }

    // Rotation (équivalent à RotateVector)
    inline vec2 rotate(const vec2& v, real degrees) {
        return glm::rotate(v, radians(degrees));
    }

    // Normalisation
    // template<typename Vec>
    inline vec2 normalized(const vec2& v) {
        return glm::normalize(v);
    }
    inline vec3 normalized(const vec3& v) {
        return glm::normalize(v);
    }
    // Projection
    inline vec2 project(const vec2& length, const vec2& direction) {
    return direction * (dot(length, direction) / dot(direction, direction));
    }

    inline vec3 project(const vec3& length, const vec3& direction) {
    return direction * (dot(length, direction) / dot(direction, direction));
    }

    // Perpendicular
    inline vec2 perpendicular(const vec2& length, const vec2& direction) {
        return length - project(length, direction);
    }

    inline vec3 perpendicular(const vec3& length, const vec3& direction) {
        return length - project(length, direction);
    }

    // Reflection
    inline vec2 reflect(const vec2& sourceVector, const vec2& normal) {
        return glm::reflect(sourceVector, normalize(normal));
    }

    inline vec3 reflect(const vec3& sourceVector, const vec3& normal) {
        return glm::reflect(sourceVector, normalize(normal));
    }

    // Angle entre vecteurs
    inline real angle(const vec2& a, const vec2& b) {
        return degrees(glm::angle(a, b));
    }

    inline real angle(const vec3& a, const vec3& b) {
        return degrees(glm::angle(a, b));
    }

    // Helper pour le rendu OpenGL
    inline const real* value_ptr(const mat4& m) {
        return glm::value_ptr(m);
    }

    // Stream operators pour l'affichage
    template<typename OStream>
    inline OStream& operator<<(OStream& os, const vec2& v) {
        return os << "(" << v.x << ", " << v.y << ")";
    }

    template<typename OStream>
    inline OStream& operator<<(OStream& os, const vec3& v) {
        return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }

    // Matrices de transformation
    inline mat4 translation(const vec3& pos) {
        return glm::translate(mat4(1.0f), pos);
    }

    inline mat4 scale(const vec3& scale) {
        return glm::scale(mat4(1.0f), scale);
    }

    inline mat4 rotation(real pitch, real yaw, real roll) {
        // Convertir en radians
        real p = radians(pitch);
        real y = radians(yaw);
        real r = radians(roll);

        // Construire la matrice de rotation dans l'ordre correct (YXZ)
        mat4 rot = mat4(1.0f);
        rot = glm::rotate(rot, y, vec3(0, 1, 0)); // Yaw autour de Y
        rot = glm::rotate(rot, p, vec3(1, 0, 0)); // Pitch autour de X
        rot = glm::rotate(rot, r, vec3(0, 0, 1)); // Roll autour de Z
        return rot;
    }

    inline mat4 xRotation(real angle) {
        return glm::rotate(mat4(1.0f), radians(angle), vec3(1, 0, 0));
    }

    inline mat4 yRotation(real angle) {
        return glm::rotate(mat4(1.0f), radians(angle), vec3(0, 1, 0));
    }

    inline mat4 zRotation(real angle) {
        return glm::rotate(mat4(1.0f), radians(angle), vec3(0, 0, 1));
    }

    inline mat4 axisAngle(const vec3& axis, real angle) {
        return glm::rotate(mat4(1.0f), radians(angle), axis);
    }

    // Fonctions de transformation pour points et vecteurs
    inline vec3 transformPoint(const vec3& point, const mat4& matrix) {
        vec4 temp = matrix * vec4(point, 1.0f);
        return vec3(temp) / temp.w;
    }

    inline vec3 transformVector(const vec3& vector, const mat4& matrix) {
        return vec3(matrix * vec4(vector, 0.0f));
    }

    // Matrices spéciales
    inline mat4 lookAt(const vec3& eye, const vec3& center, const vec3& up) {
        return glm::lookAt(eye, center, up);
    }

    inline mat4 perspective(real fov, real aspect, real near, real far) {
        return glm::perspective(radians(fov), aspect, near, far);
    }

    inline mat4 ortho(real left, real right, real bottom, real top, real near, real far) {
        return glm::ortho(left, right, bottom, top, near, far);
    }

    // Décomposition/Extraction
    inline vec3 getTranslation(const mat4& mat) {
        return vec3(mat[3]);
    }

    inline vec3 getScale(const mat4& mat) {
        return vec3(
            length(vec3(mat[0])),
            length(vec3(mat[1])),
            length(vec3(mat[2]))
        );
    }

    // Inversion
    inline mat3 inverse(const mat3& mat) {
        return glm::inverse(mat);
    }

    inline mat4 inverse(const mat4& mat) {
        return glm::inverse(mat);
    }

    // Transposition
    inline mat3 transpose(const mat3& mat) {
        return glm::transpose(mat);
    }

    inline mat4 transpose(const mat4& mat) {
        return glm::transpose(mat);
    }

    // Construction de transformation combinée
    inline mat4 transform(const vec3& scale, const vec3& eulerRotation, const vec3& translate) {
        mat4 result(1.0f);
        // Appliquer dans l'ordre : Scale, Rotation, Translation
        result = glm::translate(result, translate);
        result *= rotation(eulerRotation.x, eulerRotation.y, eulerRotation.z);
        result = glm::scale(result, scale);
        return result;
    }

    // Opérateurs de flux
    template<typename OStream>
    inline OStream& operator<<(OStream& os, const mat4& m) {
        for(int i = 0; i < 4; ++i) {
            os << "\n";
            for(int j = 0; j < 4; ++j) {
                os << m[i][j] << " ";
            }
        }
        return os;
    }

    template<typename OStream>
    inline OStream& operator<<(OStream& os, const mat3& m) {
        for(int i = 0; i < 3; ++i) {
            os << "\n";
            for(int j = 0; j < 3; ++j) {
                os << m[i][j] << " ";
            }
        }
        return os;
    }

    // Fonctions utilitaires supplémentaires
    inline bool equal(const mat4& a, const mat4& b, real epsilon = EPSILON) {
        for(int i = 0; i < 4; ++i)
            for(int j = 0; j < 4; ++j)
                if(std::abs(a[i][j] - b[i][j]) > epsilon)
                    return false;
        return true;
    }

    inline bool equal(const mat3& a, const mat3& b, real epsilon = EPSILON) {
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                if(std::abs(a[i][j] - b[i][j]) > epsilon)
                    return false;
        return true;
    }
    inline bool multiply(float* out, const float* matA, int aRows, int aCols, const float* matB, int bRows, int bCols) {
        if (aCols != bRows) {
            return false;
        }

        for (int i = 0; i < aRows; ++i) {
            for (int j = 0; j < bCols; ++j) {
                out[bCols * i + j] = 0.0f;
                for (int k = 0; k < bRows; ++k) {
                    out[bCols * i + j] += matA[aCols * i + k] * matB[bCols * k + j];
                }
            }
        }

        return true;
    }
    inline real determinant(const mat2& m) {
        return glm::determinant(m);
    }

    inline real determinant(const mat3& m) {
        return glm::determinant(m);
    }

    inline real determinant(const mat4& m) {
        return glm::determinant(m);
    }

    // Fonction Cut pour extraire une sous-matrice 3x3 d'une matrice 4x4
    inline mat3 cut(const mat4& mat, int row, int col) {
        return mat3(
            row == 0 ? vec3(mat[1][1], mat[1][2], mat[1][3]) : vec3(mat[0][1], mat[0][2], mat[0][3]),
            row == 1 ? vec3(mat[2][1], mat[2][2], mat[2][3]) : vec3(mat[1][1], mat[1][2], mat[1][3]),
            row == 2 ? vec3(mat[3][1], mat[3][2], mat[3][3]) : vec3(mat[2][1], mat[2][2], mat[2][3])
        );
    }

    // MultiplyPoint pour transformer un point par une matrice
    inline vec3 multiplyPoint(const vec3& point, const mat4& mat) {
        vec4 temp = mat * vec4(point, 1.0f);
        return vec3(temp) / temp.w;
    }
    inline vec3 multiplyVector(const vec3& vec, const mat4& mat) {
        // Pour un vecteur, on met w à 0 car on ne veut pas de translation
        vec4 temp = mat * vec4(vec, 0.0f);
        return vec3(temp);
    }

    // Pour vec3 * mat3
    inline vec3 multiplyVector(const vec3& vec, const mat3& mat) {
        return mat * vec;
    }
    // Rotation 3x3
    inline mat3 rotation3x3(real pitch, real yaw, real roll) {
        return mat3(glm::eulerAngleYXZ(radians(yaw), radians(pitch), radians(roll)));
    }

    // Fast inverse pour matrices contenant seulement rotation/translation
    inline mat4 fastInverse(const mat4& m) {
        // Assuming matrix is orthogonal (rotation) + translation
        mat4 inv = glm::transpose(m);
        vec3 trans = vec3(m[3]);
        vec3 invTrans = -(glm::transpose(mat3(m)) * trans);
        inv[3] = vec4(invTrans, 1.0f);
        return inv;
    }
    inline mat4 fromMat3(const mat3& m) {
        return mat4(
            vec4(m[0], 0.0f),
            vec4(m[1], 0.0f),
            vec4(m[2], 0.0f),
            vec4(0.0f, 0.0f, 0.0f, 1.0f)
        );
    }
}

// Littéraux pour les angles
static  math::real operator"" _deg(long double deg) {
    return math::radians(static_cast<math::real>(deg));
}

static  math::real operator"" _rad(long double rad) {
    return static_cast<math::real>(rad);
}