//
// Created by felixfifi on 26.09.20.
//

#ifndef RTX_PMM_GLM_H
#define RTX_PMM_GLM_H

///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, Sebastian Herholz, Lukas Ruppert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the author nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include "pmm.h"

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include <glm/mat2x2.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <glm/geometric.hpp>
#include <cmath>

PMM_NAMESPACE_BEGIN

// Wrapping glm main data types
    typedef glm::vec3 Vector3;
    typedef glm::vec3 Point3;
    typedef glm::vec2 Point2;
    typedef glm::mat2x2 Matrix2x2;


#define PMM_WARN(...) std::cerr << __VA_ARGS__ << std::endl;
#ifndef PMM_DISABLE_ASSERTS
#define PMM_ASSERT(cond) SAssert(cond);
#define PMM_ASSERT_MSG(cond, msg) SAssertEx(cond, msg);
#else
#define PMM_ASSERT(cond)
#define PMM_ASSERT_MSG(cond, msg)
#endif

//#define PMM_ASSERT_MSG(cond) if ( !(cond) ) { __builtin_trap(); }


    PMM_INLINE Vector3 sphericalDirection(const float theta, const float phi);

    [[nodiscard]] PMM_INLINE Vector3 squareToUniformSphere(Point2 random) {
        float theta = 2 * glm::pi<float>() * random.x;
        float phi = std::acos(1 - 2 * random.y);
        return sphericalDirection(theta, phi);
    }

    [[nodiscard]] PMM_INLINE Point2 toSphericalCoordinates(const Vector3 &dir) {
        float phi = std::atan2(dir.y, dir.x);
        float theta = acos(dir.z / glm::length(dir));
        return {phi, theta};
    }

    [[nodiscard]] PMM_INLINE Vector3 sphericalDirection(const float theta, const float phi) {
        const float sinTheta = std::sin(theta);
        return {sinTheta * std::cos(phi), sinTheta * std::sin(phi), std::cos(theta)};
    }

    [[nodiscard]] PMM_INLINE Vector3
    sphericalDirection(const float cosTheta, const float sinTheta, const float cosPhi, const float sinPhi) {
        return Vector3(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);
    }

    [[nodiscard]] PMM_INLINE Vector3 frameToWorld(const Vector3 n, const Vector3 dir) {
        Vector3 s, t;
        if (std::abs(n.x) > std::abs(n.y)) {
            float invLen = 1.0f / std::sqrt(n.x * n.x + n.z * n.z);
            t = Vector3(n.z * invLen, 0.0f, -n.x * invLen);
        } else {
            float invLen = 1.0f / std::sqrt(n.y * n.y + n.z * n.z);
            t = Vector3(0.0f, n.z * invLen, -n.y * invLen);
        }
        s = glm::cross(t, n);

        return s * dir.x + t * dir.y + n * dir.z;
    }

    // Rendering Vorlesung - Nori src/common.cpp
    PMM_INLINE void coordinateAxis(Vector3 z, Vector3 &x, Vector3 &y) {
        if (abs(z.x) > abs(z.y)) {
            float invLen = 1.0f / sqrt(z.x * z.x + z.z * z.z);
            y = Vector3(z.z * invLen, 0.0f, -z.x * invLen);
        } else {
            float invLen = 1.0f / sqrt(z.y * z.y + z.z * z.z);
            y = Vector3(0.0f, z.z * invLen, -z.y * invLen);
        }
        x = cross(y, z);
    }

    struct Frame {
        Vector3 x, y, z;

        Frame(Vector3 n) {
            z = n;
            coordinateAxis(z, x, y);
        }

        Vector3 toLocal(Vector3 v) {
            return Vector3(dot(v, x), dot(v, y), dot(v, z));
        }
    };

PMM_NAMESPACE_END

#endif //PMM_GLM_H
