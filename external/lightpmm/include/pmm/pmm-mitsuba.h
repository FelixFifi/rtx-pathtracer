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

/// Header file to wrap the inhouse data types (e.g., position, vector) and 
/// and main mapping functions (e.g., angles to vector)
/// of the Mitsuba rendering system to lightpmm.

#ifndef PMM_PMM_MITSUBA_H_
#define PMM_PMM_MITSUBA_H_

#include "pmm.h"

#include <mitsuba/mitsuba.h>
#include <mitsuba/core/point.h>
#include <mitsuba/core/vector.h>
#include <mitsuba/core/logger.h>

#include <mitsuba/core/warp.h>

PMM_NAMESPACE_BEGIN

// Wrapping Mitsubas main data types
typedef mitsuba::Vector Vector3;
typedef mitsuba::Point Point3;
typedef mitsuba::Point2 Point2;


// Using Mitsuba's logging system
#define PMM_WARN(...) SLog(mitsuba::EWarn, __VA_ARGS__);
#ifndef PMM_DISABLE_ASSERTS
#define PMM_ASSERT(cond) SAssert(cond);
#define PMM_ASSERT_MSG(cond, msg) SAssertEx(cond, msg);
#else
#define PMM_ASSERT(cond)
#define PMM_ASSERT_MSG(cond, msg)
#endif

//#define PMM_ASSERT_MSG(cond) if ( !(cond) ) { __builtin_trap(); }

// Wrapping the main conversion function of Mitsuba
[[nodiscard]] PMM_INLINE Vector3 squareToUniformSphere(Point2 random) {
    return mitsuba::warp::squareToUniformSphere(random);
}

[[nodiscard]] PMM_INLINE Point2 toSphericalCoordinates(const Vector3 &dir){
    return mitsuba::toSphericalCoordinates(dir);
}

[[nodiscard]] PMM_INLINE Vector3 sphericalDirection(const float theta, const float phi){
    return mitsuba::sphericalDirection(theta, phi);
}

[[nodiscard]] PMM_INLINE Vector3 sphericalDirection(const float cosTheta, const float sinTheta, const float cosPhi, const float sinPhi) {
    return Vector3(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);
}

[[nodiscard]] PMM_INLINE Vector3 frameToWorld(const Vector3 n, const Vector3 dir) {
    Vector3 s,t;
    if (std::abs(n.x) > std::abs(n.y)) {
        mitsuba::Float invLen = 1.0f / std::sqrt(n.x * n.x + n.z * n.z);
        t = Vector3(n.z * invLen, 0.0f, -n.x * invLen);
    } else {
        mitsuba::Float invLen = 1.0f / std::sqrt(n.y * n.y + n.z * n.z);
        t = Vector3(0.0f, n.z * invLen, -n.y * invLen);
    }
    s = cross(t, n);

    return s*dir.x+t*dir.y+n*dir.z;
}

PMM_NAMESPACE_END

#endif /* PMM_PMM_MITSUBA_H_ */
