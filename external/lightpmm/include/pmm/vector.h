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

#ifndef PMM_VECTOR_H_
#define PMM_VECTOR_H_

#include "pmm.h"

#include <string>
#include <sstream>

PMM_NAMESPACE_BEGIN

/// Class for representing a vectorized version of 3D vector.
template<typename TScalar>
struct Vec
{
public:
    TScalar x;
    TScalar y;
    TScalar z;

    PMM_INLINE Vec() = default;

    PMM_INLINE Vec(const TScalar x, const TScalar y, const TScalar z)
        : x {TScalar{x}}, y {TScalar{y}}, z {TScalar{z}}
    {
    }

    PMM_INLINE Vec(const Vector3 v)
        : Vec{v[0], v[1], v[2]}
    {
    }

    PMM_INLINE Vec<TScalar> operator*(const Vec<TScalar> v) const
    {
        Vec<TScalar> b;
        b.x = this->x*v.x;
        b.y = this->y*v.y;
        b.z = this->z*v.z;

        return b;
    }

    PMM_INLINE Vec<TScalar> &operator*=(const Vec<TScalar> v)
    {
        this->x*=v.x;
        this->y*=v.y;
        this->z*=v.z;
        return *this;
    }

    PMM_INLINE Vec<TScalar> operator*(const TScalar v)const
    {
        Vec<TScalar> b;
        b.x = this->x*v;
        b.y = this->y*v;
        b.z = this->z*v;

        return b;
    }

    PMM_INLINE Vec<TScalar> &operator*=(const TScalar v)
    {
        this->x*=v;
        this->y*=v;
        this->z*=v;
        return *this;
    }

    PMM_INLINE Vec<TScalar> operator/(const Vec<TScalar> v) const
    {
        Vec<TScalar> b;
        b.x = this->x/v.x;
        b.y = this->y/v.y;
        b.z = this->z/v.z;

        return b;
    }

    PMM_INLINE Vec<TScalar> &operator/=(const Vec<TScalar> v)
    {
        this->x/=v.x;
        this->y/=v.y;
        this->z/=v.z;
        return *this;
    }


    PMM_INLINE Vec<TScalar> operator/(const TScalar v) const
    {
        Vec<TScalar> b;
        b.x = this->x/v;
        b.y = this->y/v;
        b.z = this->z/v;

        return b;
    }

    PMM_INLINE Vec<TScalar> &operator/=(const TScalar v)
    {

        this->x/=v;
        this->y/=v;
        this->z/=v;

        return *this;
    }

    PMM_INLINE Vec<TScalar> operator+(const Vec<TScalar> v)const
    {
        Vec<TScalar> b;
        b.x = this->x+v.x;
        b.y = this->y+v.y;
        b.z = this->z+v.z;

        return b;
    }

    PMM_INLINE Vec<TScalar> &operator+=(const Vec<TScalar> v)
    {
        this->x+=v.x;
        this->y+=v.y;
        this->z+=v.z;
        return *this;
    }

    PMM_INLINE Vec<TScalar> operator-(const Vec<TScalar> v)const
    {
        Vec<TScalar> b;
        b.x = this->x-v.x;
        b.y = this->y-v.y;
        b.z = this->z-v.z;
        return b;
    }

    PMM_INLINE Vec<TScalar> &operator-=(const Vec<TScalar> v)
    {
        this->x-=v.x;
        this->y-=v.y;
        this->z-=v.z;
        return *this;
    }

    PMM_INLINE TScalar length() const
    {
        return lightpmm::sqrt(x*x+y*y+z*z);
    }

    PMM_INLINE Vector3 operator[](const uint32_t component) const
    {
        return Vector3{x[component], y[component], z[component]};
    }

    PMM_INLINE void insert(const uint32_t component, const Vector3 v)
    {
        x.insert(component, v.x);
        y.insert(component, v.y);
        z.insert(component, v.z);
    }

    std::string toString() const
    {
        std::ostringstream oss;

        oss << "Vec<TScalar<" << TScalar::Width::value << ">> [\n";
        for (size_t i=0; i<TScalar::Width::value; ++i)
            oss << "  (" << x[i] << ", " << y[i] << ", " << z[i] << ")\n";
        oss << "]";

        return oss.str();
    }
};

/// Function to calculate the dot product of to vectors.
template<typename TScalar>
PMM_INLINE TScalar dot(const Vec<TScalar> a, const Vec<TScalar> b)
{
    return a.x*b.x+a.y*b.y+a.z*b.z;
}

template <typename TScalar>
PMM_INLINE Vec<TScalar> cross(const Vec<TScalar> &v1, const Vec<TScalar> &v2) {
    return Vec<TScalar>(
        (v1.y * v2.z) - (v1.z * v2.y),
        (v1.z * v2.x) - (v1.x * v2.z),
        (v1.x * v2.y) - (v1.y * v2.x)
    );
}

template<typename TScalar>
PMM_INLINE Vec<TScalar> ifthen(const typename TScalar::BooleanType cond, const Vec<TScalar> a, const Vec<TScalar> b)
{
    return Vec<TScalar>{ifthen(cond, a.x, b.x), ifthen(cond, a.y, b.y), ifthen(cond, a.z, b.z)};
}

template<typename TScalar>
PMM_INLINE bool isfinite(const Vec<TScalar> a){
    return lightpmm::isfinite(a.x) && lightpmm::isfinite(a.y) && lightpmm::isfinite(a.z);
}

PMM_NAMESPACE_END

#endif /* PMM_VECTOR_H_ */
