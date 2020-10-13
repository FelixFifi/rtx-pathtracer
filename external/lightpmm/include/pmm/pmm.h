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

#ifndef PMM_PMM_H_
#define PMM_PMM_H_


#ifndef _MSC_VER
#define __forceinline inline __attribute__((always_inline))
#endif

#define PMM_INLINE __forceinline

#define PMM_EPSILON 1.0e-8f

#define EXPECT_TAKEN(arg) arg
#define EXPECT_NOT_TAKEN(arg) arg

#define PMM_NAMESPACE_BEGIN namespace lightpmm {
#define PMM_NAMESPACE_END }

#include <cstdint>
#include <type_traits>
#include <xmmintrin.h>

#include <string>
#include <sstream>

// disables the use of asserts which are used for testing
// and validating the integrity of the code.
#define PMM_DISABLE_ASSERTS
// Allows the use of an approximation of the exponential function to speed up the code.
#define PMM_APPROX_EXP

// once there are other options, these should be set via a compiler flag
#define PMM_USE_VCL
#define PMM_USE_GLM
//#define PMM_USE_MITSUBA

// includes the Mistuba wrapper
#ifdef PMM_USE_MITSUBA
#include "pmm-mitsuba.h"
#endif

// includes the VCL wrapper to use the VCL library for SSE/AVS support
#ifdef PMM_USE_GLM
#include "pmm-glm.h"
#endif


// includes the VCL wrapper to use the VCL library for SSE/AVS support
#ifdef PMM_USE_VCL
#include "pmm-vcl.h"
#endif

PMM_NAMESPACE_BEGIN

/// Enable the use of denormalized floats.
/// This ways values really close to zero will be set to zero
/// to speed up calcuations (no special handling needed). 
PMM_INLINE void useDenormalizedFloats(const bool &flag = true) {
    /// Get rid of de-normalized floats
    if(flag){
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
        _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
    }else{
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_OFF);
        _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_OFF); 
    }
}

//TODO: this is for floats not for SSE
PMM_INLINE bool isDenormalizedNumbersOn() {
    const float floatMin = std::numeric_limits<float>::min();
    float a = 0.f;
    int * i = (int *)&a;
    *i += 1;

    return a > 0.f && a < floatMin;
}

/// Wrapper class for SIMD vectors.
/// The werapper contains the type of the SIMD vector its size
/// and the needed memory alignment.
template<typename TType, size_t TWidth, size_t TAlignment=0>
struct alignas(TAlignment) alignas(alignof(TType)) Scalar : public TType {
    typedef std::integral_constant<size_t, TAlignment> Alignment;
    typedef std::integral_constant<size_t, TWidth> Width;
    typedef TType IntegralType;
    typedef typename std::conditional<TWidth == 4, bool4, typename std::conditional<TWidth == 8, bool8, void>::type>::type BooleanType;

    // make all of TType's constructors available
    template<typename... Ts> Scalar(Ts... parameters) : TType(parameters...) { }

    std::string toString() const
    {
        std::ostringstream oss;

        oss << "Scalar<" << TWidth << "> [\n";
        for (size_t i=0; i<TWidth; ++i)
            oss << "  " << (*this)[i] << '\n';
        oss << "]";

        return oss.str();
    }

    // all other public functions are available through derivation
};

/// Scalar type representing an SSE vector
typedef Scalar<float4, 4, 16> Scalar4;
/// Scalar type representing an AVX vector
typedef Scalar<float8, 8, 32> Scalar8;

PMM_NAMESPACE_END

#endif /* PMM_PMM_H_ */
