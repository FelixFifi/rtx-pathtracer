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


/// Header to wrap the vectorclass library to support SSE and AVX.
/// 

#ifndef PMM_PMM_VCL_H_
#define PMM_PMM_VCL_H_

#include "pmm.h"

#include <immintrin.h>

#define VCL_NAMESPACE vcl
#include "vectorclass/vectorclass.h"


#define VCL_FASTEXP
#include "vectorclass/vectormath_exp.h"
#include "vectorclass/vectormath_hyp.h"

PMM_NAMESPACE_BEGIN

typedef vcl::Vec4fb bool4;
typedef vcl::Vec4f float4;

typedef vcl::Vec8fb bool8;
typedef vcl::Vec8f float8;

PMM_INLINE float tanh(const float &a){
	return std::tanh(a);
}

PMM_INLINE float ifthen(const bool &cond, const float &a, const float &b){
	return cond ? a : b;
}

PMM_INLINE bool all(const bool &a){
	return a;
}

PMM_INLINE bool any(const bool &a){
	return a;
}

PMM_INLINE void insert(float4 &a, uint32_t i, const float &v){
	a.insert(i, v);
}

PMM_INLINE float sum(const float4 &a){
	return vcl::horizontal_add(a);
}

PMM_INLINE float4 sqrt(const float4 &a){
	return vcl::sqrt(a);
}

PMM_INLINE float4 min(const float4 &a, const float4 &b){
	return vcl::min(a,b);
}

PMM_INLINE float4 max(const float4 &a, const float4 &b){
	return vcl::max(a,b);
}

PMM_INLINE float4 ifthen(const bool4 &cond, const float4 &a, const float4 &b){
	return vcl::select(cond, a ,b);
}

PMM_INLINE bool4 is_finite(const float4 &a){
    return vcl::is_finite(a);
}

PMM_INLINE bool4 is_inf(const float4 &a){
    return vcl::is_inf(a);
}

PMM_INLINE bool all(const bool4 &a){
	return vcl::horizontal_and(a);
}

PMM_INLINE bool any(const bool4 &a){
	return vcl::horizontal_or(a);
}

PMM_INLINE bool is_entirely_finite(const float4 &a){
	return vcl::horizontal_and(vcl::is_finite(a));
}

PMM_INLINE bool is_any_inf(const float4 &a){
	return vcl::horizontal_or(vcl::is_inf(a));
}


#ifdef PMM_APPROX_EXP
/*
PMM_INLINE __m128 my_fastpow2 (__m128 p) {
    const __m128 clipp = _mm_max_ps(_mm_set1_ps(-126.0f), p);
    const __m128 w = _mm_round_ps(clipp, _MM_FROUND_TO_ZERO);
    const __m128 z = _mm_add_ps(_mm_sub_ps(clipp, w), _mm_set1_ps(1.f));

    const __m128 result =  _mm_castsi128_ps(_mm_cvtps_epi32(_mm_mul_ps(_mm_set1_ps(1 << 23),
        _mm_sub_ps( _mm_add_ps(_mm_add_ps(clipp, _mm_set1_ps(121.2740575f)), _mm_div_ps(_mm_set1_ps(27.7280233f), _mm_sub_ps(_mm_set1_ps(4.84252568f), z))), _mm_mul_ps(_mm_set1_ps(1.49012907f), z)) 
        )));
    return result;
}
*/
PMM_INLINE __m128 my_fastpow2 (__m128 p) {
    const __m128 clipp = _mm_max_ps(_mm_set1_ps(-126.0f), p);
    const __m128 w = _mm_round_ps(clipp, _MM_FROUND_TO_ZERO);
    const __m128 z = _mm_add_ps(_mm_sub_ps(clipp, w), _mm_set1_ps(1.f));

	const __m128 c_121_2740575 = _mm_set1_ps(121.2740575f);
	const __m128 c_27_7280233  = _mm_set1_ps( 27.7280233f);
	const __m128 c_4_84252568   = _mm_set1_ps( 4.84252568f);
	const __m128 c_1_49012907   = _mm_set1_ps( 1.49012907f);

    const __m128 result =  _mm_castsi128_ps(_mm_cvtps_epi32(_mm_mul_ps(_mm_set1_ps(1 << 23),
        _mm_sub_ps( _mm_add_ps(_mm_add_ps(clipp, c_121_2740575), _mm_div_ps(c_27_7280233, _mm_sub_ps(c_4_84252568, z))), _mm_mul_ps(c_1_49012907, z)) 
        )));
    return result;
}


PMM_INLINE __m128 my_fastexp (__m128 p) {
	const __m128 c_invlog_2 = _mm_set1_ps(1.442695040f);
    return my_fastpow2 (_mm_mul_ps(c_invlog_2, p));
}

template<typename Type>
PMM_INLINE Type fastpow2 (Type p) {
    const Type clipp = vcl::max(Type(-126.0f), p);
    const Type w = vcl::truncate(clipp);
    const Type z = ((clipp-w) + Type(1.f));

	const Type c_121_2740575(121.2740575f);
	const Type c_27_7280233(  27.7280233f);
	const Type c_4_84252568(   4.84252568f);
	const Type c_1_49012907(   1.49012907f);

    const Type result = vcl::reinterpret_f(vcl::roundi((Type(1 << 23)*
        (((clipp + c_121_2740575) + (c_27_7280233 / (c_4_84252568 - z))) - (c_1_49012907*z)) 
        )));
    return result;
}

template<typename Type>
PMM_INLINE Type fastexp( const Type& p){
	const Type c_invlog_2(1.442695040f);
    return fastpow2<Type>(c_invlog_2*p);
}

PMM_INLINE float4 exp(const float4 &a){
	//PMM_ASSERT(vcl::horizontal_and(float4(my_fastexp(a)) == fastexp(a)));
	//return float4(my_fastexp(a));
	return fastexp<float4>(a);
}

//from vcl, with exp replaced by fastexp
template<typename VTYPE, typename BVTYPE>
static inline VTYPE fasttanh(VTYPE const & x0) {
// The limit of abs(x) is 89.0, as defined by max_x in vectormath_exp.h for 0.5*exp(x).

    // Coefficients
    const float r0 = -3.33332819422E-1f;
    const float r1 =  1.33314422036E-1f;
    const float r2 = -5.37397155531E-2f;
    const float r3 =  2.06390887954E-2f;
    const float r4 = -5.70498872745E-3f;

    // data vectors
    VTYPE x, x2, y1, y2;
    BVTYPE x_small, x_big;                       // boolean vectors

    x = abs(x0);
    x_small = x <= 0.625f;                       // use polynomial approximation if abs(x) <= 5/8

    if (horizontal_or(x_small)) {
        // At least one element needs small method
        x2 = x*x;
        y1 = polynomial_4(x2, r0, r1, r2, r3, r4);
        y1 = mul_add(y1, x2*x, x);               // y1 = x + (x2*x)*y1;
    }
    if (!horizontal_and(x_small)) {
        // At least one element needs big method
        y2 = fastexp<VTYPE>(x+x);                           // exp(2*x)
        y2 = 1.0f - 2.0f / (y2 + 1.0f);          // tanh(x)
    }
    x_big = x > 44.4f;
    y1 = select(x_small, y1, y2);                // choose method
    y1 = select(x_big,  1.0f, y1);               // avoid overflow
    y1 = sign_combine(y1, x0);                   // get original sign

    return y1;
}

PMM_INLINE float4 tanh(const float4 &a){
    return fasttanh<float4, bool4>(a);
}

#else
PMM_INLINE float4 exp(const float4 &a){
	return vcl::exp(a);
}

PMM_INLINE float4 tanh(const float4 &a){
    return vcl::tanh(a);
}
#endif

PMM_INLINE float4 log(const float4 &a){
    return vcl::log(a);
}

PMM_INLINE void insert(float8 &a, uint32_t i, const float &v){
	a.insert(i, v);
}

PMM_INLINE float sum(const float8 &a){
	return vcl::horizontal_add(a);
}

PMM_INLINE float8 sqrt(const float8 &a){
	return vcl::sqrt(a);
}

PMM_INLINE float8 min(const float8 &a, const float8 &b){
	return vcl::min(a,b);
}

PMM_INLINE float8 max(const float8 &a, const float8 &b){
	return vcl::max(a,b);
}

#ifdef PMM_APPROX_EXP
PMM_INLINE float8 exp(const float8 &a){
	return fastexp<float8>(a);
}
PMM_INLINE float8 tanh(const float8 &a){
    return fasttanh<float8, bool8>(a);
}
#else
PMM_INLINE float8 exp(const float8 &a){
	return vcl::exp(a);
}

PMM_INLINE float8 tanh(const float8 &a){
    return vcl::tanh(a);
}
#endif

PMM_INLINE float8 log(const float8 &a){
    return vcl::log(a);
}

PMM_INLINE float8 ifthen(const bool8 &cond, const float8 &a, const float8 &b){
	return vcl::select(cond, a ,b);
}

PMM_INLINE bool8 is_finite(const float8 &a){
    return vcl::is_finite(a);
}

PMM_INLINE bool8 is_inf(const float8 &a){
    return vcl::is_inf(a);
}

PMM_INLINE bool all(const bool8 &a){
	return vcl::horizontal_and(a);
}

PMM_INLINE bool any(const bool8 &a){
	return vcl::horizontal_or(a);
}

PMM_INLINE bool is_entirely_finite(const float8 &a){
	return vcl::horizontal_and(vcl::is_finite(a));
}

PMM_INLINE bool is_any_inf(const float8 &a){
	return vcl::horizontal_or(vcl::is_inf(a));
}


PMM_NAMESPACE_END



#endif /* PMM_PMM_VCL_H_ */
