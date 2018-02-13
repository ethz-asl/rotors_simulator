/*
Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef TF_SCALAR_H
#define TF_SCALAR_H

#ifdef TF_MANAGED_CODE
//Aligned data types not supported in managed code
#pragma unmanaged
#endif


#include <math.h>
#include <stdlib.h>//size_t for MSVC 6.0
#include <cstdlib>
#include <cfloat>
#include <float.h>

#if defined(DEBUG) || defined (_DEBUG)
#define TF_DEBUG
#endif


#ifdef _WIN32

		#if defined(__MINGW32__) || defined(__CYGWIN__) || (defined (_MSC_VER) && _MSC_VER < 1300)

			#define TFSIMD_FORCE_INLINE inline
			#define ATTRIBUTE_ALIGNED16(a) a
			#define ATTRIBUTE_ALIGNED64(a) a
			#define ATTRIBUTE_ALIGNED128(a) a
		#else
			//#define TF_HAS_ALIGNED_ALLOCATOR
			#pragma warning(disable : 4324) // disable padding warning
//			#pragma warning(disable:4530) // Disable the exception disable but used in MSCV Stl warning.
//			#pragma warning(disable:4996) //Turn off warnings about deprecated C routines
//			#pragma warning(disable:4786) // Disable the "debug name too long" warning

			#define TFSIMD_FORCE_INLINE __forceinline
			#define ATTRIBUTE_ALIGNED16(a) __declspec(align(16)) a
			#define ATTRIBUTE_ALIGNED64(a) __declspec(align(64)) a
			#define ATTRIBUTE_ALIGNED128(a) __declspec (align(128)) a
		#ifdef _XBOX
			#define TF_USE_VMX128

			#include <ppcintrinsics.h>
 			#define TF_HAVE_NATIVE_FSEL
 			#define tfFsel(a,b,c) __fsel((a),(b),(c))
		#else


		#endif//_XBOX

		#endif //__MINGW32__

		#include <assert.h>
#ifdef TF_DEBUG
		#define tfAssert assert
#else
		#define tfAssert(x)
#endif
		//tfFullAssert is optional, slows down a lot
		#define tfFullAssert(x)

		#define tfLikely(_c)  _c
		#define tfUnlikely(_c) _c

#else
	
#if defined	(__CELLOS_LV2__)
		#define TFSIMD_FORCE_INLINE inline
		#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		#define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
		#define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#ifndef assert
		#include <assert.h>
		#endif
#ifdef TF_DEBUG
		#define tfAssert assert
#else
		#define tfAssert(x)
#endif
		//tfFullAssert is optional, slows down a lot
		#define tfFullAssert(x)

		#define tfLikely(_c)  _c
		#define tfUnlikely(_c) _c

#else

#ifdef USE_LIBSPE2

		#define TFSIMD_FORCE_INLINE __inline
		#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		#define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
		#define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#ifndef assert
		#include <assert.h>
		#endif
#ifdef TF_DEBUG
		#define tfAssert assert
#else
		#define tfAssert(x)
#endif
		//tfFullAssert is optional, slows down a lot
		#define tfFullAssert(x)


		#define tfLikely(_c)   __builtin_expect((_c), 1)
		#define tfUnlikely(_c) __builtin_expect((_c), 0)
		

#else
	//non-windows systems


		#define TFSIMD_FORCE_INLINE inline
		///@todo: check out alignment methods for other platforms/compilers
		///#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		///#define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
		///#define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#define ATTRIBUTE_ALIGNED16(a) a
		#define ATTRIBUTE_ALIGNED64(a) a
		#define ATTRIBUTE_ALIGNED128(a) a
		#ifndef assert
		#include <assert.h>
		#endif

#if defined(DEBUG) || defined (_DEBUG)
		#define tfAssert assert
#else
		#define tfAssert(x)
#endif

		//tfFullAssert is optional, slows down a lot
		#define tfFullAssert(x)
		#define tfLikely(_c)  _c
		#define tfUnlikely(_c) _c

#endif // LIBSPE2

#endif	//__CELLOS_LV2__
#endif


///The tfScalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
typedef double tfScalar;
//this number could be bigger in double precision
#define TF_LARGE_FLOAT 1e30



#define TF_DECLARE_ALIGNED_ALLOCATOR() \
   TFSIMD_FORCE_INLINE void* operator new(size_t sizeInBytes)   { return tfAlignedAlloc(sizeInBytes,16); }   \
   TFSIMD_FORCE_INLINE void  operator delete(void* ptr)         { tfAlignedFree(ptr); }   \
   TFSIMD_FORCE_INLINE void* operator new(size_t, void* ptr)   { return ptr; }   \
   TFSIMD_FORCE_INLINE void  operator delete(void*, void*)      { }   \
   TFSIMD_FORCE_INLINE void* operator new[](size_t sizeInBytes)   { return tfAlignedAlloc(sizeInBytes,16); }   \
   TFSIMD_FORCE_INLINE void  operator delete[](void* ptr)         { tfAlignedFree(ptr); }   \
   TFSIMD_FORCE_INLINE void* operator new[](size_t, void* ptr)   { return ptr; }   \
   TFSIMD_FORCE_INLINE void  operator delete[](void*, void*)      { }   \



		
TFSIMD_FORCE_INLINE tfScalar tfSqrt(tfScalar x) { return sqrt(x); }
TFSIMD_FORCE_INLINE tfScalar tfFabs(tfScalar x) { return fabs(x); }
TFSIMD_FORCE_INLINE tfScalar tfCos(tfScalar x) { return cos(x); }
TFSIMD_FORCE_INLINE tfScalar tfSin(tfScalar x) { return sin(x); }
TFSIMD_FORCE_INLINE tfScalar tfTan(tfScalar x) { return tan(x); }
TFSIMD_FORCE_INLINE tfScalar tfAcos(tfScalar x) { if (x<tfScalar(-1))	x=tfScalar(-1); if (x>tfScalar(1))	x=tfScalar(1); return acos(x); }
TFSIMD_FORCE_INLINE tfScalar tfAsin(tfScalar x) { if (x<tfScalar(-1))	x=tfScalar(-1); if (x>tfScalar(1))	x=tfScalar(1); return asin(x); }
TFSIMD_FORCE_INLINE tfScalar tfAtan(tfScalar x) { return atan(x); }
TFSIMD_FORCE_INLINE tfScalar tfAtan2(tfScalar x, tfScalar y) { return atan2(x, y); }
TFSIMD_FORCE_INLINE tfScalar tfExp(tfScalar x) { return exp(x); }
TFSIMD_FORCE_INLINE tfScalar tfLog(tfScalar x) { return log(x); }
TFSIMD_FORCE_INLINE tfScalar tfPow(tfScalar x,tfScalar y) { return pow(x,y); }
TFSIMD_FORCE_INLINE tfScalar tfFmod(tfScalar x,tfScalar y) { return fmod(x,y); }


#define TFSIMD_2_PI         tfScalar(6.283185307179586232)
#define TFSIMD_PI           (TFSIMD_2_PI * tfScalar(0.5))
#define TFSIMD_HALF_PI      (TFSIMD_2_PI * tfScalar(0.25))
#define TFSIMD_RADS_PER_DEG (TFSIMD_2_PI / tfScalar(360.0))
#define TFSIMD_DEGS_PER_RAD  (tfScalar(360.0) / TFSIMD_2_PI)
#define TFSIMDSQRT12 tfScalar(0.7071067811865475244008443621048490)

#define tfRecipSqrt(x) ((tfScalar)(tfScalar(1.0)/tfSqrt(tfScalar(x))))		/* reciprocal square root */


#define TFSIMD_EPSILON      DBL_EPSILON
#define TFSIMD_INFINITY     DBL_MAX

TFSIMD_FORCE_INLINE tfScalar tfAtan2Fast(tfScalar y, tfScalar x) 
{
	tfScalar coeff_1 = TFSIMD_PI / 4.0f;
	tfScalar coeff_2 = 3.0f * coeff_1;
	tfScalar abs_y = tfFabs(y);
	tfScalar angle;
	if (x >= 0.0f) {
		tfScalar r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	} else {
		tfScalar r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}
	return (y < 0.0f) ? -angle : angle;
}

TFSIMD_FORCE_INLINE bool      tfFuzzyZero(tfScalar x) { return tfFabs(x) < TFSIMD_EPSILON; }

TFSIMD_FORCE_INLINE bool	tfEqual(tfScalar a, tfScalar eps) {
	return (((a) <= eps) && !((a) < -eps));
}
TFSIMD_FORCE_INLINE bool	tfGreaterEqual (tfScalar a, tfScalar eps) {
	return (!((a) <= eps));
}


TFSIMD_FORCE_INLINE int       tfIsNegative(tfScalar x) {
    return x < tfScalar(0.0) ? 1 : 0;
}

TFSIMD_FORCE_INLINE tfScalar tfRadians(tfScalar x) { return x * TFSIMD_RADS_PER_DEG; }
TFSIMD_FORCE_INLINE tfScalar tfDegrees(tfScalar x) { return x * TFSIMD_DEGS_PER_RAD; }

#define TF_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

#ifndef tfFsel
TFSIMD_FORCE_INLINE tfScalar tfFsel(tfScalar a, tfScalar b, tfScalar c)
{
	return a >= 0 ? b : c;
}
#endif
#define tfFsels(a,b,c) (tfScalar)tfFsel(a,b,c)


TFSIMD_FORCE_INLINE bool tfMachineIsLittleEndian()
{
   long int i = 1;
   const char *p = (const char *) &i;
   if (p[0] == 1)  // Lowest address contains the least significant byte
	   return true;
   else
	   return false;
}



///tfSelect avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
TFSIMD_FORCE_INLINE unsigned tfSelect(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero) 
{
    // Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
    // Rely on positive value or'ed with its negative having sign bit on
    // and zero value or'ed with its negative (which is still zero) having sign bit off 
    // Use arithmetic shift right, shifting the sign bit through all 32 bits
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz;
    return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz)); 
}
TFSIMD_FORCE_INLINE int tfSelect(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
{
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz; 
    return static_cast<int>((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}
TFSIMD_FORCE_INLINE float tfSelect(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
{
#ifdef TF_HAVE_NATIVE_FSEL
    return (float)tfFsel((tfScalar)condition - tfScalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
#else
    return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero; 
#endif
}

template<typename T> TFSIMD_FORCE_INLINE void tfSwap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}


//PCK: endian swapping functions
TFSIMD_FORCE_INLINE unsigned tfSwapEndian(unsigned val)
{
	return (((val & 0xff000000) >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8)  | ((val & 0x000000ff) << 24));
}

TFSIMD_FORCE_INLINE unsigned short tfSwapEndian(unsigned short val)
{
	return static_cast<unsigned short>(((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
}

TFSIMD_FORCE_INLINE unsigned tfSwapEndian(int val)
{
	return tfSwapEndian((unsigned)val);
}

TFSIMD_FORCE_INLINE unsigned short tfSwapEndian(short val)
{
	return tfSwapEndian((unsigned short) val);
}

///tfSwapFloat uses using char pointers to swap the endianness
////tfSwapFloat/tfSwapDouble will NOT return a float, because the machine might 'correct' invalid floating point values
///Not all values of sign/exponent/mantissa are valid floating point numbers according to IEEE 754. 
///When a floating point unit is faced with an invalid value, it may actually change the value, or worse, throw an exception. 
///In most systems, running user mode code, you wouldn't get an exception, but instead the hardware/os/runtime will 'fix' the number for you. 
///so instead of returning a float/double, we return integer/long long integer
TFSIMD_FORCE_INLINE unsigned int  tfSwapEndianFloat(float d)
{
    unsigned int a = 0;
    unsigned char *dst = (unsigned char *)&a;
    unsigned char *src = (unsigned char *)&d;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];
    return a;
}

// unswap using char pointers
TFSIMD_FORCE_INLINE float tfUnswapEndianFloat(unsigned int a) 
{
    float d = 0.0f;
    unsigned char *src = (unsigned char *)&a;
    unsigned char *dst = (unsigned char *)&d;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];

    return d;
}


// swap using char pointers
TFSIMD_FORCE_INLINE void  tfSwapEndianDouble(double d, unsigned char* dst)
{
    unsigned char *src = (unsigned char *)&d;

    dst[0] = src[7];
    dst[1] = src[6];
    dst[2] = src[5];
    dst[3] = src[4];
    dst[4] = src[3];
    dst[5] = src[2];
    dst[6] = src[1];
    dst[7] = src[0];

}

// unswap using char pointers
TFSIMD_FORCE_INLINE double tfUnswapEndianDouble(const unsigned char *src) 
{
    double d = 0.0;
    unsigned char *dst = (unsigned char *)&d;

    dst[0] = src[7];
    dst[1] = src[6];
    dst[2] = src[5];
    dst[3] = src[4];
    dst[4] = src[3];
    dst[5] = src[2];
    dst[6] = src[1];
    dst[7] = src[0];

	return d;
}

// returns normalized value in range [-TFSIMD_PI, TFSIMD_PI]
TFSIMD_FORCE_INLINE tfScalar tfNormalizeAngle(tfScalar angleInRadians) 
{
	angleInRadians = tfFmod(angleInRadians, TFSIMD_2_PI);
	if(angleInRadians < -TFSIMD_PI)
	{
		return angleInRadians + TFSIMD_2_PI;
	}
	else if(angleInRadians > TFSIMD_PI)
	{
		return angleInRadians - TFSIMD_2_PI;
	}
	else
	{
		return angleInRadians;
	}
}

///rudimentary class to provide type info
struct tfTypedObject
{
	tfTypedObject(int objectType)
		:m_objectType(objectType)
	{
	}
	int	m_objectType;
	inline int getObjectType() const
	{
		return m_objectType;
	}
};
#endif //TFSIMD___SCALAR_H