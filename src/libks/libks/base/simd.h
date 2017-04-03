#ifndef KS_SIMD_H
#define KS_SIMD_H

#include <boost/smart_ptr.hpp>
#include <boost/bind.hpp>
#include <sys/types.h>

// SIMD Headers
#ifdef __SSE3__
#include <pmmintrin.h>
#else
#error "SSE3 support not available"
#endif

#ifdef __SSSE3__
#include <tmmintrin.h>
#else
#warning "SSSE3 support not available"
#endif

#ifdef __SSE4_1__
#include <smmintrin.h>
#else
#warning "SSE4.1 support not available"
#endif

#ifdef __SSE4_2__
#include <nmmintrin.h>
#else
#warning "SSE4.2 support not available"
#endif

/* This file contains declarations required for SIMD programming */

namespace ks {
	// Integer types
	typedef char v16qi __attribute__ ((vector_size (16), aligned (16)));
	typedef short v8hi __attribute__ ((vector_size (16), aligned (16)));
	typedef int v4si __attribute__ ((vector_size (16), aligned (16)));
	typedef long long v2di __attribute__ ((vector_size (16), aligned (16)));
	
	// Floating point types
	typedef float v4sf __attribute__ ((vector_size (16), aligned (16)));
	typedef double v2sf __attribute__ ((vector_size (16), aligned (16)));
	
	class SIMD {
	public:
		// Methods for creating constants with just one scalar
		static __always_inline const v16qi& scalar16(char c) {
			/*const v16qi ret = {c, c, c, c, c, c, c, c, c, c, c, c, c, c, c, c};
			return ret;*/
			return v16Constants[(unsigned char)c];
		}
		
		static v16qi scalar16NonLookup(char c) {
			const v16qi ret = {c, c, c, c, c, c, c, c, c, c, c, c, c, c, c, c};
			return ret;
		}
		
		static __always_inline v8hi scalar8(short s) {
			const v8hi ret = {s, s, s, s, s, s, s, s};
			return ret;
		}
		
		static __always_inline v4si scalar4(int i) {
			const v4si ret = {i, i, i, i};
			return ret;
		}
		
		static __always_inline v2di scalar2(long long i) {
			const v2di ret = {i, i};
			return ret;
		}
		
		// Methods for accessing vector elements
		
		static __always_inline char& element16(v16qi& vec, int index) {
			union Accessor {
				v16qi vector;
				char elements[16];
			} __attribute__((may_alias));
			
			return ((Accessor*)&vec)->elements[index];
		}
		
		static __always_inline short& element8(v8hi& vec, int index) {
			union Accessor {
				v8hi vector;
				short elements[8];
			} __attribute__((may_alias));
			
			return ((Accessor*)&vec)->elements[index];
		}
		
		static __always_inline int& element4(v4si& vec, int index) {
			union Accessor {
				v4si vector;
				int elements[4];
			} __attribute__((may_alias));
			
			return ((Accessor*)&vec)->elements[index];
		}
		
		static __always_inline long long& element2(v2di& vec, int index) {
			union Accessor {
				v2di vector;
				long long elements[2];
			} __attribute__((may_alias));
			
			return ((Accessor*)&vec)->elements[index];
		}
		
		static __always_inline float& element4f(v4sf& vec, int index) {
			union Accessor {
				v4sf vector;
				float elements[4];
			} __attribute__((may_alias));
			
			return ((Accessor*)&vec)->elements[index];
		}

		static __always_inline double& element2f(v2sf& vec, int index) {
			union Accessor {
				v4sf vector;
				double elements[2];
			} __attribute__((may_alias));
			
			return ((Accessor*)&vec)->elements[index];
		}

		// Allocates a memory aligned array (only use primitive types!)
		template <class T>
		static __always_inline boost::shared_array<T> alignedNew(int size, int alignment = 16) {
			
			char* ptr = new char[sizeof(T) * size + alignment -1];
			T* alignedPtr = (T*)((size_t(ptr) + alignment-1) & -alignment);
			
			AlignedDeallocator<T> d = {ptr};
			return boost::shared_array<T>(alignedPtr, d);
		}
		
		// Packed rotate left, 8-bit; Pass in constants to allow register usage optimization
		/*static __always_inline v16qi prol8(v16qi& value, const v16qi& const0x01, const v16qi& const0x80) {
			v16qi lsb = __builtin_ia32_pmaxub128(value & const0x80, const0x01);
			return (value+value) | lsb;
			// About 4 instructions
		}*/
		
		// Packed rotate left, 16-bit
		static __always_inline v8hi prol16(const v8hi& value) {
			v8hi lsb = __builtin_ia32_psrlwi128(value, 15);
			return (value+value) | lsb;
		}
			
	private:
		// Required for dealocating aligned memory areas
		template <class T>
		struct AlignedDeallocator {
			char* ptr;
			void operator() (T*) {
				delete [] ptr;
			}
		};
		
		// Lookup table for SSE 8-bit constants
		static v16qi v16Constants[256] __attribute__((aligned (16)));
	};
}

#endif