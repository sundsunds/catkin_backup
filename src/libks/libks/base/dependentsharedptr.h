#ifndef KS_DEPENDENTSHAREDPTR_H
#define KS_DEPENDENTSHAREDPTR_H

#include <boost/smart_ptr.hpp>

namespace ks {
	// Shared pointers that don't free the pointer,
	// but keep a copy of another shared pointer that shares the
	// allocated data.

	template <class T>
	class DependentSharedPtr{
	public:
		template <class U>
		static boost::shared_ptr<T> create1(boost::shared_ptr<U> dependentPtr1, T* data) {
			Deallocator1<U> d = {dependentPtr1};
			return boost::shared_ptr<T>(data, d);
		}
		
		template <class U, class V>
		static boost::shared_ptr<T> create2(boost::shared_ptr<U> dependentPtr1,
			boost::shared_ptr<V> dependentPtr2, T* data)
		{
			Deallocator2<U,V> d = {dependentPtr1, dependentPtr2};
			return boost::shared_ptr<T>(data, d);
		}
		
		template <class U, class V, class W>
		static boost::shared_ptr<T> create3(boost::shared_ptr<U> dependentPtr1, 
			boost::shared_ptr<V> dependentPtr2, boost::shared_ptr<W> dependentPtr3, T* data)
		{
			Deallocator3<U,V,W> d = {dependentPtr1, dependentPtr2, dependentPtr3};
			return boost::shared_ptr<T>(data, d);
		}
		
		template <class U, class V, class W, class X>
		static boost::shared_ptr<T> create4(boost::shared_ptr<U> dependentPtr1, 
			boost::shared_ptr<V> dependentPtr2, boost::shared_ptr<W> dependentPtr3,
			boost::shared_ptr<X> dependentPtr4, T* data)
		{
			Deallocator4<U,V,W,X> d = {dependentPtr1, dependentPtr2, dependentPtr3, dependentPtr4};
			return boost::shared_ptr<T>(data, d);
		}
	
	private:
		template <class U>
		struct Deallocator1 {
			boost::shared_ptr<U> ptr1;
			
			void operator() (T* p) {
				delete p;
				ptr1.reset();
			}
		};
		
		template <class U, class V>
		struct Deallocator2 {
			boost::shared_ptr<U> ptr1;
			boost::shared_ptr<V> ptr2;
			
			void operator() (T* p) {
				delete p;
				ptr1.reset();
				ptr2.reset();
			}
		};
		
		template <class U, class V, class W>
		struct Deallocator3 {
			boost::shared_ptr<U> ptr1;
			boost::shared_ptr<V> ptr2;
			boost::shared_ptr<W> ptr3;
			
			void operator() (T* p) {
				delete p;
				ptr1.reset();
				ptr2.reset();
				ptr3.reset();
			}
		};
		
		template <class U, class V, class W, class X>
		struct Deallocator4 {
			boost::shared_ptr<U> ptr1;
			boost::shared_ptr<V> ptr2;
			boost::shared_ptr<W> ptr3;
			boost::shared_ptr<X> ptr4;
			
			void operator() (T* p) {
				delete p;
				ptr1.reset();
				ptr2.reset();
				ptr3.reset();
				ptr4.reset();
			}
		};
	};
}

#endif
