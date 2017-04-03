#ifndef OCCPREVKEYRAY_H
#define OCCPREVKEYRAY_H

#include <vector>
#include <algorithm>
#include <octomap/OcTreeKey.h>

// Struct storing occupancy and previous key for a given key
struct OccPrevKey {
	float occupancy;
	octomap::OcTreeKey key;
	octomap::OcTreeKey prevKey;

	OccPrevKey(): occupancy(0), key(), prevKey() {
	}
	
	OccPrevKey(float occupancy, const octomap::OcTreeKey& key, const octomap::OcTreeKey& prevKey)
		:occupancy(occupancy), key(key), prevKey(prevKey) {
	}
	
	// The original hashing function did not use a prime number for key.k[1],
	// so I thing that this one is better
	struct KeyHash{
		size_t operator()(const OccPrevKey& val) const{
			// Prime number based hash
			return val.key.k[0] + 1543*val.key.k[1] + 3145739*val.key.k[2];
			
			/*
			// This should produce a more collision free hash for updates constrained to a local
			// volume. But this hash requires more time to be computed. 32-bit version
			return (val.key[0] & 0x7FF ) | ((val.key[1] & 0x7FF) << 11) | ((val.key[2] & 0x3FF) << 22);
			
			// This is guaranteed to be collision free for a 64-bit system. Don't use otherwise!
			return ((size_t)val.key[0]) | ((size_t)val.key[1] << 16) | ((size_t)val.key_[2] << 32);
			*/
		}
	};
	
	struct KeyEqual{
		bool operator()(const OccPrevKey& val1, const OccPrevKey& val2) const{
			return val1.key == val2.key;
		}
	};
};

// A ray of OccPrevKeys
class OccPrevKeyRay {
	public:
		OccPrevKeyRay () {
			ray.resize(100000);
			reset();
		}
		void reset() {
			end_of_ray = begin();
		}
		
		void addOccPrevKey(float occupancy, const octomap::OcTreeKey& key, const octomap::OcTreeKey& prevKey) {
			assert(end_of_ray != ray.end());
			end_of_ray->occupancy = occupancy;
			end_of_ray->key = key;
			end_of_ray->prevKey = prevKey;
			end_of_ray++;
		}

		unsigned int size() const { return end_of_ray - ray.begin(); }
		unsigned int sizeMax() const { return ray.size(); }

		typedef std::vector<OccPrevKey>::iterator iterator;
		typedef std::vector<OccPrevKey>::const_iterator const_iterator;
		typedef std::vector<OccPrevKey>::reverse_iterator reverse_iterator;
		
		iterator begin() { return ray.begin(); }
		iterator end() { return end_of_ray; }
		const_iterator begin() const { return ray.begin(); }
		const_iterator end() const	 { return end_of_ray; }

		reverse_iterator rbegin() { return (reverse_iterator) end_of_ray; }
		reverse_iterator rend() { return ray.rend(); }
		
		OccPrevKey& front() {return ray[0];}
		OccPrevKey& back() {return *(end_of_ray-1);}
		
		void swap(OccPrevKeyRay& other) {
			ray.swap(other.ray);
			std::swap(end_of_ray, other.end_of_ray);
		}
		
	private:
		std::vector<OccPrevKey> ray;
		std::vector<OccPrevKey>::iterator end_of_ray;
};

typedef std::tr1::unordered_set<OccPrevKey, OccPrevKey::KeyHash, OccPrevKey::KeyEqual> OccPrevKeySet;
//typedef google::dense_hash_set<OccPrevKey, OccPrevKey::KeyHash, OccPrevKey::KeyEqual> OccPrevKeySet;

#endif
