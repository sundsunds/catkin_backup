#include "robustoctree.h"
#include <iostream>
#include <cmath>
#include <list>
#include <libks/stereo/cameracalibration.h>
#include <libks/base/exception.h>
#include <libks/base/timer.h>
#include <libks/base/simd.h>

#define MODEL_Z_ERROR

// Not supported features: Bounding box, change notification

namespace occupancymap {
	using namespace octomap;
	using namespace std;
	using namespace ks;
	
	octomap::OcTreeKey RobustOcTree::ROOT_KEY(0xFFFF, 0xFFFF, 0xFFFF);

	RobustOcTree::StaticMemberInitializer RobustOcTree::ocTreeMemberInit;

	RobustOcTree::RobustOcTree(boost::shared_ptr<Parameters> parameters)
		:Base(parameters != NULL ? parameters->octreeResolution : 0.1), parameters(parameters) {
		if(parameters != NULL) {
			probHitIfOccupied = parameters->probHitIfOccupied;
			probHitIfNotOccupied = parameters->probHitIfNotOccupied;
			probHitIfNotVisible = parameters->probHitIfNotVisible;
			probMissIfOccupied = 1.0 - probHitIfOccupied;
			probMissIfNotOccupied = 1.0 - probHitIfNotOccupied;
			probMissIfNotVisible = 1.0 - probHitIfNotVisible;
			occupancyProbThres = parameters->occupancyThreshold;
			clampingThresMax = parameters->clampingThresholdMax;
			clampingThresMin = parameters->clampingThresholdMin;
			visClampMax = parameters->visibilityClampingMax;
			visClampMin = parameters->visibilityClampingMin;
			voxelErrorResolution = parameters->voxelErrorResolution;
			
			OcTreeKey key;
			if(!coordToKeyChecked(0, 0, parameters->minHeight, key))
				minZKey = 0;
			else minZKey = key[2];

			if(!coordToKeyChecked(0, 0, parameters->maxHeight, key))
				maxZKey = 0xFFFF;
			else maxZKey = key[2];
			
			CalibrationResult calib = CalibrationResult(parameters->calibFile.c_str());
			double fx = calib.P[0](0,0);
			double baseline = fabs(calib.T(0));
			
			lookup.reset(new ProbabilityLookup(parameters->octreeResolution, voxelErrorResolution, 0.1,
				parameters->maxPointDist, 0, 1, 0.05, parameters->disparityStdDev / (baseline * fx)));
			
			// Hopefully makes accesses to the hash faster
			voxelUpdateCache.max_load_factor(0.75);
			voxelUpdateCache.rehash(1000);
		} else {
			probHitIfOccupied = probHitIfNotOccupied = occupancyProbThres = clampingThresMax = 
				clampingThresMin = visClampMax = visClampMin = voxelErrorResolution = 0.0;
			maxZKey = minZKey = 0;
		}
	}

	void RobustOcTree::insertPointCloud(const octomap::Pointcloud& scan, const octomap::point3d& sensor_origin,
		const octomap::point3d& forwardVec, double maxrange, bool lazy_eval) {
		
		computeUpdate(scan, sensor_origin, forwardVec, maxrange);
		calcVisibilities(sensor_origin);

		// Perform update
		for (OccPrevKeySet::iterator it = voxelUpdateCache.begin(); it != voxelUpdateCache.end(); it++) {
			float visibility = visibilities[it->key];
			if(visibility >= visClampMin)
				updateNode(it->key, visibility, it->occupancy, lazy_eval);
		}
	}
	
	void RobustOcTree::calcVisibilities(const octomap::point3d& sensor_origin) {
		// Find the origin key
		OcTreeKey originKey;
		if ( !coordToKeyChecked(sensor_origin, originKey)) {
			OCTOMAP_WARNING_STR("origin coordinates ( " << sensor_origin << ") out of bounds in computeRayKeys");
			return;
		}
		
		visibilities.clear();
		visibilities[ROOT_KEY] = 1.0;
	
		// Create list of keys that need to be processed
		list<const OccPrevKey*> unprocessedKeys(voxelUpdateCache.size());
		list<const OccPrevKey*>::iterator it2 = unprocessedKeys.begin();
		
		for (OccPrevKeySet::iterator it1 = voxelUpdateCache.begin(); it1 != voxelUpdateCache.end(); it1++, it2++)
			(*it2) = &(*it1);
	
		// Perform processing until there are no more keys left
		while(unprocessedKeys.size() > 0) {
			for (list<const OccPrevKey*>::iterator it = unprocessedKeys.begin(); it != unprocessedKeys.end();) {
				KeyVisMap::iterator visIter = visibilities.find((*it)->prevKey);
				if(visIter != visibilities.end()) {
					// Find occupancies for the three neighbor nodes bordering
					// the visible faces
					octomap::OcTreeKey currKey = (*it)->key;
					
					int step[3] = {
						currKey[0] > originKey[0] ? 1 : -1,
						currKey[1] > originKey[1] ? 1 : -1,
						currKey[2] > originKey[2] ? 1 : -1
					};
					OcTreeKey neighborKeys[3] =  {
						OcTreeKey(currKey[0] - step[0], currKey[1], currKey[2]),
						OcTreeKey(currKey[0], currKey[1] - step[1], currKey[2]),
						OcTreeKey(currKey[0], currKey[1], currKey[2] - step[2])
					};
					
					float occupancies[3] = {0.5F, 0.5F, 0.5F};

					for(unsigned int i = 0; i<3; i++) {
						RobustOcTreeNode* neighborNode = search(neighborKeys[i]);
						if(neighborNode != NULL) {
							occupancies[i] = neighborNode->getOccupancy();
							if(occupancies[i] <= clampingThresMin)
								occupancies[i] = 0;
						}
					}
					
					// Get the probabilities that the current voxel is occluded by its neighbors
					//float occlusionProb = occupancies[0] * occupancies[1] * occupancies[2];
					float occlusionProb = min(occupancies[0], min(occupancies[1], occupancies[2]));
					float visibility = visIter->second * (parameters->probVisibleIfOccluded * occlusionProb
						+ parameters->probVisibleIfNotOccluded * (1.0F-occlusionProb));

					// clamp visibility
					if(visibility > visClampMax)
						visibility = 1.0F;
				
					visibilities[(*it)->key] = visibility;
					unprocessedKeys.erase(it++);
				} else ++it;
			}
		}
	}
	
	// Smarter implementation but slightly slower
	/*void RobustOcTree::calcVisibilities(const octomap::point3d& sensor_origin) {
		static Timer timer("Visibility");
		timer.start();
	
		// Find the origin key
		OcTreeKey originKey;
		if ( !coordToKeyChecked(sensor_origin, originKey)) {
			OCTOMAP_WARNING_STR("origin coordinates ( " << sensor_origin << ") out of bounds in computeRayKeys");
			return;
		}
		
		visibilities.clear();
		visibilities[ROOT_KEY] = 1.0; // The root is fully visible
	
		// Create list of keys that need to be processed. The map key is the predecessor node
		typedef multimap<OcTreeKey, const OccPrevKey*, KeySmaller> UnprocessedMap;
		//typedef tr1::unordered_multimap<OcTreeKey, const OccPrevKey*, OcTreeKey::KeyHash, KeySmaller> UnprocessedMap;
		UnprocessedMap unprocessedKeys;
		//unprocessedKeys.rehash(voxelUpdateCache.size() / 0.75);
		for (OccPrevKeySet::iterator it = voxelUpdateCache.begin(); it != voxelUpdateCache.end(); it++)
			unprocessedKeys.insert(UnprocessedMap::value_type(it->prevKey, &(*it)));
		
		// Stack for keys that are ready for processing. Initialize with all nodes who's
		// predecessor is the root key
		vector<UnprocessedMap::iterator> readyKeys;
		pair<UnprocessedMap::iterator, UnprocessedMap::iterator> initRange = unprocessedKeys.equal_range(ROOT_KEY);
		for(UnprocessedMap::iterator it = initRange.first; it != initRange.second; it++)
			readyKeys.push_back(it);
		
		// Perform processing until there are no more keys left
		while(unprocessedKeys.size() > 0) {
			// Process top element from ready keys list
			UnprocessedMap::iterator currentKey = readyKeys.back();
			KeyVisMap::iterator visIter = visibilities.find(currentKey->second->prevKey);
			
			// Find min occupancy for 3 neighbors
			octomap::OcTreeKey currKey = currentKey->second->key;
			int step[3] = {
				currKey[0] > originKey[0] ? 1 : -1,
				currKey[1] > originKey[1] ? 1 : -1,
				currKey[2] > originKey[2] ? 1 : -1
			};
			RobustOcTreeNode* neighborNode1 = search(OcTreeKey(currKey[0] - step[0], currKey[1], currKey[2]));
			RobustOcTreeNode* neighborNode2 = search(OcTreeKey(currKey[0], currKey[1] - step[1], currKey[2]));
			RobustOcTreeNode* neighborNode3 = search(OcTreeKey(currKey[0], currKey[1], currKey[2] - step[2]));
			
			// Update visibility estimate
			double neighbor1Occ = neighborNode1 != NULL ? neighborNode1->getOccupancy() : 0.5,
				neighbor2Occ = neighborNode2 != NULL ? neighborNode2->getOccupancy() : 0.5,
				neighbor3Occ = neighborNode3 != NULL ? neighborNode3->getOccupancy() : 0.5;

			double minOcc = min(neighbor1Occ, min(neighbor2Occ, neighbor3Occ));
			double visibility = visIter->second * (minOcc*parameters->occupancyVisibilityProb + (1.0 - minOcc));
		
			// clamp visibility
			if(visibility > visClampMax)
				visibility = 1.0;
		
			visibilities[currentKey->second->key] = visibility;
			
			// Update list of ready keys
			readyKeys.pop_back();
			pair<UnprocessedMap::iterator, UnprocessedMap::iterator> range = unprocessedKeys.equal_range(currentKey->second->key);
			for(UnprocessedMap::iterator it = range.first; it != range.second; it++)
				readyKeys.push_back(it);

			// Remove from unprocessed keys
			unprocessedKeys.erase(currentKey);
		}
		
		timer.stop();
	}*/
	
	void RobustOcTree::insertPointCloud(const Pointcloud& pc, const point3d& sensor_origin, const pose6d& frame_origin,
		double maxrange, bool lazy_eval) {
		
		// performs transformation to data and sensor origin first
		Pointcloud transformed_scan (pc);
		transformed_scan.transform(frame_origin);
		point3d transformed_sensor_origin = frame_origin.transform(sensor_origin);
		point3d forwardVec = frame_origin.transform(sensor_origin + point3d(0, 1.0, 0.0)) - transformed_sensor_origin;
		
		insertPointCloud(transformed_scan, transformed_sensor_origin, forwardVec, maxrange, lazy_eval);
	}
	
	void RobustOcTree::computeUpdate(const Pointcloud& scan, const octomap::point3d& origin, const octomap::point3d& forwardVec, double maxrange) {
		voxelUpdateCache.clear();
		prevKeyray.reset();
		
		for (int i = 0; i < (int)scan.size(); ++i) {
			// Compute a new ray
			const point3d& p = scan[i];
			computeRayKeys(origin, p, forwardVec, maxrange);
			
			// Insert cells into the update cache if they haven't been
			// part of a previous ray, or if their occupancy is higher than
			// for any previous ray.
			
			// First perform lookup against previous ray
			OccPrevKeyRay::iterator currIt = keyray.begin();
			OccPrevKeyRay::iterator currEnd = prevKeyray.size() > keyray.size() ? keyray.end() : keyray.begin() + prevKeyray.size();
			
			for(OccPrevKeyRay::iterator prevIt = prevKeyray.begin(); currIt != currEnd; currIt++, prevIt++) {
				if(currIt->key == prevIt->key) {
					// Cell already exists in previous ray
					if(currIt->occupancy > prevIt->occupancy) {
						OccPrevKeySet::iterator updateIter = voxelUpdateCache.find(*currIt);
						OccPrevKeySet::iterator insertHint = voxelUpdateCache.erase(updateIter);
						voxelUpdateCache.insert(insertHint, *currIt);
					} else {
						currIt->occupancy = prevIt->occupancy; // Keep best occupancy for next ray
					}
				} else {
					// Perform full lookup against hash map
					OccPrevKeySet::iterator updateIter = voxelUpdateCache.find(*currIt);
					if(updateIter == voxelUpdateCache.end()) {
						voxelUpdateCache.insert(*currIt);
					} else if(currIt->occupancy > updateIter->occupancy) {
						OccPrevKeySet::iterator insertHint = voxelUpdateCache.erase(updateIter);					
						voxelUpdateCache.insert(insertHint, *currIt);
					} else {
						currIt->occupancy = updateIter->occupancy; // Keep best occupancy for next ray
					}
				}
			}
			
			// Lookup remaining cells against hash map
			for(; currIt != keyray.end(); currIt++) {
				OccPrevKeySet::iterator updateIter = voxelUpdateCache.find(*currIt);
				if(updateIter == voxelUpdateCache.end()) {
					voxelUpdateCache.insert(*currIt);
				} else if(currIt->occupancy > updateIter->occupancy) {
					OccPrevKeySet::iterator insertHint = voxelUpdateCache.erase(updateIter);
					voxelUpdateCache.insert(insertHint, *currIt);
				} else {
					currIt->occupancy = updateIter->occupancy; // Keep best occupancy for next ray
				}
			}
			
			keyray.swap(prevKeyray);
		}
	}
	
	void RobustOcTree::computeRayKeys(const point3d& origin, const point3d& end, const octomap::point3d& forwardVec, double maxrange) {
		// see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
		// basically: DDA in 3D
		
		keyray.reset();
		
		point3d direction = (end - origin);
		double length = direction.norm();
		direction /= length; // normalize vector

		OcTreeKey key_origin, key_end;
		if ( !coordToKeyChecked(origin, key_origin)) {
			OCTOMAP_WARNING_STR("origin coordinates ( " << origin << " -> " << end << ") out of bounds in computeRayKeys");
			return;
		}
		if(!coordToKeyChecked(end, key_end) && !coordToKeyChecked(origin + direction*(2.0*maxrange), key_end)) {
			OCTOMAP_WARNING_STR("end coordinates ( " << origin << " -> " << end << ") out of bounds in computeRayKeys");
			return;
		}
		
		// Initialization phase -------------------------------------------------------

		int	   step[3];
		double tMax[3];
		double tDelta[3];

		OcTreeKey prev_key = ROOT_KEY;
		OcTreeKey current_key = key_origin; 

		for(unsigned int i=0; i < 3; ++i) {
			// compute step direction
			if (direction(i) > 0.0) step[i] =	1;
			else if (direction(i) < 0.0)	 step[i] = -1;
			else step[i] = 0;

			// compute tMax, tDelta
			if (step[i] != 0) {
				// corner point of voxel (in direction of ray)
				double voxelBorder = keyToCoord(current_key[i]);
				voxelBorder += step[i] * resolution * 0.5;

				tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
				tDelta[i] = this->resolution / fabs( direction(i) );
			}
			else {
				tMax[i] =	std::numeric_limits<double>::max( );
				tDelta[i] = std::numeric_limits<double>::max( );
			}
		}

		// for speedup:
		point3d origin_scaled = origin;
		origin_scaled /= resolution;	
		
		double length_to_key = (keyToCoord(key_end) - origin).norm();
		double length_scaled = (min(length_to_key, maxrange)) / resolution;
		length_scaled = length_scaled*length_scaled;
		double maxrange_scaled = maxrange / resolution;
		maxrange_scaled *= maxrange_scaled;
		
		// Conversion factor from length to depth (length of one z-step)
		double lengthToDepth = forwardVec.x() * direction.x() + forwardVec.y() * direction.y() + forwardVec.z() * forwardVec.z();
		
		double depth = length / lengthToDepth;
		double probApproxMaxDistScaled, probApproxMaxDist;
		vector<float>* lookupEntry = lookup->lookupEntry(depth);
		if(lookupEntry == NULL) {
			probApproxMaxDist = std::numeric_limits<double>::max();
			probApproxMaxDistScaled = maxrange_scaled;
		} else {
			probApproxMaxDist = lengthToDepth*(depth - (lookupEntry->size()/2.0)*voxelErrorResolution);
			probApproxMaxDistScaled = probApproxMaxDist / resolution;
			probApproxMaxDistScaled = min(probApproxMaxDistScaled*probApproxMaxDistScaled, maxrange_scaled);
		}
		double treeMax05 = tree_max_val - 0.5F;
		// Incremental phase	---------------------------------------------------------

		unsigned int dim = 0;
		double squareDistFromOrigVec[3] = {
			(current_key[0] - treeMax05 - origin_scaled(0))*(current_key[0] - treeMax05 - origin_scaled(0)),
			(current_key[1] - treeMax05 - origin_scaled(1))*(current_key[1] - treeMax05 - origin_scaled(1)),
			(current_key[2] - treeMax05 - origin_scaled(2))*(current_key[2] - treeMax05 - origin_scaled(2))};
		
		while(true) {
			// Calculate distance from origin
			double squareDistFromOrig = squareDistFromOrigVec[0] + squareDistFromOrigVec[1] + squareDistFromOrigVec[2];
			
#ifdef MODEL_Z_ERROR
			if(squareDistFromOrig < probApproxMaxDistScaled) {
				// Use approximate probabilities
				keyray.addOccPrevKey(0.0, current_key, prev_key);
			} else if(squareDistFromOrig >= maxrange_scaled) {
				// The point is too far away. Lets stop.
				break;
			} else {
				// Detailed calculation
				int index = int((sqrt(squareDistFromOrig)*resolution - probApproxMaxDist)/
					(voxelErrorResolution*lengthToDepth) + 0.5);
				
				double occupancyFactor = 1.0;
				bool done = false;
				if(index >= (int)lookupEntry->size()) {
					done = true;
					// Continue to make sure we integrate one full hit
				} else {
					occupancyFactor = (*lookupEntry)[index];
				}
				
				keyray.addOccPrevKey(occupancyFactor, current_key, prev_key);
				
				if(done)
					break;
			}
#else
			// reached endpoint?
			if (current_key == key_end) {
                keyray.addOccPrevKey(1.0, current_key, prev_key);
                break;
			} else if(squareDistFromOrig >= length_scaled) {
				// We missed it :-(
				if(length_to_key < maxrange)
					keyray.addOccPrevKey(1.0, key_end, prev_key);
				break;
			} else {
				keyray.addOccPrevKey(0.0, current_key, prev_key);
			}
#endif
			
			// find minimum tMax:
			if (tMax[0] < tMax[1]){
				if (tMax[0] < tMax[2]) dim = 0;
				else dim = 2;
			}
			else {
				if (tMax[1] < tMax[2]) dim = 1;
				else dim = 2;
			}

			// advance in direction "dim"
			prev_key = current_key;
			current_key[dim] += step[dim];
			tMax[dim] += tDelta[dim];
			squareDistFromOrigVec[dim] = current_key[dim] - treeMax05 - origin_scaled(dim);
			squareDistFromOrigVec[dim]*= squareDistFromOrigVec[dim];
			
			assert (current_key[dim] < 2*this->tree_max_val);
			
			if(current_key[2] < minZKey || current_key[2] > maxZKey)
				break; // Exceeded min or max height
				
			assert ( keyray.size() < keyray.sizeMax() - 1);
		}
	}
	
	RobustOcTreeNode* RobustOcTree::updateNode(const OcTreeKey& key, float visibility, float occupancy, bool lazy_eval) {
		// early abort (no change will happen).
		// may cause an overhead in some configuration, but more often helps
		RobustOcTreeNode* leaf = this->search(key);
		// no change: node already at minimum threshold
		if (leaf != NULL && occupancy == 0.0F && leaf->getOccupancy() <= clampingThresMin)
			return leaf;

		bool createdRoot = false;
		if (this->root == NULL){
			this->root = new RobustOcTreeNode();
			this->tree_size++;
			createdRoot = true;
		}

		return updateNodeRecurs(this->root, visibility, createdRoot, key, 0, occupancy, lazy_eval);
	}
	
	RobustOcTreeNode* RobustOcTree::updateNodeRecurs(RobustOcTreeNode* node, float visibility, bool node_just_created, const OcTreeKey& key,
		unsigned int depth, float occupancy, bool lazy_eval) {
		// Differences to OccupancyOcTreeBase implementation: only cosmetic
		
		unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
		bool created_node = false;

		assert(node);

		// follow down to last level
		if (depth < this->tree_depth) {
			if (!node->childExists(pos)) {
				// child does not exist, but maybe it's a pruned node?
				if ((!node->hasChildren()) && !node_just_created ) {
					// current node does not have children AND it is not a new node 
					// -> expand pruned node
					node->expandNode();
					this->tree_size+=8;
					this->size_changed = true;
				}
				else {
					// not a pruned node, create requested child
					node->createChild(pos);
					this->tree_size++;
					this->size_changed = true;
					created_node = true;
				}
			}

			if (lazy_eval)
				return updateNodeRecurs(node->getChild(pos), visibility, created_node, key, depth+1, occupancy, lazy_eval);
			else {
				RobustOcTreeNode* retval = updateNodeRecurs(node->getChild(pos), visibility, created_node, key, depth+1, occupancy, lazy_eval);
				// prune node if possible, otherwise set own probability
				// note: combining both did not lead to a speedup!
				if (node->pruneNode())
					this->tree_size -= 8;
				else
					node->updateOccupancyChildren();

				return retval;
			}
		}

		// at last level, update node, end of recursion
		else {
			updateNodeOccupancy(node, visibility, occupancy);
			return node;
		}
	}
	
	void RobustOcTree::updateNodeOccupancy(RobustOcTreeNode* occupancyNode, float probVisible, float occupancy) {
		float probOccupied = occupancyNode->getOccupancy();
		
		if(occupancy == 0.0F) {
			float probMiss = calcOccupiedProbability(probOccupied, probVisible, probMissIfOccupied, probMissIfNotOccupied, probMissIfNotVisible);
			occupancyNode->setOccupancy(probMiss > clampingThresMin ? probMiss : clampingThresMin);
		} else {
			float probHit = calcOccupiedProbability(probOccupied, probVisible, probHitIfOccupied, probHitIfNotOccupied, probHitIfNotVisible);
			if(occupancy == 1.0F)
				occupancyNode->setOccupancy(probHit < clampingThresMax ? probHit : clampingThresMax);
			else {
				float probMiss = calcOccupiedProbability(probOccupied, probVisible, probMissIfOccupied, probMissIfNotOccupied, probHitIfNotVisible);
				occupancyNode->setOccupancy(occupancy*probHit + (1.0F-occupancy)*probMiss);
				
				if (occupancyNode->getOccupancy() < clampingThresMin)
					occupancyNode->setOccupancy(clampingThresMin);
				else if (occupancyNode->getOccupancy() > clampingThresMax)
					occupancyNode->setOccupancy(clampingThresMax);
			}
		}
	}
	
	float RobustOcTree::calcOccupiedProbability(float probOccupied, float probVisible, float probMeasIfOccupied, float probMeasIfNotOccupied,
		float probMeasNotVisible) {
		// See our paper for a derivation of this formula
		return (probMeasIfOccupied*probVisible*probOccupied + probMeasNotVisible*(1.0F-probVisible)*probOccupied) /
			(probMeasIfOccupied*probVisible*probOccupied + probMeasIfNotOccupied*probVisible*(1.0F - probOccupied) +
				probMeasNotVisible*(1.0F-probVisible));
	}
}
