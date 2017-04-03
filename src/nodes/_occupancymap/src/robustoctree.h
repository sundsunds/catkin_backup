#ifndef ROBUSTOCTREE_H
#define ROBUSTOCTREE_H

#include <octomap/OcTreeBase.h>
#include <octomap/OcTree.h>
#include <boost/smart_ptr.hpp>
#include <tr1/unordered_map>
#include "robustoctreenode.h"
#include "parameters.h"
#include "occprevkeyray.h"
#include "probabilitylookup.h"

namespace occupancymap {
	// A more noise robust octree
	class RobustOcTree: public octomap::OcTreeBase<RobustOcTreeNode> {
	public:
		RobustOcTree(boost::shared_ptr<Parameters> parameters);
		virtual ~RobustOcTree() {}
		
		// Methods for processing scans
		virtual void insertPointCloud(const octomap::Pointcloud& scan, const octomap::point3d& sensor_origin,
			const octomap::point3d& forwardVec, double maxrange=-1., bool lazy_eval = false);
			
		virtual void insertPointCloud(const octomap::Pointcloud& pc, const octomap::point3d& sensor_origin,
			const octomap::pose6d& frame_origin, double maxrange, bool lazy_eval = false);

		// virtual constructor: creates a new object of same type
		virtual RobustOcTree* create() const {
			return new RobustOcTree(parameters);
		}
		
		std::string getTreeType() const {
			return "OcTree"; // Ensures compatibility with OcTree
		}

		// Performs thresholding of the occupancy probability for the given node
		inline bool isNodeOccupied(const RobustOcTreeNode& node) const {
			return node.getOccupancy() > occupancyProbThres;
		}

		inline bool isNodeOccupied(const RobustOcTreeNode* node) const {
			return isNodeOccupied(*node);
		}
	
	private:
		typedef octomap::OcTreeBase<RobustOcTreeNode> Base;
		typedef std::tr1::unordered_map<octomap::OcTreeKey, float, octomap::OcTreeKey::KeyHash> KeyVisMap;
		static octomap::OcTreeKey ROOT_KEY;
		
		boost::shared_ptr<Parameters> parameters;
		float voxelErrorResolution;
		float probHitIfOccupied, probHitIfNotOccupied;
		float probMissIfOccupied, probMissIfNotOccupied;
		float probHitIfNotVisible, probMissIfNotVisible;
		float occupancyProbThres;
		float clampingThresMax, clampingThresMin;
		float visClampMax, visClampMin;
		OccPrevKeyRay keyray, prevKeyray;
		OccPrevKeySet voxelUpdateCache;
		KeyVisMap visibilities;
		unsigned short int maxZKey, minZKey;
		boost::scoped_ptr<ProbabilityLookup> lookup;
							
		/**
		 * Static member object which ensures that this OcTree's prototype
		 * ends up in the classIDMapping only once
		 */
		class StaticMemberInitializer{
		public:
			StaticMemberInitializer() {
				RobustOcTree* tree = new RobustOcTree(boost::shared_ptr<Parameters>());
				octomap::AbstractOcTree::registerTreeType(tree);
			}
		};
		
		// to ensure static initialization (only once)
		static StaticMemberInitializer ocTreeMemberInit;
		
		// Casts all rays and fills the update cache
		void computeUpdate(const octomap::Pointcloud& scan, const octomap::point3d& origin, const octomap::point3d& forwardVec, double maxrange);
		
		// Casts a single ray and stores it in keyray
		void computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end, const octomap::point3d& forwardVec, double maxrange);
		
		// Manipulate occupancy probability of node directly
		RobustOcTreeNode* updateNodeRecurs(RobustOcTreeNode* node, float visibility, bool node_just_created, const octomap::OcTreeKey& key,
			unsigned int depth, float occupancy, bool lazy_eval);
		RobustOcTreeNode* updateNode(const octomap::OcTreeKey& key, float visibility, float occupancy, bool lazy_eval);
		__always_inline void updateNodeOccupancy(RobustOcTreeNode* occupancyNode, float occupancy, float measProbIfNotOcc);
		
		// Calculates the new occupancy probability
		__always_inline float calcOccupiedProbability(float probOccupied, float probVisible, float probMeasIfOccupied,
			float probMeasIfNotOccupied, float probMeasNotVisible);
		
		// Computes a visibility value for all nodes in voxelUpdateCache
		void calcVisibilities(const octomap::point3d& sensor_origin);
	};
}
#endif
