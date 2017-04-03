#ifndef OCTREEWRAPPER_H
#define OCTREEWRAPPER_H

#include <octomap/OcTree.h>
#include <boost/smart_ptr.hpp>
#include "parameters.h"

namespace occupancymap {
	// A more noise robust octree
	class OcTreeWrapper: public octomap::OcTree {
	public:
		OcTreeWrapper(boost::shared_ptr<Parameters> parameters);
		virtual ~OcTreeWrapper() {}
		
		// virtual constructor: creates a new object of same type
		virtual OcTreeWrapper* create() const {
			return new OcTreeWrapper(parameters);
		}
		
		std::string getTreeType() const {
			return "OcTree"; // Ensures compatibility with OcTree
		}
		
		virtual void insertPointCloud(const octomap::Pointcloud& scan, const octomap::point3d& sensor_origin,
			double maxrange=-1., bool lazy_eval = false);
		virtual void insertPointCloud(const octomap::Pointcloud& pc, const octomap::point3d& sensor_origin,
			const octomap::pose6d& frame_origin, double maxrange=-1., bool lazy_eval = false) {
			octomap::OcTree::insertPointCloud(pc, sensor_origin, frame_origin, maxrange, lazy_eval);
		}
		
	private:
		boost::shared_ptr<Parameters> parameters;
		
        /**
		 * Static member object which ensures that this OcTree's prototype
		 * ends up in the classIDMapping only once
		 */
		class StaticMemberInitializer{
		public:
			StaticMemberInitializer() {
				OcTreeWrapper* tree = new OcTreeWrapper(boost::shared_ptr<Parameters>());
				octomap::AbstractOcTree::registerTreeType(tree);
			}
		};
		
		// to ensure static initialization (only once)
		static StaticMemberInitializer ocTreeMemberInit;	
		
		// Prunes nodes that are outside the allowed min/max range
		void pruneMinMax();
	};
}
#endif
