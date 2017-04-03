#include "octreewrapper.h"
#include <iostream>

using namespace std;
using namespace octomap;

namespace occupancymap {
	OcTreeWrapper::StaticMemberInitializer OcTreeWrapper::ocTreeMemberInit;

	OcTreeWrapper::OcTreeWrapper(boost::shared_ptr<Parameters> parameters)
		: OcTree(parameters != NULL ? parameters->octreeResolution : 0.1), parameters(parameters) {
		if(parameters != NULL) {
			setProbHit(parameters->probabilityHit);
			setProbMiss(parameters->probabilityMiss);
			setOccupancyThres(parameters->occupancyThreshold);
			setClampingThresMax(parameters->clampingThresholdMax);
			setClampingThresMin(parameters->clampingThresholdMin);
		}
	}
	
	void OcTreeWrapper::insertPointCloud(const Pointcloud& scan, const octomap::point3d& sensor_origin,
		double maxrange, bool lazy_eval) {

		KeySet free_cells, occupied_cells;
		computeUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);
		
		int minZKey = 0, maxZKey = 0xFFFF;
		OcTreeKey key;
		if(coordToKeyChecked(0, 0, parameters->minHeight, key))
			minZKey = key[2];
		if(coordToKeyChecked(0, 0, parameters->maxHeight, key))
			maxZKey = key[2];

		// insert data into tree	-----------------------
		for (KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
			if((*it)[2] >= minZKey && (*it)[2] <= maxZKey)
				updateNode(*it, false, lazy_eval);
		}
		for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
			if((*it)[2] >= minZKey && (*it)[2] <= maxZKey)
				updateNode(*it, true, lazy_eval);
		}
		//OcTree::insertPointCloud(scan, sensor_origin, maxrange, lazy_eval);
	}
}