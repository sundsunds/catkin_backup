#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <ros/ros.h>
#include <string>


namespace occupancymap {
	// Stores all supported parameters
	class Parameters {
	public:
		Parameters(std::string nodeletName);
		
		// All parameters are public members
		std::string calibFile;
		double maxDisp;
		bool graphicalCV, graphicalSDL;
		int subsampling;
		double minHeight;
		double maxHeight;
		double octreeResolution;
		std::string worldFrame;
		double maxPointDist;
		std::string mapSavePath;
		int octreePublishInterval;
		double probabilityHit;
		double probabilityMiss;
		double probHitIfOccupied;
		double probHitIfNotOccupied;
		double probHitIfNotVisible;
		double probVisibleIfOccluded;
		double probVisibleIfNotOccluded;
		double voxelErrorResolution;
		double disparityStdDev;
		double occupancyThreshold;
		double clampingThresholdMax;
		double clampingThresholdMin;
		double visibilityClampingMax;
		double visibilityClampingMin;
		std::string offlineProcessBag;
		double bagTimeOffset;
		double bagReplayDuration;
		std::string mapLoadPath;
	
	private:
		ros::NodeHandle privateNh;
	
		template <class T>
		bool readSingleParam(const char* name, T& dst);
	};
}

#endif
