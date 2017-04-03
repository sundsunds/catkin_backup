#include "parameters.h"
#include <libks/base/exception.h>

namespace occupancymap {
	using namespace ros;

	Parameters::Parameters(std::string nodeletName)
		: privateNh(nodeletName) {
		// Read all neccessary ros parameters
		if(!readSingleParam("calib_file", calibFile) ||
			!readSingleParam("max_disparity", maxDisp) ||
			!readSingleParam("cv_window", graphicalCV) ||
			!readSingleParam("sdl_window", graphicalSDL) ||
			!readSingleParam("subsampling_factor", subsampling) ||
			!readSingleParam("min_height", minHeight) ||
			!readSingleParam("max_height", maxHeight) ||
			!readSingleParam("octree_resolution", octreeResolution) ||
			!readSingleParam("world_frame", worldFrame) || 
			!readSingleParam("max_point_dist", maxPointDist) ||
			!readSingleParam("map_save_path", mapSavePath) ||
			!readSingleParam("octree_publish_interval", octreePublishInterval) ||
			!readSingleParam("prob_hit", probabilityHit) ||
			!readSingleParam("prob_miss", probabilityMiss) ||
			!readSingleParam("prob_hit_if_free", probHitIfNotOccupied) ||
			!readSingleParam("prob_hit_if_occupied", probHitIfOccupied) ||
			!readSingleParam("prob_hit_if_not_visible", probHitIfNotVisible) ||
			!readSingleParam("prob_visible_if_occluded", probVisibleIfOccluded) ||
			!readSingleParam("prob_visible_if_not_occluded", probVisibleIfNotOccluded) ||
			!readSingleParam("disparity_std_dev", disparityStdDev) ||
			!readSingleParam("voxel_error_resolution", voxelErrorResolution) ||
			!readSingleParam("occupancy_threshold", occupancyThreshold) ||
			!readSingleParam("clamping_threshold_max", clampingThresholdMax) ||
			!readSingleParam("clamping_threshold_min", clampingThresholdMin) ||
			!readSingleParam("visibility_clamping_min", visibilityClampingMin) ||
			!readSingleParam("visibility_clamping_max", visibilityClampingMax) ||
			!readSingleParam("offline_process_bag", offlineProcessBag) ||
			!readSingleParam("bag_time_offset", bagTimeOffset) ||
			!readSingleParam("bag_replay_duration", bagReplayDuration) ||
			!readSingleParam("map_load_path", mapLoadPath)) { 
			ROS_FATAL("Not all mandatory parameters set! Please use correct launch file!");
			throw ks::Exception("Error reading parameters");
		}
	}
	
	template <class T>
	bool Parameters::readSingleParam(const char* name, T& dst) {
		if(!privateNh.getParam(name, dst)) {
			ROS_FATAL_STREAM("Parameter \"" << name << "\" not present!");
			return false;
		} else return true;
	}
}
