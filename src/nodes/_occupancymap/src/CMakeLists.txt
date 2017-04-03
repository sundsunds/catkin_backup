rosbuild_add_library(occupancy_map_nodelet
	ImageSet.h
	MultiCameraImage.h
	SharedImageSet.h	
	occupancymapnodelet.h
	occupancymapnodelet.cpp
	visualization.h
	visualization.cpp
	parameters.h
	parameters.cpp
	robustoctree.h
	robustoctree.cpp
	robustoctreenode.h
	robustoctreenode.cpp
	occprevkeyray.h
	probabilitylookup.h
	probabilitylookup.cpp
	settings.h
	octreewrapper.h
	octreewrapper.cpp
)

rosbuild_link_boost(occupancy_map_nodelet system filesystem thread)

target_link_libraries(occupancy_map_nodelet ${SDL_LIBRARY})
