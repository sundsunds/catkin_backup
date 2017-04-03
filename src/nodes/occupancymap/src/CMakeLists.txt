#add_subdirectory(occmapping)
#add_subdirectory(occupancymap)
add_library(occupancy_map_nodelet	
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

#target_link_libraries(target ${Boost_LIBRARIES}) 

#rosbuild_link_boost(occupancy_map_nodelet system filesystem thread)
target_link_libraries(occupancy_map_nodelet ${EXTERNAL_LIBRARIES})
