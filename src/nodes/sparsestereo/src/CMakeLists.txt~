add_library(sparse_stereo_nodelet
	sparsestereonodelet.cpp
	sparsestereonodelet.h
	sparsestereobase.h
	sparsestereobase.cpp
)
#rosbuild_link_boost(sparse_stereo_nodelet system filesystem thread)
target_link_libraries(sparse_stereo_nodelet ${EXTERNAL_LIBRARIES})

add_executable(sparsestereo sparsestereonode.cpp)
target_link_libraries(sparsestereo sparse_stereo_nodelet)
#rosbuild_link_boost(sparsestereo signals)
#ks_create_link(sparsestereo ${CMAKE_CURRENT_BINARY_DIR}/../bin/sparsestereo ../../../bin/sparsestereo)
