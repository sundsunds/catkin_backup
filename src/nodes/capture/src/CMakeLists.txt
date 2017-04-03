add_library(capturenodelet
	capturenodelet.cpp
	capturenodelet.h
	capturebase.h
	capturebase.cpp
)

include_directories(${Boost_INCLUDE_DIRS})

#rosbuild_link_boost(capturenodelet ${Boost_LIBRARIES})
target_link_libraries(capturenodelet ${SDL_LIBRARY} ${Boost_LIBRARIES} ${EXTERNAL_LIBRARIES})

add_executable(capture	capturenode.cpp)
target_link_libraries(capture capturenodelet)
#ks_create_link(capture ${CMAKE_CURRENT_BINARY_DIR}/../bin/capture ../../../bin/capture)


