add_library(capturenodelet
	capturenodelet.cpp
	capturenodelet.h
	capturebase.h
	capturebase.cpp
)
#rosbuild_link_boost(capturenodelet system filesystem thread)
target_link_libraries(capturenodelet ${EXTERNAL_LIBRARIES}) # ${SDL_LIBRARY})

add_executable(capture	capturenode.cpp)
target_link_libraries(capture capturenodelet)
#ks_create_link(capture ${CMAKE_CURRENT_BINARY_DIR}/../bin/capture ../../../bin/capture)


