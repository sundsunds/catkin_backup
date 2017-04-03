add_executable(testrunner
	testrunner.cpp
	hammingdistance.h
	census.h
	census.cpp
	sparsecostcube.h
	sparsecostcube.cpp
	pyramidrangedetector.h
	pyramidrangedetector.cpp
)

target_link_libraries(testrunner ${Boost_LIBRARIES} 
	${OPENGL_LIBRARIES} ${CppUnit_LIBRARY} ks)
