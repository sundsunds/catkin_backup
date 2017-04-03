#add_definitions("-DHAS_FLYCAPTURE")
#set (FoundFlyCapture true)
if(FoundFlyCapture)
	set(FlyCapFiles imageio/flycapqueue.h imageio/flycapqueue.cpp)
        message(${FlyCapFiles} " RAJ found Flycapture ")
endif(FoundFlyCapture)

add_library(ks
	stereo/correlation/censuswindow.h
	stereo/correlation/zsadwindow.h
	stereo/dense/densestereoalgorithm.h
	stereo/dense/cvblockmatch.h
	stereo/dense/cvblockmatch-inl.h
	stereo/dense/cvsgm.h
	stereo/dense/cvsgm-inl.h
	stereo/dense/winnertakesall.h
	stereo/dense/winnertakesall-inl.h
	stereo/sparse/sparsestereo.h
	stereo/sparse/sparsestereo-inl.h
	stereo/sparse/sparsecostcube.h
	stereo/sparse/sparseleftdenserightstereo.h
	stereo/sparse/sparseleftdenserightstereo-inl.h
	stereo/sparse/sparseconstraints.h
	stereo/sparse/sparseconstraints.cpp
	stereo/sparse/sparsematch.h
	stereo/sparse/sparserectification.h
	stereo/sparse/sparserectification.cpp
	stereo/sparse/densetosparse.h
	stereo/cameracalibration.h
	stereo/cameracalibration.cpp
	stereo/stereorectification.h
	stereo/stereorectification.cpp
	stereo/dense/sparsecompletion.h
	stereo/dense/sparsecompletion.cpp
	imageio/rawimage.cpp
	imageio/rawimage.h
	imageio/imagequeue.h
	imageio/imagequeue.cpp
	imageio/filequeue.h
	imageio/filequeue.cpp
	imageio/tvrawimageio.h
	imageio/tvrawimageio.cpp
	imageio/rawsequence.h
	imageio/rawsequence.cpp
	imageio/rosqueue.h
	imageio/rosqueue.cpp
	imageio/sharedimagesetpublisher.h
	imageio/sharedimagesetpublisher.cpp
	${FlyCapFiles}
	base/simd.h
	base/simd.cpp
	base/typesequal.h
	base/exception.h
	base/hammingdistance.h
	base/hammingdistance.cpp
	base/basewindow.h
	base/cvwindow.h
	base/cvwindow.cpp
	base/sdlwindow.h
	base/sdlwindow.cpp
	base/subpixelinterpolation.h
	base/argvconverter.h
	base/argvconverter.cpp
	base/crashguard.h
	base/crashguard.cpp
	base/timer.h
	base/timer.cpp
	feature/fakefast.h
	feature/fakefast.cpp
	feature/censusfeature.h
	feature/censusfeature.cpp
	feature/faster.h
	feature/featuretester.h
	#feature/faster-detect.cpp
	#feature/faster-detect.cpp.o
	#feature/faster-score.cpp
	#feature/faster-score.cpp.o
	feature/faster-dummy.cpp
	feature/faster.cpp
	feature/extendedfast.cpp
	feature/extendedfast.h
	feature/extendedfast2.h
	feature/extendedfast2.cpp
	feature/extendedfast3.h
	feature/extendedfast3.cpp
	feature/ssefast.h
	feature/ssefast-inl.h
	feature/ssefast.cpp
	feature/fast9.h
	feature/fast9-inl.h
	feature/harris.h
	feature/harris.cpp
	feature/featurereducer.h
	feature/featurereducer.cpp
	feature/pyramidrangedetector.h
	feature/pyramidrangedetector.cpp
	feature/evaluation/clusterevaluation.h
	feature/evaluation/clusterevaluation.cpp
	feature/evaluation/repeatabilityevaluation.h
	feature/evaluation/repeatabilityevaluation.cpp
	imageproc/census.h
	imageproc/census-inl.h
	imageproc/census.cpp
	imageproc/colorcoder.h
	imageproc/colorcoder.cpp
	imageproc/imageconversion.h
	imageproc/imageconversion.cpp
	imageproc/bilinearinterpolation.h
	#imageproc/noiserobustcensus.h
	#imageproc/noiserobustcensus.cpp
	#plane/planeesmestimator.h
	#plane/planeesmestimator.cpp
	#plane/groundplaneransac.h
	#plane/groundplaneransac.cpp
)


target_link_libraries(ks ${EXTERNAL_LIBRARIES} ${catkin_LIBRARIES})

if(FoundFlyCapture)
	target_link_libraries(ks ${FlyCapture_LIBRARY}  ${catkin_LIBRARIES})
endif(FoundFlyCapture)

add_dependencies(ks libks_msgs_gencpp)

if(FoundCppUnit)
	add_subdirectory(tests)
endif(FoundCppUnit)
