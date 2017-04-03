#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <limits>
#include "libks/base/timer.h"
#include "libks/imageio/flycapqueue.h"
#include "libks/imageio/rosqueue.h"
#include "libks/stereo/sparse/sparsestereo-inl.h"
#include "libks/imageproc/census-inl.h"
#include "libks/imageproc/imageconversion.h"
#include "libks/feature/featurereducer.h"
#include "libks/base/typesequal.h"
#include "libks/base/sdlwindow.h"
#include "libks/base/cvwindow.h"
#include "libks/base/crashguard.h"
#include "sparsestereobase.h"

namespace sparsestereo {
	using namespace ks;
	using namespace std;
	using namespace boost;
	using namespace boost::posix_time;
	using namespace cv;
	using namespace ros;

	SparseStereoBase::SparseStereoBase()
		: currentFrame(0) {
		CrashGuard::setup();
	}
	
	void SparseStereoBase::onInit(int argc, char** argv, NodeHandle* nh, string name) {
		if(!parseOptions(argc, argv))
			throw ks::Exception("Error parsing program options");
		
		this->name = name;
		rosInput = !capture && inputDir == "";
		
		if(nh != NULL)
			nodeHandle.reset(new NodeHandle(*nh));
		else if(nh == NULL && (!noROS || rosInput)) {
			ros::init(argc, argv, ROS_PACKAGE_NAME);
			nodeHandle.reset(new NodeHandle());
			spinner.reset(new AsyncSpinner(1));
			spinner->start();
		}
		
		stereoPublisher.reset(!noROS ? new Publisher(nodeHandle->advertise<sensor_msgs::PointCloud>("/sparsestereo/points", 0)) : NULL);
		pyramidPublisher.reset(!noROS ? new SharedImageSetPublisher(*nodeHandle, "/sparsestereo/pyramid", 0) : NULL);
		
		if(capture)
			imgQueue.reset(new StereoFlyCapQueue8U(camera1, camera2));
		else if(inputDir != "")
			imgQueue.reset(new StereoFileQueue8U(inputDir.c_str(), false/*!performanceTest*/));
		else imgQueue.reset(new StereoRosQueue8U(*nodeHandle, shared, true, camera1, camera2));
		
		if(graphicalSDL)
			window.reset(new SDLWindow(1280, 480, name.c_str()));
		else if(graphicalCV)
			window.reset(new CVWindow(1280, 480, name.c_str()));
		if(window.get() != NULL)
			colCoder.reset(new ColorCoder(0, maxDisp/*round(maxDisp * 0.75 / 10.0)*10*/, true, false));
		
		if(calibFile != "")
			rectification.reset(new StereoRectification(CalibrationResult(calibFile.c_str())));
		sparseCompletion.reset(new SparseCompletion(maxDisp, 4));
			
#ifdef SPARSE_LEFT_DENSE_RIGHT
		stereo.reset(new SparseLeftDenseRightStereo<CORRELATION, short>(maxDisp, uniqueness));
#elif defined DENSE_TO_SPARSE
		stereo.reset(new DenseToSparse<WinnerTakesAll<CORRELATION, unsigned int> >(
			WinnerTakesAll<CORRELATION, unsigned int> (maxDisp, true, uniqueness)));
#else
		stereo.reset(new SparseStereo<CORRELATION, short>(maxDisp, 1, uniqueness, rectify ? NULL: rectification.get(), false, false, leftRightStep));
#endif

#ifdef HARRIS_CORNER
		shared_ptr<Harris> leftPtr(new Harris(adaptivity, true, 2));
#elif defined FAST_CORNER
		shared_ptr<FastFeatureDetector> leftPtr(new FastFeatureDetector((int)adaptivity /*20*/, true));
#else
		shared_ptr<EXTENDED_FAST> leftPtr(new EXTENDED_FAST(true, 10, adaptivity, false, WINDOW_SIZE/2));
#endif
#ifdef IMAGE_PYRAMID
		leftFeatureDetector.reset(new PyramidRangeDetector(leftPtr, leftPtr, 3, true));
#else
		leftFeatureDetector = leftPtr;
#endif
#ifdef HARRIS_CORNER
		rightFeatureDetector.reset(new Harris(adaptivity, false, 2));
#elif defined FAST_CORNER
		rightFeatureDetector.reset(new FastFeatureDetector((int)adaptivity /*20*/, false));
#else
		rightFeatureDetector.reset(new EXTENDED_FAST(false, 10, adaptivity, false, WINDOW_SIZE/2));
#endif
	}

	void SparseStereoBase::mainLoop() {
		ptime lastTime = microsec_clock::local_time();
		int sumFeatLeft = 0, sumFeatRight = 0;
		double sumFeatMatched = 0.0;
		vector<SparseMatch> correspondences;
		StereoFrame8U::Type rectPair;
		
		int frames = 0;
		ks::Timer timer(name.c_str());
		
		while(nodeHandle == NULL || ros::ok()) {
			// Load input images
			if(!readStereoPair())
				break;
			
			if(performanceTest)
				lastTime = microsec_clock::local_time(); // Only measure the stereo matching
			
			timer.start();
			for(int i=0; i< (performanceTest ? 10 : 1); i++) {
				if(rectify)
					rectification->rectifyStereoPair(*stereoPair, &rectPair);
				else rectPair = *stereoPair;

				extractFeatures(rectPair);				
				
				// Stereo matching
				correspondences.clear();
#ifdef CENSUS_TRANSFORM
				Mat &srcLeft = censusLeft, &srcRight = censusRight;
#else
				Mat &srcLeft = rectPair.first, &srcRight = rectPair.second;
#endif
				
#if defined(SPARSE_LEFT_DENSE_RIGHT) || defined (DENSE_TO_SPARSE)
				stereo->match(srcLeft, srcRight, keypointsLeft, &correspondences);
#else
				stereo->match(srcLeft, srcRight, keypointsLeft, keypointsRight, &correspondences);
#endif						
				subpixelRefine(rectPair, &correspondences);
				
				frames++;
				sumFeatLeft += keypointsLeft.size();
				sumFeatRight += keypointsRight.size();
				sumFeatMatched += double(correspondences.size()) / keypointsLeft.size() *100.0;
			}
			
			time_duration elapsed = (microsec_clock::local_time() - lastTime);
			if(performanceTest || elapsed.total_seconds() >= 1 || interactive) {
				ROS_INFO_STREAM("Fps: " << (frames/(elapsed.total_microseconds()/1.0e6))
					<< " ; Avg. Feat. Left: " << (sumFeatLeft/double(frames))
					<< " ; Avg. Feat. Right: " << (sumFeatRight/double(frames))
					<< " ; Avg. Feat. Matched: " << (sumFeatMatched/frames) << "%");
				lastTime = microsec_clock::local_time();
				frames = 0;
				sumFeatLeft = 0;
				sumFeatRight = 0;
				sumFeatMatched = 0.0;
			}
			
			if(!noROS) {
				publishMatches(correspondences);
				publishPyramid(rectPair);
			}
			
			timer.stop();
			
			// Visualize
			if(window.get() != NULL)
				visualizeMatches(rectPair, correspondences);
			writePoints(correspondences);
			currentFrame++;
		}
	}

	bool SparseStereoBase::parseOptions(int argc, char** argv) {
		bool error = false;
		interactive = false;
		rectify = false;
		capture = false;
		uniqueness = 0.7;
		graphicalSDL = false;
		graphicalCV = false;
		maxDisp = 70;
		leftRightStep = 2;
		performanceTest = false;
		adaptivity = 1.0;
		writePath = "";
		camera1 = 0; camera2 = 1;
		noROS = false;
		maxFeatures = numeric_limits<int>::max();
		worldFrame = "/world";
		shared = false;
		
		char c;
		optind = 0;
		while ((c = getopt(argc, argv, "hr:o:IfcC:p:u:gGm:s:Pa:w:Ri:M:W:S")) != -1) {
			switch(c) {
				case 'r': calibFile = optarg; break;
				case 'o': videoFile = optarg; break;
				case 'I': interactive = true; break;
				case 'f': rectify = true; break;
				case 'c': capture = true; break;
				case 'C': sscanf(optarg,  "%d,%d", &camera1, &camera2); break;
				case 'p': pointsFile = optarg; break;
				case 'u': uniqueness = atof(optarg); break;
				case 'g': graphicalSDL = true; break;
				case 'G': graphicalCV = true; break;
				case 'm': maxDisp = atof(optarg); break;
				case 's': leftRightStep = atoi(optarg); break;
				case 'P': performanceTest = true; break;
				case 'a': adaptivity = atof(optarg); break;
				case 'w': writePath = optarg; break;
				case 'R': noROS = true; break;
				case 'i': inputDir = optarg; break;
				case 'M': maxFeatures = atoi(optarg); break;
				case 'W': worldFrame = optarg; break;
				case 'S': shared = true; break;
				case 'h':
				default: error = true; break;
			}
		}
		
		if(error || argc - optind != 0) {
			cerr << "Usage: " << endl
				 << argv[0] << " [OPTIONS]" << endl << endl
				 << "Options: " << endl
				 << "-h        Print this help message" << endl
				 << "-g        Graphical with SDL window" << endl
				 << "-G        Graphical with OpenCV window" << endl
				 << "-R        Disables ROS publishing" << endl
				 << "-r FILE   Use given calibration file for rectification" << endl
				 << "-f        Perform full rectification" << endl
				 << "-o FILE   Output to video file" << endl
				 << "-p FILE   Output points to file" << endl
				 << "-i DIR    Replay the files from the given input directory" << endl
				 << "-c        Capture from camera" << endl
				 << "-C LIST   Comma separated list of camera indices" << endl
				 << "-I        Interactive" << endl
				 << "-u N      Threshold for uniqueness constraint" << endl
				 << "-m N      Set maximum disparity" << endl
				 << "-M N      Set maximum number of features" << endl
				 << "-s N      Set step width for left/right consistency check" << endl
				 << "-a N      Set exFAST adaptivity or FAST threshold to N" << endl
				 << "-P        Run performance test" << endl
				 << "-w DIR    Write images to the given directory" << endl
				 << "-W FRAME  Sets the world frame (default: /world)" << endl
				 << "-S        Use shared memory for image transport" << endl
				 << endl;
			return false;
		}
		
		return true;
	}
	
	void SparseStereoBase::writePoints(const vector<SparseMatch>& points) {
		if(pointsFile == "")
			return; // Not activated
		
		if(!pointsFileStrm.is_open()) {
			pointsFileStrm.open(pointsFile.c_str(), ios::out);
			pointsFileStrm << "# Frame; Points Count; [ Rectified X ; Rectified Y; Image X; Image Y; Disparity ]*" << endl;
		}
		if(pointsFileStrm.fail())
			throw ks::Exception("Error writing points");
		
		pointsFileStrm << currentFrame << " " << points.size();
		for(unsigned int i=0; i<points.size(); i++)
			pointsFileStrm  <<  " " << points[i].rectLeft.x << " " << points[i].rectLeft.y
				<< " " << points[i].imgLeft->pt.x << " " << points[i].imgLeft->pt.y << " " << points[i].disparity();
		pointsFileStrm << endl;
	}
	
	// Reads the next stereo pair and stores it in a member variable
	bool SparseStereoBase::readStereoPair() {
		while(true) {
			stereoPair = imgQueue->pop();
			if(stereoPair == NULL)
				return false; // No more images

			if(rosInput)
				messageTime = ((StereoRosQueue8U*)imgQueue.get())->getLastMsgTime();
			else if(!noROS) messageTime = Time::now();
				
			if(stereoPair->first.data == NULL || stereoPair->second.data == NULL) {
				// An incomplete image. Lets keep on producing notifications
				publishMatches(vector<SparseMatch>());
			} else break;
		}
		
		if(stereoPair->first.size() != stereoPair->second.size())
			throw ks::Exception("Image sizes don't match");
				
		if(censusLeft.size() != stereoPair->first.size()) {
			censusLeft = Mat_<unsigned int>(stereoPair->first.size(), 0);
			charLeft = Mat_<char>(stereoPair->first.size(), 0);
			censusRight = Mat_<unsigned int>(stereoPair->second.size(), 0);	
			charRight = Mat_<char>(stereoPair->second.size(), 0);	
			if(window != NULL) {
				screen = colCoder->createLegendBorder(stereoPair->first.cols, stereoPair->first.rows);
				window->resize(screen.cols + stereoPair->first.cols, screen.rows);
				if(video.get() == NULL && videoFile.length() != 0) {
					// First initialization
					cout << "Writing video to " << videoFile << endl;
					video.reset(new VideoWriter(videoFile.c_str(), CV_FOURCC('D','I','V','X'), 30.0, screen.size()));
					if(!video->isOpened())
						cerr << "Unable to open video stream" << endl;
				}
			}
		}
		
		return true;
	}
	
	
	// Displays the matches as colored boxes
	void SparseStereoBase::visualizeMatches(const StereoFrame8U::Type& pair, const vector<SparseMatch>& correspondences) {
		Mat_<Vec3b> imageArea = screen(Rect(0, 0, pair.first.cols, pair.first.rows));
		cvtColor(pair.first, imageArea, CV_GRAY2BGR);
		
		/*Mat_<float> dispMap(pair.first.size(), 0);
		sparseCompletion->match(correspondences, censusLeft, censusRight, dispMap);
		Mat_<Vec3b> colImg(dispMap.size());
		colCoder->codeImage(dispMap, colImg);
		for(int y=0; y<imageArea.rows; y++)
			for(int x=0; x<imageArea.cols; x++)
				imageArea(y,x) = colImg(y/4, x/4);
				
		std_msgs::Header header;
		header.stamp = messageTime;
		header.frame_id = worldFrame;
		header.seq = currentFrame;
		static SharedImageSetPublisher densePub(*nodeHandle, "/densestereo/disparity_map", 0);
		densePub.publish(header, dispMap);*/

		for(int i=0; i<(int)correspondences.size(); i++)
			rectangle(screen, correspondences[i].imgLeft->pt - Point2f(2,2),
				correspondences[i].imgLeft->pt + Point2f(2, 2), 
				(Scalar) colCoder->getColor((float)correspondences[i].disparity()), CV_FILLED);
	
		// Visualisation for papers
		/*for(int i=0; i<(int)correspondences.size(); i++)
			rectangle(screen, correspondences[i].imgLeft->pt - Point2f(4,4),
				correspondences[i].imgLeft->pt + Point2f(4, 4), 
				(Scalar) Vec3b(0, 0, 0), CV_FILLED);
		
		for(int i=0; i<(int)correspondences.size(); i++)
			rectangle(screen, correspondences[i].imgLeft->pt - Point2f(2,2),
				correspondences[i].imgLeft->pt + Point2f(2, 2), 
				(Scalar) colCoder->getColor((float)correspondences[i].disparity()), CV_FILLED);*/
		
		window->displayStereoPair(std::pair<Mat, Mat>(screen, pair.second));
		if(interactive)
			window->waitForKey();
		window->processEvents(false);
		
		if(video || writePath != "") {
			/*Mat_<Vec3b> frame(screen.rows, screen.cols + pair.second.cols);
			Mat_<Vec3b> leftWindow = frame(Rect(0, 0, screen.cols, screen.rows)),
				rightWindow = frame(Rect(screen.cols, 0, pair.second.cols, pair.second.rows));
			screen.copyTo(leftWindow);
			cvtColor(pair.second, rightWindow, CV_GRAY2RGB);*/

			if(video)
				(*video) << screen;
		
			if(writePath != "") {
				char fileName[256];
				snprintf(fileName, sizeof(fileName), "%s/image%04d.png", writePath.c_str(), currentFrame);
				
				Mat_<Vec3b> tmp(screen.rows, screen.cols + pair.second.cols);
				Mat_<Vec3b> left = tmp(Rect(0, 0, screen.cols, screen.rows)),
					right = tmp(Rect(screen.cols, 0, pair.second.cols, screen.rows)) ;
				screen.copyTo(left);
				ImageConversion::convertToColor(pair.second, &right);
				imwrite(fileName, tmp/*screen*/);
			}
		}
	}
	
	// Extracts feature points
	void SparseStereoBase::extractFeatures(const StereoFrame8U::Type& rectPair) {
		#pragma omp parallel sections default(shared) num_threads(2)
		{
			#pragma omp section
			{
				ImageConversion::unsignedToSigned(rectPair.first, &charLeft);
#ifdef CENSUS_TRANSFORM
				//Census::transform9x3(charLeft, &censusLeft);
				Census::transform5x5(charLeft, &censusLeft);
#endif			
				keypointsLeftUnreduced.clear();
#ifdef IMAGE_PYRAMID
				static_cast<PyramidRangeDetector*>(leftFeatureDetector.get())->setUnsignedImage(&rectPair.first);
#endif			
				leftFeatureDetector->detect(rectPair.first, keypointsLeftUnreduced);
				
				FeatureReducer reducer(maxFeatures, rectPair.first.cols, rectPair.first.rows);
				reducer.reduce(keypointsLeftUnreduced, &keypointsLeft);
			}
			#pragma omp section
			{
				ImageConversion::unsignedToSigned(rectPair.second, &charRight);
#ifdef CENSUS_TRANSFORM
				//Census::transform9x3(charRight, &censusRight);
				Census::transform5x5(charRight, &censusRight);
#endif			
				keypointsRight.clear();
				rightFeatureDetector->detect(rectPair.second, keypointsRight);
			}
		}
	}
	
	// Performs subpixel refinement of matched stereo points (DOESN'T WORK VERY WELL!)
	void SparseStereoBase::subpixelRefine(const StereoFrame8U::Type& rectPair, vector<SparseMatch>* matches) {
		// Refine by stereo matching
		//stereo->subpixelRefine(censusLeft, censusRight, matches);
	
		// Refine by feature matching
		/*for(unsigned int i=0; i<matches->size(); i++) {
			if((*matches)[i].imgRight == NULL)
				continue; // No refinement possible
			
			// Right feature detector should always be a exFAST, when using subpixel refinement
			static_cast<EXTENDED_FAST*>(rightFeatureDetector.get())->subpixelRefine(
				rectPair.first,  const_cast<KeyPoint*>((*matches)[i].imgLeft), true, true);
			static_cast<EXTENDED_FAST*>(rightFeatureDetector.get())->subpixelRefine(
				rectPair.second,  const_cast<KeyPoint*>((*matches)[i].imgRight), true, false);

			if(!rectify && rectification.get() != NULL) {
				(*matches)[i].rectLeft = rectification->rectifyLeftPoint((*matches)[i].imgLeft->pt);
				(*matches)[i].rectRight = rectification->rectifyRightPoint((*matches)[i].imgRight->pt);
			} else {
				(*matches)[i].rectLeft = (*matches)[i].imgLeft->pt;
				(*matches)[i].rectRight = (*matches)[i].imgRight->pt;
			}
			
			// Due to subpixel optimization, disparities smaller than 0 might occur. In these cases,
			// we set the disparity to 0 again.
			if((*matches)[i].disparity() < 0)
				(*matches)[i].rectRight = (*matches)[i].rectLeft;
		}*/
	}
	
	// Publishes the stereo matches through ROS
	void SparseStereoBase::publishMatches(const vector<SparseMatch>& matches) {
		vector<Point3f> points3d;
		SparseMatch::projectMatches(matches, &points3d, rectification.get());
		
		sensor_msgs::PointCloudPtr msg(new sensor_msgs::PointCloud);
		msg->header.stamp = messageTime;
		msg->header.frame_id = worldFrame;
		msg->header.seq = currentFrame;
		
		sensor_msgs::ChannelFloat32 uChannel, vChannel, octChannel;
		uChannel.name = "u";
		vChannel.name = "v";
		octChannel.name = "octave";
		
		msg->channels.push_back(uChannel);
		msg->channels.push_back(vChannel);
		msg->channels.push_back(octChannel);
		
		msg->points.reserve(points3d.size());
		msg->channels[0].values.reserve(points3d.size());
		msg->channels[1].values.reserve(points3d.size());
		msg->channels[2].values.reserve(points3d.size());
		
		for(unsigned int i=0; i<points3d.size(); i++) {
			geometry_msgs::Point32 pt;
			
			// Coordinates are swapped, because PTAM assumes the
			// ground plane to be XY
			pt.x = points3d[i].x;
			pt.y = points3d[i].z;
			pt.z = -points3d[i].y;
			msg->points.push_back(pt);
			
			// u is y and v is x! Weird ROS!
			msg->channels[0].values.push_back(matches[i].imgLeft->pt.y);
			msg->channels[1].values.push_back(matches[i].imgLeft->pt.x);
			msg->channels[2].values.push_back(matches[i].imgLeft->octave);
		}
		
		stereoPublisher->publish(msg);
	}
	
	void SparseStereoBase::publishPyramid(const StereoFrame8U::Type& stereoPair) {
#ifdef IMAGE_PYRAMID
		PyramidRangeDetector* pyramidDetector = static_cast<PyramidRangeDetector*>(leftFeatureDetector.get());
		
		std_msgs::Header header;
		header.stamp = messageTime;
		header.frame_id = worldFrame;
		header.seq = currentFrame;
		
		pyramidPublisher->publish(header,
			stereoPair.first,
			pyramidDetector->getPyramidLevel(1),
			pyramidDetector->getPyramidLevel(2),
			pyramidDetector->getPyramidLevel(3)
		);
#endif
	}
}
