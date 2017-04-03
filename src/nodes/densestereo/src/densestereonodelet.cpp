#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <limits>
#include <pluginlib/class_list_macros.h>

#include "opencv2/contrib/contrib.hpp"

#include "libks/imageio/flycapqueue.h"
#include "libks/imageio/rosqueue.h"
#include "libks/base/argvconverter.h"
#include "libks/base/sdlwindow.h"
#include "libks/base/cvwindow.h"
#include "libks/base/crashguard.h"
#include "libks/base/timer.h"
#include "libks_msgs/SharedImageSet.h"
#include "libks/stereo/dense/winnertakesall-inl.h"
#include "libks/stereo/correlation/censuswindow-inl.h"
#include "libks/imageproc/census-inl.h"
#include "densestereonodelet.h"

PLUGINLIB_DECLARE_CLASS(densestereo, DenseStereoNodelet, densestereo::DenseStereoNodelet, nodelet::Nodelet)

namespace densestereo {
	using namespace ks;
	using namespace std;
	using namespace boost;
	using namespace boost::posix_time;
	using namespace cv;
	using namespace ros;

	DenseStereoNodelet::DenseStereoNodelet()
		: currentFrame(0) {
		CrashGuard::setup();
	}
	
	DenseStereoNodelet::~DenseStereoNodelet() {
		mainThread.join();
	}

	void DenseStereoNodelet::onInit() {
		if(!parseOptions())
			throw ks::Exception("Error parsing program options");
	
		rosInput = !capture && inputDir == "";
		
		if(capture)
			imgQueue.reset(new StereoFlyCapQueue8U(camera1, camera2));
		else if(inputDir != "")
			imgQueue.reset(new StereoFileQueue8U(inputDir.c_str(), !performanceTest));
		else imgQueue.reset(new StereoRosQueue8U(getNodeHandle(), shared, true, camera1, camera2, rate));
		
		stereoPublisher.reset(!noROS ? new SharedImageSetPublisher(getNodeHandle(), "/densestereo/disparity_map", 0) : NULL);
		
		if(graphicalSDL)
			window.reset(new SDLWindow(1280, 480, getName().c_str()));
		else if(graphicalCV)
			window.reset(new CVWindow(1280, 480, getName().c_str()));
		if(window.get() != NULL)
			colCoder.reset(new ColorCoder(0, maxDisp, true, false, ColorCoder::BLUE_WHITE_RED));
		
		if(calibFile != "")
			rectification.reset(new StereoRectification(CalibrationResult(calibFile.c_str(), 0.0)));
		
		// Initialize elas
		Elas::parameters param;
		param.disp_max = maxDisp;
		param.subsampling_step = subsampling;
		param.postprocess_only_left = true;
		elas.reset(new Elas(param));
		
		// Initialize SGBM
		sgbm.reset(new StereoSGBM());
		
		sgbm->preFilterCap = 63;
		sgbm->SADWindowSize = 7;//11;
		sgbm->P1 = 2*sgbm->SADWindowSize*sgbm->SADWindowSize;
		sgbm->P2 = 5*sgbm->SADWindowSize*sgbm->SADWindowSize;
		sgbm->minDisparity = 0;
		sgbm->numberOfDisparities = maxDisp;
		sgbm->uniquenessRatio = 20;
		sgbm->speckleWindowSize = 100;
		sgbm->speckleRange = 32;
		sgbm->fullDP = true;
			
		mainThread = thread(bind(&DenseStereoNodelet::mainLoop, this));
	}

	void DenseStereoNodelet::mainLoop() {
		ptime lastTime = microsec_clock::local_time();
		StereoFrame8U::Type rectPair;
		Mat_<float> dispMap;
		
		int frames = 0;
		ks::Timer timer("Dense Stereo");
		
		while(ros::ok()) {
			// Load input images
			if(!readStereoPair())
				break;

			if(performanceTest)
				lastTime = microsec_clock::local_time(); // Only measure the stereo matching
			
			timer.start();
			for(int i=0; i< (performanceTest ? 10 : 1); i++) {
				if(rectification != NULL)
					rectification->rectifyStereoPair(*stereoPair, &rectPair);
				else rectPair = *stereoPair;
				
				Size2i subSize(rectPair.first.cols/subsampling, rectPair.first.rows/subsampling);
				if(dispMap.size() != subSize)
					dispMap = Mat_<float>(subSize);
				
				// Run stereo processing
				elasStereo(rectPair, dispMap);
				//sgbmStereo(rectPair, dispMap);
				
				frames++;
			}
			
			time_duration elapsed = (microsec_clock::local_time() - lastTime);
			if(performanceTest || elapsed.total_seconds() >= 1 || interactive) {
				ROS_INFO_STREAM(getName() << ": " << (frames/(elapsed.total_microseconds()/1.0e6)));
				lastTime = microsec_clock::local_time();
				frames = 0;
			}
			
			if(!noROS)
				publishDisparities(dispMap);
				
			timer.stop();
				
			// Visualize
			if(window.get() != NULL)
				visualizeMatches(rectPair, dispMap);
			currentFrame++;
		}
	}
	
	void DenseStereoNodelet::elasStereo(const StereoFrame8U::Type rectPair, cv::Mat_<float> output) {
		static cv::Mat_<float> rightDisp;
		if(rightDisp.size() != output.size())
			rightDisp = Mat_<float>(output.size());
	
		const int dims[3] = {rectPair.first.cols, rectPair.first.rows, rectPair.first.step[0]};
		elas->process(rectPair.first.data, rectPair.second.data,
			(float*)output.data, (float*)rightDisp.data, dims);
	}
	
	void DenseStereoNodelet::sgbmStereo(const StereoFrame8U::Type rectPair, cv::Mat_<float> output) {
		static cv::Mat_<short> intDisp;
		
		if(rectPair.first.size() != output.size())
			throw ks::Exception("Subsampling not supported for SGBM");
			
		if(intDisp.size() != output.size())
			intDisp = Mat_<short>(output.size());
	
		(*sgbm)(rectPair.first, rectPair.second, intDisp);
		
		for(int y = 0; y < intDisp.rows; y++)
			for(int x = 0; x < intDisp.cols; x++)
				output(y,x) = intDisp(y,x)/16.0;
	}

	bool DenseStereoNodelet::parseOptions() {
		bool error = false;
		interactive = false;
		capture = false;
		graphicalSDL = false;
		graphicalCV = false;
		maxDisp = 70;
		performanceTest = false;
		writePath = "";
		camera1 = 0; camera2 = 1;
		noROS = false;
		worldFrame = "/world";
		shared = false;
		rate = -1;
		subsampling = 1;
		
		char c;
		ArgvConverter argConv(getMyArgv());
		
		unique_lock<mutex> lock(argConv.getoptMutex);
		optind = 0;
		while ((c = getopt(argConv.argc, argConv.argv, "hr:o:IcC:gGm:w:Ri:W:Sf:s:")) != -1) {
			switch(c) {
				case 'r': calibFile = optarg; break;
				case 'o': videoFile = optarg; break;
				case 'I': interactive = true; break;
				case 'c': capture = true; break;
				case 'C': sscanf(optarg,  "%d,%d", &camera1, &camera2); break;
				case 'g': graphicalSDL = true; break;
				case 'G': graphicalCV = true; break;
				case 'm': maxDisp = atof(optarg); break;
				case 'w': writePath = optarg; break;
				case 'R': noROS = true; break;
				case 'i': inputDir = optarg; break;
				case 'W': worldFrame = optarg; break;
				case 'S': shared = true; break;
				case 'f': rate=atof(optarg); break;
				case 's': subsampling = atoi(optarg); break;
				case 'h':
				default: error = true; break;
			}
		}
		
		if(error || argConv.argc - optind != 0) {
			cerr << "Usage: " << endl
				 << argConv.argv[0] << " [OPTIONS]" << endl << endl
				 << "Options: " << endl
				 << "-h        Print this help message" << endl
				 << "-g        Graphical with SDL window" << endl
				 << "-G        Graphical with OpenCV window" << endl
				 << "-R        Disables ROS publishing" << endl
				 << "-r FILE   Use given calibration file for rectification" << endl
				 << "-o FILE   Output to video file" << endl
				 << "-p FILE   Output points to file" << endl
				 << "-i DIR    Replay the files from the given input directory" << endl
				 << "-c        Capture from camera" << endl
				 << "-C LIST   Comma separated list of camera indices" << endl
				 << "-I        Interactive" << endl
				 << "-m N      Set maximum disparity" << endl
				 << "-P        Run performance test" << endl
				 << "-w DIR    Write images to the given directory" << endl
				 << "-W FRAME  Sets the world frame (default: /world)" << endl
				 << "-S        Use shared memory for image transport" << endl
				 << "-f N      Apply frame rate limit" << endl
				 << "-s N      Set subsampling factor to N" << endl
				 << endl;
			return false;
		}
		
		return true;
	}
	
	// Reads the next stereo pair and stores it in a member variable
	bool DenseStereoNodelet::readStereoPair() {
		Size2i oldSize;
		if(stereoPair != NULL)
			oldSize = stereoPair->first.size();
	
		while(true) {
			stereoPair = imgQueue->pop();
			if(stereoPair == NULL)
				return false; // No more images
			
			// Hack to get rid of white line in rawseeds dataset
			memcpy(stereoPair->first.data, &stereoPair->first.data[stereoPair->first.step[0]], stereoPair->first.step[0]);
			memcpy(stereoPair->second.data, &stereoPair->second.data[stereoPair->second.step[0]], stereoPair->second.step[0]);

			if(rosInput)
				messageTime = ((StereoRosQueue8U*)imgQueue.get())->getLastMsgTime();
			else if(!noROS) messageTime = Time::now();
				
			if(stereoPair->first.data == NULL || stereoPair->second.data == NULL) {
				// An incomplete image. Lets keep on producing notifications
				//publishMatches(vector<SparseMatch>());
			} else break;
		}
		
		if(stereoPair->first.size() != stereoPair->second.size())
			throw ks::Exception("Image sizes don't match");
				
		if(oldSize != stereoPair->first.size()) {
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
	void DenseStereoNodelet::visualizeMatches(const StereoFrame8U::Type& pair, const Mat_<float>& matches) {
		Mat_<Vec3b> colImg(matches.size());
               // RAJ color disparities better color code
	       //colCoder->codeImage(matches, colImg);
               cv::Mat tmp;

		double min;
		double max;
		cv::minMaxIdx(matches, &min, &max);
		cv::Mat adjMap;
		cv::convertScaleAbs(matches, tmp, 1);

               matches.convertTo(tmp, CV_8U, 255/(maxDisp));
               applyColorMap(tmp, colImg,  cv::COLORMAP_JET); //cv::COLORMAP_AUTUMN);

	
		Mat_<Vec3b> imageArea = screen(Rect(0, 0, pair.first.cols, pair.first.rows));
		int scalingFactor = imageArea.cols/colImg.cols;
		for(int y=0; y<imageArea.rows; y++)
			for(int x=0; x<imageArea.cols; x++)
				imageArea(y,x) = colImg(y/scalingFactor, x/scalingFactor);
	
		window->displayStereoPair(std::pair<Mat, Mat>(screen, pair.first));
		if(interactive)
			window->waitForKey();
		window->processEvents(false);
		
		if(video || writePath != "") {
			if(video)
				(*video) << screen;
		
			if(writePath != "") {
				char fileName[256];
				snprintf(fileName, sizeof(fileName), "%s/image%04d.png", writePath.c_str(), currentFrame);
				imwrite(fileName, /*screen*/imageArea);
				
				// DEBUG: print input too
				//snprintf(fileName, sizeof(fileName), "%s/input%04d.png", writePath.c_str(), currentFrame);
				//imwrite(fileName, pair.first);
			}
		}
	}
	
	void DenseStereoNodelet::publishDisparities(const Mat_<float>& disparityMap) {
		std_msgs::Header header;
		header.stamp = messageTime;
		header.frame_id = worldFrame;
		header.seq = currentFrame;
		
		stereoPublisher->publish(header, disparityMap);
	}
}
