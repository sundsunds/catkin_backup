#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <queue>
#include <pluginlib/class_list_macros.h>
#include "libks/imageio/flycapqueue.h"
#include "libks_msgs/ImageSet.h"
#include "libks_msgs/SharedImageSet.h"
#include "libks/base/argvconverter.h"
#include "libks/imageio/sharedimagesetpublisher.h"
#include "libks/base/crashguard.h"
#include "capturenodelet.h"

PLUGINLIB_DECLARE_CLASS(capture, CaptureNodelet, capture::CaptureNodelet, nodelet::Nodelet)

namespace capture {
	using namespace ros;
	using namespace ks;
	using namespace std;
	using namespace boost;
	using namespace boost::posix_time;
	using namespace cv;

	CaptureNodelet::CaptureNodelet() {
		CrashGuard::setup();
	}
	
	void CaptureNodelet::onInit() {
		if(!parseOptions()) {
			NODELET_FATAL("Error parsing program options");
			return;
		}
		
		countDown = false;
		window.reset(graphical ? new SDLWindow(640, 480, "Capture") : NULL);
		
		// Initialize capture source
		if(doubleStereo) {
			if(rosTopic)
				doubleStereoQueue.reset(new DoubleStereoRosQueue8U(getNodeHandle(), false, false, camera1, camera2, camera3, camera4));
			else if(replayDir !="")
				doubleStereoQueue.reset(new DoubleStereoFileQueue8U(replayDir));
			else {
				doubleStereoQueue.reset(new DoubleStereoFlyCapQueue8U(alternating, secondHalfRate,
					camera1, camera2, camera3, camera4));
				if(autoShutterMinDiff != 0)
					((DoubleStereoFlyCapQueue8U*)doubleStereoQueue.get())->adjustAutoShutter(
					autoShutterMin, autoShutterMax, autoShutterMinDiff);
			}
		}else if(stereo) {
			if(rosTopic)
				stereoQueue.reset(new StereoRosQueue8U(getNodeHandle(), false, false, camera1, camera2));
			else if(replayDir != "")
				stereoQueue.reset( new StereoFileQueue8U(replayDir));
			else {
				stereoQueue.reset(new StereoFlyCapQueue8U(camera1, camera2));
				if(autoShutterMinDiff != 0)
					((StereoFlyCapQueue8U*)stereoQueue.get())->adjustAutoShutter(
						autoShutterMin, autoShutterMax, autoShutterMinDiff);
			}
		} else {
			if(rosTopic)
				monoQueue.reset(new MonoRosQueue8U(getNodeHandle(), false, false, camera1));
			else if(replayDir != "")
				monoQueue.reset(new MonoFileQueue8U(replayDir));
			else {
				monoQueue.reset(new MonoFlyCapQueue8U(camera1, false));
				if(autoShutterMinDiff != 0) {
					((MonoFlyCapQueue8U*)monoQueue.get())->adjustAutoShutter(
						autoShutterMin, autoShutterMax, autoShutterMinDiff);
				}
			}
		}
		
		// Publish ROS topics
		publisherDefault.reset(!noROS ? new Publisher(getNodeHandle().advertise<libks_msgs::ImageSet>("capture", 0)) : NULL);
		publisherShared.reset(!noROS ? new SharedImageSetPublisher(getNodeHandle(), "capture_shared", 0) : NULL);
		
		// Start thread for writing frames
		writingThread = thread(bind(&CaptureNodelet::writeLoop, this));
		
		if(graphical)
			NODELET_INFO("Press SPACE for single frame grabbing; ENTER for sequence grabbing; C for count downs");
			
		playbackRate.reset(rate > 0 ? new Rate(rate) : NULL);
		mainThread = thread(bind(&CaptureNodelet::mainLoop, this));
	}

	void CaptureNodelet::mainLoop() {
		unsigned int framesCount = 0, fileIndex = 0;
		ptime countDownStart, lastTime = microsec_clock::local_time();
		int lastCountDown = -1;
		int lastFrames = 0;
		bool grab=false;	
		Frame frame;
		
		while(captureNextFrame(&frame) && ros::ok()) {
			if(graphical)
				displayFrame(frame);
			if(!noROS) {
				Time stamp = Time::now();
				sendSharedRosMessage(frame, framesCount, stamp);
				sendDefaultRosMessage(frame, framesCount, stamp);
			}
			
			framesCount++;
			
			// Print status message
			time_duration elapsed = (microsec_clock::local_time() - lastTime);
			if(elapsed.total_seconds() > 1) {
				double fps = ((framesCount-lastFrames)/(elapsed.total_microseconds()/1.0e6));
				if(grab || !graphical || writeQueue.size() > 0)
					NODELET_INFO("Fps: %.2f ; Grabbed frames: %d ; Queue size: %d", fps, fileIndex, (int)writeQueue.size());
				else if(!countDown) NODELET_INFO("Fps: %.2f", fps); 
				lastFrames = framesCount;
				lastTime = microsec_clock::local_time();
			}

			// Get user input
			char key = '\0';
			if(graphical)
				key = window->getPressedKey();
			if(key == '\r') {
				if(!grab)
					NODELET_INFO("Starting grabbing...");
				else NODELET_INFO("Stopping grabbing...");
				grab = !grab;
			} else if(key =='c' || key =='C') {
				countDown = !countDown;
				countDownStart = microsec_clock::local_time();
				if(countDown)
					NODELET_INFO("Starting Count Down!");
				else NODELET_INFO("Count Down Stopped!");
			}
			
			time_duration countDownElapsed = (microsec_clock::local_time() - countDownStart);
			if(countDown && countDownElapsed.total_seconds() > lastCountDown) {
				lastCountDown = countDownElapsed.total_seconds();
				cout << (3 - lastCountDown) << endl;
				if(lastCountDown == 3) {
					countDownStart = microsec_clock::local_time();
					lastCountDown = -1;
				}
			}
			
			// Write frames
			if(key == ' ' || (countDown && lastCountDown == -1))
				NODELET_INFO("Writing frame %d", fileIndex);
			if(key == ' ' || grab || grabFiles || (countDown && lastCountDown == -1)) {
				scheduleWrite(frame, fileIndex);
				fileIndex++;
			}
			if(graphical)
				window->processEvents(false);
		}
	}

	bool CaptureNodelet::parseOptions() {
		stereo = false;
		doubleStereo = false;
		writePgm = false;
		camera1 = 0; camera2 = 1; camera3 = 2; camera4 = 3;
		grabFiles = false;
		graphical = false;
		noROS = false;
		rate = -1.0;
		rosTopic = false;
		alternating = false;
		secondHalfRate = false;
		autoShutterMin = autoShutterMax = autoShutterMinDiff = 0;
		bool error = false;
				
		ArgvConverter argConv(getMyArgv());
		char c;
		
		unique_lock<mutex> lock(argConv.getoptMutex);
		optind = 0;
		while ((c = getopt(argConv.argc, argConv.argv, "sdC:pgi:Rwtr:a:AH")) != -1) {
			switch(c) {
				case 's': stereo = true; break;
				case 'd': doubleStereo = true; break;
				case 'C': sscanf(optarg,  "%d,%d,%d,%d", &camera1, &camera2, &camera3, &camera4); break;
				case 'p': writePgm = true; break;
				case 'w': grabFiles = true; break;
				case 'g': graphical = true; break;
				case 'R': noROS = true; break;
				case 'i': replayDir = optarg; break;
				case 't': rosTopic = true; noROS=true; break;
				case 'r': rate=atof(optarg); break;
				case 'a': sscanf(optarg,"%f,%f,%f", &autoShutterMin,
					&autoShutterMax, &autoShutterMinDiff);
					break;
				case 'A': alternating = true; break;
				case 'H': secondHalfRate = true; break;
				default: error = true;
			}
		}
		
		if(argConv.argc - optind != 0 || error) {
			cerr << "Usage: " << endl
				 << argConv.argv[0] << " [OPTIONS]" << endl << endl
				 << "Options: " << endl
				 << "-g        Graphical" << endl
				 << "-R        Disables ROS publishing" << endl
				 << "-s        Stereo capture (default cameras: 0 and 1)" << endl
				 << "-d        Double stereo capture (default cameras: 0 - 3)" << endl
				 << "-C LIST   Use comma separated list of camera indices" << endl
				 << "-p        Write frames as pgm (default: png)" << endl
				 << "-w        Immediately starts writing captured frames to files" << endl
				 << "-i DIR    Replay the files from the given input directory" << endl
				 << "-t        Replay from ROS topic \"capture\"" << endl
				 << "-r N      Enforce given playback rate" << endl
				 << "-a M,X,D  Sets the auto shutter settings with minimum intensity M," << endl
				 << "          maximum intensity X, minimum difference D" << endl
				 << "-A        For double stereo, alternate the stereo pairs" << endl
				 << "-H        For double stereo, second pair is published at half the image rate" << endl
				 << "-h        Print this message" << endl
				 << endl;
			return false;
		}		
		
		return true;		
	}
		
	void CaptureNodelet::writeLoop() {
		
		while(true) {
			pair<string, Mat_<unsigned char> > frame;
			{
				unique_lock<mutex> lock(writingMutex);
				if(writeQueue.size() == 0)
					writingCond.wait(lock);
				frame = writeQueue.front();
				writeQueue.pop();
			}
            if(!imwrite("/home/sun/calibration_capture/"+frame.first, frame.second))
				NODELET_ERROR("Error writing file: %s", frame.first.c_str());
		}
	}
		
	bool CaptureNodelet::captureNextFrame(Frame* dst) {
		if(rate > 0)
			playbackRate->sleep(); // Slow down playback

		if(doubleStereo) {
			dst->doubleStereo = doubleStereoQueue->pop();
			if(dst->doubleStereo == NULL ||
				(dst->doubleStereo->first.first.data == NULL &&
				dst->doubleStereo->second.second.data == NULL))
				return false;
		} else if(stereo) {
			dst->stereo = stereoQueue->pop();
			if(dst->stereo == NULL ||
				dst->stereo->second.data == NULL)
				return false;
		} else {
			 dst->mono = monoQueue->pop();
			 if(dst->mono == NULL ||
			 	dst->mono->data == NULL)
				return false;
		}
		
		return true;
	}
		
	void CaptureNodelet::displayFrame(const Frame& frame) {
		if(doubleStereo) {
			Size sz(frame.doubleStereo->first.first.size().width*2, frame.doubleStereo->first.first.size().height*2);
			if(graphical && sz != window->getSize())
				window->resize(sz);
			window->displayDoubleStereoPair(*frame.doubleStereo);
		} else if(stereo) {
			Size sz(frame.stereo->first.size().width*2, frame.stereo->first.size().height);
			if(sz != window->getSize())
				window->resize(sz);
			window->displayStereoPair(*frame.stereo);
		}
		else {
			if(frame.mono->size() != window->getSize())
				window->resize(frame.mono->size());
			window->displayImage(*frame.mono);
		}
	}
		
	void CaptureNodelet::scheduleWrite(const Frame& frame, int fileIndex) {
		char fileNames[4][17];
		for(int i=0; i<4; i++) {
			unsigned int index = fileIndex;
			if(rosTopic) {
				if(doubleStereo)
					index = static_cast<DoubleStereoRosQueue8U*>(doubleStereoQueue.get())->getLastSequenceId();
				else if(stereo)
					index = static_cast<StereoRosQueue8U*>(stereoQueue.get())->getLastSequenceId();
				else index = static_cast<MonoRosQueue8U*>(monoQueue.get())->getLastSequenceId();
			}
			
            snprintf(fileNames[i], sizeof(fileNames[i]), "image%04d_c%d.%s", index, i, writePgm ? "pgm" : "png");
		}
		
		unique_lock<mutex> lock(writingMutex);
		if(doubleStereo) {
			writeQueue.push(pair<string, Mat_<unsigned char> >(fileNames[0], frame.doubleStereo->first.first));
			writeQueue.push(pair<string, Mat_<unsigned char> >(fileNames[1], frame.doubleStereo->first.second));
			writeQueue.push(pair<string, Mat_<unsigned char> >(fileNames[2], frame.doubleStereo->second.first));
			writeQueue.push(pair<string, Mat_<unsigned char> >(fileNames[3], frame.doubleStereo->second.second));
		} else if(stereo) {
			writeQueue.push(pair<string, Mat_<unsigned char> >(fileNames[0], frame.stereo->first));
			writeQueue.push(pair<string, Mat_<unsigned char> >(fileNames[1], frame.stereo->second));
		} else writeQueue.push(pair<string, Mat_<unsigned char> >(fileNames[0], *frame.mono));
		
		writingCond.notify_one();
	}
		
	void CaptureNodelet::sendDefaultRosMessage(const Frame& frame, unsigned int frameNum, ros::Time stamp) {
		if(publisherDefault->getNumSubscribers() == 0)
			return; // No need to do any effort if nobody's listening
	
		libks_msgs::ImageSetPtr msg(new libks_msgs::ImageSet);
		
		msg->header.seq = frameNum;
		msg->header.stamp = stamp;
		msg->header.frame_id = "multi_camera_capture";
		
		if(doubleStereo) {
			msg->images.resize(4);
			convertCVImage(frame.doubleStereo->first.first, &msg->images[0], frameNum, stamp);
			convertCVImage(frame.doubleStereo->first.second, &msg->images[1], frameNum, stamp);
			convertCVImage(frame.doubleStereo->second.first, &msg->images[2], frameNum, stamp);
			convertCVImage(frame.doubleStereo->second.second, &msg->images[3], frameNum, stamp);
		} else if(stereo) {
			msg->images.resize(2);
			convertCVImage(frame.stereo->first, &msg->images[0], frameNum, stamp);
			convertCVImage(frame.stereo->second, &msg->images[1], frameNum, stamp);
		} else {
			msg->images.resize(1);
			convertCVImage(*frame.mono, &msg->images[0], frameNum, stamp);
		}
		
		publisherDefault->publish(msg);
	}
	
	void CaptureNodelet::sendSharedRosMessage(const Frame& frame, unsigned int frameNum, ros::Time stamp) {
	
		libks_msgs::SharedImageSetPtr msg(new libks_msgs::SharedImageSet);
		
		std_msgs::Header header;
		header.seq = frameNum;
		header.stamp = stamp;
		header.frame_id = "multi_camera_capture";
		
		if(doubleStereo) {
			publisherShared->publish(header, frame.doubleStereo->first.first,
				frame.doubleStereo->first.second, frame.doubleStereo->second.first,
				frame.doubleStereo->second.second);
		} else if(stereo) {
			publisherShared->publish(header, frame.stereo->first, frame.stereo->second);
		} else {
			publisherShared->publish(header, *frame.mono);
		}
	}
	
	// Converts an OpenCV image to a  ROS image
	void CaptureNodelet::convertCVImage(const Mat_<unsigned char>& in, sensor_msgs::Image* out,
		unsigned int frameNum, const Time& rosTime) {
		cv_bridge::CvImage img;
		img.encoding = "mono8";
		img.header.seq = frameNum;
		img.header.stamp = rosTime;
		img.header.frame_id = "camera";
		img.image = in;
		
		img.toImageMsg(*out);
	}
}
