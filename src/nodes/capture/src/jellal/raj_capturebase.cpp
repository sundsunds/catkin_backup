#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <queue>
#include "libks/imageio/flycapqueue.h"
#include "libks_msgs/ImageSet.h"
#include "libks_msgs/SharedImageSet.h"
#include "libks/base/argvconverter.h"
#include "libks/imageio/sharedimagesetpublisher.h"
#include "libks/base/crashguard.h"
#include "capturebase.h"

namespace capture {
	using namespace ros;
	using namespace ks;
	using namespace std;
	using namespace boost;
	using namespace boost::posix_time;
	using namespace cv;

	CaptureBase::CaptureBase() {
		CrashGuard::setup();
	}
	
	void CaptureBase::onInit(int argc, char** argv, ros::NodeHandle* nh) {
		if(!parseOptions(argc, argv)) {
			ROS_FATAL("Error parsing program options");
			return;
		}
		
		if(nh != NULL)
			nodeHandle.reset(new NodeHandle(*nh));
		else if(nh == NULL && (!noROS || rosTopic)) {
			ros::init(argc, argv, ROS_PACKAGE_NAME);
			nodeHandle.reset(new NodeHandle());
			spinner.reset(new AsyncSpinner(1));
			spinner->start();
		}
		
		countDown = false;
        window.reset(graphical ? new SDLWindow(640, 480, "Capture") : NULL);
		
		// Initialize capture source
		if(doubleStereo) {
			if(rosTopic)
				doubleStereoQueue.reset(new DoubleStereoRosQueue8U(*nodeHandle, false, false, camera1, camera2, camera3, camera4));
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
				stereoQueue.reset(new StereoRosQueue8U(*nodeHandle, false, false, camera1, camera2));
			else if(replayDir != "")
				stereoQueue.reset( new StereoFileQueue8U(replayDir));
			else {
				stereoQueue.reset(new StereoFlyCapQueue8U(camera1, camera2));
				if(autoShutterMinDiff != 0)
					((StereoFlyCapQueue8U*)stereoQueue.get())->adjustAutoShutter(
						autoShutterMin, autoShutterMax, autoShutterMinDiff);
			}

           // publish camerainfo L R
           //nodeHandle->getParam("left_camera_filename", left_camera_filename);
           //nodeHandle->getParam("right_camera_filename", right_camera_filename);

           calibFile = "/localhome/ait_jellal/calibration_stereo/calibFileFront_17102015.xml";
           std::cout << "RAJ  Calibration file : " << calibFile << std::endl;

           left_camera_info_pub =  nodeHandle->advertise<sensor_msgs::CameraInfo>("/front_stereo/left/camera_info", 1);
           right_camera_info_pub = nodeHandle->advertise<sensor_msgs::CameraInfo>("/front_stereo/right/camera_info", 1);

           //fill the static part of the CameraInfo message
           fillCameraInfoMessage(calibFile, left_camera_info_msg , LEFT_CAM);
           fillCameraInfoMessage(calibFile, right_camera_info_msg, RIGHT_CAM);

           if(calibFile != "")
               rectification.reset(new StereoRectification(CalibrationResult(calibFile.c_str(), 0.0)));

        } else {
            left_camera_info_pub =  nodeHandle->advertise<sensor_msgs::CameraInfo>("/mono/camera_info", 1);
			if(rosTopic)
				monoQueue.reset(new MonoRosQueue8U(*nodeHandle, false, false, camera1));
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

		publisherDefault.reset(!noROS ? new Publisher(nodeHandle->advertise<libks_msgs::ImageSet>("capture", 0)) : NULL);

        ros::NodeHandle my_nh;
        if(stereo){
        publisherForLibVISO2_l.reset(!noROS ? new Publisher(nodeHandle->advertise<sensor_msgs::Image>(my_nh.resolveName("left_stereo_image"), 0)) : NULL);
        publisherForLibVISO2_r.reset(!noROS ? new Publisher(nodeHandle->advertise<sensor_msgs::Image>(my_nh.resolveName("right_stereo_image"), 0)) : NULL);

        publisherForLibVISO2_l_rect.reset(!noROS ? new Publisher(nodeHandle->advertise<sensor_msgs::Image>(my_nh.resolveName("/front_stereo/left/image_rect"), 0)) : NULL);
        publisherForLibVISO2_r_rect.reset(!noROS ? new Publisher(nodeHandle->advertise<sensor_msgs::Image>(my_nh.resolveName("/front_stereo/right/image_rect"), 0)) : NULL);
        }else if(!doubleStereo){
           // mono
         publisherForLibVISO2_l.reset(!noROS ? new Publisher(nodeHandle->advertise<sensor_msgs::Image>(my_nh.resolveName("/mono/image"), 0)) : NULL);

        }

        publisherShared.reset(!noROS ? new SharedImageSetPublisher(*nodeHandle, "capture_shared", 0) : NULL);
		
		// Start thread for writing frames
		writingThread = thread(bind(&CaptureBase::writeLoop, this));
		
		if(graphical)
            ROS_INFO("Press SPACE for single frame grabbing; ENTER for sequence grabbROS_Iing; C for count downs");
			
		playbackRate.reset(rate > 0 ? new Rate(rate) : NULL);
	}

	void CaptureBase::mainLoop() {
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
                sendVISORosMessage(frame, framesCount, stamp);
				sendDefaultRosMessage(frame, framesCount, stamp);
			}
			
			framesCount++;
			
			// Print status message
			time_duration elapsed = (microsec_clock::local_time() - lastTime);
			if(elapsed.total_seconds() > 1) {
				double fps = ((framesCount-lastFrames)/(elapsed.total_microseconds()/1.0e6));
				if(grab || !graphical || writeQueue.size() > 0)
					ROS_INFO("Fps: %.2f ; Grabbed frames: %d ; Queue size: %d", fps, fileIndex, (int)writeQueue.size());
				else if(!countDown) ROS_INFO("Fps: %.2f", fps); 
				lastFrames = framesCount;
				lastTime = microsec_clock::local_time();
			}

			// Get user input
			char key = '\0';
			if(graphical)
				key = window->getPressedKey();
			if(key == '\r') {
				if(!grab)
					ROS_INFO("Starting grabbing...");
				else ROS_INFO("Stopping grabbing...");
				grab = !grab;
			} else if(key =='c' || key =='C') {
				countDown = !countDown;
				countDownStart = microsec_clock::local_time();
				if(countDown)
					ROS_INFO("Starting Count Down! /localhome/ait_jellal/calib_imgs/image **RAJ**");
				else ROS_INFO("Count Down Stopped!");
			}
			
			time_duration countDownElapsed = (microsec_clock::local_time() - countDownStart);
			if(countDown && countDownElapsed.total_seconds() > lastCountDown) {
				lastCountDown = countDownElapsed.total_seconds();
				cout << (1 - lastCountDown) << endl; //3raj
				if(lastCountDown == 1) { //3raj
					countDownStart = microsec_clock::local_time();
					lastCountDown = -1;
				}
			}
			
			// Write frames
			if(key == ' ' || (countDown && lastCountDown == -1))
				ROS_INFO("Writing frame %d", fileIndex);
			if(key == ' ' || grab || grabFiles || (countDown && lastCountDown == -1)) {
				scheduleWrite(frame, fileIndex);
				fileIndex++;
			}
			if(graphical)
				window->processEvents(false);
		}
	}

	bool CaptureBase::parseOptions(int argc, char** argv) {
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
				
		char c;
		optind = 0;
		while ((c = getopt(argc, argv, "sdC:pgi:Rwtr:a:AH")) != -1) {
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
		
		if(argc - optind != 0 || error) {
			cerr << "Usage: " << endl
				 << argv[0] << " [OPTIONS]" << endl << endl
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
		
	void CaptureBase::writeLoop() {
        ROS_WARN("Store pictures in /localhome/ait_jellal/dataset");
		while(true) {
			pair<string, Mat_<unsigned char> > frame;
			{
				unique_lock<mutex> lock(writingMutex);
				if(writeQueue.size() == 0)
					writingCond.wait(lock);
				frame = writeQueue.front();
				writeQueue.pop();
			}
                        // /localhome/ait_jellal/dataset
			if(!imwrite("/localhome/ait_jellal/dataset/"+frame.first, frame.second))
				ROS_ERROR("Error writing file: %s", frame.first.c_str());
		}
	}
		
	bool CaptureBase::captureNextFrame(Frame* dst) {
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
		
	void CaptureBase::displayFrame(const Frame& frame) {
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
		
	void CaptureBase::scheduleWrite(const Frame& frame, int fileIndex) {
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
		
	void CaptureBase::sendDefaultRosMessage(const Frame& frame, unsigned int frameNum, ros::Time stamp) {
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
	
	void CaptureBase::sendSharedRosMessage(const Frame& frame, unsigned int frameNum, ros::Time stamp) {
	
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

    void CaptureBase::sendVISORosMessage(const Frame& frame, unsigned int frameNum, ros::Time stamp) {
        //if(publisherForLibVISO2_l->getNumSubscribers() == 0)
        //    return; // No need to do any effort if nobody's listening

        if(doubleStereo) {
            //
            ROS_INFO("RAJ sendVISORosMessage publish doubleStereo not yet implemented ");
        } else if(stereo) {
            sensor_msgs::ImagePtr msg_l = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame.stereo->first).toImageMsg();
            sensor_msgs::ImagePtr msg_r = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame.stereo->second).toImageMsg();

            msg_l->header.seq = frameNum;
            msg_l->header.stamp = stamp;
            msg_l->header.frame_id = "multi_camera_capture";

            msg_r->header.seq = frameNum;
            msg_r->header.stamp = stamp;
            msg_r->header.frame_id = "multi_camera_capture";

            //convertCVImage(frame.doubleStereo->first.first, &msg->images[0], frameNum, stamp);
            publisherForLibVISO2_l->publish(msg_l);
            publisherForLibVISO2_r->publish(msg_r);

            bool publish_rectified_imgs =true;
            if(publish_rectified_imgs)
            {
              ks::StereoFrame8U::Type rectPair;
              if(rectification != NULL){
            //   rectification->rectifyStereoPair(std::pair<frame.stereo->first,frame.stereo->second >, &rectPair);
              rectification->rectifyLeftImage((frame.stereo->first),   &(rectPair.first) );
              rectification->rectifyRightImage((frame.stereo->second), &(rectPair.second) ) ;

             }//  else  rectPair = frame.stereo; //*stereoPair;

              sensor_msgs::ImagePtr msg_l_rect = cv_bridge::CvImage(std_msgs::Header(), "mono8", rectPair.first).toImageMsg();
              sensor_msgs::ImagePtr msg_r_rect = cv_bridge::CvImage(std_msgs::Header(), "mono8", rectPair.second).toImageMsg();
              /*
              //rectPair.first
              Size sz(rectPair.first.size().width*2, rectPair.first.size().height);
              //printf(" size rect %d , %d ", rectPair.first.size().width , rectPair.first.size().height);
              if(sz != window->getSize())
                          window->resize(sz);
              window->displayStereoPair(rectPair);
              */

              msg_l_rect->header.seq = frameNum;
              msg_l_rect->header.stamp = stamp;
              msg_l_rect->header.frame_id = "base_link";

              msg_r_rect->header.seq = frameNum;
              msg_r_rect->header.stamp = stamp;
              msg_r_rect->header.frame_id = "base_link";
              publisherForLibVISO2_l_rect->publish(msg_l_rect);
              publisherForLibVISO2_r_rect->publish(msg_r_rect);

            }

            left_camera_info_msg.header.seq =frameNum ;
            left_camera_info_msg.header.stamp = stamp ;
            right_camera_info_msg.header.seq  =frameNum ;
            right_camera_info_msg.header.stamp = stamp;

            left_camera_info_pub.publish(left_camera_info_msg);
            right_camera_info_pub.publish(right_camera_info_msg);
            //stereo_camera_info_pub.publish(stereo_camera_info);

        } else {
            //
            sensor_msgs::ImagePtr msg_l = cv_bridge::CvImage(std_msgs::Header(), "mono8", *frame.mono).toImageMsg();

            msg_l->header.seq = frameNum;
            msg_l->header.stamp = stamp;
            msg_l->header.frame_id = "multi_camera_capture";

            //convertCVImage(frame.doubleStereo->first.first, &msg->images[0], frameNum, stamp);
            publisherForLibVISO2_l->publish(msg_l);
            bool pub_info = true;
            if(pub_info){
              left_camera_info_msg.header.seq =frameNum ;
              left_camera_info_msg.header.stamp = stamp ;
      
              left_camera_info_pub.publish(left_camera_info_msg);
           }
        }


    }

	
	// Converts an OpenCV image to a  ROS image
	void CaptureBase::convertCVImage(const Mat_<unsigned char>& in, sensor_msgs::Image* out,
		unsigned int frameNum, const Time& rosTime) {
		cv_bridge::CvImage img;
		img.encoding = "mono8";
		img.header.seq = frameNum;
		img.header.stamp = rosTime;
		img.header.frame_id = "camera";
		img.image = in;
		
		img.toImageMsg(*out);
	}

    // raj

    //fill CameraInfo message from an opencv xml calibration file
    // int which_camera  Left or Right (stereo)
    int CaptureBase::fillCameraInfoMessage(const std::string & fileName, sensor_msgs::CameraInfo &msg, int which_camera)
    {
          int ii;
          cv::Mat KK(3,3,CV_64FC1);
          cv::Mat DD(5,1,CV_64FC1);
          cv::Mat SZ(2,1, CV_16UC1);

          cv::FileStorage fs;

          //open file
          fs.open(fileName, cv::FileStorage::READ);
          if ( !fs.isOpened() )
          {
                std::cout << "WARNING: Camera Calibration File " << fileName << " Not Found." << std::endl;
                std:: cout << "WARNING: camera_info topic will not provide right data" << std::endl;
                return -1;
          }

          //fill static part of the message
          msg.header.frame_id = "/camera"; //"raj_stereo_camera";
          msg.binning_x = 0;
          msg.binning_x = 0;
          msg.roi.width = 0;
          msg.roi.height = 0;
          for (ii=0; ii<9; ii++) msg.R[ii] = 0;

          fs["size"] >> SZ;
          msg.height =  SZ.at<int>(0);
          msg.width  =  SZ.at<int>(1);

          //msg.width = (int)fs["image_width"];
          //msg.height = (int)fs["image_height"];
          if(which_camera == LEFT_CAM){
             fs["M1"] >> KK;
          }else if(which_camera ==RIGHT_CAM)
          {
             fs["M2"] >> KK;
          }
          else{std:: cout << "WARNING: which camera ? is it left or right" << std::endl;
              return -1;
          }
          for (ii=0; ii<9; ii++) msg.K[ii] = KK.at<double>(ii);

          msg.distortion_model = "plumb_bob";


          if(which_camera == LEFT_CAM){
              fs["D1"] >> DD;
          }else if(which_camera == RIGHT_CAM)
          {
              fs["D2"] >> DD;
          }
          for (ii=0; ii<5; ii++) msg.D.push_back(DD.at<double>(ii));

          //close file and return
          fs.release();
          return 0;
    }
 }

