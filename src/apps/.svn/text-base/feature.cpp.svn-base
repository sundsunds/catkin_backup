#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/smart_ptr.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/timer.hpp>
#include <fstream>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/fast_corner.h>
#include "libks/imageio/filequeue.h"
#include "libks/imageio/flycapqueue.h"
#include "libks/base/sdlwindow.h"
#include "libks/feature/fakefast.h"
#include "libks/feature/ssefast.h"
#include "libks/feature/faster.h"
#include "libks/feature/extendedfast.h"
#include "libks/feature/extendedfast2.h"
#include "libks/feature/extendedfast3.h"
#include "libks/feature/harris.h"
#include "libks/feature/featurereducer.h"
#include "libks/imageproc/imageconversion.h"

using namespace ks;
using namespace std;
using namespace boost;
using namespace boost::posix_time;
using namespace cv;
using namespace CVD;

class Feature {
public:
    Feature(): frame(0) {}

	int run(int argc, char** argv) {
		if(!parseOptions(argc, argv))
			return 1;
			
		SDLWindow window(640, 480, "Features");

		scoped_ptr<MonoImageQueue8U> imgQueue;
		if(capture)
			imgQueue.reset(new MonoFlyCapQueue8U());
		else imgQueue.reset(new MonoFileQueue8U(inputDir.c_str()));

		Mat_<Vec3b> screen;
		Mat_<char> charImg;
		int frames = 0;
		int featuresCount = 0;
		ptime lastTime;
		double totalElapsed = 0;
		unsigned int totalFeatures = 0;
		
		int repetitions = performanceTest ? 10 : 1;
		
		SSEFast fast;
		
		while(true) {
			// Load input images
			MonoFrame8U::ConstPtr left = imgQueue->pop();
			
			if(left == NULL)
				break; // No more images
			
			if(screen.size() != left->size()) {
				screen = Mat_<Vec3b>(left->size());
				charImg = Mat_<char>(left->size());
				//window.resize(screen.size());
			}
			cvtColor(*left, screen, CV_GRAY2BGR);
			
			vector<KeyPoint> keypoints;
			//Mat_<unsigned char> edgeMap(left->size(), 0);
			lastTime = microsec_clock::local_time();
			
			FeatureReducer reducer(500, left->cols, left->rows);
			
			// Convert to cvd sub-image, no data is copied
			BasicImage<byte> cvdImg(left->data, ImageRef(left->cols, left->rows));
			if((int)left->step[0] != left->cols)
				throw ks::Exception("Padding not supported by CVD");
			// Convert to cvd basic image, no data is copied
			vector<ImageRef> cvdCorners;
			
			for(int i=0; i<repetitions; i++) {
				keypoints.clear();
				ImageConversion::unsignedToSigned(*left, &charImg);
				
				//FAST(*left, keypoints, algorithmParam > 0 ? algorithmParam : 20, true);
				//FASTER(*left, keypoints, algorithmParam > 0 ? algorithmParam : 20);
				
				ExtendedFAST exFast(true, 10, algorithmParam > 0 ? algorithmParam : 1.2, false);
				exFast.detect(*left, keypoints);
				
				//ExtendedFAST2 exFast(true, 10, algorithmParam > 0 ? algorithmParam : 1.0, false);
				//((FeatureDetector*)&exFast)->detect(charImg, keypoints);
				
				//ExtendedFAST3 exFast(true, 10, algorithmParam > 0 ? algorithmParam : 1.0, false);
				//((FeatureDetector*)&exFast)->detect(*left, keypoints);
								
				/*cvdCorners.clear();
				//fast_corner_detect_9_nonmax(cvdImg, cvdCorners, algorithmParam > 0 ? algorithmParam : 20);
				fast_corner_detect_9(cvdImg, cvdCorners, algorithmParam > 0 ? algorithmParam : 20);
				featuresCount+=cvdCorners.size();*/
				
				/*StarDetector detector; //Use presets
				detector(*left, keypoints);*/
				/*SURF detector;
				detector(*left, Mat(), keypoints);*/
												
				//fast.detect(charImg, algorithmParam > 0 ? algorithmParam : 20, &keypoints, false);
				
				//GoodFeaturesToTrackDetector shito(10000, algorithmParam > 0 ? algorithmParam: 0.001, 0, 3, false);
				//shito.detect(*left, keypoints);
				
				//Harris harris(algorithmParam > 0 ? algorithmParam: 0.00005, true);
				//harris.detect(*left, keypoints);
				
				/*vector<KeyPoint> reduced;
				reducer.reduce(keypoints, &reduced);
				keypoints = reduced;*/
				
				frames++; frame++;
				featuresCount+=keypoints.size();
			}
			
			
			if(performanceTest) {
				double elapsed = (microsec_clock::local_time() - lastTime).total_microseconds()/1.0e6;
				totalElapsed += elapsed;
				//cout << "Fps: " << frames/elapsed
				//	<< "; Avg. Features: " << featuresCount/float(frames) << endl;
				totalFeatures += featuresCount;
				frames = 0;
			}
			featuresCount = 0;
			
			vector<Point2f> points;
			if(keypoints.size() != 0) {
				// Convert opencv keypoints 
				KeyPoint::convert(keypoints, points);
				writePoints(points);
			} else {
				// Convert cvd points
				points.reserve(cvdCorners.size());
				for(unsigned int i=0; i<cvdCorners.size(); i++)
					points.push_back(Point2f(cvdCorners[i].x, cvdCorners[i].y));
			}
			
			//goodFeaturesToTrack(*left, points, 1000, 0.01, 5, Mat(), 3, false);
			
			// Visualize
			for(unsigned int i=0; i<points.size(); i++)
				rectangle(screen, Point2f(points[i].x-2, points[i].y-2),
					Point2f(points[i].x+2, points[i].y+2), (Scalar) Vec3b(0, 0, 255), CV_FILLED);
			for(unsigned int i=0; i<points.size(); i++)
				rectangle(screen, Point2f(points[i].x-1, points[i].y-1),
					Point2f(points[i].x+1, points[i].y+1), (Scalar) Vec3b(0, 255, 255), CV_FILLED);
				//screen(points[i].y, points[i].x) = Vec3b(0, 255, 255);
			
			if(writePath != "") {
				char fileName[256];
				snprintf(fileName, sizeof(fileName), "%s/image%04d.png", writePath.c_str(), frame);
				imwrite(fileName, screen);
			}
			
			window.displayImage(screen);
			if(interactive)
			    window.waitForKey();
			else window.processEvents(false);
		}	
		
		
		if(performanceTest)
			cout << "Total Avg Time: " << totalElapsed / frame <<   
				"; Total Avg Features: " << double(totalFeatures) / frame << endl;
		
		return 0;
	}

private:
	string inputDir;
	bool capture;
	bool interactive;
	string pointsFile;
	double algorithmParam;
	string writePath;
	bool performanceTest;
	
	int frame;
	fstream pointsFileStrm;
	
	bool parseOptions(int argc, char** argv) {
		capture = false;
		interactive = false;
		algorithmParam = -1.0;
		writePath = "";
		performanceTest = false;
		
		char c;
		while ((c = getopt(argc, argv, "cip:s:w:P")) != -1) {
			switch(c) {
				case 'P': performanceTest = true; break;
				case 'c': capture = true; break;
				case 'i': interactive = true; break;
				case 'p': pointsFile = optarg; break;
				case 's': algorithmParam = atof(optarg); break;
				case 'w': writePath = optarg; break;
				default: return false;
			}
		}
		
		if((capture && argc - optind != 0) || (!capture && argc - optind != 1)) {
			cerr << "Usage: " << endl
				 << argv[0] << " [OPTIONS] [INPUT-DIR]" << endl << endl
				 << "Options: " << endl
				 << "-c       Capture from camera" << endl
				 << "-i       Interactive" << endl
				 << "-p FILE  Output points to file" << endl
				 << "-s VAL   Sets primary feature algorithm parameter to given value" << endl
				 << "-w DIR   Write results to directory" << endl
				 << "-P       Starts performance test" << endl;
			return false;
		}
		
		if(!capture)
			inputDir = argv[argc-1];
		return true;
	}
	
	// Write matched points to file
	void writePoints(const vector<Point2f>& points) {
		if(pointsFile == "")
			return; // Not activated
		
		if(!pointsFileStrm.is_open()) {
			pointsFileStrm.open(pointsFile.c_str(), ios::out);
			pointsFileStrm << "# Frame; Points Count; [X; Y]*" << endl;
		}
		if(pointsFileStrm.fail())
			throw ks::Exception("Error writing points");
		
		pointsFileStrm << frame << " " << points.size();
		for(unsigned int i=0; i<points.size(); i++)
			pointsFileStrm  <<  " " << points[i].x << " " << points[i].y;
		pointsFileStrm << endl;
	}
};


int main(int argc, char** argv) {
	Feature feat;
	return feat.run(argc, argv);
}
