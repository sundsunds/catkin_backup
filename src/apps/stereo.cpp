#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/timer.hpp>
/*#include "libks/stereo/dense/cvblockmatch-inl.h"
#include "libks/stereo/dense/cvsgm-inl.h"*/
#include "libks/stereo/dense/winnertakesall-inl.h"
#include "libks/stereo/correlation/censuswindow-inl.h"
#include "libks/stereo/correlation/weightedcensus.h"
#include "libks/feature/censusfeature.h"
#include "libks/imageio/filequeue.h"
#include "libks/base/sdlwindow.h"
#include "libks/imageproc/census-inl.h"
#include "libks/imageproc/noiserobustcensus.h"
#include "libks/imageproc/colorcoder.h"
#include <cstdlib>

using namespace ks;
using namespace std;
using namespace boost;
using namespace boost::posix_time;
using namespace cv;

float uniqueness = 0.7;
bool interactive = false;
string writeDir = "", dispDir = "";

bool parseOptions(int argc, char** argv) {

	char c;
	bool failed = false;
	
	while ((c = getopt(argc, argv, "iw:d:u:")) != -1) {
		switch(c) {
			case 'i': interactive = true; break;
			case 'u': uniqueness = atof(optarg); break;
			case 'w': writeDir = optarg; break;
			case 'd': dispDir = optarg; break;
			default:
				failed = true;
				break;
		}
	}
	
	if(argc - optind != 1 || failed) {
		cerr << "Usage: " << endl
			 << argv[0] << " [OPTIONS] INPUT-DIR" << endl << endl
			 << "Options: " << endl
			 << "-i      Interactive" << endl
			 << "-u X    Set uniqueness to X" << endl
			 << "-w DIR  Write color coded output to DIR" << endl
			 << "-d DIR  Write disparity data to DIR" << endl;
		return false;
	}
	else return true;
}

int main(int argc, char** argv) {
	if(!parseOptions(argc, argv))
		return 1;
	
	SDLWindow window(640, 480, "Disparities");

	const int maxDisp = 115;
	//CvBlockMatch<unsigned char> stereo(maxDisp);
	//CvSGM<unsigned char, unsigned char> stereo(0, maxDisp, 5);
	WinnerTakesAll<CensusWindow<5>, unsigned int> stereo(maxDisp, true, uniqueness);
	
	Mat_<Vec3b> screen;
	ColorCoder colCoder(0, maxDisp, true, true);
	StereoFileQueue8U imgQueue(argv[argc-1]);

	Mat_<unsigned char> disp;
	int frames = 0;
	timer time;
	ptime lastTime = microsec_clock::local_time();
	
	while(true) {
		// Load input images
		StereoFrame8U::ConstPtr stereoPair = imgQueue.pop();
		if(stereoPair == NULL)
			break; // No more images
		
		Mat_<unsigned char> left = stereoPair->first,
			right = stereoPair->second;
		
		if(disp.size() != left.size() || frames == 0) {
			disp = 	Mat_<unsigned char>(left.size());
			screen = colCoder.createLegendBorder(stereoPair->first.cols, stereoPair->first.rows);
			window.resize(screen.cols, screen.rows);
		}
		
		Mat_<unsigned int> cenLeft(left.rows, left.cols), cenRight(right.rows, right.cols);
		Census::transform5x5(left, &cenLeft);
		Census::transform5x5(right, &cenRight);
		
		//
		/*CensusFeature censusFeat;
		Mat_<unsigned char> weightLeft(cenLeft.rows, cenLeft.cols), weightRight(cenRight.rows, cenRight.cols);
		censusFeat.compute5x5(cenLeft, &weightLeft);
		censusFeat.compute5x5(cenRight, &weightRight);
		WeightedCensus<5> weight(1.0, weightLeft, weightRight);
		WinnerTakesAll<WeightedCensus<5>, unsigned int> stereo(maxDisp, true, uniqueness, weight);*/
		//
		stereo.match(cenLeft, cenRight, &disp);
		
		frames++;
		
		time_duration elapsed = (microsec_clock::local_time() - lastTime);
		if(elapsed.total_seconds() >= 1 || interactive) {
			cout << "Fps: " << (frames/(elapsed.total_microseconds()/1.0e6)) << endl;
			time.restart();	
		}
		
		Mat_<Vec3b> imgArea = screen(Rect(0, 0, stereoPair->first.cols, stereoPair->first.rows));
		colCoder.codeImage(disp, imgArea);
		window.displayImage(screen);
		
		char fileName[20];
		snprintf(fileName, sizeof(fileName), "image%04d.png", frames);
		
		if(writeDir != "")
			imwrite(writeDir + "/" + fileName, screen);
		if(dispDir != "")
			imwrite(dispDir + "/" + fileName, disp);
		
		window.processEvents(false);
		if(interactive)
			window.waitForKey();
	}	
	
	return 0;
}
