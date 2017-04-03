#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <boost/smart_ptr.hpp>

#include "libks/base/sdlwindow.h"
#include "libks/stereo/cameracalibration.h"
#include "libks/imageio/filequeue.h"

using namespace ks;
using namespace cv;
using namespace std;

int main(int argc, char** argv) { 
	// Parse options
	bool interactive = false;
	bool error = false;
	
	char c;
	while ((c = getopt(argc, argv, "i")) != -1) {
		switch(c) {
			case 'i': interactive = true; break;
			default: error = true;
		}
	}
	
	if(argc - optind != 2 || error) {
		cerr << "Usage: " << endl
			 << argv[0] << "[OPTIONS] INPUT-DIR OUTPUT-FILE" << endl << endl
			 << "Options: " << endl
			 << "-i    Interactive" << endl
                         << "Please note that pattern size = 11x7 0.018" << endl;
		return 1;
	}
	
	// Initialize
	SDLWindow window(1280, 480, "Calibration");
	boost::scoped_ptr<StereoImageQueue8U> imgQueue(new StereoFileQueue8U(argv[argc-2]));
	
	// Default	
	//CameraCalibration calib(Size(9,8), 0.02, false);
        
    //raj
    //CameraCalibration calib(Size(11,9), 0.02, false);
    //sunds
    CameraCalibration calib(Size(11,7), 0.018, false);
    //CameraCalibration calib(Size(9,6), 0.01, false);
	// Auckland
	//CameraCalibration calib(Size(8,8), 0.09, true);
	
	// Rawseeds
	//CameraCalibration calib(Size(9,5), 0.1, false, 1);
	
	pair<Mat_<Vec3b>, Mat_<Vec3b> > outputPair;
	int frame = 0;
	
	while(true) {
		StereoFrame8U::ConstPtr imagePair = imgQueue->pop();
		if(imagePair == NULL) {
			break;
        }
		Size winSize(imagePair->first.size().width + imagePair->second.size().width,
			max(imagePair->first.size().height, imagePair->second.size().height));
		if(window.getSize() != winSize)
			window.resize(winSize);
		
		cout << "Frame: " << frame++ << endl;	
		calib.addCalibrationPair(*imagePair, &outputPair);
		window.displayStereoPair(outputPair);
		
		if(interactive)
			window.waitForKey();
		window.processEvents(false);
	}
	
	calib.calibrate().writeToFile(argv[argc-1]);
	
	return 0;
}
