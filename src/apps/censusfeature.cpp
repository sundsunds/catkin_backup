#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/timer.hpp>
#include "libks/imageio/filequeue.h"
#include "libks/base/sdlwindow.h"
#include "libks/feature/censusfeature.h"
#include "libks/imageproc/census-inl.h"
//#include "libks/imageproc/noiserobustcensus.h"
#include "libks/imageproc/colorcoder.h"
#include <cstdlib>

using namespace ks;
using namespace std;
using namespace boost;
using namespace cv;

int main(int argc, char** argv) {
	SDLWindow window(640, 480, "Features");

	if(argc != 2) {
		cerr << "Usage: " << argv[0] << " INPUT-DIR" << endl;
		return 1;
	}

	StereoFileQueue8U imgQueue(argv[1]);

	Mat_<unsigned char> result;
	Mat_<Vec3b> screen;
	Mat_<unsigned int> censusImage;
	//Mat_<unsigned long long> censusImage;
	int frames = 0;
	timer time;
	int lastFrames = 0;
	
	CensusFeature censusFeat;
	int featuresCounter = 0;
	ColorCoder colCoder(0, 5, true, true);
	
	while(true) {
		// Load input images
		StereoFrame<unsigned char>::ConstPtr pair = imgQueue.pop();
		if(pair == NULL)
			break;
			
		Mat_<unsigned char> left = pair->first;
		
		if(result.size() != left.size()) {
			result = Mat_<unsigned char>(left.size(), (unsigned char)0);
			censusImage = Mat_<unsigned int>(left.size());
			screen = colCoder.createLegendBorder(left.cols, left.rows);
			window.resize(screen.size());
		}
		
		//NoiseRobustCensus::average5x5(&left, &censusImage, true);
		Census::transform5x5(left, &censusImage);
		//Census::transform9x7(left, &censusImage);
		
		//vector<Point2i> features;
		censusFeat.compute5x5(censusImage, &result);
		//censusFeat.compute9x7(censusImage, &result);
		
		// Visualize		
		colCoder.codeImage(result, screen);
		//normalize(result, left, 0, 255, NORM_MINMAX);
		imwrite("output.png", result);
		//cvtColor(left, screen, CV_GRAY2BGR);
		
		/*featuresCounter += features.size();
		for(unsigned int i=0; i<features.size(); i++) 
			rectangle(screen, Point2f(features[i].x-1, features[i].y-1),
				Point2f(features[i].x+1, features[i].y+1), (Scalar) Vec3b(0, 0, 255), CV_FILLED);*/
		
		frames++;
		
		usleep(30000);
		
		if(time.elapsed() > 1.0) {
			cout << "Fps: " << ((frames-lastFrames)/time.elapsed()) 
				<< "; Avg. Features " << ((double)featuresCounter/(frames-lastFrames)) << endl;
			lastFrames = frames;
			featuresCounter = 0;
			time.restart();
		}
		
		window.displayImage(screen);
		window.processEvents(false);
		window.waitForKey();
	}	
	
	return 0;
}
