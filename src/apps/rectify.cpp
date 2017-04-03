#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/smart_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <unistd.h>

#include "libks/base/sdlwindow.h"
#include "libks/imageproc/colorcoder.h"
#include "libks/stereo/cameracalibration.h"
#include "libks/stereo/stereorectification.h"
#include "libks/imageio/filequeue.h"
#include "libks/imageio/flycapqueue.h"

using namespace ks;
using namespace cv;
using namespace std;
using namespace boost;
using namespace boost::posix_time;

class Rectification {
public:
	Rectification() : fileIndex(1) {}

	int run(int argc, char** argv) {
		if(!parseOptions(argc, argv))
			return 1;
		
		SDLWindow window(1280, 480, "Rectification");
		CalibrationResult res(calibFile.c_str());
		
		scoped_ptr<StereoImageQueue8U> imgQueue;
		if(capture)
			imgQueue.reset(new StereoFlyCapQueue8U(camera1, camera2));
		else imgQueue.reset(new StereoFileQueue8U(inputDir.c_str()));
		
		StereoRectification rect(res, cubic ? StereoRectification::Cubic : StereoRectification::Linear);
		StereoFrame8U::Type outputPair;
		
		while(true) {
			StereoFrame8U::ConstPtr imagePair = imgQueue->pop();
			if(imagePair == NULL)
				break;
			Size winSize(imagePair->first.size().width + imagePair->second.size().width,
				max(imagePair->first.size().height, imagePair->second.size().height));
			if(window.getSize() != winSize)
				window.resize(winSize);
			
			if(inverseGrid)
				outputPair = StereoFrame8U::Type(imagePair->first.clone(), imagePair->second.clone());
			else rect.rectifyStereoPair(*imagePair, &outputPair);
				
			pair<Mat_<Vec3b>, Mat_<Vec3b> > colPair;
			if(grid || inverseGrid || epilines) {
				drawGrid(rect, outputPair, &colPair);
				window.displayStereoPair(colPair);
			}		
			else window.displayStereoPair(outputPair);
			if(outputDir != "")
				writeStereoPair(colPair.first.data != NULL ? (pair<Mat,Mat>) colPair : (pair<Mat,Mat>)outputPair);
			
			if(performanceTest) {
				int frames = 0; 
				ptime lastTime = microsec_clock::local_time();
				int lastFrames = 0;
				while(window.getPressedKey() == '\0') {
					rect.rectifyStereoPair(*imagePair, &outputPair);
					frames++;
					
					time_duration elapsed = (microsec_clock::local_time() - lastTime);
					if(elapsed.total_seconds() > 1) {
						cout << "Fps: " << ((frames-lastFrames)/(elapsed.total_microseconds()/1.0e6)) << endl;
						lastFrames = frames;
						lastTime = microsec_clock::local_time();
					}
					window.processEvents(false);
				}
			}
			
			if(interactive)
				window.waitForKey();
			else window.processEvents(false);
		}
		
		return 0;
	}

private:
	int fileIndex;
	bool performanceTest;
	bool interactive;
	bool grid;
	bool epilines;
	bool inverseGrid;
	bool capture;
	bool cubic;
	string calibFile;
	string inputDir;
	string outputDir;
	int camera1, camera2;

	bool parseOptions(int argc, char** argv) {
		performanceTest = false;
		interactive = false;
		grid = false;
		inverseGrid = false;
		capture = false;
		outputDir = "";
		cubic = true;
		epilines = false;
		camera1 = 0; camera2 = 1;
		
		char c;
		while ((c = getopt(argc, argv, "Pignc::o:be")) != -1) {
			switch(c) {
				case 'P': performanceTest = true; break;
				case 'i': interactive = true; break;
				case 'g': grid = true; break;
				case 'n': inverseGrid = true; break;
				case 'e': epilines = true; break;
				case 'c':
					capture = true;
					if(optarg != NULL)
						sscanf(optarg,  "%d,%d", &camera1, &camera2);
					break;
				case 'o': outputDir = optarg; break;
				case 'b': cubic = true; break;
				default: return false;
			}
		}
		
		if((capture && argc - optind != 1) || (!capture && argc - optind != 2)) {
			cerr << "Usage: " << endl
				 << argv[0] << " [OPTIONS] CALIB-FILE [INPUT-DIR]" << endl << endl
				 << "Options: " << endl
				 << "-P        Run performance test" << endl
				 << "-i        Interactive" << endl
				 << "-g        Draw grid" << endl
				 << "-e        Draw epilines grid" << endl
				 << "-n        Draw inverse grid" << endl
				 << "-c [LIST] Capture from camera. Camera indices can be given separated by commas." << endl
				 << "-o DIR    Output images to directory" << endl
				 << "-b        Cubic interpolation" << endl;
			return false;
		}
		
		if(!capture) {
			inputDir = argv[argc-1];
			calibFile = argv[argc -2];
		} else calibFile = argv[argc -1];
		return true;
	}

	void drawGrid(const StereoRectification& rect, const pair<Mat_<unsigned char>, Mat_<unsigned char> >& input,
		pair<Mat_<Vec3b>, Mat_<Vec3b> >* output) {
		
		Epiline::setMaxEpilineLength(input.first.cols);
		
		cvtColor(input.first, output->first, CV_GRAY2BGR);
		cvtColor(input.second, output->second, CV_GRAY2BGR);
		ColorCoder colCoder(0, input.first.rows, false, false);

		double epilineError = 0;
		int count= 0;

		int ystart = 0, yend = input.first.rows;
		/*if(epilines) {
			ystart = rect.highPrecisionRectifyLeftPoint(Point2f(input.first.cols/2, 0)).y;
			yend = rect.highPrecisionRectifyLeftPoint(Point2f(input.first.cols/2, input.first.rows-1)).y;
		}*/

		for(int y=ystart; y<yend; y+=40) {
			Point2f lastL(-1,-1), lastR(-1,-1);
			Vec3b col = Vec3b(0, 0, 255);//colCoder.getColor((float)y);
			Epiline leftEpiline, rightEpiline;
			
			double offsetLeft = 0.0, offsetRight = 0.0;
			if(inverseGrid) {
				// Get epiline
				leftEpiline = rect.getLeftEpiline(Point2f(input.first.cols/2, y));
				rightEpiline = rect.getRightEpiline(Point2f(input.second.cols/2, y));
				Point2f hiLeft = rect.highPrecisionRectifyLeftPoint(Point2f(input.first.cols/2, y));
				Point2f hiRight = rect.highPrecisionRectifyRightPoint(Point2f(input.first.cols/2, y));
				//Point2f hiLeft = rect.rectifyLeftPoint(Point2f(input.first.cols/2, y));
				//Point2f hiRight = rect.rectifyRightPoint(Point2f(input.first.cols/2, y));
				offsetLeft = y - hiLeft.y; offsetRight = y - hiRight.y;
			}
			
			for(int x=1; x<input.first.cols-1; x++) {
				Point2f ptLeft, ptRight;
				if(inverseGrid) {
					// Project back only for estimating the accuracy
					ptLeft = Point2f(x, leftEpiline.at(x));
					ptRight = Point2f(x, rightEpiline.at(x));
					if(ptLeft.y > 0) {
						epilineError += fabs(rect.highPrecisionRectifyLeftPoint(ptLeft).y + offsetLeft - y);
						count++;
					} else ptLeft = Point2f(-1, -1);
					
					if(ptRight.y > 0) {
						epilineError += fabs(rect.highPrecisionRectifyRightPoint(ptRight).y + offsetRight - y);
						count++;
					} else ptRight = Point2f(-1, -1);
				}
				else if(epilines) {
					ptLeft = Point2i(x, rect.highPrecisionRectifyLeftPoint(Point2f(input.first.cols/2, y)).y);
					ptRight = Point2i(x, rect.highPrecisionRectifyRightPoint(Point2f(input.first.cols/2, y)).y);
				} else {
					// Get point through rectification
					ptLeft = rect.rectifyLeftPoint(Point2i(x, y));
					ptRight = rect.rectifyRightPoint(Point2i(x, y));
				}
					
				/*if(x % 40 == 0) {
					if(ptLeft != Point2f(-1, -1))
						rectangle(output->first, Point2f(ptLeft.x-3, ptLeft.y-3),
							Point2f(ptLeft.x+3, ptLeft.y+3), (Scalar)col, CV_FILLED);
					if(ptRight != Point2f(-1, -1))
						rectangle(output->second, Point2f(ptRight.x-3, ptRight.y-3),
							Point2f(ptRight.x+3, ptRight.y+3), (Scalar)col, CV_FILLED);
				}*/
				
				if(lastL.x >= 0 && ptLeft != Point2f(-1, -1))
					line(output->first, lastL, ptLeft, (Scalar)col, 2);
				if(lastR.x >= 0 && ptRight != Point2f(-1, -1))
					line(output->second, lastR, ptRight, (Scalar)col, 2);
					
				lastL = ptLeft; lastR = ptRight;
			}
		}
		
		if(inverseGrid)
			cout << "Epiline error: " << epilineError/count << endl;
	}
	
	void writeStereoPair(const pair<Mat, Mat>& stereoFrame) {
		char fileName1[17], fileName2[17];
		snprintf(fileName1, sizeof(fileName1), "image%04d_c0.png", fileIndex);
		snprintf(fileName2, sizeof(fileName2), "image%04d_c1.png", fileIndex++);
		
		bool success = imwrite(outputDir + "/" + fileName1, stereoFrame.first);
		success = success && imwrite(outputDir + "/" + fileName2, stereoFrame.second);
		if(!success)
			cerr << "Error writing file(s)!" << endl;
	}
};

int main(int argc, char** argv) {
	Rectification rect;
	return rect.run(argc, argv);
	
	return 0;
}
