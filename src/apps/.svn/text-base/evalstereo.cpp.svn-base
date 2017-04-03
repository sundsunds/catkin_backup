#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <list>

#include "libks/base/sdlwindow.h"
#include "libks/imageio/filequeue.h"
#include "libks/stereo/sparse/sparsestereo.h"
#include "libks/feature/evaluation/clusterevaluation.h"

#define DEFAULT_EPSILON 1.0

using namespace std;
using namespace cv;
using namespace ks;
using namespace boost;

class EvalSparseStereo {
public:
	EvalSparseStereo(): frame(-1) {}

	int run(int argc, char** argv) {
		if(!parseOptions(argc, argv))
			return 1;
		
		if(!average)
			cout << "# Frame, percentage of bad pixels" << endl;	
		
		double sumBPP = 0, sumCluster = 0;
		unsigned int clusterCount = 0;
		
		while((truthFrame = truthQueue->pop()) != NULL) {
			if((sparse && !readNextSparse()) || (!sparse && !readNextDense())) {
				cerr << "Unexpected end of disparity data!" << endl;
				return 1;
			}
				
			double bpp = evalBadPixelPercentage();
			double cluster = evalCluster();
			
			if(!average)
				cout << frame << " " << bpp << " " << cluster << endl;
			else {
				sumBPP+=bpp;
				if(cluster >= 0) {
					sumCluster+=cluster;
					clusterCount++;
				}
			}
			
			frame++;
		}
		
		if(average) {
			cout << sumBPP/(frame+1) << " " << sumCluster/clusterCount << endl;
		}
		
		return 0;
	}

private:
	double epsilon;
	bool average;
	bool sparse;
	
	fstream pointsStrm;
	int frame;
	list<KeyPoint> keyPoints;
	vector<SparseMatch> matches;
	scoped_ptr<MonoRawFileQueue> truthQueue;
	MonoFrame32F::ConstPtr truthFrame;
	scoped_ptr<MonoImageQueue8U> denseQueue;

	// Parses all program options
	bool parseOptions(int argc, char** argv) {
		average = false;
		sparse = false;
		epsilon = DEFAULT_EPSILON;
		bool dense = false;
		
		char c;
		bool fail = false;
		while ((c = getopt(argc, argv, "ae:dp")) != -1) {
			switch(c) {
				case 'a': average = true; break;
				case 'e': epsilon = atof(optarg); break;
				case 'd': dense = true; break;
				case 'p': sparse = true; break;
				default: fail = true;
			}
		}
		
		if(fail || argc - optind != 2 || (dense && sparse) || (!dense && !sparse)) {
			cerr << "Usage: " << endl
				 << argv[0] << " [OPTIONS] GROUND-TRUTH-DIR INPUT-FILE/DIR" << endl << endl
				 << "Options: " << endl
				 << "-a      Calculate the average measure for the entire sequence" << endl
				 << "-e N    Sets Epsilon to N" << endl
				 << "-p      Input is a sparse points file" << endl
				 << "-d      Input is a directory with dense disparity map" << endl;
			return false;
		}
		
		truthQueue.reset(new MonoRawFileQueue(argv[argc-2]));
		if(sparse)
			pointsStrm.open(argv[argc-1], ios::in);
		else denseQueue.reset(new MonoFileQueue8U(argv[argc-1]));
		return true;
	}

	// Reads all points for the current frame from a sparse points file
	bool readNextSparse() {
		string line;
		matches.clear();
		keyPoints.clear();
		
		do {
			getline(pointsStrm, line);
			
			if(pointsStrm.eof())
				return false; //We reached the end
			else if(pointsStrm.fail()) {
				cerr << "Error reading input file" << endl;
				return false;
			}
		}
		while(line.length() != 0 && line[0] == '#');
		
		int matchesCount, readOffset;
		const char* parsePos = line.c_str();
		
		sscanf(parsePos, "%d %d%n", &frame, &matchesCount, &readOffset);
		parsePos += readOffset;
		
		for(int i=0; i<matchesCount; i++) {
			float rectX, rectY, imgX, imgY, disp;
			sscanf(parsePos, " %f %f %f %f %f%n", &rectX, &rectY, &imgX, &imgY, &disp, &readOffset);
			parsePos += readOffset;
			keyPoints.push_back(KeyPoint(Point2f(imgX, imgY), 0));
			matches.push_back(SparseMatch(&keyPoints.back(), NULL, Point2f(rectX, rectY), Point2f(rectX-disp, rectY)));
		}
		
		return true;
	}
	
	// Reads all points for the current frame from a dense disparity map
	bool readNextDense() {
		matches.clear();
		keyPoints.clear();
	
		MonoFrame8U::ConstPtr dispMap = denseQueue->pop();
		if(dispMap == NULL)
			return false;
	
		for(int y=0; y<dispMap->rows; y++)
			for(int x=0; x<dispMap->cols; x++)
				if((*dispMap)(y,x) != 255) {
					keyPoints.push_back(KeyPoint(Point2f(x, y), 0));
					matches.push_back(SparseMatch(&keyPoints.back(), NULL, Point2f(x, y), Point2f(x-(*dispMap)(y,x), y)));
				}
				
		frame++;		
		return true;
	}
	
	// Evaluates the percentage of bad pixels for the current frame
	double evalBadPixelPercentage() {
		unsigned int badPixels = 0;
		for(unsigned int i=0; i<matches.size(); i++) {
			double truthDisp = (*truthFrame)(int(matches[i].imgLeft->pt.y+0.5), int(matches[i].imgLeft->pt.x+0.5));
			if(fabs(truthDisp - matches[i].disparity()) > epsilon)
				badPixels++;
		}
		
		return badPixels / double(matches.size());
	}
	
	// Performs a clustering evaluation
	double evalCluster() {
		ClusterEvaluation clusterEval;
		vector<Point2f> pointsVec(matches.size());
		for(unsigned int i=0; i<matches.size(); i++)
			pointsVec[i] = matches[i].imgLeft->pt;
		return clusterEval.evaluate(pointsVec, Rect(0, 0, truthFrame->cols, truthFrame->rows));
	}
};


int main(int argc, char** argv) {
	EvalSparseStereo eval;
	return eval.run(argc, argv);
}
