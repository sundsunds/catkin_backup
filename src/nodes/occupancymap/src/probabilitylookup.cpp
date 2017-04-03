#include "probabilitylookup.h"
#include <cmath>
#include <vector>
#include <iostream>
#include <map>
#include <iomanip>
#include <fstream>

using namespace std;

ProbabilityLookup::ProbabilityLookup(double ocTreeResolution, double voxelResolution, double depthResolution,
	double maxDepth, double probMin, double probMax, double cutOffLimit, double depthErrorScale)
	:ocTreeResolution(ocTreeResolution), voxelResolution(voxelResolution),
	depthResolution(depthResolution), probMin(probMin) {
	
	vector<float> negativeRange, positiveRange;
	
	cout << "Constructing probability lookup table..." << endl;
	for(unsigned int zScaled = 1;; zScaled++) {
		double z = zScaled*depthResolution;
		if(z > maxDepth)
			break;
	
		double sigma = depthErrorScale*z*z;
		
		// Compute negative range
		negativeRange.clear();
		for(int i=0;; i--) {
			double prob = computeCDF(i*voxelResolution, 0, sigma);
			negativeRange.push_back((float)(prob * (probMax - probMin) + probMin));
			if(prob < cutOffLimit)
				break;
		}
		
		// Compute positive range
		positiveRange.clear();
		for(unsigned int i=1;i<negativeRange.size(); i++) {
			double prob = computeCDF(i*voxelResolution, 0, sigma);
			positiveRange.push_back((float)(prob * (probMax - probMin) + probMin));
		}
		
		lookupTable.push_back(vector<float>(negativeRange.size() + positiveRange.size()));
		copy(negativeRange.rbegin(), negativeRange.rend(), lookupTable.back().begin());
		copy(positiveRange.begin(), positiveRange.end(), lookupTable.back().begin() + negativeRange.size());
	}
	
    //fstream strm("/home/schauwecker/debug.csv", ios::out);
    fstream strm("/tmp/debug.csv", ios::out);
	for(unsigned int zScaled = 0; zScaled < lookupTable.size(); zScaled++)
		strm << (zScaled * depthResolution) << ";";
	strm << endl;
	
	for(unsigned int dist = 0;; dist++) {
	    bool written = false;
		for(unsigned int zScaled = 0; zScaled < lookupTable.size(); zScaled++) {
			if(dist < lookupTable[zScaled].size()) {
			    written = true;
				strm << lookupTable[zScaled][dist];
			} else strm << probMax;
			strm << ";";
		}
		strm << endl;
		if(!written)
		    break;
	}
	
	cout << "Lookup table construction finished." << endl;
}

double ProbabilityLookup::computeCDF(double x, double mean, double sigma) {
	return 0.5 * (1.0 + erf((x - mean) / (sigma * M_SQRT2)));
}

double ProbabilityLookup::computeRangeProbability(double mean, double sigma, double minRange, double maxRange) {
	return computeCDF(maxRange, mean, sigma) - computeCDF(minRange, mean, sigma);
}

void ProbabilityLookup::printStatistics() {
	cout << "Probability Lookup Table Statistics" << endl
		 << "===================================" << endl << endl;
	
	unsigned int totalSize = 0;
	map<int, int> histogram;
	
	for(unsigned int i = 0; i < lookupTable.size(); i++) {
		totalSize += sizeof(lookupTable[i]) + sizeof(float)* lookupTable[i].size();
		
		if(histogram.count(lookupTable[i].size()) == 0)
			histogram[lookupTable[i].size()] = 1;
		else histogram[lookupTable[i].size()]++;
	}
	
	cout << "Histogram:" << endl;
	for(map<int, int>::iterator iter = histogram.begin(); iter!= histogram.end(); iter++) {
		cout << setw(4) << iter->first << ": " << setw(8) << iter->second << endl;
	}
	double dist = 7;
	cout << endl << "Example entry at " << dist << " m:" << endl;
	vector<float>* example = lookupEntry(dist);
	if(example != NULL) { 
		for(unsigned int i=0; i<example->size(); i++) {
			cout << setw(4) << (i*voxelResolution) << ": " << setw(8) << (*example)[i] << endl;
		}
	}
	
	cout << endl << "Total size: " << (totalSize/1024) << " KB" << endl;
}
