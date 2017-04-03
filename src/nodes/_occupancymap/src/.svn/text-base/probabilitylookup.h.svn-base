#ifndef PROBABILITYNLOOKUP_H
#define PROBABILITYLOOKUP_H

#include <boost/multi_array.hpp>

// Lookup table for the probability distribution of
// endpoint observations in the occupancy map
class ProbabilityLookup {
public:
	ProbabilityLookup(double ocTreeResolution, double voxelResolution, double depthResolution,
		double maxDepth, double probMin, double probMax, double cutOffLimit, double depthErrorScale);
	
	void printStatistics();
	
	/*float lookupProbability(double z, double dist) {
		int distScaled = (int)(dist / voxelResolution + 0.5);
		std::vector<float>* entry = lookupEntry(z);
		if(entry != NULL && distScaled < (int)entry->size())
			return (*entry)[distScaled];
		else return probMin;
	}*/
	
	std::vector<float>* lookupEntry(double z) {
		int zScaled = (int)(z / depthResolution + 0.5);
		if(zScaled >= (int)lookupTable.size() || zScaled < 0)
			return NULL;
		else return &lookupTable[zScaled];
	}
	
	/*double lookupZRange(double z) {
		std::vector<float>* entry = lookupEntry(z);
		if(entry != NULL)
			return (entry->size()/2+1)*voxelResolution;
		else return 0.0;
	}*/
	
private:
	double ocTreeResolution;
	double voxelResolution;
	double depthResolution;
	float probMin;
	
	std::vector<std::vector<float> > lookupTable;
	
	double computeCDF(double x, double mean, double sigma);
	double computeRangeProbability(double mean, double sigma, double minRange, double maxRange);
};

#endif
