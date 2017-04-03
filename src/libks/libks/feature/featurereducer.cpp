#include "libks/feature/featurereducer.h"
#include <iostream>
#include <cstdlib>
#include <algorithm>

namespace ks {
	using namespace std;
	using namespace cv;
	
	FeatureReducer::FeatureReducer(unsigned int maxFeatures, int imgWidth, int imgHeight)
		: maxFeatures(maxFeatures), imgWidth(imgWidth), imgHeight(imgHeight)/*offsets(imgHeight, imgWidth)*/ {
	}
	
	void FeatureReducer::reduce(const vector<KeyPoint>& in, vector<KeyPoint>* out) {
		//initOffsets(in);
		//testOffsets(in);
		
		if(in.size() <= maxFeatures) {
			// No Feaure reduction neccessary
			*out = in;
			return;
		}
		out->clear();
		out->reserve(maxFeatures);
		
		//static const int xTiles = 4, yTiles = 3;
		static const int xTiles = 5, yTiles = 4;
		vector<const KeyPoint*> tiles[yTiles][xTiles];
		int tileWidth = imgWidth / xTiles;
		int tileHeight = imgHeight / yTiles;
		
		// Fill tiles
		for(unsigned int i=0; i<in.size(); i++)
			tiles[int(in[i].pt.y / tileHeight)][int(in[i].pt.x / tileWidth)].push_back(&in[i]);
		
		float fraction = maxFeatures / (float)in.size();
		float residual = 0;
		
		for(int y=0; y<yTiles; y++)
			for(int x=0; x<xTiles; x++) {
				vector<const KeyPoint*>& tile = tiles[y][x];
				// Sort current tile by score
				sort(tile.begin(), tile.end(), KeyPointComp());
				
				// Determine number of features to keep
				float numFeat = fraction * tile.size() + residual;
				unsigned int intNumFeat = (unsigned int) numFeat;
				residual = numFeat - intNumFeat;
				
				// Copy best features
				for(unsigned i = 0; i < intNumFeat; i++)
					out->push_back(*tile[i]);
			}
	}
	
	/*void FeatureReducer::testOffsets(const vector<KeyPoint>& points) {
		for(unsigned int i=1; i<points.size()-1; i++)
			if(offsets(points[i].pt.y, points[i].pt.x) != i ||
				offsets(points[i-1].pt.y, points[i-1].pt.x+1) != i) {
				cerr << "Offsets wrong for " << points[i].pt << endl;
				exit(1);
				}
		cerr << "All offsets OK" << endl;
	}
	
	void FeatureReducer::initOffsets(const vector<KeyPoint>& points) {
		Point lastPoint(-1, 0);
		
		for(unsigned int i=0; i<=points.size(); i++) {
			int offset = i;
			Point endPoint;
			
			if(i < points.size())
				endPoint = points[i].pt;
			else {
				endPoint = Point(offsets.cols-1, offsets.rows-1);
				offset = i-1; // Fill up the last offset till the end
			} 
			
			
			Point p(lastPoint.x, lastPoint.y);
			do {
				// Increment point by one
				p.x++;
				if(p.x >= offsets.cols) {
					p.y++;
					p.x = 0;
				}
				
				// Set offset
				offsets(p.y, p.x) = offset;	
			} while(p != endPoint);
			
			lastPoint = endPoint;
		}
	}*/
}
