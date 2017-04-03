#include <octomap/OcTree.h>
#include <exception>
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;
using namespace octomap;

int main(int argc, char** argv) {
	try {
		if(argc != 2) {
			cout << "Usage: " << argv[0] << "octree-file" << endl;
			return 1;
		}
		
		// Read input file
		cout << "Reading octree..." << endl;
		OcTree* octree = dynamic_cast<OcTree*>(AbstractOcTree::read(argv[1]));
		
		if(octree == NULL) {
			cout << "Reading failed!" << endl;
			return 1;
		}
		
		cout << "Reading done" << endl;
		
		// Get bounding box
		double minX, minY, minZ;
		double maxX, maxY, maxZ;
		octree->getMetricMin(minX, minY, minZ);
		octree->getMetricMax(maxX, maxY, maxZ);
		
		cout << "X range: " << minX << " - " << maxX << endl;
		
		int numCells = (maxX - minX) / octree->getResolution() + 2;
		vector<double> scores(numCells, 0);
		
		cout << "Searching best x-divider" << endl;
		
		// Calculate score for all x-values
		unsigned int maxDepth = octree->getTreeDepth();
		for (OcTree::leaf_iterator it = octree->begin_leafs(); it != octree->end_leafs(); it++) {
			if(it.getDepth() == maxDepth && octree->isNodeOccupied(*it)) {
				for(int xIdx = 0; xIdx < numCells; xIdx++) {
					double x = minX + (xIdx - 1) * octree->getResolution();
					
					double scoreInc = 1.0 / fabs(x - it.getX());
					if(it.getX() < x)
						scores[xIdx] += scoreInc*scoreInc;
					else scores[xIdx] -= scoreInc*scoreInc;
				}
			}
		}
		
		// Extract best x-value
		double minScore = 1e10;
		int minIdx = -1;
		
		for(int xIdx = 0; xIdx < numCells; xIdx++) {
			if(fabs(scores[xIdx]) < minScore) {
				minScore = fabs(scores[xIdx]);
				minIdx = xIdx;
			}
		}
		
		double divX = minX + (minIdx - 1) * octree->getResolution();
		cout << "Best dividing x-value: " << divX << endl;
		cout << "Score: " << minScore << endl;
		
		// Write divided octrees for debugging
		cout << "Writting debugging trees" << endl;
		OcTree upperTree(octree->getResolution()), lowerTree(octree->getResolution());
		
		for (OcTree::leaf_iterator it = octree->begin_leafs(); it != octree->end_leafs(); it++) {
			if(it.getDepth() == maxDepth && octree->isNodeOccupied(*it)) {
				if(it.getX() < divX)
					upperTree.updateNode(it.getKey(), true);
				else lowerTree.updateNode(it.getKey(), true);
			}
		}
		
		fstream upperStrm("upper_tree.bt", ios::out | ios::binary | ios::trunc),
			lowerStrm("lower_tree.bt", ios::out | ios::binary | ios::trunc);
			
		upperTree.writeBinary(upperStrm);
		lowerTree.writeBinary(lowerStrm);
		upperStrm.close();
		lowerStrm.close();
		
		delete octree;
		return 0;
	} catch (const exception& e) {
		cout << e.what() << endl;
		return 1;
	}
}
