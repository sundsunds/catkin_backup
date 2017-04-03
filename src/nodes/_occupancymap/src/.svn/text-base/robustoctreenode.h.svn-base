#ifndef ROBUSTOCTREENODE_H
#define ROBUSTOCTREENODE_H

#include <octomap/OcTree.h>

namespace occupancymap {
	// An octree node that counts its observations and can be pruned if
	// not observed often enough
	class RobustOcTreeNode : public octomap::OcTreeDataNode<float> {		
	public:
		RobustOcTreeNode() : OcTreeDataNode<float>(0.5) {}
		RobustOcTreeNode(const RobustOcTreeNode& rhs) : OcTreeDataNode<float>(rhs) {}

		// children
		inline RobustOcTreeNode* getChild(unsigned int i) {
			return static_cast<RobustOcTreeNode*> (OcTreeDataNode<float>::getChild(i));
		}
		inline const RobustOcTreeNode* getChild(unsigned int i) const {
			return static_cast<const RobustOcTreeNode*> (OcTreeDataNode<float>::getChild(i));
		}

		bool createChild(unsigned int i) {
			if (children == NULL) allocChildren();
			children[i] = new RobustOcTreeNode();
			return true;
		}
		
		// Returns the occupancy probability
		inline float getOccupancy() const { return value; }
		
		// Sets the occupancy probability
		inline void setOccupancy(float p) { value = p; }
		
		// Methods for compatibility with octomap
		inline void setLogOdds(float l) {value = octomap::probability(l);}
		inline float getLogOdds() const {return octomap::logodds(value);}
		
		// Returns mean of all children's occupancy probabilities, in log odds
		double getMeanChildProbability() const{
			int ctr = 0;
			double sum = 0;
			for (unsigned int i=0; i<8; i++)
				if (childExists(i)) {
					sum += getChild(i)->getOccupancy();
					ctr++;
				}
			if(ctr)
				return sum/ctr;
			else return 0;
		}
		
		// Returns maximum of children's occupancy probabilities
		double getMaxChildProbability() const {
			float max = 0;
			for (unsigned int i=0; i<8; i++) {
				if (childExists(i)) {
					float p = getChild(i)->getOccupancy();
					if(p > max)
						max = p;
				}
			}
					
			return max;
		}
		
		// Update this node's occupancy according to its children's maximum occupancy
		inline void updateOccupancyChildren() {
			this->setOccupancy(this->getMaxChildProbability()); // conservative
		}
		
		// Shadow base class implementations, and simulates logodds
		std::ostream& writeValue(std::ostream &s) const;
		std::istream& readValue(std::istream &s);
	};
}

#endif
