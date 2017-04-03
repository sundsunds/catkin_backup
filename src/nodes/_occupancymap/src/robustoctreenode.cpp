#include "robustoctreenode.h"
#include <iostream>

namespace occupancymap {
	using namespace std;
	using namespace octomap;

	ostream& RobustOcTreeNode::writeValue(ostream &s) const {	
		// 1 bit for each children; 0: empty, 1: allocated
		std::bitset<8> children;

		for (unsigned int i=0; i<8; i++) {
		  if (childExists(i))
			children[i] = 1;
		  else
			children[i] = 0;
		}

		char children_char = (char) children.to_ulong();
		float logVal = logodds(value);
		s.write((const char*) &logVal, sizeof(logVal));
		s.write((char*)&children_char, sizeof(char));

		// write children's children
		for (unsigned int i=0; i<8; i++) {
		  if (children[i] == 1) {
			((RobustOcTreeNode*)this->getChild(i))->writeValue(s);
		  }
		}
		return s;
	}
	
	istream& RobustOcTreeNode::readValue(istream &s) {
		char children_char;

		// read data:
		float logVal;
		s.read((char*) &logVal, sizeof(logVal));
		value = probability(logVal);
		
		s.read((char*)&children_char, sizeof(char));
		std::bitset<8> children ((unsigned long long) children_char);

		for (unsigned int i=0; i<8; i++) {
			if (children[i] == 1){
				createChild(i);
				getChild(i)->readValue(s);
			}
		}
		return s;
	}
}