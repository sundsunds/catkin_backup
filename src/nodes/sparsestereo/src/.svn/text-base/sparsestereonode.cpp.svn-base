#include <ros/ros.h>
#include <exception>
#include <iostream>
#include "sparsestereobase.h"

int main(int argc, char** argv) {
	try {
		sparsestereo::SparseStereoBase ssbase;
		ssbase.onInit(argc, argv, NULL, "Sparse Stereo");
		ssbase.mainLoop();
	} catch (const std::exception& ex) {
		std::cerr << "Fatal exception: " << ex.what() << std::endl;
		return 1;
	}
	return 0;
}
