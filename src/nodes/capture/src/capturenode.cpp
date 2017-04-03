#include <ros/ros.h>
#include <exception>
#include <iostream>
#include "capturebase.h"

int main(int argc, char** argv) {
	try {
		capture::CaptureBase cbase;
		cbase.onInit(argc, argv, NULL);
		cbase.mainLoop();
	} catch (const std::exception& ex) {
		std::cerr << "Fatal exception: " << ex.what() << std::endl;
		return 1;
	}
	return 0;
}
