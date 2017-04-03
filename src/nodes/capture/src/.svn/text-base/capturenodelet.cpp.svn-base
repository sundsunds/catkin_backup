#include <pluginlib/class_list_macros.h>
#include "libks/base/argvconverter.h"
#include "capturenodelet.h"

PLUGINLIB_DECLARE_CLASS(capture, CaptureNodelet, capture::CaptureNodelet, nodelet::Nodelet)

namespace capture {
	using namespace boost;
	using namespace ks;

	CaptureNodelet::CaptureNodelet() {
	}
	
	CaptureNodelet::~CaptureNodelet() {
		mainThread.join();
	}

	void CaptureNodelet::onInit() {
		ArgvConverter argConv(getMyArgv());
		unique_lock<mutex> lock(argConv.getoptMutex);
		capture.onInit(argConv.argc, argConv.argv, &getNodeHandle());
		mainThread = thread(bind(&CaptureNodelet::mainLoop, this));
	}

	void CaptureNodelet::mainLoop() {
		capture.mainLoop();
	}
}
