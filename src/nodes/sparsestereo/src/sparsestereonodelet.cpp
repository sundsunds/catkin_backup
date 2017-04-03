#include <pluginlib/class_list_macros.h>
#include "sparsestereonodelet.h"
#include "libks/base/argvconverter.h"

PLUGINLIB_DECLARE_CLASS(sparsestereo, SparseStereoNodelet, sparsestereo::SparseStereoNodelet, nodelet::Nodelet)

namespace sparsestereo {
	using namespace boost;
	using namespace ks;

	SparseStereoNodelet::SparseStereoNodelet() {
	}
	
	SparseStereoNodelet::~SparseStereoNodelet() {
		mainThread.join();
	}

	void SparseStereoNodelet::onInit() {
		ArgvConverter argConv(getMyArgv());
		unique_lock<mutex> lock(argConv.getoptMutex);
		sparsestereo.onInit(argConv.argc, argConv.argv, &getNodeHandle(), getName());
		mainThread = thread(bind(&SparseStereoNodelet::mainLoop, this));
	}

	void SparseStereoNodelet::mainLoop() {
		sparsestereo.mainLoop();
	}
}
