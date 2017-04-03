#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include "capturebase.h"

namespace capture {
	class CaptureNodelet: public nodelet::Nodelet {
	public:
		CaptureNodelet();
		~CaptureNodelet();

		// Performs general initializations
		virtual void onInit();

	private:
		CaptureBase capture;
	
		boost::thread mainThread;

		// Runs the processing loop
		void mainLoop();
	};
}
