#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include "sparsestereobase.h"

namespace sparsestereo {
	class SparseStereoNodelet: public nodelet::Nodelet {
	public:
		SparseStereoNodelet();
		~SparseStereoNodelet();

		// Performs general initializations
		virtual void onInit();

	private:
		SparseStereoBase sparsestereo;
	
		boost::thread mainThread;

		// Runs the processing loop
		void mainLoop();
	};
}
