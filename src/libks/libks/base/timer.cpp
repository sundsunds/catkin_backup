#include "timer.h"

namespace ks {
	using namespace std;
	using namespace boost;
	using namespace boost::posix_time;

	Timer::Timer(const char* name): name(name), started(false) {
	}
	
	Timer::~Timer() {
		cout << "Timer \"" << name << "\" Avg: " << (sumTime *1000.0/ count)
			<< " ms / Total: " << (sumTime*1000.0) << " ms" << endl;
	}
	
	void Timer::start() {
		if(started)
			cerr << "Timer \"" << name << "\" already started!" << endl;
		else {
			started = true;
			startTime = microsec_clock::local_time();
		}
	}
	
	void Timer::stop() {
		if(!started)
			cerr << "Timer \"" << name << "\" not started!" << endl;
		else {
			count ++;
			sumTime += (microsec_clock::local_time() - startTime).total_nanoseconds()*1.0e-9;
			started = false;
		}
	}
}
