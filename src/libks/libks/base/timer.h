#ifndef KS_TIMER_H
#define KS_TIMER_H

#include <boost/date_time/posix_time/posix_time.hpp>

namespace ks {
	// Tracks execution times for program parts.
	// Usage: Create a static object and call start/stop at the beginning/end
	// of the critical block
	class Timer {
		public:
		
		Timer(const char* name);
		~Timer();
		
		void start();
		void stop();
		
		private:
		std::string name;
		double sumTime;
		double count; 
		bool started;
		boost::posix_time::ptime startTime;
	};
}

#endif
