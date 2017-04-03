#ifndef KS_CRASHGUARD_H
#define KS_CRASHGUARD_H

#include <signal.h>
#include <string>

namespace ks {
	// Installs a signal handler that executes gdb on a crash
	class CrashGuard {
	public:
		// Installs the crashguard
		static void setup();
		
	private:
		CrashGuard();
		~CrashGuard();
	
		static CrashGuard* instance;
		pid_t pid;
		std::string executable;
	
		// Variables needed for signal handling   
		int lastBadSignal;
		struct sigaction coreAction;
		struct sigaction oldActions[_NSIG];
		
		void initSignals();
		static const char* signalToStr(int signo);
		static void badSignals(int signo);
	};
}

#endif
