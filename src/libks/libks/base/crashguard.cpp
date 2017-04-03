#include "crashguard.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <climits>

namespace ks {
	using namespace std;
	
	CrashGuard* CrashGuard::instance = NULL;
	
	void CrashGuard::setup() {
		if(instance == NULL)
			instance = new CrashGuard();
	}

	CrashGuard::CrashGuard(): executable() {
		pid = getpid();
		initSignals();
	}
	
	CrashGuard::~CrashGuard() {
	}

	void CrashGuard::initSignals()
	{
		lastBadSignal = -1;

		coreAction.sa_handler = CrashGuard::badSignals;
		coreAction.sa_flags = SA_RESTART /*| SA_RESETHAND*/;

		sigaction(SIGQUIT, &coreAction, &(oldActions[SIGQUIT]));
		sigaction(SIGILL , &coreAction, &(oldActions[SIGILL]));
		sigaction(SIGABRT, &coreAction, &(oldActions[SIGABRT]));
		sigaction(SIGFPE , &coreAction, &(oldActions[SIGFPE]));
		sigaction(SIGSEGV, &coreAction, &(oldActions[SIGSEGV]));
		sigaction(SIGBUS , &coreAction, &(oldActions[SIGBUS]));
		sigaction(SIGSYS , &coreAction, &(oldActions[SIGSYS]));
		sigaction(SIGTRAP, &coreAction, &(oldActions[SIGTRAP]));
		sigaction(SIGXCPU, &coreAction, &(oldActions[SIGXCPU]));
		sigaction(SIGXFSZ, &coreAction, &(oldActions[SIGXFSZ]));
		//sigaction(SIGIOT, &userterm, NULL); // synonym for SIGABRT
	}

	const char* CrashGuard::signalToStr(int signo) {
		switch (signo) {
			case SIGHUP:
				return "SIGHUP";
			case SIGINT:
				return "SIGINT";
			case SIGQUIT:
				return "SIGQUIT";
			case SIGILL:
				return "SIGILL";
			case SIGABRT:
				return "SIGABRT";
			case SIGFPE:
				return "SIGFPE";
			case SIGKILL:
				return "SIGKILL";
			case SIGSEGV:
				return "SIGSEGV";
			case SIGPIPE:
				return "SIGPIPE";
			case SIGALRM:
				return "SIGALRM";
			case SIGTERM:
				return "SIGTERM";
			case SIGUSR1:
				return "SIGUSR1";
			case SIGUSR2:
				return "SIGUSR2";
			case SIGCHLD:
				return "SIGCHLD";
			case SIGCONT:
				return "SIGCONT";
			case SIGSTOP:
				return "SIGSTOP";
			case SIGTSTP:
				return "SIGTSTP";
			case SIGTTIN:
				return "SIGTTIN";
			case SIGTTOU:
				return "SIGTTOU";
			case SIGBUS:
				return "SIGBUS";
			case SIGPROF:
				return "SIGPROF";
			case SIGSYS:
				return "SIGSYS";
			case SIGTRAP:
				return "SIGTRAP";
			case SIGURG:
				return "SIGURG";
			case SIGVTALRM:
				return "SIGVTALRM";
			case SIGXCPU:
				return "SIGXCPU";
			case SIGXFSZ:
				return "SIGXFSZ";
			case SIGSTKFLT:
				return "SIGSTKFLT";
			case SIGIO:
				return "SIGIO";
			case SIGPWR:
				return "SIGPWR";
			case SIGWINCH:
				return "SIGWINCH";
			default:
				return "UNKNOWN";
		}
	}
	
	void CrashGuard::badSignals(int signo) {
		static const char* gdbExec = "/usr/bin/gdb";
		static bool backtracing = false;
		static bool backtracingDone = false;

		if(instance->pid != getpid())
			return;

		if(instance->lastBadSignal == signo && backtracingDone == false)
			return; // do nothing
			
		instance->lastBadSignal = signo;

		cerr << "FATAL-ERROR: " << strsignal(signo) << " (" << signalToStr(signo) << ")" << endl;
		if(backtracing == false) {
			backtracing = true;
			bool gdbAvailable = (access(gdbExec, R_OK | X_OK) == 0);

			if (gdbAvailable) {
				static const char* gdbFileName = "/tmp/crashguard.gdb";
				ofstream gdbFile(gdbFileName, ios::out | ios::trunc);
				gdbFile << "echo \\n\\nShort backtrace:\\n\\n" << endl << "bt" << endl ;
				gdbFile << "echo \\n\\nFull backtrace:\\n\\n" << endl << "bt full" << endl;
				gdbFile << "echo \\n\\nThreads:\\n\\n" << endl << "info threads" << endl;
				gdbFile << "echo \\n\\nShort backtraces for all threads:\\n" << endl
					<< "thread apply all bt" << endl;
				gdbFile << "echo \\n\\nFull backtraces for all threads:\\n" << endl
					<< "thread apply all bt full" << endl;
				gdbFile.close();

				int tid = (int)syscall(__NR_gettid);;
				if (tid < 0) 
					tid = instance->pid;

				char datestr[200];
				memset(datestr, 0x00, sizeof(datestr));
				time_t t = time(NULL);
				struct tm * tmp = localtime(&t);
				bool dateval = (strftime(datestr, sizeof(datestr), "%Y%m%d%H%M%S", tmp) != 0);
				
				char execPath[PATH_MAX] = "";
				readlink("/proc/self/exe", execPath, sizeof(execPath));
				
				char execCopy[PATH_MAX] = "";
				strcpy(execCopy, execPath);
				char* execName = basename(execCopy);
				
				char cmd[PATH_MAX*3];
				memset(cmd, 0x00, sizeof(cmd));
				snprintf(cmd, 4096, "%s -x %s -batch %s %d > /tmp/backtrace-%s-%s.%d 2>&1", gdbExec, gdbFileName,
						execPath, tid, execName, dateval ? datestr : "", instance->pid);
				cerr << "Trying to start GDB: " << cmd << endl;
				system(cmd);
				unlink(gdbFileName);
			} else cerr << "Not able to write backtrace. Can't read/execute gdb from: " << gdbExec << endl;
			backtracingDone = true;
		}

		if (backtracingDone)
			instance->oldActions[signo].sa_handler(signo); // Call the original signal handler
	}
}
