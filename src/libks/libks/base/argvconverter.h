#ifndef KS_ARGVCONVERTER_H
#define KS_ARGVCONVERTER_H

#include <boost/thread.hpp>
#include <vector>
#include <string>
#include <cstring>

namespace ks {
	// Used for converting arguments of a nodelet to
	// argc/argv format
	class ArgvConverter {
	public:
		char** argv;
		int argc;
		static boost::mutex getoptMutex;
		
		ArgvConverter(std::vector<std::string> args) {
			argv = new char*[args.size()+2];
			for(unsigned int i=0; i<args.size(); i++) {
				argv[i+1] = new char[args[i].length() + 1];
				std::strcpy(argv[i+1], args[i].c_str());
			}
			argc = (int)args.size()+1;
			
			argv[0] = new char[strlen("[NODELET]")+1];
			strcpy(argv[0], "[NODELET]");
			argv[argc] = NULL;
		}
		
		~ArgvConverter() {
			for(int i=0; i<=argc; i++)
				delete [] argv[i];
			delete [] argv;
		}
	};
}

#endif
