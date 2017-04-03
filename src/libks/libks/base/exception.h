#ifndef KS_EXCEPTION_H
#define KS_EXCEPTION_H

#include <exception>
#include <string>

namespace ks {
	// Base class for all exceptions
	class Exception: public std::exception {
	public:
		Exception(const char* msg)
			: message(msg) {}
		Exception(std::string msg)
			: message(msg) {}
		~Exception() throw() {}
		
		virtual const char* what() const throw()
			{return message.c_str();}
			
	private:
		std::string message;
	};
}
#endif
