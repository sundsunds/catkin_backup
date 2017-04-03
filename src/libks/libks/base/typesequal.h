// Macros for enabling template type comparisons

#ifndef KS_TYPESEQUAL_H
#define KS_TYPESEQUAL_H

namespace ks {

	template <typename T, typename U>
	struct TypesEqual {
		enum { result = false };
	};

	template <typename T>
	struct TypesEqual<T, T> {
		enum { result = true };
	};

}

#endif
