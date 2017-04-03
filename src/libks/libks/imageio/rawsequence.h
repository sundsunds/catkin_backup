#ifndef KS_RAWSEQUENCE_H
#define KS_RAWSEQUENCE_H

#include <vector>
#include "libks/imageio/rawimage.h"
#include "libks/base/exception.h"

namespace ks {
	// Stores a sequence of raw images
	class RawSequence {

	public:
		// Crates an empty sequence
		RawSequence() {};
		// Creates a new sequence with the given dimensions
		RawSequence(unsigned int width, unsigned int height,
			unsigned int num);
		// Loads a sequence form a file
		RawSequence(const char* file);
		
		~RawSequence();
		
		// Adds an image to the sequence
		void add(const RawImage& image);
		
		// Gets the number of images
		unsigned int getCount() const {return images.size();}
		
		// Gets the n-th image of the sequence
		RawImage& operator[](unsigned int num) {return *(images[num]);}
		const RawImage& operator[](unsigned int num) const {return *(images[num]);}
		
		// Saves the sequence to a file
		void save(const char* file);

		// Exception class thrown when a decoding error occures
		class DecodingException: public Exception {
		public:
			DecodingException(): Exception(
				"Error decoding raw image sequence") {}
		};

	private:
		std::vector<RawImage*> images;
		static const unsigned int magicNumber;
	};
}

#endif
