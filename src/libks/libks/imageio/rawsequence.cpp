#include "libks/imageio/rawsequence.h"
#include <fstream>

namespace ks {
	using namespace std;

	const unsigned int RawSequence::magicNumber=0xA31F9318; // Some random number

	RawSequence::RawSequence(unsigned int width, unsigned int height,
		unsigned int num)
		:images(num) {
		for(unsigned int i=0; i<num; i++)
			images[i] = new RawImage(width, height);	
	}

	void RawSequence::add(const RawImage& image){
		if(images.size() > 0 &&
			(images.front()->getWidth() != image.getWidth() ||
			images.front()->getHeight() != image.getHeight()))
			throw Exception("Image dimensions of sequence don't match");
		images.push_back(new RawImage(image));
	}

	RawSequence::RawSequence(const char* file) {
		fstream strm(file, ios::in | ios::binary);
		unsigned int magic = 0;
		strm.read((char*)&magic, sizeof(magic));
		if(magic != magicNumber)
			throw DecodingException();
		unsigned int width, height, num;
		strm.read((char*)&width, sizeof(width));
		strm.read((char*)&height, sizeof(height));
		strm.read((char*)&num, sizeof(num));
		
		float* buffer=new float[width*height];
		for(unsigned int i=0; i<num; i++) {
			strm.read((char*)buffer, sizeof(float) * width*height);
			images.push_back(new RawImage(width, height, buffer));
		}
		delete []buffer;
	}

	RawSequence::~RawSequence() {
		for(unsigned int i=0; i<images.size(); i++)
			delete images[i];
	}

	void RawSequence::save(const char* file) {
		fstream strm(file, ios::out | ios::binary);
		strm.write((char*)&magicNumber, sizeof(magicNumber));
		unsigned int buffer = images.front()->getWidth();
		strm.write((char*)&buffer, sizeof(buffer));
		buffer = images.front()->getHeight();
		strm.write((char*)&buffer, sizeof(buffer));
		buffer = images.size();
		strm.write((char*)&buffer, sizeof(buffer));
		for(unsigned int i=0; i<images.size(); i++) {
			strm.write((char*)images[i]->getValues(),
				sizeof(float) * images[i]->getWidth() * images[i]->getHeight());
		}
	}
}
