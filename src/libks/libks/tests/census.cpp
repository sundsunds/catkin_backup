#include "libks/imageproc/census-inl.h"
#include "libks/tests/census.h"

#include <iostream>
#include <iomanip>
using namespace std;

namespace ks {
	using namespace cv;

	char TestCensus::pixel[16*8] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,
		1, 0, 1, 1, 1, 1, 0, 1, 1, 1,  0, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 1, 1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,
		1, 0, 0, 0, 1, 1, 0, 1, 1, 1,  0, 0, 0, 0, 0, 0,
		1, 0, 0, 0, 1, 1, 0, 1, 1, 1,  0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,
		1, 0, 1, 1, 1, 1, 0, 1, 1, 1,  0, 0, 0, 0, 0, 0,
		1, 0, 1, 0, 1, 0, 1, 1, 0, 1,  0, 0, 0, 0, 0, 0,
	};

	TestCensus::TestCensus()
		:testImageInt(8, 16, pixel), testImageFloat(8, 16) {
		for(int y=0; y<testImageInt.rows; y++)
			for(int x=0; x<testImageInt.cols; x++)
				testImageFloat(y, x) = testImageInt(y, x);
	}
	
	void TestCensus::setUp() {
		censusImage = Mat_<unsigned int>(testImageInt.size(), 0).clone();
	}
	
	void TestCensus::test9x3Int() {
		census.transform9x3(testImageInt, &censusImage);
		//cout << hex << censusImage(2,4) << endl << hex << censusImage(3, 5) << endl;
		CPPUNIT_ASSERT(censusImage(2, 4) == 0x2138EE4 /*0b010000100111000111011100100*/);
		CPPUNIT_ASSERT(censusImage(3, 5) == 0x63F91C8 /*0b110001111111001000111001000*/);
	}
	
	void TestCensus::test9x3Float() {
		census.transform9x3(testImageFloat, &censusImage);
		Mat_<unsigned int> refImage(censusImage.size());
		census.transform9x3(testImageInt, &refImage);
		
		for(int y=2; y<5; y++)
			for(int x=4; x<12; x++)
				CPPUNIT_ASSERT(censusImage(y, x) == refImage(y, x));
	}
	
	void TestCensus::test5x5Int() {
		census.transform5x5(testImageInt, &censusImage);
		CPPUNIT_ASSERT(censusImage(3, 4) == 0x018E73F /*0b0000110001110011100111111*/);
		CPPUNIT_ASSERT(censusImage(4, 5) == 0x0394BE2 /*0b0001110010100101111100010*/);
	}
	
	void TestCensus::test5x5Float() {
		census.transform5x5(testImageFloat, &censusImage);
		Mat_<unsigned int> refImage(censusImage.size());
		census.transform5x5(testImageInt, &refImage);
		
		for(int y=2; y<5; y++)
			for(int x=2; x<14; x++)
				CPPUNIT_ASSERT(censusImage(y, x) == refImage(y, x));
	}
}