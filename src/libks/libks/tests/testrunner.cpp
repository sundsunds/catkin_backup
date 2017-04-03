#include <cppunit/ui/text/TestRunner.h>
#include "libks/tests/hammingdistance.h"
#include "libks/tests/census.h"
#include "libks/tests/sparsecostcube.h"
#include "libks/tests/pyramidrangedetector.h"

using namespace CppUnit;
using namespace ks;

int main(int argc, char **argv)
{
	TextUi::TestRunner runner;
	runner.addTest(TestHammingDistance::suite());
	runner.addTest(TestCensus::suite());
	runner.addTest(TestSparseCostCube::suite());
	runner.addTest(TestPyramidRangeDetector::suite());
	runner.run();
	return 0;
}
          