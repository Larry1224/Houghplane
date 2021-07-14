#include <iostream>
#include <fstream>
#include <cstdint>
#include "inc/stopwatch.hpp"
#include "inc/HoughPlane.hpp"
#include <fstream>

using namespace std;

int main(int argc, char **argv)
{
	if (argc != 7)
	{
		cerr << argc << endl;
		cerr << "\n"
			 << argv[0] << " [point cloud data pts file] [n_theta] [n_phi] [n_rho] [threshold] [output ptx filename]";
		return 255;
	}

	// Task 1 - get command line arguments
	const auto inputCloudData = argv[1];
	const uint32_t n_theta = atoi(argv[2]);
	const uint32_t n_phi = atoi(argv[3]);
	const uint32_t n_rho = atoi(argv[4]);
	const auto threshold = atof(argv[5]);
	const auto outputCloudData = argv[6];

	stopwatch t[5];

	// Task 2 - allocate memory & read point-cloud data
	t[0].start();
	t[1].start();
	mydata data;
	if (!readPointCloud(inputCloudData, data))
	{
		cerr << "\nError reading the point-cloud data, file: " << inputCloudData;
		return 254;
	}

	// Task 3 - Center the point-cloud data and get the maximum possible rho
	float rho_max = centerPointCloudToOrigin(data);
	t[1].stop();

	// Task 4 - prepare accumulator
	t[2].start();
	prepareAccumulator(data, rho_max, n_theta, n_phi, n_rho);

	// Task 5 - do 3D standard Hough Transform
	houghTransform(data);
	t[2].stop();

	// Task 6 - Find resultant plane parameters
	t[3].start();
	identifyPlaneParameters(data, threshold);
	t[3].stop();

	// Task 7 - Output ptx file
	t[4].start();
	if (!outputPtxFile(data, outputCloudData))
	{
		cerr << "\nError writing output file: " << outputCloudData;
	}
	// Task 8 - release all allocated memory
	release(data);

	t[4].stop();
	t[0].stop();

	// task 9 - output timing
	cout << "\n[Timing], ";
	for (const auto &i : t)
	{
		cout << i.elapsedTime() << ", ";
	}

	return 0;
}
