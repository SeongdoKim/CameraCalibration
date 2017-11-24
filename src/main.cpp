#include <vector>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "utils.h"
#include "CameraCalibration.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
	string optionfile = "./option.ini";

	if (argc == 2) {
		optionfile = string(argv[1]);
		if (!isValidFile(optionfile)) {
			cout << "Invalid setting file type \'" << argv[1] << "\'" << endl;
			return EXIT_FAILURE;
		}
	}

	Settings settings;

	settings.load(optionfile);

	Mat cameraMatrix, distortCoeff;
	CameraCalibration calibration;

	if (!calibration.calibrate(settings)) {
		return EXIT_FAILURE;
	}

#ifdef _WIN32
	system("pause");
#endif

	return EXIT_SUCCESS;
}
