#include <vector>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "utils.h"
#include "CameraCalibration.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
	if (argc < 4) {
		cout << "Usage: " << argv[0] << " [camera_setting] [input_folder] [output_folder]" << endl;
		return EXIT_FAILURE;
	}

	string setting_file(argv[1]);
	CameraCalibration calibration(setting_file);

	if (!calibration.isGood()) {
		cout << "Invalid camera setting from the file: " << setting_file << endl;
		return EXIT_FAILURE;
	}

	calibration.undistortImages(argv[2], argv[3]);

	return EXIT_SUCCESS;
}
