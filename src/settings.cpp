#include <opencv2/opencv.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "settings.h"
#include "utils.h"

using namespace std;

bool Settings::load(string filename) {
	std::string value;
	boost::property_tree::ptree ptree;

	try {
		boost::property_tree::read_ini(filename, ptree);
	}
	catch (const boost::property_tree::ptree_error &e) {
		cout << e.what() << endl;
		return false;
	}

	if (ptree.empty()) {
		cout << "The file is empty!" << endl;
		return false;
	}

	if (!read(ptree, "input_folder", inputFolder))
		return false;

	if (!read(ptree, "num_corners_h", value))
		return false;
	boardSize.width = stoi(value);

	if (!read(ptree, "num_corners_w", value))
		return false;
	boardSize.height = stoi(value);

	if (!read(ptree, "square_size", value))
		return false;
	squareSize = stof(value);

	if (!read(ptree, "output_filename", outputFilename))
		return false;

	if (!read(ptree, "output_foldername", outputFoldername))
		outputFilename = "./output";

	if (read(ptree, "write_features", value))
		writeFeatures = to_bool(value);

	if (read(ptree, "write_extrinsic", value))
		writeExtrinsic = to_bool(value);

	if (read(ptree, "zero_tangential", value)) {
		zeroTangential = to_bool(value);
		if (zeroTangential) flags |= cv::CALIB_ZERO_TANGENT_DIST;
	}

	if (read(ptree, "fix_aspect_ratio", value)) {
		aspectRatio = stof(value) < 0.f ? 0.f : stof(value);
		flags |= cv::CALIB_FIX_ASPECT_RATIO;
	}

	if (read(ptree, "fix_principal_point", value)) {
		fixPrincipalPoint = to_bool(value);
		if (fixPrincipalPoint) flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
	}

	if (read(ptree, "flip_horizontal", value))
		flipHorizontal = to_bool(value);

	if (read(ptree, "show_undistorted_images", value))
		showUndistortedImages = to_bool(value);

	return true;
}
