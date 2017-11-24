#pragma once

#include <string>
#include <opencv2/opencv.hpp>

struct Settings {
	/**
	 * the folder containing the calibration images.
	 * All the images must contain the same checkerboard.
	 */
	std::string inputFolder;

	/**
	 * The number of inner corners per one of board in each dimension.
	 */
	cv::Size boardSize;

	/**
	 * The number of frames to use for calibration.
	 * Set it to a negative value for using entire images in given folder.
	 */
	int numFrames = -1;

	/**
	 * Square size in user-defined units, e.g., 20mm as 20
	 */
	float squareSize;

	/**
	 * The output filename for intrinsic [and extrinsic] parameters
	 */
	std::string outputFilename;

	/**
	 * The output filename for intrinsic [and extrinsic] parameters
	 */
	std::string outputFoldername;

	/**
	 * Write detected feature points if set to true
	 */
	bool writeFeatures = false;

	/**
	 * Write extrinsic parameters
	 */
	bool writeExtrinsic = false;

	/**
	 * Assume zero tangential distortion if set to true
	 */
	bool zeroTangential = false;

	/**
	 * Fix aspect ratio (fx/fy)
	 */
	float aspectRatio = 1.f;

	/**
	 * Fix the principal point at the center
	 */
	bool fixPrincipalPoint = true;

	/**
	 * Flip the captured images around the horizontal axis
	 */
	bool flipHorizontal = true;

	/**
	 * Fix the principal point at the center
	 */
	bool showUndistortedImages = true;

	/**
	 * Calibration flags
	 */
	int flags;

	/**
	 * Load settings from given ini file
	 */
	bool load(std::string filename);
};
