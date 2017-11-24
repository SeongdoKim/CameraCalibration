#pragma once

#include <vector>

#include "settings.h"

class CameraCalibration {
private:
	/**
	 * Computed camera matrix
	 */
	cv::Mat mCameraMatrix;

	/**
	 * Distortion coefficient, where the number of elements between 4 and 8:
	 * [k1, k2, p1, p2, [k3, [k4, k5, k6]]], where p1 and p2 are tangential distortion
	 * coefficient.
	 */
	cv::Mat mDistortCoeffs;

private:
	static double computeReprojectionErrors(
		const std::vector<std::vector<cv::Point3f>>& objectPoints,
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
		std::vector<float>& perViewErrors);

	bool runCalibration(std::vector<std::vector<cv::Point2f>> imagePoints,
		cv::Size imageSize, cv::Size boardSize,
		float squareSize, float aspectRatio,
		int flags, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
		std::vector<float>& reprojErrs,
		double& totalAvgErr);

	void saveCameraParams(const std::string& filename,
		cv::Size imageSize, cv::Size boardSize,
		float squareSize, float aspectRatio, int flags,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const std::vector<float>& reprojErrs,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		double totalAvgErr);

	bool calibrate(const Settings& settings,
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		const cv::Size imageSize);

public:
	CameraCalibration();
	CameraCalibration(std::string filename);
	~CameraCalibration();

	/**
	 * Collect calibration images from given folder, then calibrate the camera
	 */
	bool calibrate(const Settings& settings);

	/**
	 * Undistort images in the given folder
	 */
	void undistortImages(const std::string folderpath, const std::string outputFolderpath);
};
