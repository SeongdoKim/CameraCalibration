#include "CameraCalibration.h"
#include "utils.h"

#include <boost/filesystem.hpp>

using namespace cv;
using namespace std;

CameraCalibration::CameraCalibration() {
}

CameraCalibration::CameraCalibration(string filename) {
	// TODO: read camera matrix and distortion coefficient from given file
}

CameraCalibration::~CameraCalibration() {
}

double CameraCalibration::computeReprojectionErrors(
	const vector<vector<Point3f>>& objectPoints,
	const vector<vector<Point2f>>& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++) {
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return sqrt(totalErr / totalPoints);
}

bool CameraCalibration::runCalibration(vector<vector<Point2f>> imagePoints,
	Size imageSize, Size boardSize,
	float squareSize, float aspectRatio,
	int flags, vector<Mat>& rvecs, vector<Mat>& tvecs,
	vector<float>& reprojErrs,
	double& totalAvgErr)
{
	mCameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flags & CALIB_FIX_ASPECT_RATIO)
		mCameraMatrix.at<double>(0, 0) = aspectRatio;

	mDistortCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f>> objectPoints(1);
	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			objectPoints[0].push_back(Point3f(float(j*squareSize),
			float(i*squareSize), 0));

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, mCameraMatrix,
		mDistortCoeffs, rvecs, tvecs, flags | CALIB_FIX_K4 | CALIB_FIX_K5);
	///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
	cout << "RMS error reported by calibrateCamera: " << rms << endl;
	cout << "Calibration matrix:\n" << mCameraMatrix << endl;
	cout << "Distortion coefficients: " << mDistortCoeffs.t() << endl;

	bool ok = checkRange(mCameraMatrix) && checkRange(mDistortCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, mCameraMatrix, mDistortCoeffs, reprojErrs);

	return ok;
}

void CameraCalibration::saveCameraParams(const string& filename,
	Size imageSize, Size boardSize,
	float squareSize, float aspectRatio, int flags,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const vector<float>& reprojErrs,
	const vector<vector<Point2f> >& imagePoints,
	double totalAvgErr)
{
	FileStorage fs(filename, FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm *t2 = new tm;
#ifdef _WIN32
	localtime_s(t2, &tt);
#else
	t2 = localtime(&tt);
#endif
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nframes" << (int)max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;

	if (flags & CALIB_FIX_ASPECT_RATIO)
		fs << "aspectRatio" << aspectRatio;

	fs << "flags" << flags;

	fs << "camera_matrix" << mCameraMatrix;
	fs << "distortion_coefficients" << mDistortCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
		fs << "extrinsic_parameters" << bigmat;
	}

	if (!imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}
}

bool CameraCalibration::calibrate(const Settings& settings,
	const vector<vector<Point2f>>& imagePoints, const Size imageSize) {
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(imagePoints, imageSize, settings.boardSize, settings.squareSize,
		settings.aspectRatio, settings.flags, rvecs, tvecs, reprojErrs, totalAvgErr);

	cout << (ok ? "Calibration succeeded" : "Calibration failed") <<
		". avg reprojection error = " << totalAvgErr << endl;

	if (ok) {
		saveCameraParams(settings.outputFilename, imageSize,
			settings.boardSize, settings.squareSize, settings.aspectRatio,
			settings.flags,
			settings.writeExtrinsic ? rvecs : vector<Mat>(),
			settings.writeExtrinsic ? tvecs : vector<Mat>(),
			settings.writeExtrinsic ? reprojErrs : vector<float>(),
			settings.writeFeatures ? imagePoints : vector<vector<Point2f>>(),
			totalAvgErr);
	}

	return ok;
}

bool CameraCalibration::calibrate(const Settings& settings) {
	// read image files from given folder
	int num_frames = settings.numFrames;
	vector<String> filelist;
	vector<Mat> images;
	vector<vector<Point2f>> imagePoints;

	glob(settings.inputFolder, filelist);

	if (num_frames < 0 || num_frames > filelist.size())
		num_frames = filelist.size();

	if (!settings.outputFoldername.empty()) {
		checkFolder(settings.outputFoldername);
	}

	for (int i = 0, j = 0; i < filelist.size(); i++) {
		Mat image = imread(filelist[i]), imageGrey;
		if (image.empty()) continue;
		images.push_back(image.clone());

		vector<Point2f> pointbuf;
		cvtColor(image, imageGrey, COLOR_BGR2GRAY);

		bool found = findChessboardCorners(image, settings.boardSize, pointbuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

		if (!found) continue;

		// improve the found corners' coordinate accuracy
		cornerSubPix(imageGrey, pointbuf, Size(11, 11),
			Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

		imagePoints.push_back(pointbuf);

		drawChessboardCorners(image, settings.boardSize, Mat(pointbuf), found);

		{
			boost::filesystem::path p(settings.outputFoldername);
			if (boost::filesystem::exists(p)) {
				boost::filesystem::path filename(filelist[i].c_str());
				boost::filesystem::path output_path(p.string() + string() + "/" + filename.stem().string() + "_features.png");
				imwrite(output_path.string(), image);
			}
		}

		if (++j == num_frames) break;
	}

	if (!calibrate(settings, imagePoints, images[0].size()))
		return false;

	if (settings.showUndistortedImages) {
		// Check if output folder is exist
		boost::filesystem::path p(settings.outputFoldername);
		if (boost::filesystem::exists(p)) {
			for (int i = 0; i < images.size(); i++) {
				Mat output;
				boost::filesystem::path filename(filelist[i].c_str());
				boost::filesystem::path output_path(p.string() + "/" + filename.stem().string() + "_undistort.png");
				undistort(images[i], output, mCameraMatrix, mDistortCoeffs);
				imwrite(output_path.string(), output);
			}
		}
	}

	return true;
}

void CameraCalibration::undistortImages(const string folderpath, const string outputFolderpath) {
	if (mCameraMatrix.empty() || mDistortCoeffs.empty() ||
		folderpath.empty() || outputFolderpath.empty())
		return;

	vector<String> filelist;

	glob(folderpath, filelist);

	if (!checkFolder(outputFolderpath))
		return;

	boost::filesystem::path p(outputFolderpath);

	for (int i = 0; i < filelist.size(); i++) {
		Mat image = imread(filelist[i]), output;
		if (image.empty()) continue;

		boost::filesystem::path filename(filelist[i].c_str());
		boost::filesystem::path output_path(p.string() + "/" + filename.stem().string() + "_undistort.png");

		undistort(image, output, mCameraMatrix, mDistortCoeffs);
		imwrite(output_path.string(), output);
	}
}
