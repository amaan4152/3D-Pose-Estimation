//Camera Calibration - OpenCV only

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

//sleep command
#include<unistd.h>
unsigned int microsecond = 1000000;


//----------------USER OPTIONS---------------------
//lower confidence bound to ignore keypoints
double confidenceBound = 0.1;
//Definition of checkerboard used in cameraCalibration
int boardWidth = 24;
int boardHeight = 13;
float squareSize = 0.5;			//inches
//Number of frames to process for camera calibration
int numFrames = 100;
//---------------END USER OPTIONS------------------

cv::Size boardSize(boardWidth, boardHeight);

//declaration of cameras for dualCameraCapture
cv::VideoCapture camera1;
cv::VideoCapture camera2;

//grayscale frame definitions
cv::Mat gray1, gray2;

//declare intrinsic camera matrices
cv::Mat cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2;

//declare extrinsic matrices
cv::Mat R, T, E, F;
cv::Mat R1, R2, P1, P2, Q;
cv::Rect validRoi1, validRoi2;

//file management declaration
cv::FileStorage fs;

//declare frame vars for openCV capture
cv::Mat frame1;
cv::Mat frame2;

//Stereo Calibration
int cameraCalibration()
{
	//definition of vectors used
	std::vector<std::vector<cv::Point3f>> objectPoints;
	std::vector<std::vector<cv::Point2f>> imagePoints1, imagePoints2;
	std::vector<cv::Point2f> corners1, corners2;
	cv::Size imageSize;

	//resize imagePoints for number of frames
	imagePoints1.resize(numFrames);
	imagePoints2.resize(numFrames);

	int i, numSuccess;

	numSuccess = 0;

	//open cameras 
	//open video capture
	camera1.open(0, cv::CAP_ANY);
	camera2.open(2, cv::CAP_ANY);
	//err detection
	if (!camera1.isOpened())
	{
		std::cerr << "camera1 failed to capture";
		return -1;
	}

	if (!camera2.isOpened())
	{
		std::cerr << "camera2 failed to capture";
		return -1;
	}

	for (i = 5; i > 1; i--)
	{
		std::cout << "Calibrating in " << std::to_string(i) << std::endl;
		usleep(microsecond);
	}

	//for each frame
	for (i = 0; i < numFrames; i++)
	{
		//capture frames
		camera1 >> frame1;
		camera2 >> frame2;

		//convert to grayscale
		cv::cvtColor(frame1, gray1, cv::COLOR_BGR2GRAY);
		cv::cvtColor(frame2, gray2, cv::COLOR_BGR2GRAY);

		//show grayscale images
		cv::imshow("gray1", gray1);
		cv::imshow("gray2", gray2);
		if (cv::waitKey(30) >= 0)
			break;


		//if either camera is not running, report error
		if (frame1.empty() | frame2.empty())
		{
			std::cerr << "ERROR: frames are empty!";
			return -1;
		}

		//resize imageSize to match frames
		if (imageSize == cv::Size())
			imageSize = frame1.size();
		else if (frame1.size() != imageSize | frame2.size() != imageSize)	//if either frames are not equal in size, report error
		{
			std::cerr << "ERROR: frames are not the same size!";
			return -1;
		}

		//boolean if checkerboard is found
		bool found1 = false;
		bool found2 = false;

		//find chessboard corners in frame
		found1 = cv::findChessboardCorners(gray1, boardSize, corners1,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
		found2 = cv::findChessboardCorners(gray2, boardSize, corners2,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

		//break if chessboard not found
		if (!found1 | !found2)
		{
			std::cout << i << ". Chessboard not found!" << std::endl;
			continue;
		}

		std::cout << i << ". Chessboard found!" << std::endl;

		//configuration of sub-pixel refinement
		cv::Size winSize = cv::Size(5, 5);
		cv::Size zeroZone = cv::Size(-1, -1);
		cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);

		//refine corner coordinates to sub-pixel
		cv::cornerSubPix(gray1, corners1, winSize, zeroZone, criteria);
		cv::cornerSubPix(gray2, corners2, winSize, zeroZone, criteria);

		std::cout << i << ". Found corners!" << std::endl;
		//output corner array
		imagePoints1[numSuccess] = corners1;
		imagePoints2[numSuccess] = corners2;
		numSuccess++;
	}
	//cleanup grayscale images
	cv::destroyAllWindows();

	//fit array size to number of processed pairs
	imagePoints1.resize(numSuccess);
	imagePoints2.resize(numSuccess);
	objectPoints.resize(numSuccess);

	//create array of object points
	for (i = 0; i < numSuccess; i++)
	{
		for (int j = 0; j < boardSize.height; j++)
			for (int k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(cv::Point3f(k*squareSize, j*squareSize, 0));
	}

	std::cout << "Finding intrinsic matrices...\n";
	//find intrinsic matrices
	cameraMatrix1 = initCameraMatrix2D(objectPoints, imagePoints1, imageSize, 0);
	cameraMatrix2 = initCameraMatrix2D(objectPoints, imagePoints2, imageSize, 0);


	std::cout << "Camera Matrix 1\n";
	std::cout << cameraMatrix1 << std::endl;
	std::cout << "Camera Matrix 2\n";
	std::cout << cameraMatrix2 << std::endl;


	std::cout << "Running stereo calibration ...\n";
	std::cout << "Can take up to 10 minutes depending on frame count, please be patient!\n";



	//stereo calibration
	double rms = stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
		cameraMatrix1, distCoeffs1,
		cameraMatrix2, distCoeffs2,
		imageSize, R, T, E, F,
		cv::CALIB_FIX_ASPECT_RATIO +
		cv::CALIB_ZERO_TANGENT_DIST +
		cv::CALIB_USE_INTRINSIC_GUESS +
		cv::CALIB_SAME_FOCAL_LENGTH +
		cv::CALIB_RATIONAL_MODEL +
		cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

	std::cout << "Stereo Calibration finished: RMS error= " << rms << std::endl;

	//Saving intrinsics
	std::cout << "Saving intrinsic parameters..." << std::endl;
	fs.open("intrinsics.yml", cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix1 << "D1" << distCoeffs1 <<
			"M2" << cameraMatrix2 << "D2" << distCoeffs2;
		fs.release();
	}
	else
		std::cout << "Error: can not save the intrinsic parameters\n";

	//Stereo Rectification
	std::cout << "Calculating rectification transforms..." << std::endl;
	stereoRectify(cameraMatrix1, distCoeffs1,
		cameraMatrix2, distCoeffs2,
		imageSize, R, T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi1, &validRoi2);

	//Saving extrinsics
	fs.open("extrinsics.yml", cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q << "E" << E << "F" << F;
		fs.release();
	}
	else
		std::cout << "Error: can not save the extrinsic parameters\n";


	return 0;
}

int main(int argc, char *argv[])
{
	// Capture 3D coordinates using 2D images
	return cameraCalibration();
}