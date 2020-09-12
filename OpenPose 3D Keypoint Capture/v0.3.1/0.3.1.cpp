//OpenPose 3D Keypoint Capture
//Aaron Schmitz
//Current Version: 0.3.0


// Third-party dependencies
#include <opencv2/opencv.hpp>

// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY

// OpenPose dependencies
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>

// Custom OpenPose flags
// Producer
DEFINE_string(image_dir, "dualcameras/",
	"Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).");
// Display
DEFINE_bool(no_display, false,
	"Enable to disable the visual display.");



//string dependency
#include <string>

//User Options
bool display2DCoords = false;
bool calibrate = false;
bool body = false;
bool handL = false;
bool handR = true;

//define which keypoints to find 3D coordinates
std::array<int, 3> CVBodyKeypoints = {0, 16, 17};
int bodyKeypointNum = CVBodyKeypoints.size();		//note: keypointNum must be manually entered into the MATLAB array definitions
//define hand keypoints
std::array<int, 3> CVHandKeypoints = { 0, 8, 4 };
int handKeypointNum = CVHandKeypoints.size();

//lower confidence bound to ignore keypoints
double confidenceBound = 0.1;

//Definition of checkerboard used in cameraCalibration
int boardWidth = 24;
int boardHeight = 13;
float squareSize = 0.5;			//inches
//Number of frames to process for camera calibration
int numFrames = 100;
cv::Size boardSize(boardWidth, boardHeight);

//Tracking number of people to prevent errors w/null values
int numberPeopleDetected;
int numberHandLDetected;
int numberHandRDetected;

//declaration of cameras for dualCameraCapture
cv::VideoCapture camera1;
cv::VideoCapture camera2;

//openCV double array definitions
cv::Mat camera1CV(bodyKeypointNum, 2, CV_64F);
cv::Mat camera2CV(bodyKeypointNum, 2, CV_64F);
cv::Mat camera1CVL(bodyKeypointNum, 2, CV_64F);
cv::Mat camera2CVL(bodyKeypointNum, 2, CV_64F);
cv::Mat camera1CVR(bodyKeypointNum, 2, CV_64F);
cv::Mat camera2CVR(bodyKeypointNum, 2, CV_64F);


//UndistortPoints Matrices
cv::Mat camera1Distorted(bodyKeypointNum, 1, CV_64FC2);
cv::Mat camera2Distorted(bodyKeypointNum, 1, CV_64FC2);
cv::Mat camera1Undistorted(bodyKeypointNum, 1, CV_64FC2);
cv::Mat camera2Undistorted(bodyKeypointNum, 1, CV_64FC2);
cv::Mat camera1toTriangulate(bodyKeypointNum, 1, CV_64FC2);
cv::Mat camera2toTriangulate(bodyKeypointNum, 1, CV_64FC2);

//UndistortPoints Hand Matrices
cv::Mat camera1DistortedH(handKeypointNum, 1, CV_64FC2);
cv::Mat camera2DistortedH(handKeypointNum, 1, CV_64FC2);
cv::Mat camera1UndistortedH(handKeypointNum, 1, CV_64FC2);
cv::Mat camera2UndistortedH(handKeypointNum, 1, CV_64FC2);
cv::Mat camera1toTriangulateH(handKeypointNum, 1, CV_64FC2);
cv::Mat camera2toTriangulateH(handKeypointNum, 1, CV_64FC2);

//4D keypoint matrix
cv::Mat points4D(bodyKeypointNum, 4, CV_64F);
cv::Mat points4DH(bodyKeypointNum, 4, CV_64F);
cv::Mat points3D(bodyKeypointNum, 3, CV_64F);
cv::Mat points3DL(handKeypointNum, 3, CV_64F);
cv::Mat points3DR(handKeypointNum, 3, CV_64F);

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

//filepath for saved jpegs (change for your system if necessary). These act as constantly updated video feeds in the form of a distinct jpg file for each camera used.
std::string filepath1 = "./dualcameras/camera1.jpg";
std::string filepath2 = "./dualcameras/camera2.jpg";

//declare frame vars for openCV capture
cv::Mat frame1;
cv::Mat frame2;

//declare arrays for captured keypoints
op::Array<double> camera1body;
op::Array<double> camera1handL;
op::Array<double> camera1handR;
op::Array<double> camera2body;
op::Array<double> camera2handL;
op::Array<double> camera2handR;

double x, y, z, w;

// This worker will just read and return all the jpg files in a directory
bool display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
	try
	{

		// User's displaying/saving/other processing here
			// datum.cvOutputData: rendered frame with pose or heatmaps
			// datum.poseKeypoints: Array<float> with the estimated pose
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			// Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
			const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
			if (!cvMat.empty())
				cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
			else
				op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
		}
		else
			op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
		const auto key = (char)cv::waitKey(1);
		return (key == 27);
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
		return true;
	}
}

//saves body keypoints to array
void outputBodyKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr, op::Array<float> arr, int whichCamera)
{
	try
	{
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			arr = datumsPtr->at(0)->poseKeypoints;
			numberPeopleDetected = arr.getSize(0);
			for (int i = 0; i < bodyKeypointNum; i++)					//for each keypoint
			{
				int j = CVBodyKeypoints[i];							//get selected keypoints
				if (numberPeopleDetected != 0)						//check for null values to prevent errors
				{
					double score = arr[{ 0, j, 2 }];					//get confidence val
					//op::opLog(score, op::Priority::High);			//for testing
					if (score > confidenceBound)	//check confidence
					{
						switch (whichCamera)
						{
						default:
							op::opLog("Error: outputBodyKeypoint camera selection", op::Priority::High);
							exit(-1);
						case 0:
							camera1CV.at<double>(i, 0) = arr[{0, j, 0}];	//assign x val
							camera1CV.at<double>(i, 1) = arr[{0, j, 1}];	//assing y val
							if (display2DCoords == true)
							{
								op::opLog("--------------Camera 1 body ouptut!--------------", op::Priority::High);	//for testing
								op::opLog("Keypoint " + std::to_string(j), op::Priority::High);		//testing
								op::opLog("Confidence: " + std::to_string(score), op::Priority::High);
								op::opLog("X: " + std::to_string(camera1CV.at<double>(i, 0)), op::Priority::High);
								op::opLog("Y: " + std::to_string(camera1CV.at<double>(i, 1)), op::Priority::High);
							}
							break;
						case 1:
							camera2CV.at<double>(i, 0) = arr[{0, j, 0}];	//assign x val
							camera2CV.at<double>(i, 1) = arr[{0, j, 1}];	//assing y val
							if (display2DCoords == true)
							{
								op::opLog("--------------Camera 2 body ouptut!--------------", op::Priority::High);	//for testing
								op::opLog("Keypoint " + std::to_string(j), op::Priority::High);		//testing
								op::opLog("Confidence: " + std::to_string(score), op::Priority::High);
								op::opLog("X: " + std::to_string(camera2CV.at<double>(i, 0)), op::Priority::High);
								op::opLog("Y: " + std::to_string(camera2CV.at<double>(i, 1)), op::Priority::High);
							}
						}
					}
					else
						op::opLog("Low confidence at keypoint " + std::to_string(j), op::Priority::High);
				}
				else
					op::opLog("No bodies detected", op::Priority::High);
			}
		}
		else
			op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}

//for left hand
void outputHandLKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr, op::Array<float> arr, int whichCamera)
{
	try
	{
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			arr = datumsPtr->at(0)->handKeypoints[0];
			numberHandLDetected = arr.getSize(0);
			for (int i = 0; i < handKeypointNum; i++)					//for each keypoint
			{
				int j = CVHandKeypoints[i];							//get selected keypoints
				if (numberHandLDetected != 0)						//check for null values to prevent errors
				{
					double score = arr[{ 0, j, 2 }];					//get confidence val
					//op::opLog(score, op::Priority::High);			//for testing
					if (score > confidenceBound)	//check confidence
					{
						switch (whichCamera)
						{
						default:
							op::opLog("Error: outputHandLKeypoint camera selection", op::Priority::High);
							exit(-1);
						case 0:
							camera1CVL.at<double>(i, 0) = arr[{0, j, 0}];	//assign x val
							camera1CVL.at<double>(i, 1) = arr[{0, j, 1}];	//assing y val
							if (display2DCoords == true)
							{
								op::opLog("--------------Camera 1 left hand ouptut!--------------", op::Priority::High);	//for testing
								op::opLog("Keypoint " + std::to_string(j), op::Priority::High);		//testing
								op::opLog("Confidence: " + std::to_string(score), op::Priority::High);
								op::opLog("X: " + std::to_string(camera1CVL.at<double>(i, 0)), op::Priority::High);
								op::opLog("Y: " + std::to_string(camera1CVL.at<double>(i, 1)), op::Priority::High);
							}
							break;
						case 1:
							camera2CVL.at<double>(i, 0) = arr[{0, j, 0}];	//assign x val
							camera2CVL.at<double>(i, 1) = arr[{0, j, 1}];	//assing y val
							if (display2DCoords == true)
							{
								op::opLog("--------------Camera 2 left hand ouptut!--------------", op::Priority::High);	//for testing
								op::opLog("Keypoint " + std::to_string(j), op::Priority::High);		//testing
								op::opLog("Confidence: " + std::to_string(score), op::Priority::High);
								op::opLog("X: " + std::to_string(camera2CVL.at<double>(i, 0)), op::Priority::High);
								op::opLog("Y: " + std::to_string(camera2CVL.at<double>(i, 1)), op::Priority::High);
							}
						}
					}
					else
						op::opLog("Low confidence at keypoint " + std::to_string(j), op::Priority::High);
				}
				else
					op::opLog("No bodies detected", op::Priority::High);
			}
		}
		else
			op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}

//for right hand
void outputHandRKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr, op::Array<float> arr, int whichCamera)
{
	try
	{
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			arr = datumsPtr->at(0)->handKeypoints[1];
			numberHandRDetected = arr.getSize(0);
			for (int i = 0; i < handKeypointNum; i++)					//for each keypoint
			{
				int j = CVHandKeypoints[i];							//get selected keypoints
				if (numberHandRDetected != 0)						//check for null values to prevent errors
				{
					double score = arr[{ 0, j, 2 }];					//get confidence val
					//op::opLog(score, op::Priority::High);			//for testing
					if (score > confidenceBound)	//check confidence
					{
						switch (whichCamera)
						{
						default:
							op::opLog("Error: outputHandRKeypoint camera selection", op::Priority::High);
							exit(-1);
						case 0:
							camera1CVR.at<double>(i, 0) = arr[{0, j, 0}];	//assign x val
							camera1CVR.at<double>(i, 1) = arr[{0, j, 1}];	//assing y val
							if (display2DCoords == true)
							{
								op::opLog("--------------Camera 1 right hand ouptut!--------------", op::Priority::High);	//for testing
								op::opLog("Keypoint " + std::to_string(j), op::Priority::High);		//testing
								op::opLog("Confidence: " + std::to_string(score), op::Priority::High);
								op::opLog("X: " + std::to_string(camera1CVR.at<double>(i, 0)), op::Priority::High);
								op::opLog("Y: " + std::to_string(camera1CVR.at<double>(i, 1)), op::Priority::High);
							}
							break;
						case 1:
							camera2CVR.at<double>(i, 0) = arr[{0, j, 0}];	//assign x val
							camera2CVR.at<double>(i, 1) = arr[{0, j, 1}];	//assing y val
							if (display2DCoords == true)
							{
								op::opLog("--------------Camera 2 right hand ouptut!--------------", op::Priority::High);	//for testing
								op::opLog("Keypoint " + std::to_string(j), op::Priority::High);		//testing
								op::opLog("Confidence: " + std::to_string(score), op::Priority::High);
								op::opLog("X: " + std::to_string(camera2CVR.at<double>(i, 0)), op::Priority::High);
								op::opLog("Y: " + std::to_string(camera2CVR.at<double>(i, 1)), op::Priority::High);
							}
						}
					}
					else
						op::opLog("Low confidence at keypoint " + std::to_string(j), op::Priority::High);
				}
				else
					op::opLog("No bodies detected", op::Priority::High);
			}
		}
		else
			op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}

//captures frames from each camera and saves them as distinct jpg files.
void dualCameraCapture()
{
	camera1 >> frame1;
	camera2 >> frame2;

	//save frame as jpeg
	cv::imwrite(filepath1, frame1);
	cv::imwrite(filepath2, frame2);

}

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
		//test output
		//std::cout << corners1 << std::endl;
		//std::cout << corners2 << std::endl;
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
	std::cout << "Saving intrinsic parameters..." << rms << std::endl;
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
	std::cout << "Calculating rectification transforms..." << rms << std::endl;
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

//OpenPose configuration
void configureWrapper(op::Wrapper& opWrapper)
{
	try
	{
		// Configuring OpenPose

		// logging_level
		op::checkBool(
			0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
			__LINE__, __FUNCTION__, __FILE__);
		op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
		op::Profiler::setDefaultX(FLAGS_profile_speed);

		// Applying user defined configuration - GFlags to program variables
		// outputSize
		const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
		// netInputSize
		const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
		// faceNetInputSize
		const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
		// handNetInputSize
		const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
		// poseMode
		const auto poseMode = op::flagsToPoseMode(FLAGS_body);
		// poseModel
		const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
		// JSON saving
		if (!FLAGS_write_keypoint.empty())
			op::opLog(
				"Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json`"
				" instead.", op::Priority::Max);
		// keypointScaleMode
		const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
		// heatmaps to add
		const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
			FLAGS_heatmaps_add_PAFs);
		const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
		// >1 camera view?
		const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
		// Face and hand detectors
		const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
		const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
		// Enabling Google Logging
		const bool enableGoogleLogging = true;

		// Pose configuration (use WrapperStructPose{} for default and recommended configuration)
		const op::WrapperStructPose wrapperStructPose{
			poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
			FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
			poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
			FLAGS_part_to_show, op::String(FLAGS_model_folder), heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
			(float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
			op::String(FLAGS_prototxt_path), op::String(FLAGS_caffemodel_path),
			(float)FLAGS_upsampling_ratio, enableGoogleLogging };
		opWrapper.configure(wrapperStructPose);
		// Face configuration (use op::WrapperStructFace{} to disable it)
		const op::WrapperStructFace wrapperStructFace{
			FLAGS_face, faceDetector, faceNetInputSize,
			op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
			(float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold };
		opWrapper.configure(wrapperStructFace);
		// Hand configuration (use op::WrapperStructHand{} to disable it)
		const op::WrapperStructHand wrapperStructHand{
			FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
			op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
			(float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold };
		opWrapper.configure(wrapperStructHand);
		// Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
		const op::WrapperStructExtra wrapperStructExtra{
			FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads };
		opWrapper.configure(wrapperStructExtra);
		// Output (comment or use default argument to disable any output)
		const op::WrapperStructOutput wrapperStructOutput{
			FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
			op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
			FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
			op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
			op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
			op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
			op::String(FLAGS_udp_port) };
		opWrapper.configure(wrapperStructOutput);
		// No GUI. Equivalent to: opWrapper.configure(op::WrapperStructGui{});
		// Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
		if (FLAGS_disable_multi_thread)
			opWrapper.disableMultiThreading();
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}

void triangulatePoints()
{
	if (body)
	{
		//get keypoints
		camera1Distorted = camera1CV;
		camera2Distorted = camera2CV;

		//undistort and correct points
		cv::undistortPoints(camera1Distorted, camera1Undistorted, cameraMatrix1, distCoeffs1, R1, P1);
		cv::undistortPoints(camera2Distorted, camera2Undistorted, cameraMatrix2, distCoeffs2, R2, P2);

		//triangulatePoints requires a different type of matrix to function
		camera1toTriangulate = camera1Undistorted;
		camera2toTriangulate = camera2Undistorted;

		//triangulate points
		cv::triangulatePoints(P1, P2, camera1toTriangulate, camera2toTriangulate, points4D);

		//for each keypoint
		std::cout << "---------------Body Keypoint Output-----------------" << std::endl;
		for (int i = 0; i < bodyKeypointNum; i++)
		{
			//retrieve coords (triangulatePoints turns matrix)
			int j = CVBodyKeypoints[i];
			x = points4D.at<double>(0, i);
			y = points4D.at<double>(1, i);
			z = points4D.at<double>(2, i);
			w = points4D.at<double>(3, i);

			//divide by fourth dimension to get x, y, and z 3D coords
			x = x / w;
			y = y / w;
			z = z / w;

			points3D.at<double>(0, i) = x;
			points3D.at<double>(1, i) = y;
			points3D.at<double>(2, i) = z;

			//Output coords
			op::opLog("Keypoint " + std::to_string(j), op::Priority::High);
			op::opLog("X: " + std::to_string(points3D.at<double>(0, i)), op::Priority::High);
			op::opLog("Y: " + std::to_string(points3D.at<double>(1, i)), op::Priority::High);
			op::opLog("Z: " + std::to_string(points3D.at<double>(2, i)), op::Priority::High);
		}
	}
	
	if (handL)
	{
		//Left Hand
		//get keypoints
		camera1DistortedH = camera1CVL;
		camera2DistortedH = camera2CVL;

		//undistort and correct points
		cv::undistortPoints(camera1DistortedH, camera1UndistortedH, cameraMatrix1, distCoeffs1, R1, P1);
		cv::undistortPoints(camera2DistortedH, camera2UndistortedH, cameraMatrix2, distCoeffs2, R2, P2);

		//triangulatePoints requires a different type of matrix to function
		camera1toTriangulateH = camera1UndistortedH;
		camera2toTriangulateH = camera2UndistortedH;

		//triangulate points
		cv::triangulatePoints(P1, P2, camera1toTriangulateH, camera2toTriangulateH, points4DH);

		//for each keypoint
		std::cout << "----------------Left Hand Keypoint Output-----------------" << std::endl;
		for (int i = 0; i < handKeypointNum; i++)
		{
			//retrieve coords (triangulatePoints turns matrix)
			int j = CVHandKeypoints[i];
			x = points4DH.at<double>(0, i);
			y = points4DH.at<double>(1, i);
			z = points4DH.at<double>(2, i);
			w = points4DH.at<double>(3, i);

			//divide by fourth dimension to get x, y, and z 3D coords
			x = x / w;
			y = y / w;
			z = z / w;

			points3DL.at<double>(0, i) = x;
			points3DL.at<double>(1, i) = y;
			points3DL.at<double>(2, i) = z;

			//Output coords
			op::opLog("Keypoint " + std::to_string(j), op::Priority::High);
			op::opLog("X: " + std::to_string(points3DL.at<double>(0, i)), op::Priority::High);
			op::opLog("Y: " + std::to_string(points3DL.at<double>(1, i)), op::Priority::High);
			op::opLog("Z: " + std::to_string(points3DL.at<double>(2, i)), op::Priority::High);
		}
	}
	if(handR)
	{
		//Right hand
		//get keypoints
		
		camera1DistortedH = camera1CVR;
		camera2DistortedH = camera2CVR;

		

		//undistort and correct points
		cv::undistortPoints(camera1DistortedH, camera1UndistortedH, cameraMatrix1, distCoeffs1, R1, P1);
		cv::undistortPoints(camera2DistortedH, camera2UndistortedH, cameraMatrix2, distCoeffs2, R2, P2);


		//triangulatePoints requires a different type of matrix to function
		camera1toTriangulateH = camera1UndistortedH;
		camera2toTriangulateH = camera2UndistortedH;

		//triangulate points
		cv::triangulatePoints(P1, P2, camera1toTriangulateH, camera2toTriangulateH, points4DH);

		

		//for each keypoint
		std::cout << "-------------Right Hand Keypoint Output--------------" << std::endl;
		for (int i = 0; i < handKeypointNum; i++)
		{
			//retrieve coords (triangulatePoints turns matrix)
			int j = CVHandKeypoints[i];
			x = points4DH.at<double>(0, i);
			y = points4DH.at<double>(1, i);
			z = points4DH.at<double>(2, i);
			w = points4DH.at<double>(3, i);

			//divide by fourth dimension to get x, y, and z 3D coords
			x = x / w;
			y = y / w;
			z = z / w;

			points3DR.at<double>(0, i) = x;
			points3DR.at<double>(1, i) = y;
			points3DR.at<double>(2, i) = z;

			//Output coords
			op::opLog("Keypoint " + std::to_string(j), op::Priority::High);
			op::opLog("X: " + std::to_string(points3DR.at<double>(0, i)), op::Priority::High);
			op::opLog("Y: " + std::to_string(points3DR.at<double>(1, i)), op::Priority::High);
			op::opLog("Z: " + std::to_string(points3DR.at<double>(2, i)), op::Priority::High);
		}
	}
}

//main function to capture 3D keypoints
int cameraCapture3D()
{
	try
	{
		//open video capture
		camera1.open(0, cv::CAP_ANY);
		camera2.open(1, cv::CAP_ANY);
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

		if (calibrate == true)
		{
			op::opLog("Attempting to calibrate cameras, please display checkerboard!", op::Priority::High);
			for (int i = 5; i > 0; i--)
			{
				op::opLog("Calibrating in " + std::to_string(i), op::Priority::High);
				_sleep(1000);
			}
			cameraCalibration();
		}

		//get intrinsic matrices
		fs.open("intrinsics.yml", cv::FileStorage::READ);
		if (fs.isOpened())
		{
			fs["M1"] >> cameraMatrix1;
			fs["D1"] >> distCoeffs1;
			fs["M2"] >> cameraMatrix2;
			fs["D2"] >> distCoeffs2;
			fs.release();
		}
		else
		{
			std::cout << "Error: cannot open intrinsics.yml file\n";
			exit(-1);
		}

		//get projection matrices
		fs.open("extrinsics.yml", cv::FileStorage::READ);
		if (fs.isOpened())
		{
			fs["R1"] >> R1;
			fs["R2"] >> R2;
			fs["P1"] >> P1;
			fs["P2"] >> P2;
			fs["E"] >> E;
			fs["F"] >> F;
			fs.release();
		}
		else
		{
			std::cout << "Error: cannot open extrinsics.yml file\n";
			exit(-1);
		}


		//display projection matrices for testing
		//std::cout << P1 << std::endl;
		//std::cout << P2 << std::endl;

		//initiate openPose
		op::opLog("Starting OpenPose demo...", op::Priority::High);
		const auto opTimer = op::getTimerInit();

		//Flags added
		if(handL || handR)
			FLAGS_hand = true;
		else
			FLAGS_hand = false;
		//FLAGS_write_json = true;    //unneccesary for current implementation, could be useful

		// Configuring OpenPose
		op::opLog("Configuring OpenPose...", op::Priority::High);
		op::Wrapper opWrapper{ op::ThreadManagerMode::Asynchronous };   //Set to asynchronous input
		configureWrapper(opWrapper);

		// Starting OpenPose
		op::opLog("Starting thread(s)...", op::Priority::High);
		opWrapper.start();

		// Read frames on directory
		const auto imagePaths = op::getFilesOnDirectory(FLAGS_image_dir, op::Extensions::Images);

		//loop until user presses escape
		while (true)
		{
			//capture frame from each camera
			dualCameraCapture();

			// Process and display images
			for (const auto& imagePath : imagePaths)	//for loop processses one frame from each camera before looping
			{
				const cv::Mat cvImageToProcess = cv::imread(imagePath);
				const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImageToProcess);
				auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);
				if (datumProcessed != nullptr)
				{
					//identify the camera associated with the frame processed using directory path
					//op::opLog(imagePath, op::Priority::High);		//used to test for correct path
					if (imagePath == "dualcameras\\camera1.jpg")	//two forward slashes = one literal forward slash
					{
						if(body)
							outputBodyKeypoints(datumProcessed, camera1body, 0);				//body
						if(handL)
							outputHandLKeypoints(datumProcessed, camera1handL, 0);				//handL
						if(handR)
							outputHandRKeypoints(datumProcessed, camera1handR, 0);				//handR
					}
					else if (imagePath == "dualcameras\\camera2.jpg")
					{
						if (body)
							outputBodyKeypoints(datumProcessed, camera2body, 1);				//body
						if (handL)
							outputHandLKeypoints(datumProcessed, camera2handL, 1);				//handL
						if (handR)
							outputHandRKeypoints(datumProcessed, camera2handR, 1);				//handR
					}
					else
					{
						op::opLog("Image Path Error", op::Priority::High);
						return(-1);
					}
					if (!FLAGS_no_display)	//openPose exit detection
					{
						const auto userWantsToExit = display(datumProcessed);
						if (userWantsToExit)
						{
							op::opLog("User pressed Esc to exit demo.", op::Priority::High);
							break;
						}
					}
				}
				else
					op::opLog("Image " + imagePath + " could not be processed.", op::Priority::High);
			}

			//triangulate keypoints
			if((numberPeopleDetected != 0 && body) || (numberHandLDetected !=0 && handL) || (numberHandRDetected !=0 && handR))
				triangulatePoints();

			

			//output frame
			cv::imshow("camera 1", frame1);
			cv::imshow("camera 2", frame2);

			//esc to exit for opencV
			char c = (char)cv::waitKey(25);
			if (c == 27)
				break;
		}
		// Measuring total time
		op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

		//dualcamera cleanup
		camera1.release();
		camera2.release();
		cv::destroyAllWindows();


		// Return
		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}
}

int main(int argc, char *argv[])
{
	// Parsing command line flags
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	// Capture 3D coordinates using 2D images
	return cameraCapture3D();
}
