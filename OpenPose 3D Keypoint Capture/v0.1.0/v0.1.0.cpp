//OpenPose 3D Keypoint Capture
//Aaron Schmitz
//Current Version: 0.1.0
// Third-party dependencies
#include <opencv2/opencv.hpp>
// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>

// Custom OpenPose flags
// Producer
DEFINE_string(image_dir, "dualcameras/",
	"Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).");
// Display
DEFINE_bool(no_display, false,
	"Enable to disable the visual display.");

//Unneccesary for implementation, but could be useful
#include <openpose/pose/wPoseExtractor.hpp>
#include <openpose/pose/poseParameters.hpp>


//declaration of cameras for dualCameraCapture
cv::VideoCapture camera1;
cv::VideoCapture camera2;

//string dependency
#include <string>

//Matlab include, path depends on MATLAB install
#include <C:/Program Files/MATLAB/R2019b/extern/include/MatlabEngine.hpp>
#include <C:/Program Files/MATLAB/R2019b/extern/include/MatlabDataArray.hpp>
//#include "mat.h"

//MATLAB Init (initialize engine and data array factory)
std::unique_ptr<matlab::engine::MATLABEngine> matlabPtr = matlab::engine::startMATLAB();
matlab::data::ArrayFactory factory;

//define which keypoints to find 3D coordinates
std::array<int, 3> matlabKeypoints = {0,16,17};
int keypointNum = matlabKeypoints.size();		//note: keypointNum must be manually entered into the MATLAB array definitions



//lower confidence bound to ignore keypoints
float confidenceBound = 0.1;


//MATLAB array definitions
matlab::data::TypedArray<float> camera1MATLAB = factory.createArray<float>({ 2,3 });
matlab::data::TypedArray<float> camera2MATLAB = factory.createArray<float>({ 2,3 });
auto worldcoords = factory.createArray<float>({ 3,3 });


const char *file = "C:/Users/Aaron Schmitz/Desktop/Double Camera Pics/cameraParams.mat";



//filepath for saved jpegs (change for your system). These act as constantly updated video feeds in the form of a distinct jpg file for each camera used.
std::string filepath1 = "C:/Users/Aaron Schmitz/Desktop/openpose-master/dualcameras/camera1.jpg";
std::string filepath2 = "C:/Users/Aaron Schmitz/Desktop/openpose-master/dualcameras/camera2.jpg";

//declare frame vars for openCV capture
cv::Mat frame1;
cv::Mat frame2;

//declare arrays for captured keypoints
op::Array<float> camera1body;
op::Array<float> camera1handL;
op::Array<float> camera1handR;
op::Array<float> camera2body;
op::Array<float> camera2handL;
op::Array<float> camera2handR;

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

//prints keypoints directly to console, heavily modified in outputBodyKeypoints
void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
	try
	{
		// Example: How to use the pose keypoints
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			op::opLog("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);
			op::opLog("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString(), op::Priority::High);
			op::opLog("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString(), op::Priority::High);
			op::opLog("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString(), op::Priority::High);
		}
		else
			op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}

//modified printKeypoints, saves body keypoints to array
void outputBodyKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr, op::Array<float> arr, int whichCamera)
{
	try
	{
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			arr = datumsPtr->at(0)->poseKeypoints;
			//op::opLog("Body Keypoints:");			//testing
			//op::opLog(arr.toString());
			const auto numberPeopleDetected = arr.getSize(0);
			for (int i = 0; i < keypointNum; i++)					//for each keypoint
			{
				int j = matlabKeypoints[i];							//get selected keypoints
				if (numberPeopleDetected != 0)						//check for null values to prevent errors
				{
					float score = arr[{ 0, j, 2 }];					//get confidence val
					op::opLog(score, op::Priority::High);			//for testing
					if (score > confidenceBound)	//check confidence
					{
						switch (whichCamera)
						{
						default:
							op::opLog("Error: outputBodyKeypoint camera selection", op::Priority::High);
							exit(-1);
						case 0:
							op::opLog("Keypoint " + std::to_string(j), op::Priority::High);		//testing
							camera1MATLAB[0][i] = arr[{0, j, 0}];	//assign x val
							camera1MATLAB[1][i] = arr[{0, j, 1}];	//assing y val
							op::opLog(camera1MATLAB[0][i], op::Priority::High);
							op::opLog(camera1MATLAB[1][i], op::Priority::High);
							break;
						case 1:
							op::opLog("Keypoint " + std::to_string(j), op::Priority::High);		//testing
							camera2MATLAB[0][i] = arr[{0, j, 0}];	//assign x val
							camera2MATLAB[1][i] = arr[{0, j, 1}];	//assing y val
							op::opLog(camera2MATLAB[0][i], op::Priority::High);
							op::opLog(camera2MATLAB[1][i], op::Priority::High);
						}
					}
					else
						op::opLog("Low confidence at keypoint " + i, op::Priority::High);
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

//for hand capture, similar to outputBodyKeypoints (outdated, see BodyKeypoints)
void outputHandKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr, op::Array<float> arrL, op::Array<float> arrR)
{
	try
	{
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			arrL = datumsPtr->at(0)->handKeypoints[0];
			arrR = datumsPtr->at(0)->handKeypoints[1];
			op::opLog("Left Hand Keypoints:");
			op::opLog(arrL.toString());
			op::opLog("Right Hand Keypoints:");
			op::opLog(arrR.toString());
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

	//output frame
	//cv::imshow("camera 1", frame1);
	//cv::imshow("camera 2", frame2);

	//save frame as jpeg
	cv::imwrite(filepath1, frame1);
	cv::imwrite(filepath2, frame2);

}

//MATLAB call to triangulate 3D points
void triangulate()
{
	//worldcoords = matlabPtr->feval<float>(u"triangulate", camera1MATLAB, camera2MATLAB, cameraParameters);
	op::opLog("----------test-------------", op::Priority::High);
	op::opLog(worldcoords[0][0], op::Priority::High);
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

		//open cameraParams file
		

		
		//initiate openPose
		op::opLog("Starting OpenPose demo...", op::Priority::High);
		const auto opTimer = op::getTimerInit();

		//Flags added
		FLAGS_hand = true;
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
						op::opLog("--------------Camera 1 ouptut!--------------", op::Priority::High);	//for testing
						outputBodyKeypoints(datumProcessed, camera1body, 0);							//body
						//outputHandKeypoints(datumProcessed, camera1handL, camera1handR);				//hand
					}
					else if (imagePath == "dualcameras\\camera2.jpg")
					{
						op::opLog("--------------Camera 2 ouptut!--------------", op::Priority::High);	//for testing
						outputBodyKeypoints(datumProcessed, camera2body, 0);							//body
						//outputHandKeypoints(datumProcessed, camera2handL, camera2handR);				//hand
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
					//triangulate();
				}
				else
					op::opLog("Image " + imagePath + " could not be processed.", op::Priority::High);
			}

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

		

		//MATLAB cleanup
		matlab::engine::terminateEngineClient();

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
