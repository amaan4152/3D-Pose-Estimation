// ------------------------- OpenPose C++ API Tutorial - Example 10 - Custom Input -------------------------
// Asynchronous mode: ideal for fast prototyping when performance is not an issue.
// In this function, the user can implement its own way to create frames (e.g., reading his own folder of images)
// and emplaces/pushes the frames to OpenPose.

// Third-party dependencies
#include <opencv2/opencv.hpp>
// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>

//necessary headers
#include <thread>
#include <iostream>
#include <fstream>
#include <atomic>
#include <chrono>

//data to be accessed and modified by any producer threads
std::atomic<bool> threadBreak(false);
std::atomic<bool> mClosed(false);

/*
reference code: https://putuyuwono.wordpress.com/2015/05/29/multi-thread-multi-camera-capture-using-opencv/

This class is to be used to generate an object that initialize all camera objects.
Each camera is then put on a seperate thread to retrieve frames from them.
*/
class inputObj
{
public:

    std::vector<std::thread*> threads;
    std::vector<int> camsId;
    std::vector<cv::VideoCapture*> cams;
    std::vector<cv::Mat> inputData;

    inputObj(std::vector<int> camIndicies):
        camsId{camIndicies}
    {
        //initialize cameras
        inputData.resize(camsId.size());
        for(int i = 0; i < camsId.size(); i++)
        {
            cv::VideoCapture *v;
            v = new cv::VideoCapture(camsId.at(i));
            cams.push_back(v);
            if(!(cams.at(i)->isOpened()))
            {
                mClosed.store(true);
                op::error("No cams opened.", __LINE__, __FUNCTION__, __FILE__);
            }
        }

        //put each camera for frame retrieval on a seperate thread.
        //e.g. camera_A -> thread_A
        //     camera_B -> thread_B
        //     ...      ->    ...
        threadCams();
    }

    //after threads have finished grabbing frames, join them to main thread and release all camera objects.
    ~inputObj()
    {
        for(int r = 0; r < cams.size(); r++)
        {
            threads.at(r)->join();
            std::cout << "--RELEASE BEGIN--" << std::endl;
            cams.at(r)->release();
            std::cout << "--RELEASE SUCCESS--" << std::endl;
        }
    }



private:
    //capture frames from associated camera index (function to be threaded).
    void captureFrame(int index)
    {
        cv::VideoCapture *vidTemp;
        vidTemp = cams.at(index);
        unsigned long long mFc = 0ull;
        while(!mClosed.load())
        {
            cv::Mat frame;
            std::cout << "--INSIDE THREAD--" << std::endl;
            (*vidTemp) >> frame;
            if(frame.empty())
            {
                mClosed.store(true);
                op::error("Empty frame detected, closing program.", __LINE__, __FUNCTION__, __FILE__);
            }
            else
            {
                inputData.at(index) = frame;
                frame.release();
                ++mFc;
            }
            if(threadBreak.load())
            {
                std::cout << "--EXITING THREAD LOOP--" << std::endl;
                break;
            }
        }
        (*vidTemp).release();
    }

    //generate threads to this object
    void threadCams()
    {
        for(int i = 0; i < cams.size(); i++)
        {
            std::cout << "--STARTING THREADS--" << std::endl;
            std::thread *t;
            t = new std::thread(&inputObj::captureFrame, this, i);
            threads.push_back(t);
            std::cout << "Thread ID:" << threads.at(i)->get_id() << " being instantiated." << std::endl;
        }
        if(threads.size() != cams.size())
        {
            op::error("PUSHING THREADS ERROR", __LINE__, __FUNCTION__, __FILE__);
        }
    }
};



class generator_3D
{
public:

    generator_3D(const std::string & cam_params_path)
    {
        paramReader.readParameters(cam_params_path);
        cameraProjMatrix = paramReader.getCameraMatricies();
        cameraIntrinsics = paramReader.getCameraIntrinsics();
        cameraDistCoeffs = paramReader.getCameraDistortions();

        for(int i = 0; i < cameraProjMatrix.size(); i++)
        {

        }
    }

    void getKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> & datumsPtr)
    {
        if(datumsPtr != nullptr && !datumsPtr->empty())
        {

            const auto & arrayBody = datumsPtr->at(0)->poseKeypoints;
            row_size = arrayBody.getSize(1);
            //assume 1 person
            //iterate through each body part
            for(int part = 0; part < row_size; part++)
            {
                //get (x,y)
                keypoints.push_back(cv::Point2f(arrayBody[{0,part,0}], arrayBody[{0,part,1}]));

            }
        }
        else
        {
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
        }

    }

    void

private:
    op::CameraParameterReader paramReader;
    std::vector<cv::Point2f> keypoints;
    const std::vector<op::Matrix> & cameraProjMatricies;
    const std::vector<op::Matrix> & cameraIntrinsics;
    const std::vector<op::Matrix> & cameraDistCoeffs;
};




void printKeypoints(int index, const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    if(datumsPtr != nullptr && !datumsPtr->empty())
    {
        op::opLog("\n----------CAMERA " + std::to_string(index) + "----------\n");
        // Accesing each element of the keypoints
        op::opLog("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);
        op::opLog("Body keypoints: " + std::to_string(datumsPtr->at(0)->poseKeypoints.getNumberDimensions()), op::Priority::High);
    }
    else
    {
        op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
}





//create a datumsPtr for a camera
std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> createDatum(const cv::Mat & cvInputData)
{
    // Close program when empty frame
    if (mClosed.load())
    {
        op::opLog("Last frame read and added to queue. Closing program after it is processed.", op::Priority::High);
        // This funtion stops this worker, which will eventually stop the whole thread system once all the frames
        // have been processed
        mClosed.store(true);
        return nullptr;
    }
    else
    {
        // Create new datum
        auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
        datumsPtr->emplace_back();
        auto& datumPtr = datumsPtr->at(0);
        datumPtr = std::make_shared<op::Datum>();

        // Fill datum
        datumPtr->cvInputData = OP_CV2OPCONSTMAT(cvInputData);

        // If empty frame -> return nullptr
        if (datumPtr->cvInputData.empty())
        {
            op::opLog("Empty frame detected. Closing program.",
                    op::Priority::High);
            mClosed.store(true);
            datumsPtr = nullptr;
        }

        return datumsPtr;
    }
}

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
            (float)FLAGS_upsampling_ratio, enableGoogleLogging};
        opWrapper.configure(wrapperStructPose);
        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{
            FLAGS_face, faceDetector, faceNetInputSize,
            op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
            (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold};
        opWrapper.configure(wrapperStructFace);
        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{
            FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
            op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
            (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold};
        opWrapper.configure(wrapperStructHand);
        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads};
        opWrapper.configure(wrapperStructExtra);
        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
            op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
            FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
            op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
            op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
            op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
            op::String(FLAGS_udp_port)};
        opWrapper.configure(wrapperStructOutput);
        // GUI (comment or use default argument to disable any visual output)
        const op::WrapperStructGui wrapperStructGui{
            op::flagsToDisplayMode(FLAGS_display, FLAGS_3d), !FLAGS_no_gui_verbose, FLAGS_fullscreen};
        opWrapper.configure(wrapperStructGui);
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}


int main(int argc, char *argv[])
{

    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    try
    {
        op::opLog("Starting OpenPose demo...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Required flags to enable 3-D
        FLAGS_3d = false;
        FLAGS_number_people_max = 1;
        FLAGS_3d_min_views = 2;
        FLAGS_output_resolution = "400x320"; // Optional, but otherwise it gets too big to render in real time
        // FLAGS_3d_views = X; // Not required because it only affects OpenPose producers (rather than custom ones)

        // Configuring OpenPose
        op::opLog("Configuring OpenPose...", op::Priority::High);
        //must be Asynchronous to enable asynch input for multi cam frame capture
        //and asynch output for retrieving keypoint data from prcessed datumsPtr
        op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
        configureWrapper(opWrapper);

        // Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
        op::opLog("Starting thread(s)...", op::Priority::High);
        opWrapper.start();


        //set up all user objects
        std::vector<int> cameras{0, 1};
        inputObj inputReciever = inputObj(cameras);
        std::vector<cv::Mat> output;
        bool userWantsToExit = false;
        //begin user process to retrieve captured frames from each user thread
        while(!userWantsToExit && !mClosed.load())
        {

            if (!opWrapper.isRunning())
            {
                op::opLog("OpenPose wrapper is no longer running, exiting video.", op::Priority::High);
                break;
            }
            for(int c = 0; c < cameras.size(); c++)
            {
                cv::Mat f;
                //get frame from camera with index c from its associated thread
                f = inputReciever.inputData.at(c);
                if(!f.empty())
                {
                    //create datumsPtr for camera
                    //auto datumToProcess = createDatum(f);
                    auto datumToProcess = opWrapper.emplaceAndPop(OP_CV2OPCONSTMAT(f));
                    printKeypoints(c, datumToProcess);
                    /*
                    if (datumToProcess != nullptr)
                    {
                        //emplace datumsPtr to OpenPose
                        auto successfullyEmplaced = opWrapper.waitAndEmplace(datumToProcess);
                        if (!successfullyEmplaced)
                            op::opLog("Processed datum could not be emplaced.", op::Priority::High);
                    }
                    */
                }
                else
                {
                    std::cout << "EMPTY FRAME RECIEVED!" << std::endl;
                }
            }
            //for thread saftey
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        }
        std::cout << "EXITED" << std::endl;
        //cause all user threads to break out of infintie while loop to capture frames
        threadBreak.store(true);
        //release all cameras
        inputReciever.~inputObj();



        op::opLog("Stopping thread(s)", op::Priority::High);
        opWrapper.stop();

        // Measuring total time
        op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);
        return 0;
    }
    catch (const std::exception&)
    {
        return -1;
    }


    // Running tutorialApiCpp
    //return tutorialApiCpp(inputReciever.mGetFrame(), inputReciever.getFrameCount(), inputReciever.isFinished());
}
