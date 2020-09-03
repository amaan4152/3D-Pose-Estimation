#ifndef MULTI_CAMERA_SYS_HPP
#define MULTI_CAMERA_SYS_HPP

//3rd party dependencies
#include <opencv2/opencv.hpp>
//necessary std headers
#include <iostream>
#include <thread>
#include <atomic>

//data to be accessed and modified by any producer threads
extern std::atomic<bool> threadBreak(false);
extern std::atomic<bool> mClosed(false);
cv::setNumThreads(0); //disables multi-threading for opencv

/*
reference code: https://putuyuwono.wordpress.com/2015/05/29/multi-thread-multi-camera-capture-using-opencv/

This class is to be used to generate an object that initialize all camera objects.
Each camera is then put on a seperate thread to retrieve frames from them.
*/

class MultiCam
{
public:
    std::vector<std::thread*> threads;
    std::vector<int> camsId;
    std::vector<cv::VideoCapture*> cams;
    std::vector<cv::Mat> inputData;

    MultiCam(std::vector<int> camIndicies);
    ~MultiCam();

private:

    void captureFrame(int index);
    void threadCams();

};




//implementation
MultiCam::MultiCam(std::vector<int> camIndicies):
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
            std::cerr << "No cams opened." << std::endl;
        }
    }
    //put each camera for frame retrieval on a seperate thread.
    //e.g. camera_A -> thread_A
    //     camera_B -> thread_B
    //     ...      ->    ...
    MultiCam::threadCams();
}
//after threads have finished grabbing frames, join them to main thread and release all camera objects.
MultiCam::~MultiCam()
{
    for(int r = 0; r < cams.size(); r++)
    {
        threads.at(r)->join();
        cams.at(r)->release();
    }
}

//capture frames from associated camera index (function to be threaded).
void MultiCam::captureFrame(int index)
{
    cv::VideoCapture *vidTemp;
    vidTemp = cams.at(index);
    unsigned long long mFc = 0ull;
    while(!mClosed.load())
    {
        cv::Mat frame;
        (*vidTemp) >> frame;
        if(frame.empty())
        {
            mClosed.store(true);
            std::cerr << "Empty frame detected, closing program." << std::endl;
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
void MultiCam::threadCams()
{
    for(int i = 0; i < cams.size(); i++)
    {
        std::cout << "--STARTING THREADS--" << std::endl;
        std::thread *t;
        t = new std::thread(&MultiCam::captureFrame, this, i);
        threads.push_back(t);
        std::cout << "Thread ID:" << threads.at(i)->get_id() << " being instantiated." << std::endl;
    }
    if(threads.size() != cams.size())
    {
        std::cerr << "PUSHING THREADS ERROR" << std::endl;
    }
}


#endif //MULTI_CAMERA_SYS_HPP
