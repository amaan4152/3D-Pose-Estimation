#ifndef MULTI_CAMERA_SYS_HPP
#define MULTI_CAMERA_SYS_HPP

//3rd party dependencies
#include <opencv2/opencv.hpp>
//OpenPose dependencies
#include <openpose/headers.hpp>
//necessary std headers
#include <iostream>
#include <thread>
#include <atomic>

//data to be accessed and modified by any producer threads
extern std::atomic<bool> threadBreak(false);
extern std::atomic<bool> mClosed(false);

/*
reference code: https://putuyuwono.wordpress.com/2015/05/29/multi-thread-multi-camera-capture-using-opencv/

This class is to be used to generate an object that initialize all camera objects.
Each camera is then put on a seperate thread to retrieve frames from them.
*/

class multiCameraSys_TH
{
public:
    std::vector<std::thread*> threads;
    std::vector<int> camsId;
    std::vector<cv::VideoCapture*> cams;
    std::vector<cv::Mat> inputData;

    multiCameraSys_TH(std::vector<int> camIndicies);
    ~multiCameraSys_TH();

private:

    void captureFrame(int index);
    void threadCams();

};




//implementation
multiCameraSys_TH::multiCameraSys_TH(std::vector<int> camIndicies):
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
    multiCameraSys_TH::threadCams();
}
//after threads have finished grabbing frames, join them to main thread and release all camera objects.
multiCameraSys_TH::~multiCameraSys_TH()
{
    for(int r = 0; r < cams.size(); r++)
    {
        threads.at(r)->join();
        cams.at(r)->release();
    }
}

//capture frames from associated camera index (function to be threaded).
void multiCameraSys_TH::captureFrame(int index)
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
void multiCameraSys_TH::threadCams()
{
    for(int i = 0; i < cams.size(); i++)
    {
        std::cout << "--STARTING THREADS--" << std::endl;
        std::thread *t;
        t = new std::thread(&multiCameraSys_TH::captureFrame, this, i);
        threads.push_back(t);
        std::cout << "Thread ID:" << threads.at(i)->get_id() << " being instantiated." << std::endl;
    }
    if(threads.size() != cams.size())
    {
        op::error("PUSHING THREADS ERROR", __LINE__, __FUNCTION__, __FILE__);
    }
}





#endif //MULTI_CAMERA_SYS_HPP
