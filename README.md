# 3D-Pose-Estimation
Utilizing OpenPose to estimate 2D pose and transform into 3D pose with stereo camera system. 

Send any questions to aaron.schmitz@cooper.edu

Current Team: Aaron and Amaan

---------------------------------------------------------------------------

Current Implementation includes:

-OpenCV stereo camera calibration (Aaron)

-Keypoint recovery using OpenPose and two USB cameras (Aaron)

-Triangulation using camera intrinsics/extrinsics and 2D keypoint data (Aaron)

-Multithreading to capture camera frames simultaneously (Amaan)

-Integrate multithreading with triangulation (Amaan)

-Easily selectable user options (Aaron)

---------------------------------------------------------------------------

To-Do:

-Create GUI with integrated user options (Aaron)

-Translate 3D coordinates to skeleton for visualization in Unity (Aaron)

-Add support to wirelessly send data to a Raspberry Pi or as a radio signal (Aaron)

-Translate 3D coordinates to skeleton for visualization in Blender (Amaan)

-Output 3D matrix and linked keypoints as JSON file (Amaan/Aaron)

---------------------------------------------------------------------------

Future Plans:

-Android and IOS application to capture frames from two phones instead of two USB webcams (Aaron)

-Create custom models to use instead of OpenPose using TensorFlow
