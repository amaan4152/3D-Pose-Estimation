# 3D-Pose-Estimation
Utilizing OpenPose to estimate 2D pose and transform into 3D pose with stereo camera system. 
Send any questions to aaron.schmitz@cooper.edu
Current Team: Aaron and Amaan

Current Implementation includes:
-OpenCV stereo camera calibration
-Keypoint recovery using OpenPose and two USB cameras
-Triangulation using camera intrinsics/extrinsics and 2D keypoint data
-Multithreading to capture camera frames simultaneously

To-Do:
-Separate OpenPose dependency from core functions (Everyone)
-Integrate multithreading with triangulation (Amaan)
-Easily selectable user options (Aaron)
-Translate 3D coordinates to skeleton for visualization in Blender (Amaan)
-Translate 3D coordinates to skeleton for visualization in Unity (Aaron)
-Android and IOS application to capture frames from two phones instead of two USB webcams (Aaron)

Future Plans:
-Create custom models to use instead of OpenPose using TensorFlow
-Create GUI with integrated user options
