--------------------README----------------------
-----------OPENPOSE 3D KEYPOINT CAPTURE---------
------------- Aaron Schmitz, 8/24/20------------
Prerequisites:
OPENPOSE with Visual Studio 2017
MATLAB
	For MATLAB integration, see
	https://www.mathworks.com/matlabcentral/answers/348269-how-do-i-set-up-microsoft-visual-studio-2017-for-slrt
------------------------------------------------
This program is designed to capture 2D keypoint data from two USB webcams,
identify the specified keypoints using OpenPose, and triangulate 3D coordinates
with a MATLAB integration.

This version is not yet feature-complete:
	It currently captures frames from two webcams, finds the desired keypoints using
Openpose via a shared directory, and translates specified keypoints to MATLAB-friendly
arrays for use with the triangulate function.
	Missing features are the inclusion of the cameraParams MATLAB object file, which is 
required for the use of triangulate and is generated externally using the
Stereo Camera Calibrator MATLAB application.

I strongly recommend looking at the program itself in addition to this readme,
taking note of the in-line comments to better understand the function of each component.
In additon, I recommend overwriting an OpenPose example project to avoid formatting a new project to
match OpenPose's requirements.
NOTE: apologies for the mess!
--------------------CAPTURE---------------------
In order to circumvent the limitations of OpenPose, OpenCV was used directly to capture
frames from each webcam simultaneously. These were saved as jpg files to a shared directory to a
specified filepath for each camera. Each iteration of the main loop retrieves the last frame
captured from each camera and overwrites the jpg file associated with the appropriate camera.
--------------OpenPose Integration -------------
OpenPose is configured to look at this shared directory of jpg files instead of a webcam to
avoid the limitations of the program in a simple manner. OpenPose finds the keypoints of the
jpg files in the directory and identifies which image is from which camera using the filepath.
Each camera's keypoints are saved to a specified array for further use, which is overwritten each loop.
Additionally, the confidence values for each keypoint are checked, and must be greater than
the specified bound for the keypoints to be used.
The total number of keypoints, as well as the specified keypoints, can be edited depending on use.
NOTE: The current setup uses pixel measurements, but a flag can be used to normalize keypoints
to a 0-1 scale if required.
----------------MATLAB Integration--------------
Once the keypoints are captured, they are saved to a MATLAB friendly array.
NOTE: MATLAB array declarations do not accept int values, and so must be manually editied. To be improved.
These MATLAB arrays are matched in accordance with the keypoint selection matrix for use with triangulate.
The unused triangulate function calls the associated MATLAB script to output a 3 by X matrix of
world coordinates. 
The last roadblock to a functional prototype is integrating the externally generated
cameraParams object from the MATLAB Stereo Camera Calibration app.