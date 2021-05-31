## Team
  **3D Pose Estimation Team:**  
&nbsp;&nbsp;&nbsp;&nbsp;[Amaan Rahman](mailto:amaan.rahman@cooper.edu)  
&nbsp;&nbsp;&nbsp;&nbsp;[Aaron Schmitz](mailto:aaron.schmitz@cooper.edu)  
    
  **Jetson Nano Team:**  
&nbsp;&nbsp;&nbsp;&nbsp;Alexa Jakob  

  **Annotations Team:**  
&nbsp;&nbsp;&nbsp;&nbsp;Lucia Rhode  
&nbsp;&nbsp;&nbsp;&nbsp;Esther Wang  
    
## Problem Statement
  Develop a means to estimate an individual's pose optimally and efficiently with an arbitrary low-cost stereo camera setup. 
 
## Accomplishments
  - Developed a triangulation algorithm to determine depth coordinate given 2D pose estimation output from OpenPose
  - Developed script to support _N_ camera synchronization with OpenCV
  - Incorporated an in-built calibration feature within source code
  - Real-time 3D model synchronization with video feed via joint-angle estimation with 3D joint data

## Current Status
  - Fixing calibration bugs
  - Developing Unity support with 3D joint data
  - Starting custom 3D pose estimation model development

## Goals
  - Develop portable low-cost kit for 3D pose estimation algorithms
  - Apply 3D pose estimation model to VR/AR and computer animation applications
  - Develop iOS and Android support
  - Migrate from stereo to monocular camera setup
