# Real-Time 3D Blender Model Synchronization with 3D Pose Data 
&nbsp;&nbsp;&nbsp;&nbsp;This module containes demo videos and scripts for real-time configuration between 3D pose estimation data, wrapped in JSON, and an arbitrary model in Blender. 

## 3D Pose Data
&nbsp;&nbsp;&nbsp;&nbsp;The data is the output of the 3D pose estimation model being utilized (_OpenPose is the current model_). Currently, the data has to be packaged in a specific JSON format for the Blender python script to read the file appropriately. The JSON format follows a strict layout in order for compliance with current python script:   
  - _JSON Object_ = Pose keypoint \# (in accordance to selected formate: **Body25 in this case**)
  - _JSON Array_ = ( _label_ = "xyz",  _array_ = \[**x**, **y**, **z**] )

The JSON formatting can be adjusted as needed (extra data can be packaged as well such as frame number), but the code portion in the python script for reading the JSON file has to be adjusted accordingly.    

  Example of JSON 3D pose data: 

```JSON
{
	"frame_num":288,
	
	"0":{
		"xyz":
		[
			13.8075,-2.73637,-1.09852
		]
	},
	"1":{
		"xyz":
		[
			13.8075,-2.73637,-1.09852
		]
	},
	"2":{
		"xyz":
		[
			13.8075,-2.73637,-1.09852
		]
	}
}
```

## Piping 3D Pose Data into Blender
&nbsp;&nbsp;&nbsp;&nbsp;The python script `poseMappingTemp_v3.0.py` reads the JSON file as it is continuously updated while the stereo cameras are video capturing. Subtle latency persists between the instance a frame is pushed into OpenPose and the JSON data is updated, thus the script handles "blank" JSON data appropriately. 
### Joint-Angle Estimation 
&nbsp;&nbsp;&nbsp;&nbsp;Updating the pose of an arbitrary model in Blender can be accomplished in two ways: **joint-coordinate updating** or **joint-angle updating**. The python script handles the latter case primarily due to its ease in calculation. Joint-angle estimation can be accomplished with either Euler angles or quaternions; Eurler angles introduce the gimbal lock issue causing uncessary complications, whereas quaternions are computationally more efficient and introduce no gimbal lock issue (quaternions >> euler angles :) ). This script computes the quaternion of each joint-angle, and performs linear interpolation for smooth rotational transitions; local coordinate axis orientations are accounted for as well to ensure joint-angle estimation is accomplished appropriaptely to yield correct rotations. 
