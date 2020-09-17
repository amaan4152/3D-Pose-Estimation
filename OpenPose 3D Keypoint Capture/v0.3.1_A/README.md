#Multithreaded Multicamera Support
The header file MultiCameraSys.hpp allows each camera object in a set of n cameras to be in a unique thread.

All individual camera threads run asynchronously and run at the same time.

To properly incorporate this multithreaded multicamera support, add the multicamera object into your Openpose code
such that the object is instantiated once. To retrieve the frames from thread #k associated with camera index #k,
grab frames from the object's inputData array where index #k corresponds with thread #k. From then on, you can proceed however
you want to once you have grabbed frames per camera. 
