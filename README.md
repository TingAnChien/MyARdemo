# Brief
This project is an simple AR demo by using OpenCV. I draw an little box man and display it
on the scene capture by the webcam.

To create an augmented reality, we need an reference such as an AR tag. Then we use the 
information given by the detection of the tag, which should be loaded into pattern library in 
advance. After detecting the pattern, then we can find the extrinsic parameter of camera. 
So we can transform the 3D model to 2D and draw it on image plane. 

Here is the [demo video](https://www.youtube.com/watch?v=kr7XexDnWBU).

# Installation
You should install Visual Studio before you build this project.

 1. Open `\build\vs2013\MyARdemo.sln`
 2. Press *F5* to build and run.

Or you can just execute the program `\Release\MyARdemo.exe`.
