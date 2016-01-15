# Brief
This project is an AR demo by using OpenCV. I design an box man 3D model and display it
on the scene capture by the webcam.

To make this model as a reality. We need the information given by detecting the AR tag, which 
should be loaded into pattern library in advance. After detecting the pattern, then we can find 
the extrinsic parameter of camera. So we can transform the 3D model to 2D and draw it on image plane. 

Here is the [demo video](https://www.youtube.com/watch?v=kr7XexDnWBU).

# Installation
You should install Visual Studio before you build this project.
1. Open \build\vs2013\MyARdemo.sln
2. Press **F5** to build and run.

If you just want to execute the program, ust open \Release\MyARdemo.exe.
