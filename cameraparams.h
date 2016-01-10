#ifndef _CAMERAPARAMS_H
#define _CAMERAPARAMS_H
#include <opencv2/core/core.hpp>
using namespace cv;

	//CvMat* intrinsic = (CvMat*)cvLoad("intrinsic.xml");	
	//CvMat* distor = (CvMat*)cvLoad("distortion.xml");
	
	double camD[] = {6.7649431228632795e+02, 0., 3.8262188058832749e+02, 0.,
	5.9941193806780484e+02, 1.6894241981264270e+02, 0., 0., 1.};
	double distCoeffD[] = {5.5318827974857022e-02, -1.0129523116603711e+00,
	3.8895464611792836e-02, 2.5365684020675693e-02,
	2.6020235726385716e+00, 0., 0., 8.1013197871984710e-01};
	Mat cameraMatrix = Mat(3,3,CV_64FC1,camD);
	Mat distortions = Mat(5,1,CV_64FC1,distCoeffD);

	//Mat cameraMatrix = cvarrToMat(intrinsic);
	//Mat distortions = cvarrToMat(distor);



#endif


