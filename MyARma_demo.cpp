#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cameraparams.h"
#include "Mypatterndetector.h"
#include "Mymodel.h"

using namespace std;
using namespace cv;
using namespace myARma;

#define PAT_SIZE 64//equal to pattern_size variable (see below)
#define SAVE_VIDEO 0 //if true, it saves the video in "output.avi"
#define NUM_OF_PATTERNS 3// define the number of patterns you want to use

char* filename1="pattern1.png";//id=1
char* filename2="pattern2.png";//id=2
char* filename3="pattern3.png";//id=3

static int loadPattern(const char* , std::vector<cv::Mat>& , int& );

int main(int argc, char** argv){

	std::vector<cv::Mat> patternLibrary;
	std::vector<Pattern> detectedPattern;
	int patternCount=0;

	/*create patterns' library using rotated versions of patterns 
	*/
	loadPattern(filename1, patternLibrary, patternCount);
#if (NUM_OF_PATTERNS==2)
	loadPattern(filename, patternLibrary, patternCount);
#endif
#if (NUM_OF_PATTERNS==3)
	loadPattern(filename2, patternLibrary, patternCount);
	loadPattern(filename3, patternLibrary, patternCount);
#endif

	cout << patternCount << " patterns are loaded." << endl;	

	int norm_pattern_size = PAT_SIZE;
	double fixed_thresh = 40;
	double adapt_thresh = 5;//non-used with FIXED_THRESHOLD mode
	int adapt_block_size = 45;//non-used with FIXED_THRESHOLD mode
	double confidenceThreshold = 0.35;
	int mode = 2;//1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD

	PatternDetector myDetector( fixed_thresh, adapt_thresh, adapt_block_size, confidenceThreshold, norm_pattern_size, mode);

	CvCapture* capture = cvCaptureFromCAM(0);
	
	Mat imgMat;
	vector<Model> model;
	model.resize(1);
	int k=0 , orientation;
	while(1){ //modify it for longer/shorter videos
		
		IplImage* img = cvQueryFrame(capture);
		Mat imgMat = Mat(img);
		double tic=(double)cvGetTickCount();

		//run the detector
		myDetector.detect(imgMat, cameraMatrix, distortions, patternLibrary, detectedPattern); 
		
		//augment the input frame (and print out the properties of pattern if you want)
		for (unsigned int i =0; i<detectedPattern.size(); i++){
			//cout << "key=" << k << endl;
			//keyboard setting
			if (k == 115){ //back
				orientation = model[0].getOrientation();
				if (orientation == 3){
					model[0].move(1, 1);//X:0 Y:1 Z:2 sign
					model[0].moveHand(2,1);
					model[0].moveLeg(1,1);//X:0 Y:1 sign
				}
				else if (orientation == 4)
					model[0].turn(PI / 2);
				else if (orientation == 1)
					model[0].turn(PI );
				else if (orientation == 2)
					model[0].turn(-PI / 2);
			}
			if (k == 119){ //forward
				orientation = model[0].getOrientation();
				if (orientation == 1){
					model[0].move(1, -1);//X:0 Y:1 sign
					model[0].moveHand(2, 1);
					model[0].moveLeg(1,-1);
				}
				else if (orientation == 2)
					model[0].turn(PI / 2);
				else if (orientation == 3)
					model[0].turn(PI);
				else if (orientation == 4)
					model[0].turn(-PI / 2);
			}
			if (k == 97){ //left
				orientation = model[0].getOrientation();
				if (orientation == 2){
					model[0].move(0, -1);//X:0 Y:1 sign
					model[0].moveHand(2, 1);
					model[0].moveLeg(0,-1);
				}
				else if (orientation == 3)
					model[0].turn(PI / 2);
				else if (orientation == 4)
					model[0].turn(PI);
				else if (orientation == 1)
					model[0].turn(-PI / 2);
			}
			if (k == 100){ //rigth
				orientation = model[0].getOrientation();
				if (orientation == 4){
					model[0].move(0, 1);//X:0 Y:1 sign
					model[0].moveHand(2, 1);
					model[0].moveLeg(0,1);
				}
				else if (orientation == 1)
					model[0].turn(PI / 2);
				else if (orientation == 2)
					model[0].turn(PI);
				else if (orientation == 3)
					model[0].turn(-PI / 2);
			}
			if (k == 101){ //up
				model[0].moveHand(2,1);
				model[0].move(2, -1);//X:0 Y:1 Z:2 sign
			}
			if (k == 113){ //down
				model[0].moveHand(2,1);
				model[0].move(2, 1);//X:0 Y:1 sign
			}
			model[0].updateModel();
			//draw model
			detectedPattern.at(i).draw(imgMat, cameraMatrix, distortions, model[0].modelPts);
		}
		
		imshow("result", imgMat);
		k = cvWaitKey(100);
		//cout << "Press" << k << endl;
		

		detectedPattern.clear();
	}

	cvReleaseCapture(&capture);

	return 0;

}

int loadPattern(const char* filename, std::vector<cv::Mat>& library, int& patternCount){
	Mat img = imread(filename,0);
	
	if(img.cols!=img.rows){
		return -1;
		printf("Not a square pattern");
	}

	int msize = PAT_SIZE; 

	Mat src(msize, msize, CV_8UC1);
	Point2f center((msize-1)/2.0f,(msize-1)/2.0f);
	Mat rot_mat(2,3,CV_32F);
	
	resize(img, src, Size(msize,msize));
	Mat subImg = src(Range(msize/4,3*msize/4), Range(msize/4,3*msize/4));
	library.push_back(subImg);
	
	//Mat getRotationMatrix2D(Point2f center, double angle, double scale)
	rot_mat = getRotationMatrix2D( center, 90, 1.0);

	for (int i=1; i<4; i++){
		Mat dst= Mat(msize, msize, CV_8UC1);
		rot_mat = getRotationMatrix2D( center, -i*90, 1.0);
		warpAffine( src, dst , rot_mat, Size(msize,msize));
		Mat subImg = dst(Range(msize/4,3*msize/4), Range(msize/4,3*msize/4));
		library.push_back(subImg);	
		
	}

	patternCount++;
	return 1;
}

