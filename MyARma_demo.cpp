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

	CvCapture* capture = cvCaptureFromCAM(1);
	if (!capture){
		cout << "Using default camera" << endl;
		capture = cvCaptureFromCAM(0);
	}
	if (!capture){
		cout << "No camera detected" << endl;
	}
	Mat imgMat;
	vector<Model> model;
	model.resize(1);
	int k=0 , orientation;
	vector<int> order, posY;
	while(1){ //modify it for longer/shorter videos
		
		IplImage* img = cvQueryFrame(capture);
		Mat imgMat = Mat(img);
		Point3f target;

		//run the detector
		myDetector.detect(imgMat, cameraMatrix, distortions, patternLibrary, detectedPattern); 
		
		//augment the input frame (and print out the properties of pattern if you want)
		if (detectedPattern.size() != 0){
			for (unsigned int i = 0; i<model.size(); i++){
				//cout << "key=" << k << endl;
				//keyboard setting
				if (i == 0){//model by player
					switch (k){
					case 115://back
						model[i].goBack();
						break;
					case 119://forward
						model[i].goFoward();
						break;
					case 97://left
						model[i].goLeft();
						break;
					case 100://rigth
						model[i].goRight();
						break;
					case 101: //up
						model[i].goUp();
						break;
					case 113://down
						model[i].goDown();
						break;
					case 112:
						Model tmp;
						model.push_back(tmp);
						break;
					}

					if (k == 101 || k == 113){
						model[i].build();
					}
					if (model[i].modelPts.rows == 60){
						model[i].updateModel();
					}
					else
						model[i].build();
				}
				else{
					target = model[i-1].getCenter();
					orientation = model[i - 1].getOrientation();
					//target = model[0].getCenter();
					//cout << "target = "  << target << endl;
					model[i].goTo(target,orientation);
				}				
				//determine the drawing order by position in y axis
				if (i == 0){
					order.push_back(0);
				}
				else if (model[i - 1].getCenter().y >= model[i].getCenter().y){
					order.insert(order.begin(), i);
				}
				else if (model[i - 1].getCenter().y < model[i].getCenter().y){
					order.push_back(i);
				}
			}
			//draw in order
			for (unsigned int i = 0; i<model.size(); i++){
				//draw model in inverse order
				detectedPattern.at(0).draw(imgMat, cameraMatrix, distortions, model[order[i]].modelPts, order[i]);
			}
			order.clear();
		}	
		
		imshow("result", imgMat);
		k = cvWaitKey(30);
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

