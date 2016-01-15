#include "Mypatterndetector.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

namespace myARma
{
	PatternDetector::PatternDetector(const double param1, const double param2, const int param3, const double param4, const int param5, const int thresh_mode)
{
	thresh1 = param1;//for image thresholding
	thresh2 = param2;// for adaptive thresholding
	block_size = param3;//for adaptive image thresholding
	mode = thresh_mode;//1 means fixed threshol, otherwise the adaptive algorithm is enabled
	confThreshold = param4;//bound for accepted similarities between detected patterns and loaded patterns
	normSize = param5;// the size of normalized ROI 
	normROI = Mat(normSize,normSize,CV_8UC1);//normalized ROI
	
	//Masks for exterior(black) and interior area inside the pattern
	patMask = Mat::ones(normSize,normSize,CV_8UC1);
	Mat submat = patMask(cv::Range(normSize/4,3*normSize/4), cv::Range(normSize/4, 3*normSize/4));
	submat = Scalar(0);
	
	patMaskInt = Mat::zeros(normSize,normSize,CV_8UC1);
	submat = patMaskInt(cv::Range(normSize/4,3*normSize/4), cv::Range(normSize/4, 3*normSize/4));
	submat = Scalar(1);


	//corner of normalized area
	norm2DPts[0] = Point2f(0,0);
	norm2DPts[1] = Point2f(normSize-1,0);
	norm2DPts[2] = Point2f(normSize-1,normSize-1);
	norm2DPts[3] = Point2f(0,normSize-1);

}

void PatternDetector::detect(const Mat& frame, const Mat& cameraMatrix, const Mat& distortions, vector<Mat>& library, vector<Pattern>& foundPatterns)
{

	patInfo out;
	Point2f roi2DPts[4];
	Mat binImage2;

	//binarize image
	convertAndBinarize(frame, binImage, grayImage, mode);
	binImage.copyTo(binImage2);

	int avsize = (binImage.rows+binImage.cols)/2;

	vector<vector<Point>> contours;
	vector<Point> polycont;

	//find contours in binary image
	cv::findContours(binImage2, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	unsigned int i;
	Point p;
	int pMinX, pMinY, pMaxY, pMaxX;

	for(i=0; i<contours.size(); i++){
		Mat contourMat = Mat (contours[i]);
		//找輪廓的長度，去跟平均的長度比較，太大太小的都不考慮
		const double per = arcLength( contourMat, true);
		//check the perimeter
		if (per>(avsize/4) && per<(4*avsize)) {
			polycont.clear();
			//利用approxPolyDP找出多邊形的角，不是四邊形的就不考慮
			//另外底下的isContourConvex確認是否為凸多邊形，若為凹多邊形則不考慮
			approxPolyDP( contourMat, polycont, per*0.02, true);
			//check rectangularity and convexity
			if (polycont.size()==4 && isContourConvex(Mat (polycont))){
				//locate the 2D box of contour,
				//利用polycont找到的四個點，抓左上與右下來框矩形
				p = polycont.at(0);
				pMinX = pMaxX = p.x;
				pMinY = pMaxY = p.y;
				int j;
				for(j=1; j<4; j++){
					p = polycont.at(j);
					if (p.x<pMinX){
						pMinX = p.x;
						}
					if (p.x>pMaxX){
						pMaxX = p.x;
						}
					if (p.y<pMinY){
						pMinY = p.y;
						}
					if (p.y>pMaxY){
						pMaxY = p.y;
						}
				}
				Rect box(pMinX, pMinY, pMaxX-pMinX+1, pMaxY-pMinY+1);
				Mat img = frame.clone();
				rectangle(img,box,Scalar(0,0,255),3,8);
				//find the upper left vertex
				double d;
				double dmin=(4*avsize*avsize);
				int v1=-1;
				for (j=0; j<4; j++){
					d = norm(polycont.at(j));
					if (d<dmin) {
					dmin=d;
					v1=j;
					}
				}

				//store vertices in refinedVertices and enable sub-pixel refinement if you want
				vector<Point2f> refinedVertices;
				refinedVertices.clear();
				for(j=0; j<4; j++){
					refinedVertices.push_back(polycont.at(j));
				}
				
				//refine corners
				cornerSubPix(grayImage, refinedVertices, Size(3,3), Size(-1,-1), TermCriteria(1, 3, 1));				
				
				//rotate vertices based on upper left vertex; this gives you the most trivial homogrpahy 
				for(j=0; j<4;j++){
					roi2DPts[j] = Point2f(refinedVertices.at((4+v1-j)%4).x - pMinX, refinedVertices.at((4+v1-j)%4).y - pMinY);
				}
				//把拍到的圖片轉到pattern的空間軸上
				//normalize the ROI (find homography and warp the ROI)
				normalizePattern(grayImage, roi2DPts, box, normROI);
				
				IplImage nnn = (IplImage) normROI;
				
				//library是Pattern的library
				const int retvalue = identifyPattern(normROI, library, out);
				
				//用retvalue來判定是不是我們要的那個pattern，只有是1才表示對的
				//push-back pattern in the stack of foundPatterns and find its extrinsics
				if (retvalue>0) {
					Pattern patCand;
					patCand.id = out.index;
					patCand.orientation = out.ori;
					patCand.confidence = out.maxCor;
					//cout << "Id: " << patCand.id << endl;

					for (j=0; j<4; j++){
						patCand.vertices.push_back(refinedVertices.at((8-out.ori+v1-j)%4));
					}
					//find the transformation (from camera CS to pattern CS)
					patCand.getExtrinsics(patCand.size, cameraMatrix, distortions);
					foundPatterns.push_back(patCand);
				}
			}
		}
	}
}


void PatternDetector::convertAndBinarize(const Mat& src, Mat& dst1, Mat& dst2, int thresh_mode)
{

	//dst1: binary image
	//dst2: grayscale image

	if (src.channels()==3){
		cvtColor(src, dst2, CV_BGR2GRAY);
	}
	else {
		src.copyTo(dst2);
	}
	
	if (thresh_mode == 1){
		threshold(dst2, dst1, thresh1, 255, CV_THRESH_BINARY_INV);
	}
	else {
		adaptiveThreshold( dst2, dst1, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, block_size, thresh2);
	}
	dilate( dst1, dst1, Mat());
	
}


void PatternDetector::normalizePattern(const Mat& src, const Point2f roiPoints[], Rect& rec, Mat& dst)
{
	

	//compute the homography
	Mat Homo(3,3,CV_32F);
	//roiPoints是我們影像上的四個點，norm2DPts是原本圖片的Size的四個點
	Homo = getPerspectiveTransform( roiPoints, norm2DPts);
	//subImg把我們要的那塊切出來，再透過投影轉回原本的座標
	cv::Mat subImg = src(cv::Range(rec.y, rec.y+rec.height), cv::Range(rec.x, rec.x+rec.width));
	//warp the input based on the homography model to get the normalized ROI
	cv::warpPerspective( subImg, dst, Homo, Size(dst.cols, dst.rows));
	
}

int PatternDetector::identifyPattern(const Mat& src, std::vector<cv::Mat>& loadedPatterns, patInfo& info)
{
	if (loadedPatterns.size()<1){
		printf("No loaded pattern");
		return -1;
	}

	unsigned int j;
	int i;
	double tempsim;
	double N = (double)(normSize*normSize/4);
	double nom, den;

	
	Scalar mean_ext, std_ext, mean_int, std_int;

	meanStdDev(src, mean_ext, std_ext, patMask);
	meanStdDev(src,mean_int, std_int, patMaskInt);

	if ((mean_ext.val[0]>mean_int.val[0]))
		return -1;


	Mat inter = src(cv::Range(normSize/4,3*normSize/4), cv::Range(normSize/4,3*normSize/4));
	
	double normSrcSq = pow(norm(inter),2);

	//zero_mean_mode;
	int zero_mean_mode = 1;
	//相似判斷
	//use correlation coefficient as a robust similarity measure
	info.maxCor = -1.0;
	for (j=0; j<(loadedPatterns.size()/4); j++){
		for(i=0; i<4; i++){
			
			double const nnn = pow(norm(loadedPatterns.at(j*4+i)),2);

			if (zero_mean_mode ==1){

				double const mmm = mean(loadedPatterns.at(j*4+i)).val[0];
			
				nom = inter.dot(loadedPatterns.at(j*4+i)) - (N*mean_int.val[0]*mmm);
				den = sqrt( (normSrcSq - (N*mean_int.val[0]*mean_int.val[0]) ) * (nnn - (N*mmm*mmm) ) );
				tempsim = nom/den;
			}
			else 
			{
				tempsim = inter.dot(loadedPatterns.at(j*4+i))/(sqrt(normSrcSq*nnn));
			}

			if(tempsim>info.maxCor){
				info.maxCor = tempsim;
				info.index = j+1;
				info.ori = i;
			}
		}
	}

	//cout << "MaxCor: " << info.maxCor << endl;
	//cout << "Ori: " << info.ori << endl;

	//如果是1表示是對的，如果是0表示不正確
	if (info.maxCor>confThreshold)
		return 1;
	else
		return 0;

}

};
