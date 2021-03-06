#include "Mypattern.h"
#include <iostream>
using namespace cv;
using namespace std;

namespace myARma {

	Pattern::Pattern(double param1){
		id =-1;
		size = param1;
		orientation = -1;
		confidence = -1;

		rotVec = (Mat_<float>(3,1) << 0, 0, 0);
		transVec = (Mat_<float>(3,1) << 0, 0, 0);
		rotMat = Mat::eye(3, 3, CV_32F);		
	}

	//convert rotation vector to rotation matrix (if you want to proceed with other libraries)
	void Pattern::rotationMatrix(const Mat& rotation_vector, Mat& rotation_matrix)
	{
		Rodrigues(rotation_vector, rotation_matrix);		
	}

	void Pattern::showPattern()
	{
		cout << "Pattern ID: " << id << endl;
		cout << "Pattern Size: " << size << endl;
		cout << "Pattern Confedince Value: " << confidence << endl;
		cout << "Pattern Orientation: " << orientation << endl;
		rotationMatrix(rotVec, rotMat);
		cout << "Exterior Matrix (from pattern to camera): " << endl;
		for (int i = 0; i<3; i++){
		cout << rotMat.at<float>(i,0) << "\t" << rotMat.at<float>(i,1) << "\t" << rotMat.at<float>(i,2) << " |\t"<< transVec.at<float>(i,0) << endl;
		}
	}

	void Pattern::getExtrinsics(int patternSize, const Mat& cameraMatrix, const Mat& distortions)
	{

		CvMat objectPts;//header for 3D points of pat3Dpts
		CvMat imagePts;//header for 2D image points of pat2Dpts 
		CvMat intrinsics = cameraMatrix;
		CvMat distCoeff = distortions;
		CvMat rot = rotVec;
		CvMat tra = transVec;
//		CvMat rotationMatrix = rotMat; // projectionMatrix = [rotMat tra];

		CvPoint2D32f pat2DPts[4];
		for (int i = 0; i<4; i++){
			pat2DPts[i].x = this->vertices.at(i).x;
			pat2DPts[i].y = this->vertices.at(i).y;
		}

		//3D points in pattern coordinate system
		CvPoint3D32f pat3DPts[4];
		pat3DPts[0].x = 0.0;
		pat3DPts[0].y = 0.0;
		pat3DPts[0].z = 0.0;
		pat3DPts[1].x = patternSize;
		pat3DPts[1].y = 0.0;
		pat3DPts[1].z = 0.0;
		pat3DPts[2].x = patternSize;
		pat3DPts[2].y = patternSize;
		pat3DPts[2].z = 0.0;
		pat3DPts[3].x = 0.0;
		pat3DPts[3].y = patternSize;
		pat3DPts[3].z = 0.0;

		cvInitMatHeader(&objectPts, 4, 3, CV_32FC1, pat3DPts);
		cvInitMatHeader(&imagePts, 4, 2, CV_32FC1, pat2DPts);
		
		//find extrinsic parameters
		cvFindExtrinsicCameraParams2(&objectPts, &imagePts, &intrinsics, &distCoeff, &rot, &tra);
		Mat rvec = cvarrToMat(&rot,true);
		Mat tvec = cvarrToMat(&tra,true);
	}

	void Pattern::draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix, Mat modelPts,int id)
	{
		/*for(int i=0;i<vertices.size();i++){
			rectangle(frame,Point(vertices[i].x-5,vertices[i].y-5),Point(vertices[i].x+5,vertices[i].y+5),Scalar(0,0,i*50),3,8);
		}*/
		rectangle(frame, Point(vertices[0].x - 5, vertices[0].y - 5), Point(vertices[0].x + 5, vertices[0].y + 5), Scalar(0, 0,  50), 3, 8);
		CvScalar color = cvScalar(255,255,255);
		
		switch (id){
			case 0:
				 color = cvScalar(255,255,0);
				break;
			default:
				 color = cvScalar(150,0,255);
				break;
		}

		//model 3D points: they must be projected to the image plane
		
		
		std::vector<cv::Point2f> model2ImagePts;
		/* project model 3D points to the image. Points through the transformation matrix 
		(defined by rotVec and transVec) "are transfered" from the pattern CS to the 
		camera CS, and then, points are projected using camera parameters 
		(camera matrix, distortion matrix) from the camera 3D CS to its image plane
		*/
		projectPoints(modelPts, rotVec, transVec, camMatrix, distMatrix, model2ImagePts); 
		//draw cube, or whatever

		//head
		line(frame, model2ImagePts, color, 12);
		//cout << (modelPts.rows - 12) / 8 << endl;
		for (int i = 1; i < (modelPts.rows - 12) / 8; i++){
			line(frame, model2ImagePts, color, 12+8*i);
		}

		cv::line(frame, model2ImagePts.at(1), model2ImagePts.at(2), color, 2);
		cv::line(frame, model2ImagePts.at(2), model2ImagePts.at(3), color, 2);
		cv::line(frame, model2ImagePts.at(1), model2ImagePts.at(3), color, 2);

		for (int i = 0; i<4; i++){
			cv::line(frame, model2ImagePts.at(i % 4 + 4), model2ImagePts.at((i + 1) % 4 + 4), color, 2);
		}

		for (int i = 0; i<4; i++){
			cv::line(frame, model2ImagePts.at(i % 4 + 8), model2ImagePts.at((i + 1) % 4 + 8), color, 2);
		}

		//draw the line that reflects the orientation. It indicates the bottom side of the pattern
		//cv::line(frame, model2ImagePts.at(21), model2ImagePts.at(22), cvScalar(0, 0, 255), 3);
		model2ImagePts.clear();
	}

	void Pattern::line(Mat& frame, vector<cv::Point2f> model2ImagePts, CvScalar color, int offset){

		int i;
		for (i = 0; i<4; i++){
			cv::line(frame, model2ImagePts.at(i % 4 + offset), model2ImagePts.at((i + 1) % 4 + offset), color, 3);
		}
		for (i = 4; i<7; i++){
			cv::line(frame, model2ImagePts.at(i % 8 + offset), model2ImagePts.at((i + 1) % 8 + offset), color, 3);
		}
		cv::line(frame, model2ImagePts.at(7 + offset), model2ImagePts.at(4 + offset), color, 3);

		for (i = 0; i<4; i++){
			cv::line(frame, model2ImagePts.at(i + offset), model2ImagePts.at(i + 4 + offset), color, 3);
		}


		/*
		for (i =0; i<4; i++){
		cv::line(frame, model2ImagePts.at(i%4), model2ImagePts.at((i+1)%4), color, 3);
		}
		for (i =4; i<7; i++){
		cv::line(frame, model2ImagePts.at(i%8), model2ImagePts.at((i+1)%8), color, 3);
		}
		cv::line(frame, model2ImagePts.at(7), model2ImagePts.at(4), color, 3);

		for (i =0; i<4; i++){
		cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i+4), color, 3);
		}
		//draw the line that reflects the orientation. It indicates the bottom side of the pattern
		cv::line(frame, model2ImagePts.at(2), model2ImagePts.at(3), cvScalar(80,255,80), 3);
		model2ImagePts.clear();

		*/
		/*
		cv::line(frame, model2ImagePts.at(0), model2ImagePts.at(1), color, 3);
		cv::line(frame, model2ImagePts.at(0), model2ImagePts.at(3), color, 3);
		cv::line(frame, model2ImagePts.at(0), model2ImagePts.at(4), cvScalar(80, 255, 80), 3);
		*/
	}

}
