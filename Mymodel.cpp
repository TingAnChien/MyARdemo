#include "Mymodel.h"
#include <iostream>
using namespace cv;
using namespace std; 

	Model::Model(double para){
		size = para;
		hand_zdir = -1;
		leg_ydir = 1;
		speed = -5.0;
		hand_up = 140;
		hand_down = 120;
		leg_range = 20;
		hand_range = 20;

		floor = (Mat_<float>(4, 3) <<
			0, 0, 0,
			100, 0, 0,
			100, 100, 0,
			0, 100, 0       //3 floor	
			)*size / 100;
		head = (Mat_<float>(8, 3) <<
			10, 10, -230,
			10, 90, -230,
			90, 90, -230,
			90, 10, -230,     //7 
			10, 10, -150,
			10, 90, -150,
			90, 90, -150,
			90, 10, -150     //11 head
			)*size / 100;
		body = (Mat_<float>(8, 3) <<
			20, 20, -150,
			20, 80, -150,
			80, 80, -150,
			80, 20, -150,     //15
			20, 20, -60,
			20, 80, -60,
			80, 80, -60,
			80, 20, -60     //19 body
			)*size / 100;
		legLeft = (Mat_<float>(8, 3) <<
			30, 40, -60,
			30, 55, -60,
			45, 55, -60,
			45, 40, -60,     //23
			30, 40, 0,
			30, 55, 0,
			45, 55, 0,
			45, 40, 0         //27 leg left
			)*size / 100;
		legRight = (Mat_<float>(8, 3) <<
			55, 40, -60,
			55, 55, -60,
			70, 55, -60,
			70, 40, -60,     //31
			55, 40, 0,
			55, 55, 0,
			70, 55, 0,
			70, 40, 0         //35 leg right
			)*size / 100;
		handLeft = (Mat_<float>(8, 3) <<
			20, 60, -135,
			20, 60, -120,
			20, 45, -120,
			20, 45, -135,     //39
			-20, 60, -140,
			-20, 60, -125,
			-20, 45, -125,
			-20, 45, -140     //43 hand left
			)*size / 100;
		handRight = (Mat_<float>(8, 3) <<
			80, 60, -135,
			80, 60, -120,
			80, 45, -120,
			80, 45, -135,     //47
			120, 60, -140,
			120, 60, -125,
			120, 45, -125,
			120, 45, -140         //51 hand right		
			)*size / 100;

		modelPts.push_back(floor);
		modelPts.push_back(head);
		modelPts.push_back(body);
		modelPts.push_back(legLeft);
		modelPts.push_back(legRight);
		modelPts.push_back(handLeft);
		modelPts.push_back(handRight);
	}
	/*
	void Model::moveHand(){

		if (hand_zdir < 0){
			if ((-handLeft.at<float>(4, 2)) >= (hand_down * size / 100)){
				//cout << handLeft.at<float>(4, 2) << endl;
				//cout << (120 * size / 100) << endl;
				handLeft.rowRange(4, 8).col(2) -= speed*size / 100;
				handRight.rowRange(4, 8).col(2) -= speed*size / 100;
			}
			else
				hand_zdir = 1;
		}
		else{
			if ((-handLeft.at<float>(4, 2)) <= (hand_up * size / 100)){
				handLeft.rowRange(4, 8).col(2) += (0, 0, speed)*size / 100;
				handRight.rowRange(4, 8).col(2) += (0, 0, speed)*size / 100;
			}
			else
				hand_zdir = -1;
		}
	}
	*/
	void Model::moveLeg(int c, int s){
		float src_center = (head.at<float>(0, c) + head.at<float>(2, c)) / 2.0F;//x
		if (s < 0){
			if (leg_ydir < 0){
				if ((legLeft.at<float>(4, c)) <= (src_center + leg_range* size / 100)){
					legLeft.rowRange(4, 8).col(c) -= speed*size / 100;
					legRight.rowRange(4, 8).col(c) += speed*size / 100;
				}
				else
					leg_ydir = 1;
			}
			else{
				if ((legLeft.at<float>(4, c)) >= src_center){
					legLeft.rowRange(4, 8).col(c) += speed*size / 100;
					legRight.rowRange(4, 8).col(c) -= speed*size / 100;
				}
				else
					leg_ydir = -1;
			}
		}
		else{
			if (leg_ydir < 0){
				if ((legLeft.at<float>(4, c)) <= (src_center)){
					legLeft.rowRange(4, 8).col(c) -= speed*size / 100;
					legRight.rowRange(4, 8).col(c) += speed*size / 100;
				}
				else
					leg_ydir = 1;
			}
			else{
				if ((legLeft.at<float>(4, c)) >= (src_center - leg_range* size / 100)){
					legLeft.rowRange(4, 8).col(c) += speed*size / 100;
					legRight.rowRange(4, 8).col(c) -= speed*size / 100;
				}
				else
					leg_ydir = -1;
			}
		}
	}

	void Model::moveHand(int c,int s){
		float src_center = (-head.at<float>(0, c) - 100*size/100);//z
		if (s < 0){
			if (hand_zdir < 0){
				if ((-handLeft.at<float>(4, c)) >= (src_center + hand_range* size / 100)){
					handLeft.rowRange(4, 8).col(c) += speed*size / 100;
					handRight.rowRange(4, 8).col(c) += speed*size / 100;
				}
				else
					hand_zdir = 1;
			}
			else{
				if ((-handLeft.at<float>(4, c)) <= src_center){
					handLeft.rowRange(4, 8).col(c) -= speed*size / 100;
					handRight.rowRange(4, 8).col(c) -= speed*size / 100;
				}
				else
					hand_zdir = -1;
			}
		}
		else{//up
			if (hand_zdir < 0){
				if ((-handLeft.at<float>(4, c)) <= (src_center)){
					handLeft.rowRange(4, 8).col(c) += speed*size / 100;
					handRight.rowRange(4, 8).col(c) += speed*size / 100;
				}
				else
					hand_zdir = 1;
			}
			else{
				if ((-handLeft.at<float>(4, c)) >= (src_center - hand_range* size / 100)){
					handLeft.rowRange(4, 8).col(c) -= speed*size / 100;
					handRight.rowRange(4, 8).col(c) -= speed*size / 100;
				}
				else
					hand_zdir = -1;
			}
		}
	}

	void Model::move(int c, int s){

		head.col(c) -= speed*s*size / 100;
		body.col(c) -= speed*s*size / 100;
		handLeft.col(c) -= speed*s*size / 100;
		handRight.col(c) -= speed*s*size / 100;
		legLeft.col(c) -= speed*s*size / 100;
		legRight.col(c) -= speed*s*size / 100;

	}

	void Model::turn(double angRad){
		float cols = head.at<float>(0, 0) + head.at<float>(2, 0);//x
		float rows = head.at<float>(0, 1) + head.at<float>(2, 1);//y
		Point2f src_center(cols / 2.0F, rows / 2.0F);

		rotate(head, src_center, angRad);
		rotate(body, src_center, angRad);
		rotate(handLeft, src_center, angRad);
		rotate(handRight, src_center, angRad);
		rotate(legLeft, src_center, angRad);
		rotate(legRight, src_center, angRad);

		//Point2f  tmp = (legLeft.at<float>(4, 0), leg_front);
		//leg_front = rotatePoint(tmp, src_center, angRad).x;
		//leg_end = rotatePoint(tmp, src_center, angRad).x;
	}

	void Model::rotate(Mat part,Point2f src_center, double angRad){
		Point2f inPoint , outPoint;
		for (int i = 0; i < 8; i++){
			inPoint.x = part.at<float>(i, 0);
			inPoint.y = part.at<float>(i, 1);
			outPoint = rotatePoint(inPoint, src_center, angRad);
			part.at<float>(i, 0) = outPoint.x;
			part.at<float>(i, 1) = outPoint.y;
		}
	}

	Point2f Model::rotate2d(const cv::Point2f& inPoint, const double& angRad)
	{
		cv::Point2f outPoint;
		//CW rotation
		outPoint.x = std::cos(angRad)*inPoint.x - std::sin(angRad)*inPoint.y;
		outPoint.y = std::sin(angRad)*inPoint.x + std::cos(angRad)*inPoint.y;
		return outPoint;
	}

	Point2f Model::rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad)
	{
		return rotate2d(inPoint - center, angRad) + center;
	}

	int Model::getOrientation(){
		float diffx = head.at<float>(6, 0) - head.at<float>(5, 0);
		float diffy = head.at<float>(6, 1) - head.at<float>(5, 1);
		if (diffy == 0){
			diffy = head.at<float>(7, 1) - head.at<float>(6, 1);
			if (diffy > 0)
				return 1;
			else
				return 3;
		}
		else if(diffx == 0){
			diffx = head.at<float>(7, 0) - head.at<float>(6, 0);
			if (diffx > 0)
				return 2;
			else
				return 4;
		}
	}

	void Model::build(){

	}

	void Model::updateModel(){
		modelPts.release();
		modelPts.push_back(floor);
		modelPts.push_back(head);
		modelPts.push_back(body);
		modelPts.push_back(legLeft);
		modelPts.push_back(legRight);
		modelPts.push_back(handLeft);
		modelPts.push_back(handRight);
		//cout << "old " << modelPts.rowRange(0, 4).col(1) << endl;
		//modelPts.rowRange(0, 52).col(1) += speed*size / 100;
	}
