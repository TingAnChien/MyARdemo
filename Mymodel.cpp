#include "Mymodel.h"
#include <iostream>
using namespace cv;
using namespace std; 

	Model::Model(double para){
		size = para;
		hand_zdir = -1;
		leg_ydir = 1;
		speed = -10.0;
		hand_up = 140;
		hand_down = 120;
		leg_range = 20;
		hand_range = 20;
		ChangeHand = false;
		Build = false;

		face = (Mat_<float>(12, 3) <<
			0, 0, 0,
			50, 90, -170,
			55, 90, -165,
			45, 90, -165,//mouse
			30, 90, -200,
			40, 90, -200,
			40, 90, -190,
			30, 90, -190,//eye
			60, 90, -200,
			70, 90, -200,
			70, 90, -190,
			60, 90, -190      //3 face	
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
		handLeftWalk = (Mat_<float>(8, 3) <<
			5, 45, -135,
			5, 60, -135,
			20, 60, -135,
			20, 45, -135,     //23
			5, 45, -75,
			5, 60, -75,
			20, 60, -75,
			20, 45, -75         //27 leg left
			)*size / 100;
		handRightWalk = (Mat_<float>(8, 3) <<
			95, 45, -135,
			95, 60, -135,
			80, 60, -135,
			80, 45, -135,     //23
			95, 45, -75,
			95, 60, -75,
			80, 60, -75,
			80, 45, -75         //27 leg left
			)*size / 100;

		modelPts.push_back(face);
		modelPts.push_back(head);
		modelPts.push_back(body);
		modelPts.push_back(legLeft);
		modelPts.push_back(legRight);
			if (ChangeHand){
				modelPts.push_back(handLeft);
				modelPts.push_back(handRight);
		}
		else{
			modelPts.push_back(handLeftWalk);
			modelPts.push_back(handRightWalk);
		}
	}

	void Model::moveLeg(int c, int s){
		float src_center = (head.at<float>(0, c) + head.at<float>(2, c)) / 2.0F;//x
		if (s < 0){
			if (leg_ydir < 0){
				if ((legLeft.at<float>(4, c)) <= (src_center + leg_range* size / 100)){
					legLeft.rowRange(4, 8).col(c) -= speed*size / 100;
					legRight.rowRange(4, 8).col(c) += speed*size / 100;
					handRightWalk.rowRange(4, 8).col(c) -= speed*size / 100;
					handLeftWalk.rowRange(4, 8).col(c) += speed*size / 100;
				}
				else
					leg_ydir = 1;
			}
			else{
				if ((legLeft.at<float>(4, c)) >= src_center){
					legLeft.rowRange(4, 8).col(c) += speed*size / 100;
					legRight.rowRange(4, 8).col(c) -= speed*size / 100;
					handRightWalk.rowRange(4, 8).col(c) += speed*size / 100;
					handLeftWalk.rowRange(4, 8).col(c) -= speed*size / 100;
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
					handRightWalk.rowRange(4, 8).col(c) -= speed*size / 100;
					handLeftWalk.rowRange(4, 8).col(c) += speed*size / 100;
				}
				else
					leg_ydir = 1;
			}
			else{
				if ((legLeft.at<float>(4, c)) >= (src_center - leg_range* size / 100)){
					legLeft.rowRange(4, 8).col(c) += speed*size / 100;
					legRight.rowRange(4, 8).col(c) -= speed*size / 100;
					handRightWalk.rowRange(4, 8).col(c) += speed*size / 100;
					handLeftWalk.rowRange(4, 8).col(c) -= speed*size / 100;
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

		face.col(c) -= speed*s*size / 100;
		head.col(c) -= speed*s*size / 100;
		body.col(c) -= speed*s*size / 100;
		handLeft.col(c) -= speed*s*size / 100;//still need to move
		handRight.col(c) -= speed*s*size / 100;
		handLeftWalk.col(c) -= speed*s*size / 100;
		handRightWalk.col(c) -= speed*s*size / 100;
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
		rotate(handLeftWalk, src_center, angRad);
		rotate(handRightWalk, src_center, angRad);
		rotate(legLeft, src_center, angRad);
		rotate(legRight, src_center, angRad);
		//roate face 12 vertex
		Point2f inPoint, outPoint;
		for (int i = 0; i < 12; i++){
			inPoint.x = face.at<float>(i, 0);
			inPoint.y = face.at<float>(i, 1);
			outPoint = rotatePoint(inPoint, src_center, angRad);
			face.at<float>(i, 0) = outPoint.x;
			face.at<float>(i, 1) = outPoint.y;
		}
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

	Point3f Model::getCenter(){
		float cols = head.at<float>(0, 0) + head.at<float>(2, 0);//x
		float rows = head.at<float>(0, 1) + head.at<float>(2, 1);//y
		//cout << "getcenter" << Point3f(cols / 2.0F, rows / 2.0F, legLeft.at<float>(4, 2)) << endl;
		return Point3f(cols / 2.0F, rows / 2.0F, legLeft.at<float>(4,2));
	}

	void Model::build(){
		Mat cube = (Mat_<float>(8, 3));		
		head.rowRange(0, 4).colRange(0, 2).copyTo(cube.rowRange(0, 4).colRange(0, 2));
		cube.rowRange(0, 4).col(2) = 0;
		head.rowRange(0, 4).colRange(0, 2).copyTo(cube.rowRange(4, 8).colRange(0, 2));
		legLeft.rowRange(4, 8).col(2).copyTo(cube.rowRange(4, 8).col(2));
		//cube.rowRange(4, 8).col(2) = (head.rowRange(0, 4).col(2) + ((float)260 * size / 100));
		modelPts.release();
		modelPts.push_back(face);
		modelPts.push_back(head);
		modelPts.push_back(body);
		modelPts.push_back(legLeft);
		modelPts.push_back(legRight);
		if (ChangeHand){
			modelPts.push_back(handLeft);
			modelPts.push_back(handRight);
		}
		else{
			modelPts.push_back(handLeftWalk);
			modelPts.push_back(handRightWalk);
		}
		modelPts.push_back(cube);
	}

	void Model::updateModel(){
		/*cout << "head" << head << endl;
		cout << "before "<< modelPts.rowRange(4, 12) << endl;
		face.copyTo(modelPts.rowRange(0, 4));
		head.copyTo(modelPts.rowRange(4, 12));
		body.copyTo(modelPts.rowRange(12, 20));
		legLeft.copyTo(modelPts.rowRange(20, 28));
		legRight.copyTo(modelPts.rowRange(28, 36));
		handLeft.copyTo(modelPts.rowRange(36, 44));
		handRight.copyTo(modelPts.rowRange(44, 48) );
		cout << modelPts.rowRange(4, 12) << endl;*/
		modelPts.release();
		modelPts.push_back(face);
		modelPts.push_back(head);
		modelPts.push_back(body);
		modelPts.push_back(legLeft);
		modelPts.push_back(legRight);
		if (ChangeHand){
			modelPts.push_back(handLeft);
			modelPts.push_back(handRight);
		}
		else{
			modelPts.push_back(handLeftWalk);
			modelPts.push_back(handRightWalk);
		}
		//cout << "old " << modelPts.rowRange(0, 4).col(1) << endl;
		//modelPts.rowRange(0, 52).col(1) += speed*size / 100;
	}

	void Model::goBack(){
		int orientation =getOrientation();
		if (orientation == 3){
			move(1, 1);//X:0 Y:1 Z:2 sign
			//moveHand(2, 1);
			moveLeg(1, 1);//X:0 Y:1 sign
		}
		else if (orientation == 4)
			turn(PI / 2);
		else if (orientation == 1)
			turn(PI);
		else if (orientation == 2)
			turn(-PI / 2);
		ChangeHand = false;
	}

	void Model::goFoward(){
		int orientation = getOrientation();
		if (orientation == 1){
			move(1, -1);//X:0 Y:1 sign
			//moveHand(2, 1);
			moveLeg(1, -1);
		}
		else if (orientation == 2)
			turn(PI / 2);
		else if (orientation == 3)
			turn(PI);
		else if (orientation == 4)
			turn(-PI / 2);
		ChangeHand = false;
	}
	void Model::goLeft(){
		int orientation = getOrientation();
		if (orientation == 2){
			move(0, -1);//X:0 Y:1 sign
			//moveHand(2, 1);
			moveLeg(0, -1);
		}
		else if (orientation == 3)
			turn(PI / 2);
		else if (orientation == 4)
			turn(PI);
		else if (orientation == 1)
			turn(-PI / 2);
		ChangeHand = false;
	}
	void Model::goRight(){
		int orientation = getOrientation();
		if (orientation == 4){
			move(0, 1);//X:0 Y:1 sign
			//moveHand(2, 1);
			moveLeg(0, 1);
		}
		else if (orientation == 1)
			turn(PI / 2);
		else if (orientation == 2)
			turn(PI);
		else if (orientation == 3)
			turn(-PI / 2);
		ChangeHand = false;
	}
	void Model::goUp(){
		moveHand(2, 1);
		move(2, -1);//X:0 Y:1 Z:2 sign
		ChangeHand = true;
	}
	void Model::goDown(){
		moveHand(2, 1);
		move(2, 1);//X:0 Y:1 sign
		ChangeHand = true;
	}
	
	void Model::goTo(Point3f target, int orientation){
		Point3f current = getCenter();
		int current_orientation = getOrientation();
		//cout << "orientation = " << orientation << endl;
		//cout << "current = " << current << endl;
		//cout << "target = " << target << endl;
		if (orientation == 3){
			if ((current.y - target.y) < -50 ){
				goBack();
				//cout << "goBack" << endl;
			}
			else if ((current.y - target.y) > -50 ){
				if (abs(current.x - target.x) > 40){
					goFoward();
					//cout << "goFoward" << endl;
				}
				else{
					turn(PI / 2);
					move(0, 4);
				}					
			}
			else{
				if ((current.x - target.x) > 0){
					goLeft();
					//cout << "goLeft" << endl;
				}
				else if ((current.x - target.x) < 0){
					goRight();
					//cout << "goRight" << endl;
				}
				else{
					if (current_orientation == 4)
						turn(PI/2);
					else if (current_orientation == 2)
						turn(-PI / 2);
				}
			}
		}
		if (orientation == 1){
			if ((current.y - target.y) < 50 ){
				if (abs(current.x - target.x) > 40){
					goBack();
					//cout << "goBack" << endl;
				}
				else{
					turn(PI / 2);
					move(0, 4);
				}
			}
			else if ((current.y - target.y) > 50 ){
				goFoward();
				//cout << "goFoward" << endl;
			}
			else{
				if ((current.x - target.x) > 0){
					goLeft();
					//cout << "goLeft" << endl;
				}
				else if ((current.x - target.x) < 0){
					goRight();
					//cout << "goRight" << endl;
				}
				else{
					if (current_orientation == 4)
						turn(-PI / 2);
					else if (current_orientation == 2)
						turn(PI / 2);
				}
			}
		}

		if (orientation == 2){
			if ((current.x - target.x) > 50 ){
				goLeft();
				//cout << "goLeft" << endl;
			}
			else if ((current.x - target.x) < 50 ){
				if (abs(current.y - target.y) > 40){
					goRight();
					//cout << "goRight" << endl;
				}
				else{
					turn(-PI / 2);
					move(1, 4);
				}
			}
			else{
				if ((current.y - target.y) < 0){
					goBack();
					//cout << "goBack" << endl;
				}
				else if ((current.y - target.y) > 0){
					goFoward();
					//cout << "goFoward" << endl;
				}				
				else{
					if (current_orientation == 3)
						turn(PI / 2);
					else if (current_orientation == 1)
						turn(-PI / 2);
				}
			}
		}
		if (orientation == 4){
			if ((current.x - target.x) > -50 ){
				if (abs(current.y - target.y) > 40){
					goLeft();
					//cout << "goLeft" << endl;
				}
				else{
					turn(-PI / 2);
					move(1, 4);
				}
			}
			else if ((current.x - target.x) < -50 ){
				goRight();
				//cout << "goRight" << endl;
			}
			else{
				if ((current.y - target.y) < 0){
					goBack();
					//cout << "goBack" << endl;
				}
				else if ((current.y - target.y) > 0){
					goFoward();
					//cout << "goFoward" << endl;
				}
				else{
					if (current_orientation == 3)
						turn(-PI / 2);
					else if (current_orientation == 1)
						turn(PI / 2);
				}
			}
		}
		
		if ((current.z - target.z) > 0){
			goUp();
			//cout << "goUp" << endl;			
			Build = true;
		}
		else if ((current.z - target.z) < 0){
			goDown();
			//cout << "goDown" << endl;
			Build = true;
		}


		if (Build){
			build();
		}
		else{
			modelPts.release();
			modelPts.push_back(face);
			modelPts.push_back(head);
			modelPts.push_back(body);
			modelPts.push_back(legLeft);
			modelPts.push_back(legRight);
			modelPts.push_back(handLeft);
			modelPts.push_back(handRight);
		}
	}