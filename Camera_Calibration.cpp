#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include<iostream>
using namespace std;
int n_boards = 0; //Will be set by input list
int board_dt = 90; //Wait 90 frames per chessboard view
int board_w;
int board_h;
int main() {
	CvCapture* capture;// = cvCreateCameraCapture( 0 );
	// assert( capture );
	board_w = 5;
	board_h = 7;
	n_boards = 5;
	//board_dt = 20;
	int board_n = board_w * board_h;// 计=糴*蔼
	int board_nx = (board_w - 1) * (board_h - 1);// ず场à计=(糴-1)*(蔼-1)
	CvSize board_sz = cvSize(board_w - 1, board_h - 1);// ず场row㎝columnà计
	capture = cvCreateCameraCapture(0);
	if (!capture) {
		printf("\nCouldn't open the camera\n");
		system("pause");
		return -1;
	}
	cvNamedWindow("Calibration");
	cvNamedWindow("Raw Video");
	//ALLOCATE STORAGE
	CvMat* image_points = cvCreateMat(n_boards*board_n, 2, CV_32FC1);
	CvMat* object_points = cvCreateMat(n_boards*board_n, 3, CV_32FC1);
	CvMat* point_counts = cvCreateMat(n_boards, 1, CV_32SC1);
	CvMat* intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* distortion_coeffs = cvCreateMat(4, 1, CV_32FC1);

	CvPoint2D32f* corners = new CvPoint2D32f[board_n];
	int corner_count;
	int successes = 0;
	int step, frame = 0;

	IplImage *image = cvQueryFrame(capture);
	IplImage *gray_image = cvCreateImage(cvGetSize(image), 8, 1);//subpixel
	// CAPTURE CORNER VIEWS LOOP UNTIL WE?橵E GOT n_boards
	// SUCCESSFUL CAPTURES (ALL CORNERS ON THE BOARD ARE FOUND)
	int countx = 0;// 耝Ω计
	while (successes < n_boards) {
		//Skip every board_dt frames to allow user to move chessboard
		if ((frame++ % board_dt) == 0) {
			cout << "!!!Turn" << endl;
			//Find chessboard corners:
			int found = cvFindChessboardCorners(
				image, board_sz, corners, &corner_count,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
				);
			cout << "found: " << found << endl;
			cout << "corner_count: " << corner_count << endl;
			//Get Subpixel accuracy on those corners
			cvCvtColor(image, gray_image, CV_BGR2GRAY);
			cvFindCornerSubPix(gray_image, corners, corner_count,
				cvSize(10, 10), cvSize(-1, -1), cvTermCriteria(
				CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			//Draw it
			cvDrawChessboardCorners(image, board_sz, corners,
				corner_count, found);
			//cvShowImage( "Calibration", image );
			//cvShowImage("Raw Video", image);

			// If we got a good board, add it to our data
			if (corner_count == board_nx) {
				countx++;
				cvShowImage("Calibration", image); //show in color if we did collect the image
				step = successes*board_n;
				for (int i = step, j = 0; j<board_n; ++i, ++j) {
					CV_MAT_ELEM(*image_points, float, i, 0) = corners[j].x;
					CV_MAT_ELEM(*image_points, float, i, 1) = corners[j].y;
					CV_MAT_ELEM(*object_points, float, i, 0) = j / board_w;
					CV_MAT_ELEM(*object_points, float, i, 1) = j%board_w;
					CV_MAT_ELEM(*object_points, float, i, 2) = 0.0f;
				}
				CV_MAT_ELEM(*point_counts, int, successes, 0) = board_n;
				successes++;
				printf("Collected our %d of %d needed chessboard images\n", successes, n_boards);
			}
			else{
				countx++;
				cvShowImage("Calibration", gray_image); //Show Gray if we didn't collect the image
				cout << countx << endl;
			}
		} //end skip board_dt between chessboard capture

		//Handle pause/unpause and ESC
		int c = cvWaitKey(15);
		if (c == 'p'){
			c = 0;
			while (c != 'p' && c != 27){
				c = cvWaitKey(250);
			}
		}
		if (c == 27)
			return 0;
		image = cvQueryFrame(capture); //Get next image
		cvShowImage("Raw Video", image);
	} //END COLLECTION WHILE LOOP.
	cvDestroyWindow("Calibration");
	printf("\n\n*** CALLIBRATING THE CAMERA...");
	//ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
	CvMat* object_points2 = cvCreateMat(successes*board_n, 3, CV_32FC1);
	CvMat* image_points2 = cvCreateMat(successes*board_n, 2, CV_32FC1);
	CvMat* point_counts2 = cvCreateMat(successes, 1, CV_32SC1);
	//TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
	for (int i = 0; i<successes*board_n; ++i){
		CV_MAT_ELEM(*image_points2, float, i, 0) =
			CV_MAT_ELEM(*image_points, float, i, 0);
		CV_MAT_ELEM(*image_points2, float, i, 1) =
			CV_MAT_ELEM(*image_points, float, i, 1);
		CV_MAT_ELEM(*object_points2, float, i, 0) =
			CV_MAT_ELEM(*object_points, float, i, 0);
		CV_MAT_ELEM(*object_points2, float, i, 1) =
			CV_MAT_ELEM(*object_points, float, i, 1);
		CV_MAT_ELEM(*object_points2, float, i, 2) =
			CV_MAT_ELEM(*object_points, float, i, 2);
	}
	for (int i = 0; i<successes; ++i){ //These are all the same number
		CV_MAT_ELEM(*point_counts2, int, i, 0) =
			CV_MAT_ELEM(*point_counts, int, i, 0);
	}
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);
	// At this point we have all of the chessboard corners we need.
	// Initialize the intrinsic matrix such that the two focal
	// lengths have a ratio of 1.0
	CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 1.0f;
	CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 1.0f;
	//CALIBRATE THE CAMERA!
	cvCalibrateCamera2(
		object_points2, image_points2,
		point_counts2, cvGetSize(image),
		intrinsic_matrix, distortion_coeffs,
		NULL, NULL, 0  //CV_CALIB_FIX_ASPECT_RATIO
		);
	// SAVE THE INTRINSICS AND DISTORTIONS
	printf(" *** DONE!\n\nStoring Intrinsics.xml and Distortions.xml files\n\n");
	cvSave("Intrinsics.xml", intrinsic_matrix);
	cvSave("Distortion.xml", distortion_coeffs);

	// EXAMPLE OF LOADING THESE MATRICES BACK IN:
	CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
	CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");

	// Build the undistort map which we will use for all
	// subsequent frames.
	IplImage* mapx = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	IplImage* mapy = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	cvInitUndistortMap(
		intrinsic,
		distortion,
		mapx,
		mapy
		);
	// Just run the camera to the screen, now showing the raw and
	// the undistorted image.
	IplImage *r = cvCreateImage(cvGetSize(image), 8, 1);//subpixel
	IplImage *g = cvCreateImage(cvGetSize(image), 8, 1);//subpixel
	IplImage *b = cvCreateImage(cvGetSize(image), 8, 1);//subpixel
	cvNamedWindow("Undistort");
	while (image) {
		cvShowImage("Raw Video", image); // Show raw image
		//system("pause");
		cvSplit(image, r, g, b, NULL);
		cvRemap(r, r, mapx, mapy); // Undistort image
		cvRemap(g, g, mapx, mapy); // Undistort image
		cvRemap(b, b, mapx, mapy); // Undistort image
		cvMerge(r, g, b, NULL, image);
		//cvRemap( image, t, mapx, mapy );     // Undistort image
		cvShowImage("Undistort", image);     // Show corrected image
		//Handle pause/unpause and ESC
		int c = cvWaitKey(15);
		if (c == 'p'){
			c = 0;
			while (c != 'p' && c != 27){
				c = cvWaitKey(250);
			}
		}
		if (c == 27)
			break;
		image = cvQueryFrame(capture);
	}
	return 0;
}