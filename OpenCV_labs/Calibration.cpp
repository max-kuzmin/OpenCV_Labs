#include "stdafx.h"
#include "Common.h"

using namespace std;
using namespace cv;


// Hit ‘p’ to pause/unpause, ESC to quit

int Calibration(int argc, char* argv[]) {
	if (argc != 4) {
		printf("ERROR: Wrong number of input parameters\n");
		return -1;
	}

	const int board_dt = 20; //Wait 20 frames per chessboard view
	int board_w;
	int board_h;
	vector<string> images_list;

	board_w = atoi(argv[1]);
	board_h = atoi(argv[2]);
	readStringList(argv[3], images_list);
	vector<string> images_list2(images_list);

	int board_n = board_w * board_h;
	CvSize board_sz = cvSize(board_w, board_h);
	cvNamedWindow("Calibration");
	//ALLOCATE STORAGE
	CvMat* image_points = cvCreateMat(images_list.size()*board_n, 2, CV_32FC1);
	CvMat* object_points = cvCreateMat(images_list.size()*board_n, 3, CV_32FC1);
	CvMat* point_counts = cvCreateMat(images_list.size(), 1, CV_32SC1);
	CvMat* intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);
	CvPoint2D32f* corners = new CvPoint2D32f[board_n];
	int corner_count;
	int successes = 0;
	int step;


	IplImage* image;


	while (images_list.size() > 0) {

		image = cvLoadImage((ExePath() + images_list.back()).c_str(), CV_LOAD_IMAGE_COLOR);
		images_list.pop_back();
		//Find chessboard corners:
		int found = cvFindChessboardCorners(
			image, board_sz, corners, &corner_count,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
		);
		//Get Subpixel accuracy on those corners
		IplImage *gray_image = cvCreateImage(cvGetSize(image), 8, 1);//subpixel
		cvFindCornerSubPix(gray_image, corners, corner_count,
			cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(
				CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		//Draw it
		cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);
		cvShowImage("Calibration", image);
		// If we got a good board, add it to our data
		if (corner_count == board_n) {
			step = successes*board_n;
			for (int i = step, j = 0; j < board_n; ++i, ++j) {
				CV_MAT_ELEM(*image_points, float, i, 0) = corners[j].x;
				CV_MAT_ELEM(*image_points, float, i, 1) = corners[j].y;
				CV_MAT_ELEM(*object_points, float, i, 0) = j / board_w;
				CV_MAT_ELEM(*object_points, float, i, 1) = j%board_w;
				CV_MAT_ELEM(*object_points, float, i, 2) = 0.0f;
			}
			CV_MAT_ELEM(*point_counts, int, successes, 0) = board_n;
			successes++;
		}

		int c = cvWaitKey(15);
		if (c == 'p') {
			c = 0;
			while (c != 'p' && c != 27) {
				c = cvWaitKey(250);
			}
		}
		if (c == 27)
			return 0;


	} //END COLLECTION WHILE LOOP.
	  //ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
	CvMat* object_points2 = cvCreateMat(successes*board_n, 3, CV_32FC1);
	CvMat* image_points2 = cvCreateMat(successes*board_n, 2, CV_32FC1);
	CvMat* point_counts2 = cvCreateMat(successes, 1, CV_32SC1);
	//TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
	//Below, we write out the details in the next two loops. We could
	//instead have written:
	//image_points->rows = object_points->rows = \
	//successes*board_n; point_counts->rows = successes;
//
	for (int i = 0; i < successes*board_n; ++i) {
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
	for (int i = 0; i < successes; ++i) { //These are all the same number
		CV_MAT_ELEM(*point_counts2, int, i, 0) =
			CV_MAT_ELEM(*point_counts, int, i, 0);
	}
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);
	// At this point we have all of the chessboard corners we need.
	// Initialize the intrinsic matrix such that the two focal
	// lengths have a ratio of 1.0
	//
	CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 1.0f;
	CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 1.0f;
	//CALIBRATE THE CAMERA!
	cvCalibrateCamera2(
		object_points2, image_points2,
		point_counts2, cvGetSize(image),
		intrinsic_matrix, distortion_coeffs,
		NULL, NULL, 0 //CV_CALIB_FIX_ASPECT_RATIO
	);
	// SAVE THE INTRINSICS AND DISTORTIONS
	cvSave((ExePath() + "Intrinsics.xml").c_str(), intrinsic_matrix);
	cvSave((ExePath() + "Distortion.xml").c_str(), distortion_coeffs);

	// EXAMPLE OF LOADING THESE MATRICES BACK IN:
	//CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
	//CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");
	// Build the undistort map that we will use for all
	// subsequent frames.
	//
	IplImage* mapx = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	IplImage* mapy = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	cvInitUndistortMap(
		intrinsic_matrix,
		distortion_coeffs,
		mapx,
		mapy
	);
	// Just run the camera to the screen, now showing the raw and
	// the undistorted image.
	//
	CreateDirectoryA((ExePath() + "undistorted\\").c_str(), NULL);
	cvNamedWindow("Undistort");
	while (images_list2.size() > 0) {
		string dirname = images_list2.back();
		IplImage* t = cvLoadImage((ExePath() + dirname).c_str(), CV_LOAD_IMAGE_COLOR);
		images_list2.pop_back();

		cvShowImage("Calibration", t); // Show raw image
		cvRemap(t, image, mapx, mapy); // Undistort image
		cvReleaseImage(&t);
		cvShowImage("Undistort", image); // Show corrected image
										 //Handle pause/unpause and ESC

		int last = dirname.find_last_of("\\") + 1;
		string name = dirname.substr(last, dirname.length() - last);
		cvSaveImage((ExePath() + "undistorted\\" + name).c_str(), image);

		int c = cvWaitKey(15);
		if (c == 'p') {
			c = 0;
			while (c != 'p' && c != 27) {
				c = cvWaitKey(250);
			}
		}
		if (c == 27)
			break;

	}
	return 0;
}