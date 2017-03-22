#include "stdafx.h"
#include "Common.h"
#include "opencv2/nonfree/nonfree.hpp"
#include "Pano.h"

using namespace std;
using namespace cv;


//images must be put from left to right
int StereoDepth(int argc, char* argv[]) {

	if (argc != 5) {
		printf("ERROR: Wrong number of input parameters\n");
		return -1;
	}

	Mat image2 = imread(ExePath() + argv[1]);
	Mat image1 = imread(ExePath() + argv[2]);
	Mat image_gray1, image_gray2;
	cvtColor(image1, image_gray1, COLOR_BGR2GRAY);
	cvtColor(image2, image_gray2, COLOR_BGR2GRAY);

	//find corners on two images
	vector<Point2f> corners1, corners2;
	FindStereoCorners(atoi(argv[3]), image_gray1, image_gray2, corners1, corners2);


	if (corners1.size() < 8)
	{
		cout << "Not enought detected corners, try change threshold";
		waitKey();
		return 0;
	}

	//find fundamental matrix
	Mat fundamental = findFundamentalMat(corners1, corners2, CV_FM_RANSAC);

	//find homography matrices
	Mat homo1, homo2;
	stereoRectifyUncalibrated(corners1, corners2, fundamental, image1.size(), homo1, homo2, 20);

	//rectificate image
	Mat image_rect1, image_rect2;
	warpPerspective(image_gray1, image_rect1, homo1, image1.size());
	warpPerspective(image_gray2, image_rect2, homo2, image2.size());

	namedWindow("image_rect1", CV_WINDOW_NORMAL);
	imshow("image_rect1", image_rect1);
	resizeWindow("image_rect1", 500, 500);
	namedWindow("image_rect2", CV_WINDOW_NORMAL);
	imshow("image_rect2", image_rect2);
	resizeWindow("image_rect2", 500, 500);

	//compute depth map
	Mat depthMap;
	StereoVar stereoVar;
	StereoBM stereoBM;
	stereoBM.state.obj->SADWindowSize = atoi(argv[4]);

	stereoVar(image_rect1, image_rect2, depthMap);
	imwrite(ExePath() + "depth_map_stereo_var.jpg", depthMap);

	stereoBM(image_rect1, image_rect2, depthMap);
	imwrite(ExePath() + "depth_map_stereo_bm.jpg", depthMap);

	waitKey();

	//compute with usual homography method
	Mat homo_mono = findHomography(corners1, corners2, CV_RANSAC);
	warpPerspective(image_gray1, image_rect1, homo_mono, image1.size());

	imshow("image_rect1", image_rect1);
	imshow("image_rect2", image_gray2);

	stereoVar(image_rect1, image_gray2, depthMap);
	imwrite(ExePath() + "depth_map_mono_var.jpg", depthMap);

	stereoBM(image_rect1, image_gray2, depthMap);
	imwrite(ExePath() + "depth_map_mono_bm.jpg", depthMap);

	waitKey();

	return 0;
}