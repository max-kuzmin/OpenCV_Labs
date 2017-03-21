#include "stdafx.h"
#include "Common.h"
#include "opencv2/nonfree/nonfree.hpp"

using namespace std;
using namespace cv;


void FindStereoCorners(int threshold, Mat image_gray1, Mat image_gray2, vector<Point2f> &corners1, vector<Point2f> &corners2) {
	//-- Step 1: Detect the keypoints using SURF Detector
	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	SurfFeatureDetector detector(threshold);
	detector.detect(image_gray1, keypoints1);
	detector.detect(image_gray2, keypoints2);

	//-- Step 2: Calculate descriptors (feature vectors)
	detector.compute(image_gray1, keypoints1, descriptors1);
	detector.compute(image_gray2, keypoints2, descriptors2);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	vector<DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);

	//-- Quick calculation of max and min distances between keypoints
	double max_dist = 0; double min_dist = 100;
	for (size_t i = 0; i < matches.size(); i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//-- Use only "good" matches (i.e. whose distance is less than 3*min_dist )
	for (size_t i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance < 3 * min_dist) {
			Point2f p1 = keypoints1[matches[i].queryIdx].pt;
			Point2f p2 = keypoints2[matches[i].trainIdx].pt;
			corners1.push_back(p1);
			corners2.push_back(p2);
		}
	}
}


//images must be put from left to right
int Pano(int argc, char* argv[]) {

	if (argc != 4) {
		printf("ERROR: Wrong number of input parameters\n");
		return -1;
	}

	Mat image1 = imread(ExePath() + argv[1]);
	Mat image2 = imread(ExePath() + argv[2]);

	Mat image_gray1, image_gray2;
	cvtColor(image1, image_gray1, COLOR_BGR2GRAY);
	cvtColor(image2, image_gray2, COLOR_BGR2GRAY);

	

	//find matches
	vector<Point2f> corners1, corners2;
	FindStereoCorners(atoi(argv[3]), image_gray1, image_gray2, corners1, corners2);

	//draw corners
	Mat image1_copy = image1.clone();
	Mat image2_copy = image2.clone();

	for (size_t i = 0; i < corners1.size(); i++)
	{
		int n1 = rand() % 255, n2 = rand() % 255, n3 = rand() % 255;
		putText(image1_copy, to_string(i), corners1[i], FONT_HERSHEY_SIMPLEX, 0.75, Scalar(n1, n2, n3), 2);
		putText(image2_copy, to_string(i), corners2[i], FONT_HERSHEY_SIMPLEX, 0.75, Scalar(n1, n2, n3), 2);
	}

	namedWindow("Corners1", CV_WINDOW_NORMAL);
	imshow("Corners1", image1_copy);
	resizeWindow("Corners1", 500, 500);
	namedWindow("Corners2", CV_WINDOW_NORMAL);
	imshow("Corners2", image2_copy);
	resizeWindow("Corners2", 500, 500);

	// Find the Homography Matrix
	Mat homography = findHomography(corners1, corners2, CV_RANSAC);
	// Use the Homography Matrix to warp the images
	//первая картинка преобразовывается с помощью гомографии
	Mat result;
	warpPerspective(image1, result, homography, Size(image1.cols + image2.cols, image1.rows));
	//вторая картинка просто вставляется поверх первой в левую половину
	Mat half(result, Rect(0, 0, image2.cols, image2.rows));
	image2.copyTo(half);

	namedWindow("Result", CV_WINDOW_NORMAL);
	imshow("Result", result);
	resizeWindow("Result", 500, 500);
	imwrite(ExePath() + "pano.jpg", result);

	waitKey();

	return 0;
}