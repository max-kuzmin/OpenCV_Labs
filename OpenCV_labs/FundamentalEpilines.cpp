#include "stdafx.h"
#include "Common.h"

using namespace std;
using namespace cv;

int count1 = 0, count2 = 0;
Mat2f corners1, corners2;
Mat image1, image2;



/**
* \brief Compute and draw the epipolar lines in two images
*      associated to each other by a fundamental matrix
*
* \param title     Title of the window to display
* \param F         Fundamental matrix
* \param img1      First image
* \param img2      Second image
* \param points1   Set of points in the first image
* \param points2   Set of points in the second image matching to the first set
* \param inlierDistance      Points with a high distance to the epipolar lines are
*                not displayed. If it is negative, all points are displayed
**/
template <typename T1, typename T2>
static void drawEpipolarLines(const std::string& title, const cv::Matx<T1, 3, 3> F,
	const cv::Mat& img1, const cv::Mat& img2,
	const std::vector<cv::Point_<T2>> points1,
	const std::vector<cv::Point_<T2>> points2,
	const float inlierDistance = -1)
{
	CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());
	cv::Mat outImg(img1.rows, img1.cols * 2, CV_8UC3);
	cv::Rect rect1(0, 0, img1.cols, img1.rows);
	cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);
	/*
	* Allow color drawing
	*/
	if (img1.type() == CV_8U)
	{
		cv::cvtColor(img1, outImg(rect1), CV_GRAY2BGR);
		cv::cvtColor(img2, outImg(rect2), CV_GRAY2BGR);
	}
	else
	{
		img1.copyTo(outImg(rect1));
		img2.copyTo(outImg(rect2));
	}
	std::vector<cv::Vec<T2, 3>> epilines1, epilines2;
	cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
	cv::computeCorrespondEpilines(points2, 2, F, epilines2);

	CV_Assert(points1.size() == points2.size() &&
		points2.size() == epilines1.size() &&
		epilines1.size() == epilines2.size());

	cv::RNG rng(0);
	for (size_t i = 0; i < points1.size(); i++)
	{
		if (inlierDistance > 0)
		{
			if (distancePointLine(points1[i], epilines2[i]) > inlierDistance ||
				distancePointLine(points2[i], epilines1[i]) > inlierDistance)
			{
				//The point match is no inlier
				continue;
			}
		}
		/*
		* Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
		*/
		cv::Scalar color(rng(256), rng(256), rng(256));

		cv::line(outImg(rect2),
			cv::Point(0, -epilines1[i][2] / epilines1[i][1]),
			cv::Point(img1.cols, -(epilines1[i][2] + epilines1[i][0] * img1.cols) / epilines1[i][1]),
			color, 5);
		cv::circle(outImg(rect1), points1[i], 3, color, 5, CV_AA);

		cv::line(outImg(rect1),
			cv::Point(0, -epilines2[i][2] / epilines2[i][1]),
			cv::Point(img2.cols, -(epilines2[i][2] + epilines2[i][0] * img2.cols) / epilines2[i][1]),
			color, 5);
		cv::circle(outImg(rect2), points2[i], 3, color, 5, CV_AA);
	}
	cv::imshow(title, outImg);
	cv::waitKey(1);
}

template <typename T>
static float distancePointLine(const cv::Point_<T> point, const cv::Vec<T, 3>& line)
{
	//Line is given as a*x + b*y + c = 0
	return std::fabsf(line(0)*point.x + line(1)*point.y + line(2))
		/ std::sqrt(line(0)*line(0) + line(1)*line(1));
}


void onMouse(int event, int x, int y, int, void* n)
{
	if (event != EVENT_LBUTTONDOWN)
		return;

	if ((int)n == 1 && count1 < 8) {
		corners1.push_back(Point2f(x, y));
		circle(image1, Point2f(x, y), 20, Scalar(0, 0, 255), 5, 8, 0);
		imshow("Corners1", image1);
		count1++;
	}
	else if ((int)n == 2 && count2 < 8) {
		corners2.push_back(Point2f(x, y));
		circle(image2, Point2f(x, y), 20, Scalar(0, 0, 255), 5, 8, 0);
		imshow("Corners2", image2);
		count2++;
	}
}




int FundamentalEpilines(int argc, char* argv[]) {

	if (argc != 3 && argc != 5) {
		printf("ERROR: Wrong number of input parameters\n");
		return -1;
	}

	image1 = imread(ExePath() + argv[1]);
	//transpose(image1, image1);
	image2 = imread(ExePath() + argv[2]);
	//transpose(image2, image2);

	Mat image_gray1;
	cvtColor(image1, image_gray1, COLOR_BGR2GRAY);
	Mat image_gray2;
	cvtColor(image2, image_gray2, COLOR_BGR2GRAY);

	if (argc == 5) {
		FileStorage fs(ExePath() + argv[3], FileStorage::READ);
		fs["corners"] >> corners1;
		fs.release();
		FileStorage fs2(ExePath() + argv[4], FileStorage::READ);
		fs2["corners"] >> corners2;
		fs2.release();
	}
	else {
		namedWindow("Corners1", CV_WINDOW_NORMAL);
		imshow("Corners1", image1);
		setMouseCallback("Corners1", onMouse, (void*)1);
		namedWindow("Corners2", CV_WINDOW_NORMAL);
		imshow("Corners2", image2);
		setMouseCallback("Corners2", onMouse, (void*)2);

		while (count1 < 8 || count2 < 8) {
			waitKey(250);
		}

		FileStorage fs(ExePath() + "corners1.xml", FileStorage::WRITE);
		fs << "corners" << corners1;
		fs.release();
		FileStorage fs2(ExePath() + "corners2.xml", FileStorage::WRITE);
		fs2 << "corners" << corners2;
		fs2.release();
	}


	Mat fundamental = findFundamentalMat(corners1, corners2, CV_FM_8POINT);
	FileStorage fs3(ExePath() + "fundamental.xml", FileStorage::WRITE);
	fs3 << "fundamental" << fundamental;
	fs3.release();


	vector<Point2f> dots1, dots2;
	for (int i = 0; i < 8; i++)
	{
		dots1.push_back(corners1.at<Point2f>(i, 0));
		dots2.push_back(corners2.at<Point2f>(i, 0));
	}

	namedWindow("Epilines", CV_WINDOW_NORMAL);
	drawEpipolarLines("Epilines", (Matx<float, 3, 3>)fundamental, image1, image2, dots1, dots2);
	resizeWindow("Epilines", 500, 500);


	waitKey();


	return 0;
}