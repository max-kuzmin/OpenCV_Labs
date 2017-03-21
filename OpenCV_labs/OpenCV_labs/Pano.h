#include "stdafx.h"
using namespace cv;

int Pano(int argc, char* argv[]);
void FindStereoCorners(int threshold, Mat image_gray1, Mat image_gray2, vector<Point2f> &corners1, vector<Point2f> &corners2);