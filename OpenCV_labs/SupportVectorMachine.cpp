#include "stdafx.h"

using namespace cv;

enum blob {
	one = 0,
	two = 1,
	three = 2,
	four = 3
};

int SupportVectorMachine(int argc, char* argv[]) {
	// Data for visual representation
	int width = 512, height = 512;
	Mat image = Mat::zeros(height, width, CV_8UC3);


	// Set up training data
	float labels[16] = { blob::one, blob::one, blob::one, blob::one,
						blob::two, blob::two, blob::two, blob::two,
						blob::three, blob::three, blob::three, blob::three,
						blob::four, blob::four, blob::four, blob::four };
	Mat labelsMat(16, 1, CV_32FC1, labels);

	float trainingData[16][2] = { { 52, 12 },{ 23, 10 },{ 30, 30 },{ 2, 20 },
								{ 312, 312 },{ 230, 310 },{ 450, 500 },{ 350, 500 },
								{ 52, 312 },{ 123, 310 },{ 130, 430 },{ 2, 420 },
								{ 352, 12 },{ 423, 10 },{ 330, 30 },{ 502, 20 } };
	Mat trainingDataMat(16, 2, CV_32FC1, trainingData);

	// Set up SVM's parameters
	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::POLY;
	params.degree = 1;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

	// Train the SVM
	CvSVM SVM;
	SVM.train(trainingDataMat, labelsMat, Mat(), Mat(), params);

	Vec3b green(0, 255, 0), blue(255, 0, 0);
	// Show the decision regions given by the SVM
	for (int i = 0; i < image.rows; ++i)
		for (int j = 0; j < image.cols; ++j)
		{
			Mat sampleMat = (Mat_<float>(1, 2) << j, i);
			float response = SVM.predict(sampleMat);

			if (response == blob::one)
				image.at<Vec3b>(i, j) = Vec3b(255, 0, 0);
			else if (response == blob::two)
				image.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
			else if (response == blob::three)
				image.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
			else if (response == blob::four)
				image.at<Vec3b>(i, j) = Vec3b(128, 128, 128);
		}

	// Show the training data
	int thickness = -1;
	int lineType = 8;
	for (int i = 0; i < trainingDataMat.size[0]; i++)
	{
		Point p = Point(trainingDataMat.at<float>(i, 0), trainingDataMat.at<float>(i, 1));
		circle(image, p, 5, Scalar(0, 0, 0), thickness, lineType);
	}

	// Show support vectors
	thickness = 2;
	lineType = 8;
	int c = SVM.get_support_vector_count();

	for (int i = 0; i < c; ++i)
	{
		const float* v = SVM.get_support_vector(i);
		circle(image, Point((int)v[0], (int)v[1]), 6, Scalar(255, 255, 255), thickness, lineType);
	}

	imshow("SVM dots classifier Example", image); // show it to the user
	cv::waitKey(0);

	return 0;
}
