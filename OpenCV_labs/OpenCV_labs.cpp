#include "stdafx.h"
#include "Calibration.h"
#include "Rectification.h"
#include "FundamentalEpilines.h"
#include "Pano.h"
#include "StereoDepth.h"
#include "Segmentation.h"
#include "BackgroundExtraction.h"
#include "FaceDetection.h"

using namespace std;
using namespace cv;


int main(int argc, char* argv[])
{
	cout << "1. Calibration: board_w=14, board_h=10, images_list_file=imgs.xml, percent=50\n";
	cout << "2. Rectification: board_w=14, board_h=10, intrinsic=file.xml, distortion=file.xml, image=file.png\n";
	cout << "3. FundamentalEpilines: image1=file.png, image2=file.png, [corners1=file.xml, corners2=file.xml]\n";
	cout << "4. Pano: image1=file.png, image2=file.png, threshold=1000\n";
	cout << "5. StereoDepth: image1=file.png, image2=file.png, threshold=1000, window=21\n";
	cout << "6. Segmentation: image1=file.png, thresh=0.4\n";
	cout << "7. BackgroundExtraction: -vid <video filename> |-img <image filename> \n";
	cout << "8. FaceDetection: -face=haarcascade_frontalface_alt.xml -eyes=haarcascade_eye_tree_eyeglasses.xml\n";

	char key;
	cin >> key;
	switch (key)
	{
	case '1':
		Calibration(argc, argv);
		break;
	case '2':
		Rectification(argc, argv);
		break;
	case '3':
		FundamentalEpilines(argc, argv);
		break;
	case '4':
		Pano(argc, argv);
		break;
	case '5':
		StereoDepth(argc, argv);
		break;
	case '6':
		Segmentation(argc, argv);
		break;
	case '7':
		BackgroundExtraction(argc, argv);
		break;
	case '8':
		FaceDetection(argc, argv);
		break;
	default:
		break;
	}
	return 0;
}

