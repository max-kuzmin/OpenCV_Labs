#include "stdafx.h"
#include "Calibration.h"
#include "Rectification.h"
#include "FundamentalEpilines.h"
#include "Pano.h"
#include "StereoDepth.h"

using namespace std;
using namespace cv;


int main(int argc, char* argv[])
{
	cout << "1. Calibration: board_w=14, board_h=10, images_list_file=imgs.xml, percent=50\n";
	cout << "2. Rectification: board_w=14, board_h=10, intrinsic=file.xml, distortion=file.xml, image=file.png\n";
	cout << "3. FundamentalEpilines: image1=file.png, image2=file.png, [corners1=file.xml, corners2=file.xml]\n";
	cout << "4. Pano: image1=file.png, image2=file.png, threashold=1000\n";
	cout << "5. StereoDepth: image1=file.png, image2=file.png, threashold=1000, window=21\n";

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
	default:
		break;
	}
	return 0;
}

