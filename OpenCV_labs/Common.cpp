#include <opencv2/opencv.hpp>
#include <windows.h>

using namespace std;
using namespace cv;

string ExePath() {
	char buffer[MAX_PATH];
	GetModuleFileNameA(NULL, buffer, MAX_PATH);
	string::size_type pos = string(buffer).find_last_of("\\/");
	return string(buffer).substr(0, pos) + "\\";
}

bool readStringList(const string& filename, vector<string>& l)
{
	l.clear();
	FileStorage fs(ExePath() + filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ && n.type() != FileNode::STR)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}