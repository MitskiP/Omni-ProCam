#pragma once

#include <opencv2/aruco.hpp>

class Sheet {
public:
	Sheet(int, Size2f, float, Point2f, Point2f, Point2f, string);
	~Sheet();
	int id;
	Size2f dim;
	float mSize;
	Point2f mPos;
	Point2f tl, br; // top left, bottom right, will be calculated to percentage of mSize with origin @ mPos
	
	string mediaFile;
	VideoCapture media;
	Mat nextFrame();
private:
	void resetMedia();
};
Sheet::~Sheet() {
	if (media.isOpened())
		media.release();
}
Sheet::Sheet(int a, Size2f b, float c, Point2f d, Point2f e, Point2f f, string z) {
	id = a; dim = b; mSize = c; mPos = d; tl = e; br = f;
	tl -= mPos;
	br -= mPos;
	tl = tl / mSize;
	br = br / mSize;
	
	mediaFile = z;
	resetMedia();
}
Mat Sheet::nextFrame() {
	Mat res;
	media >> res;
	while (res.empty()) {
		resetMedia();
		media >> res;
	}
	return res;
}
void Sheet::resetMedia() {
	media.open(mediaFile);
	if (!media.isOpened()) {
		cerr << "invalid input: " << mediaFile << endl;
		exit(32);
	}
}

//Sheet::~SharedQueue(){}

