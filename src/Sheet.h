#pragma once

#include <opencv2/aruco.hpp>

class Sheet {
public:
	Sheet(int, Size2f, float, Point2f, Point2f, Point2f, string, bool, int);
	~Sheet();
	int id;
	Size2f dim;
	float mSize;
	Point2f mPos;
	Point2f tl, br; // top left, bottom right, will be calculated to percentage of mSize with origin @ mPos
	
	string mediaFile;
	VideoCapture media;
	Mat frame;
	Mat nextFrame();
	Size getContentSize();
	bool transp;
	int frameSteps;

	long updatedFrame;
	Vec3d realWorld[4];
	Point2d projection[4];
	Mat perspTrans;
private:
	Mat nextFrame(int);
	void resetMedia();
};
Sheet::~Sheet() {
	if (media.isOpened())
		media.release();
}
Sheet::Sheet(int markerID, Size2f sheetSize, float markerSize, Point2f markerTopLeft, Point2f drawingTopLeft, Point2f drawingBottomRight, string mediaFileStr, bool transpose, int framesPerProjFrame) {
	id = markerID; dim = sheetSize; mSize = markerSize; mPos = markerTopLeft; tl = drawingTopLeft; br = drawingBottomRight;
	tl -= mPos;
	br -= mPos;
	tl = tl / mSize;
	br = br / mSize;
	
	updatedFrame = -99999999;
	
	frameSteps = framesPerProjFrame;
	transp = transpose;
	mediaFile = mediaFileStr;
	resetMedia();
}
Mat Sheet::nextFrame() {
	return nextFrame(frameSteps);
}
Mat Sheet::nextFrame(int fs) {
	for (int i = 0; i < fs; i++)
		media.read(frame);
	bool newFrame = fs > 0 || frame.empty();
	while (frame.empty()) {
		resetMedia();
		media >> frame;
	}
	//flip(res, res, 1);
	if (newFrame && transp)
		transpose(frame, frame);
	return frame;
}
void Sheet::resetMedia() {
	media.open(mediaFile);
	if (!media.isOpened()) {
		cerr << "invalid input: " << mediaFile << endl;
		exit(32);
	}
	if (frameSteps == 0)
		frame = nextFrame(1);
}
Size Sheet::getContentSize() {
	return Size(media.get(CAP_PROP_FRAME_WIDTH), media.get(CAP_PROP_FRAME_HEIGHT));
}

//Sheet::~SharedQueue(){}

