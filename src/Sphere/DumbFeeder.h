#pragma once

#include "opencv2/opencv.hpp"

class WorldFeeder;
#include "WorldFeeder.h"

using namespace cv;
using namespace std;

class DumbFeeder : public WorldFeeder {
public:
	DumbFeeder() {
		Mat m = Mat(1, 1, DEFINED_CV_TYPE);
		m.at<Vec3b>(0, 0) = Vec3b(0, 255, 0);
		updateFrame(m);
	}
	void invalidateMaths() { }
	Point locate(double a, double b) {
		return Point(0, 0);
	}
};
