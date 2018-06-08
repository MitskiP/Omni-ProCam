#pragma once

#include "opencv2/opencv.hpp"
#include "WorldFeeder.h"

using namespace cv;
using namespace std;

class FisheyeFeeder : public WorldFeeder {
private:

	double aperture;
	Point2d anchor;

	double anchorRotationMatrixArray[3][3];
	
	int frameRadius;

public:

	FisheyeFeeder(double, Point2d, int);
	~FisheyeFeeder();

	void setAperture(double d) { aperture = d; }
	void setAnchor(Point2d p) { anchor = p; }

	Point locate(double, double);
};
