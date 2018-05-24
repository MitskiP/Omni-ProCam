#pragma once

#include <memory>
#include <vector>
#include <limits>
#include "opencv2/opencv.hpp"

class SphereWorld;
#include "SphereWorld.h"

using namespace cv;
using namespace std;

class WorldFeeder {
private:
protected:

	SphereWorld *sworld;

public:
	shared_ptr<Mat> frame;

	WorldFeeder();
	virtual ~WorldFeeder();

	static Mat genRotMat(Vec3d);
	
	void connect(SphereWorld*);

	virtual Point locate(double, double) = 0;
	
	void updateFrame(shared_ptr<Mat> &f) { frame = f; }
};
