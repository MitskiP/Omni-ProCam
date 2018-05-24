#pragma once

#include <memory>
#include <vector>
#include <limits>
#include "opencv2/opencv.hpp"

class SphereWorld;
class PxInfo;

using namespace cv;
using namespace std;

class WorldRetriever {
private:

	shared_ptr<Mat> frame;
	
protected:

	SphereWorld *sworld;
	PxInfo **accessTable;
	int width;
	int height;

public:

	WorldRetriever();
	virtual ~WorldRetriever();
	void deleteAccessTable();
	
	virtual void invalidateMaths() = 0;
	void connect(SphereWorld*);
	
	shared_ptr<Mat> getFrame();
	virtual void postProcessFrame(shared_ptr<Mat>&) {}

	static Point2d rotateCoordinateArray(double, double, double[3][3]);
	static Mat genRotMat(Vec3d);
};
