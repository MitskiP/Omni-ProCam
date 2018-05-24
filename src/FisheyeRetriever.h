#pragma once

#include "opencv2/opencv.hpp"
#include "WorldRetriever.h"

using namespace cv;
using namespace std;

class FisheyeRetriever : public WorldRetriever {
private:

	Mat rotation;
	double compression;
	double aperture;

public:

	FisheyeRetriever(double, double);
	
	void setRotation(Mat m) { rotation = m; }

	void invalidateMaths();

	void postProcessFrame(shared_ptr<Mat>&);
};
