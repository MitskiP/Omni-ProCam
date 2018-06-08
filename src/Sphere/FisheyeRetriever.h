#pragma once

#include "opencv2/opencv.hpp"
#include "WorldRetriever.h"

using namespace cv;
using namespace std;

class FisheyeRetriever : public WorldRetriever {
private:

	double compression;
	double aperture;

public:

	FisheyeRetriever(double, double);
	
	void invalidateMaths();

	void postProcessFrame(shared_ptr<Mat>&);
};
