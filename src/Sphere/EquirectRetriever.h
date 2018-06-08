#pragma once

#include "opencv2/opencv.hpp"
#include "WorldRetriever.h"

using namespace cv;
using namespace std;

class EquirectRetriever : public WorldRetriever {
private:

	double compression;
	double hangle, vangle;

public:

	EquirectRetriever(double, double, double);

	void invalidateMaths();
};
