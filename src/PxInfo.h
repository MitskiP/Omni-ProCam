#pragma once

#include <memory>
#include <vector>
#include <limits>
#include "opencv2/opencv.hpp"

class WorldFeeder;

using namespace cv;
using namespace std;

class PxInfo {
public:
	Point p;
	WorldFeeder *wf;
	PxInfo() {}
	PxInfo(Point pp, WorldFeeder *w) {
		p = pp; wf = w;
	}
};
