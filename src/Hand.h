#pragma once

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class Hand {
private:
	Point pos;
	Point vel;
	
	int label;
	bool tracked;

public:
	int id;

	Hand(Point, int);
	Point getPos() { return pos; }
	void setPos(Point x) { pos = x; }
	Point getVel() { return vel; }
	void setVel(Point x) { vel = x; }
	int getLabel() { return label; }
	void setLabel(int i) { label = i; }
	bool isTracked() { return tracked; }
	void setTracked(bool i) { tracked = i; }
	
	void update(Hand);
	Point movedPos() { return pos+vel; }
	double distance2(Hand);
	double distance2(Point);
};
