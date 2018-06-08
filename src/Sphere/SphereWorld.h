#pragma once

#include "opencv2/opencv.hpp"

#define DEFINED_CV_TYPE CV_8UC3

class WorldFeeder;
class PxInfo;
class DumbFeeder;

using namespace cv;
using namespace std;

class SphereWorld {
private:

	DumbFeeder *df;
	double radius;
	vector<WorldFeeder*> feeders;
	
	double *sinTable;
	double *sinTableCenter;
	double *cosTableCenter;

public:

	SphereWorld(double);
	~SphereWorld();
	void generateTrigonometryTable();
	double sinSW(double);
	double cosSW(double);

	double getRadius() { return radius; }
	
	void addFeeder(WorldFeeder*);
	PxInfo getDumbPxInfo();
	PxInfo getPxInfo(double, double);
};
