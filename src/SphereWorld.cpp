#include "DumbFeeder.h"
#include "SphereWorld.h"
#include "PxInfo.h"
#include "WorldFeeder.h"

SphereWorld::SphereWorld(int r) {
	radius = r;
	feeders = vector<WorldFeeder*>();
	df = new DumbFeeder();
	df->connect(this);
	generateTrigonometryTable();
}
SphereWorld::~SphereWorld() {
	delete df;
	delete[] sinTable;
}
void SphereWorld::generateTrigonometryTable() {
	int max = radius * CV_PI;
	sinTable = new double[max*3];
	for (int i = 0; i < max*3; i++) {
		int x = i - max;
		sinTable[i] = sin(x * CV_PI / max);		// sin(t), t \in [-*CV_PI; CV_PI*2]
	}
	sinTableCenter = sinTable + max;			// points to sin(0)
	cosTableCenter = sinTable + max + max/2;	// points to cos(0)
}
double SphereWorld::cosSW(double d) {
	int index = d * radius;
	return cosTableCenter[index];
}
double SphereWorld::sinSW(double d) {
	int index = d * radius;
	return sinTableCenter[index];
}
void SphereWorld::addFeeder(WorldFeeder *wf) {
	feeders.push_back(wf);
}
PxInfo SphereWorld::getDumbPxInfo() {
	return PxInfo(Point(0, 0), feeders[0]);
}
PxInfo SphereWorld::getPxInfo(double longitude, double latitude) {
	// TODO: implement this when using more than 1 feeder (excluding feeder[0])
	Point p = feeders[1]->locate(longitude, latitude);
	if (p.x == -1) {
		return getDumbPxInfo();
	}
	return PxInfo(p, feeders[1]);
}
