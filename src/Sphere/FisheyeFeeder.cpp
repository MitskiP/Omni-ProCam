#include "FisheyeFeeder.h"

#define sin(x) sworld->sinSW(x)
#define cos(x) sworld->cosSW(x)

FisheyeFeeder::FisheyeFeeder(double d, Point2d anc, int fr) {
	aperture = d;
	anchor = anc;
	frameRadius = fr;
}
FisheyeFeeder::~FisheyeFeeder() {
}
Point FisheyeFeeder::locate(double longitude, double latitude) {

	#ifndef DEGREE_90
	double refx = anchor.x, refy = anchor.y;
	double r = 2* acos(sin(refy) * sin(latitude) + cos(refy)*cos(latitude)*cos(longitude-refx)) / aperture;
	double theta = atan2(cos(latitude)*sin(longitude-refx), cos(refy)*sin(latitude) - sin(refy)*cos(latitude)*cos(longitude-refx)); // 'mirror' by *-1
	#else
	// case: 90°
	double r = 2*(CV_PI/2 + latitude) / aperture;
	double theta = longitude;
	#endif

	//double r = 2*atan2(sqrt(px*px + pz*pz), py) / aperture;
	//double theta = atan2(pz, px);
	if (r > 1.0) {
		return Point(-1, -1);
	}
	double x = r * cos(theta);
	double y = r * sin(theta);

	double fx = x*frameRadius+frameRadius;
	double fy = y*frameRadius+frameRadius;

	return Point(fx, fy);
}
