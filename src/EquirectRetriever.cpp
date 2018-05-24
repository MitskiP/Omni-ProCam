#include "EquirectRetriever.h"
#include "SphereWorld.h"
#include "PxInfo.h"

EquirectRetriever::EquirectRetriever(double h, double v, double comp) {
	compression = comp;
	hangle = h;
	vangle = v;
	accessTable = nullptr;
	rotation = genRotMat(Vec3d(0, 0, 0));
}
void EquirectRetriever::invalidateMaths() {
	// http://paulbourke.net/dome/dualfish2sphere/
	width = hangle * sworld->getRadius() * compression;
	height = vangle * sworld->getRadius() * compression;
	double totalEquirectWidth  = 2*CV_PI * sworld->getRadius() * compression;
	double totalEquirectHeight =   CV_PI * sworld->getRadius() * compression;

	double rotationArray[3][3] = {
		{rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2)},
		{rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2)},
		{rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2)}
	};
	
	deleteAccessTable();
	accessTable = new PxInfo*[height];
	for (int i = 0; i < height; i++) {
		accessTable[i] = new PxInfo[width];
	}
	Point2d viewpoint;
	for (int eyr = 0; eyr < height; eyr++) {
		double ey = (double)(eyr - height/2) / (totalEquirectHeight/2);
		double latitude = ey * CV_PI/2;
		for (int exr = 0; exr < width; exr++) {
			double ex = (double)(exr - width/2) / (totalEquirectWidth/2);
			double longitude = ex * CV_PI;
			viewpoint = rotateCoordinateArray(longitude, latitude, rotationArray);
			accessTable[eyr][exr] = sworld->getPxInfo(viewpoint.x, viewpoint.y);
		}
	}
}
