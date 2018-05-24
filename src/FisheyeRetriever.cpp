#include "FisheyeRetriever.h"
#include "SphereWorld.h"
#include "PxInfo.h"

FisheyeRetriever::FisheyeRetriever(double a, double comp) {
	compression = comp;
	aperture = a;
	accessTable = nullptr;
	rotation = genRotMat(Vec3d(0, 0, 0));
}
void FisheyeRetriever::invalidateMaths() {
	width = aperture * sworld->getRadius() * compression;
	height = width;

	double anchorRotationMatrixArray[3][3] = {
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
		double y = (double)(eyr - height/2) / (height/2);
		for (int exr = 0; exr < width; exr++) {
			double x = (double)(exr - width/2) / (width/2);

			double r = sqrt(x*x + y*y) * aperture / 2;
			double theta = atan2(y, x) - CV_PI/2;

			#ifndef DEGREE_90
			double vec[] = {
				-sin(theta) * sin(r),
				cos(theta) * sin(r),
				cos(r)
			};
			//Mat target = anchorRotationMatrix * Mat(v);
			double px = anchorRotationMatrixArray[0][0] * vec[0] + anchorRotationMatrixArray[0][1] * vec[1] + anchorRotationMatrixArray[0][2] * vec[2];
			double py = anchorRotationMatrixArray[1][0] * vec[0] + anchorRotationMatrixArray[1][1] * vec[1] + anchorRotationMatrixArray[1][2] * vec[2];
			double pz = anchorRotationMatrixArray[2][0] * vec[0] + anchorRotationMatrixArray[2][1] * vec[1] + anchorRotationMatrixArray[2][2] * vec[2];

			double longitude = atan2(px, pz);
			double latitude = asin(py);
			#else
			double latitude = r - CV_PI/2;
			double longitude = theta;
			#endif
			accessTable[eyr][exr] = sworld->getPxInfo(longitude, latitude);
			
			//viewpoint = rotateCoordinateArray(longitude, latitude, rotationArray);
			//accessTable[eyr][exr] = sworld->getPxInfo(viewpoint.x, viewpoint.y);
		}
	}
}
void FisheyeRetriever::postProcessFrame(shared_ptr<Mat> &mat) {
	circle(*mat, Point(mat->cols/2, mat->rows/2), mat->cols/2, Scalar(255, 0, 0), mat->cols/100);
}
