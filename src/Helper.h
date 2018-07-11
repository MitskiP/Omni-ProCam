#pragma once

#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ccalib/omnidir.hpp"

using namespace std;

Mat genRotMat(Vec3d theta) {
	// x - default axis; y - vertical axis; z - depth axis
	// Calculate rotation about x axis
	Mat R_x = (Mat_<double>(3,3) <<
			   1,		0,			  0,
			   0,		cos(theta[0]), -sin(theta[0]),
			   0,		sin(theta[0]), cos(theta[0])
			   );
	// Calculate rotation about y axis
	Mat R_y = (Mat_<double>(3,3) <<
			   cos(theta[1]),	0,	sin(theta[1]),
			   0,				1,	0,
			   -sin(theta[1]),	0,	cos(theta[1])
			   );
	 
	// Calculate rotation about z axis
	Mat R_z = (Mat_<double>(3,3) <<
			   cos(theta[2]),	-sin(theta[2]),	0,
			   sin(theta[2]),	cos(theta[2]),	0,
			   0,				0,				1);
	// Combined rotation matrix
	Mat R = R_z * R_y * R_x;
	return R;
}
Vec3d rotateVector(double vec[3], double rotation[3][3]) {
	//Mat mult = rotation * Mat(vec);
	double x = rotation[0][0] * vec[0] + rotation[0][1] * vec[1] + rotation[0][2] * vec[2];
	double y = rotation[1][0] * vec[0] + rotation[1][1] * vec[1] + rotation[1][2] * vec[2];
	double z = rotation[2][0] * vec[0] + rotation[2][1] * vec[1] + rotation[2][2] * vec[2];
	return Vec3d(x, y, z);
}
/*Point2d rotateCoordinateArray(double longitude, double latitude, double rotation[3][3]) {
	double vec[] = {
		cos(latitude) * sin(longitude),
		sin(latitude),
		cos(latitude) * cos(longitude)
	};
	Vec3d rotated = rotateVector(vec, rotation);
	Point2d res;
	res.x = atan2(rotated[0], rotated[2]);
	res.y = asin(rotated[1]);
	return res;
}*/
Point2d pxToAngles(double diameter, double aperture, double x, double y) {
	x -= diameter/2;
	y -= diameter/2;
	double r = sqrt(x*x + y*y) * aperture / diameter;
	double theta = atan2(y, x) + CV_PI/2; // 0° = top
	double longitude = theta;
	//double latitude = r - CV_PI/2;
	double latitude = CV_PI/2 - r;
	return Point2d(longitude, latitude);
}
Point2d pxToAngles(double diameter, double aperture, Point2d xy) {
	return pxToAngles(diameter, aperture, xy.x, xy.y);
}
Point2d anglesToPx(double diameter, double aperture, Point2d a) {
	double r = (CV_PI/2 - a.y) * diameter / aperture;
	double theta = a.x;
	double x = r * sin(theta) + diameter / 2;
	double y = r * -cos(theta) + diameter / 2;
	return Point2d(x, y);
}
Vec3d anglesToVector(Point2d a, double r) {
	return Vec3d(
		cos(a.y) * sin(a.x),
		sin(a.y),
		cos(a.y) * cos(a.x)
	) * r;
}
Point2d vectorToAngles(Vec3d v) {
	return Point2d(
		atan2(v[0], v[2]),
		asin(v[1] / norm(v))
	);
}

Mat toPerspective(Mat &orig, double aperture, Mat rotation, Point2d targetAngle) {
	double rotationMatrixArray[3][3] = {
		{rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2)},
		{rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2)},
		{rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2)}
	};

	Mat dup = orig.clone();
	
	double radius = orig.cols / aperture;
	Size size(
		tan(targetAngle.x/2) * radius * 2,
		tan(targetAngle.y/2) * radius * 2
	);
	Mat res(size.height, size.width, orig.type());
	for (int y = 0; y < size.height; y++) {
		double dy = -y + (double)size.height/2;
		for (int x = 0; x < size.width; x++) {
			double dx = -x+(double)size.width/2;
			double v[] = { dx, dy, radius };
			Vec3d rotated = rotateVector(v, rotationMatrixArray);
			Point2d f = anglesToPx(orig.cols, aperture, vectorToAngles(rotated));
			
			if (f.x >= orig.cols || f.y >= orig.rows || f.x < 0 || f.y < 0)
				res.at<Vec3b>(y, x) = Vec3b(0, 255, 0);
			else {
				res.at<Vec3b>(y, x) = dup.at<Vec3b>(f.y, f.x);
				orig.at<Vec3b>(f.y, f.x) += Vec3b(32, 32, 32);
			}
		}
	}
	return res;
}


/*
static double avgAngle(double a, double b) {
	if (a >= 360)
		a -= 360;
	if (b >= 360)
		b -= 360;
	double sum = a + b;
	if (sum > 360 && sum < 540)
		sum = sum - 180;
	return sum / 2;
}*/



// https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/samples/detect_board_charuco.cpp
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	FileStorage fs(filename, FileStorage::READ);
	if(!fs.isOpened())
		return false;
	fs["Camera_Matrix"] >> camMatrix;
	if (camMatrix.empty())
		fs["camera_matrix"] >> camMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;
	if (distCoeffs.empty())
		fs["distortion_coefficients"] >> distCoeffs;
	return true;
}

