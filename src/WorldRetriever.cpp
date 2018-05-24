#include "WorldRetriever.h"
#include "WorldFeeder.h"
#include "SphereWorld.h"
#include "PxInfo.h"

WorldRetriever::WorldRetriever() {
}
WorldRetriever::~WorldRetriever() {
	deleteAccessTable();
}
void WorldRetriever::deleteAccessTable() {
	if (accessTable != nullptr) {
		for (int y = 0; y < height; y++) {
			delete[] accessTable[y];
		}
		delete[] accessTable;
	}
}
void WorldRetriever::connect(SphereWorld *sw) {
	sworld = sw;
	rotation = genRotMat(Vec3d(0, 0, 0));
}
shared_ptr<Mat> WorldRetriever::getFrame() {
	shared_ptr<Mat> mat = make_shared<Mat>(height, width, DEFINED_CV_TYPE);
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			PxInfo *p = &(accessTable[y][x]);
			mat->at<Vec3b>(y, x) = p->wf->frame->at<Vec3b>(p->p.y, p->p.x);
		}
	}
	postProcessFrame(mat);
	return mat;
}
Point2d WorldRetriever::rotateCoordinateArray(double longitude, double latitude, double rotation[3][3]) {
	double vec[] = {
		cos(latitude) * sin(longitude),
		sin(latitude),
		cos(latitude) * cos(longitude)
	};
	//Mat mult = rotation * Mat(vec);
	double x = rotation[0][0] * vec[0] + rotation[0][1] * vec[1] + rotation[0][2] * vec[2];
	double y = rotation[1][0] * vec[0] + rotation[1][1] * vec[1] + rotation[1][2] * vec[2];
	double z = rotation[2][0] * vec[0] + rotation[2][1] * vec[1] + rotation[2][2] * vec[2];

	Point2d res;
	res.x = atan2(x, z);
	res.y = asin(y);
	return res;
}
Mat WorldRetriever::genRotMat(Vec3d theta) {
	// x - default axis; y - vertical axis; z - depth axis

	// ATTENTION: do not use fast sin/cos here - rotation matrices might lose their properties otherwise!!
	//            something like norm(v') != norm(v) after v'=M*v

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
