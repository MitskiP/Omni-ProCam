#include "WorldFeeder.h"

WorldFeeder::WorldFeeder() {
}
WorldFeeder::~WorldFeeder() {
}
void WorldFeeder::connect(SphereWorld *sw) {
	sworld = sw;
	sworld->addFeeder(this);
}
Mat WorldFeeder::genRotMat(Vec3d theta) {
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
