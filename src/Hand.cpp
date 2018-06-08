#include "Hand.h"

Hand::Hand(Point p, int i) {
	setPos(p);
	setLabel(i);
	setVel(Point(0, 0));
	setTracked(true);
	id = rand()%1000;
}
void Hand::update(Hand h) {
	// calculate new velocity
	vel = h.pos - pos;
	pos = h.pos;
	label = h.label;
	tracked = true;
}
double Hand::distance2(Hand p) {
	return distance2(p.pos);
}
double Hand::distance2(Point p) {
	return (p.x-pos.x)*(p.x-pos.x) + (p.y-pos.y)*(p.y-pos.y);
}
//void HandTracker::update(Mat frame) {
//}
