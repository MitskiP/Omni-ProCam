#pragma once

#include <stdio.h>
#include <vector>
#include <memory>
#include "opencv2/opencv.hpp"
#include "Hand.h"

using namespace cv;
using namespace std;

class MarkerTracker {
private:
	static const int TRACKING_DISTANCE2_THRESHOLD = 20*20;

	vector<Hand> appearedHands;
	vector<Hand> trackedHands;

	void generateAppearedHands();
	void findExistingHands();
	void removeDisappearedHands();
	void addAppearedHands();

	shared_ptr<Mat> frame;

	int nLabels;
	Mat labelMask, stats, centroids;

	Mat skinMask;
	Mat debugFrame;
	Mat frameHSV, skinMaskBGR, skinMaskHSV;

	void removeLabel(int);
	void findConnectedComponents();
	void calculateSkinMask(bool, bool);

public:
	MarkerTracker();
	void update(shared_ptr<Mat>&);
	Mat &getSkinFrame();
	Mat &getConnectedComponentsFrame();
};
