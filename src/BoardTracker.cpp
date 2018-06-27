#include <ctime>
#include <ratio>
#include <chrono>
#include <thread>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <chrono>
#include <memory>

#include "SharedQueue.h"
#include "SharedVideoCapture.h"
#include "Sphere/SphereWorld.h"
#include "Sphere/FisheyeFeeder.h"
#include "Sphere/EquirectRetriever.h"
#include "Sphere/FisheyeRetriever.h"
#include "MarkerTracker.h"
#include "BetterBlobDetector.h"


using namespace cv;
using namespace std;
using namespace std::chrono;


Mat origFile;
Mat rotationMatrix;

class Page {
public:
	Page(shared_ptr<Mat> &m, high_resolution_clock::time_point t) { frame = m; time = t; }
	shared_ptr<Mat> frame;
	high_resolution_clock::time_point time;
};

bool localFile = false;
SharedVideoCapture cap(1); // open the default camera
SharedQueue<Page> frames;

void cameraThread() {
	while (cap.isOpened()) {
		shared_ptr<Mat> frame = make_shared<Mat>();
		cap.read(*frame);
		if (frame->empty())
			break;
		frames.push_back(Page(frame, high_resolution_clock::now()));
		if (localFile)
			this_thread::sleep_for(std::chrono::milliseconds(66));
			//waitKey(66); // 66.66ms = 15 fps
	}
	cap.release();
}
/*
top view:
up: +z
right: +x
depth: y (front: +, back: -)
theta: clockwise, 0° = up
*/
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
Point2d anglesToPx(double diameter, double aperture, Point2d a) {
	double r = (CV_PI/2 - a.y) * diameter / aperture;
	double theta = a.x;
	double x = r * sin(theta) + diameter / 2;
	double y = r * -cos(theta) + diameter / 2;
	return Point2d(x, y);
}

int main(int argc, char** argv) {
	/*
	cout << pxToAngles(10.0, CV_PI, 2.5, 2.5) * 180/CV_PI << endl;
	cout << anglesToVector(Point2d(-45*CV_PI/180, 26*CV_PI/180), 10.0) << endl;
	//cout << anglesToVector(Point2d(180*CV_PI/180, 45*CV_PI/180), 10.0) << endl;
	cout << vectorToAngles(Vec3d(-6, 4, 6)) * 180/CV_PI << endl;
	cout << anglesToPx(10.0, CV_PI, Point2d(-45*CV_PI/180, 26*CV_PI/180)) << endl;
	return 0;
	//*/
	
	if (argc == 2) {
		cap.open(string(argv[1]));
		localFile = true;
	}
	if(!cap.isOpened())  // check if we succeeded
		return -1;


	Mat image = imread("frame1.png", CV_LOAD_IMAGE_COLOR);
	//Point2f imgPts[] = { Point2f(0, 0), Point2f(image.cols, 0), Point2f(0, image.rows), Point2f(image.cols, image.rows) };
	float imgDelta = 19.613f + 39.687f / 2;
	Point2f imgPts[] = { Point2f(imgDelta, imgDelta), Point2f(image.cols-imgDelta, 0), Point2f(0, image.rows-imgDelta), Point2f(image.cols-imgDelta, image.rows-imgDelta) };


	cap.set(CAP_PROP_FRAME_HEIGHT, 1200);
	cap.set(CAP_PROP_FRAME_WIDTH, 1200);

	int w = cap.get(CAP_PROP_FRAME_WIDTH);
	int h = cap.get(CAP_PROP_FRAME_HEIGHT);

	cout << w << ", " << h << endl;

	Mat border = Mat(768, 128, DEFINED_CV_TYPE, Scalar(0, 0, 0));

	// prepare required variables for the main loop
	bool is_first_frame = true;
	int sleep;
	high_resolution_clock::time_point start_time;
	high_resolution_clock::time_point end_time;
	duration<double, std::milli> time_span;
	high_resolution_clock::time_point last_frame_time = high_resolution_clock::now();

	// start camera thread
	thread t1(cameraThread);






	// pattern configuration
	//double realBlobRadius = 3.105/2; // cm
	double realBlobRadius = 3.969/2; // cm
	Size patternsize(2, 2); //number of centers
	// cut out about 170-180 degree out of 235
	double sourceAperture = 235 * CV_PI/180;
	double targetAperture = 148 * CV_PI/180;
	// projector position relative to camera. in cm.
	//Vec3d projPos(0, 0, 20);
	//Vec3d projPos(0, 0, 19);
	Vec3d projPos(0, -7.49213, 18.5437);
	double projectorAngularCorrectionStep = 1*CV_PI/180;
	bool debug180 = true;
	
	double camRadius = w / sourceAperture;


	SimpleBlobDetector::Params params;
	//*// https://www.learnopencv.com/blob-detection-using-opencv-python-c/
	// Change thresholds
//	params.minThreshold = 80;
//	params.maxThreshold = 200;
	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 92;
//	params.maxArea = 2500;
	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.7; // 0.785 = square
	// Filter by Convexity
//	params.filterByConvexity = true;
//	params.minConvexity = 0.87;
	// Filter by Inertia
//	params.filterByInertia = true;
//	params.minInertiaRatio = 0.01;//*/
	CirclesGridFinderParameters cgparams;
	/*cgparams.minDensity = 10;
	cgparams.densityNeighborhoodSize = Size2f(16, 16);
	cgparams.minDistanceToAddKeypoint = 20;
	cgparams.kmeansAttempts = 100;
	cgparams.convexHullFactor = 1.1f;
	cgparams.keypointScale = 1;
	cgparams.minGraphConfidence = 9;
	cgparams.vertexGain = 1;
	cgparams.vertexPenalty = -0.6f;
	cgparams.edgeGain = 1;
	cgparams.edgePenalty = -0.6f;
	cgparams.existingVertexGain = 10000;
	cgparams.minRNGEdgeSwitchDist = 5.f;
	cgparams.gridType = CirclesGridFinderParameters::GridType::ASYMMETRIC_GRID;*/

	BetterBlobDetector blobDetector(params);
	Ptr<SimpleBlobDetector> simpleBlobDetector = SimpleBlobDetector::create(params);
	vector<KeyPoint> blobs;

	int lastFrameKey = -1;
	//while (cap.isOpened() || frames.size() > 0) {
	while (cap.isOpened()) {

		if (frames.size() == 0) {
			//printf("waiting for new frame\n");
			continue;
		}

		// start timer and read next frame with timestamp
		start_time = high_resolution_clock::now();
		if (frames.size() > 1) {
//			printf("SKIPPING %d frames\n", frames.size()-1);
			while (frames.size() > 2)
				frames.pop_front();
			last_frame_time = frames.front().time;
			frames.pop_front();
		}
		Page p = frames.front();
		frames.pop_front();
		duration<double, std::milli> frame_duration = p.time - last_frame_time;
		last_frame_time = p.time;
		

		Rect roi;
		roi.width = p.frame->cols * targetAperture / sourceAperture;
		roi.height = p.frame->rows * targetAperture / sourceAperture;
		roi.x = (p.frame->cols - roi.width) / 2;
		roi.y = (p.frame->rows - roi.height) / 2;
		Mat crop = (*p.frame)(roi);

		Mat gray; //source image
		cvtColor(crop, gray, CV_BGR2GRAY);

		blobDetector.detect(gray, blobs);
		vector< vector<Point> > blobContours = blobDetector.getContours();
		for (size_t i = 0; i < blobContours.size(); i++)
			drawContours(crop, blobContours, i, Scalar(255, 255, 0));

		//simpleBlobDetector->detect(gray, blobs);
		//drawKeypoints(crop, blobs, crop, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		vector<Point2f> centers; //this will be filled by the detected centers
		bool patternfound = findCirclesGrid(gray, patternsize, centers, CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING, simpleBlobDetector, cgparams);

		//Mat out(p.frame->rows, p.frame->cols, p.frame->type(), Scalar(0,0,0));
		Mat out(crop.rows, crop.cols, crop.type(), Scalar(0, 0, 0));
		drawChessboardCorners(out, patternsize, Mat(centers), patternfound);
		drawChessboardCorners(crop, patternsize, Mat(centers), patternfound);

		if (patternfound) {
			//Point pts[] = { centers[0], centers[1], centers[3], centers[7], centers[9], centers[8] };
			Point pts[] = { centers[0], centers[1], centers[3], centers[2] };
			fillConvexPoly(out, pts, sizeof(pts)/sizeof(pts[0]), Scalar(255, 0, 0));

			vector<KeyPoint> blobs2(centers.size());
			vector<vector<Point>> blobContours2(centers.size());
			for (size_t k = 0; k < centers.size(); k++) {
				for (size_t i = 0; i < blobs.size(); i++) {
					if (blobs[i].pt == centers[k]) {
						blobs2[k] = blobs[i];
						blobContours2[k] = blobContours[i];
						blobs.erase(blobs.begin() + i);
						blobContours.erase(blobContours.begin() + i);
						break;
					}
				}
			}
			vector<RotatedRect> minRect(blobContours2.size());
			for(size_t i = 0; i < blobContours2.size(); i++) {
				minRect[i] = minAreaRect(Mat(blobContours2[i]));
			}

			//drawKeypoints(out, blobs2, out, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			//drawKeypoints(crop, blobs2, crop, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			vector<double> distances(centers.size());
			double avgDist = 0;
			for (size_t i = 0; i < blobs2.size(); i++) {
				drawContours(out, blobContours2, i, Scalar(255, 255, 0)); // Draw the largest contour using previously 
				double diameter = max(minRect[i].size.width, minRect[i].size.height);
				//double theta = blobs2[i].size / camRadius;
				double theta = diameter / camRadius;
				double distance = realBlobRadius / tan(theta/2);
				distances[i] = distance;
				avgDist += distance / blobs2.size();
			}

			double avgDistProj = 0;
			for (size_t i = 0; i < centers.size(); i++) {
				Point2d angles = pxToAngles(crop.rows, targetAperture, centers[i].x, centers[i].y);
				//Vec3d vec = anglesToVector(angles, distances[i]);
				Vec3d vec = anglesToVector(angles, avgDist);
				avgDistProj += norm(projPos - vec) / centers.size();
				vec += -projPos;
				angles = vectorToAngles(vec);
				centers[i] = anglesToPx(crop.rows, targetAperture, angles);
			}
			//cout << "dist: " << avgDist << " from proj: " << avgDistProj << endl;
			//Point pts2[] = { centers[0], centers[1], centers[3], centers[7], centers[9], centers[8] };
			Point pts2[] = { centers[0], centers[1], centers[3], centers[2] };
			fillConvexPoly(out, pts2, sizeof(pts2)/sizeof(pts2[0]), Scalar(0, 0, 255));
			
			Point2f pts3a[] = { centers[0], centers[1], centers[2], centers[3] }; // vertical
			//Point2f pts3a[] = { centers[3], centers[2], centers[1], centers[0] }; // vertical
			Point2f pts3b[] = { centers[1], centers[3], centers[0], centers[2] }; // horizontal
			float dx = norm(centers[0] - centers[1]);
			float dy = norm(centers[0] - centers[2]);
			dx = max(dx, -dx);
			dy = max(dy, -dy);
			Mat perspTrans;
			if (dx > dy) // horizontal
				perspTrans = getPerspectiveTransform(imgPts, pts3b);
			else // vertical
				perspTrans = getPerspectiveTransform(imgPts, pts3a);
			
			#ifdef DEBUG
			Mat warp;
			warpPerspective(image, warp, perspTrans, Size(out.cols, out.rows));
			int numberOfPixels = warp.rows * warp.cols;
			int ch = warp.channels();
			CV_DbgAssert(ch == 3);
			uchar* fptr = reinterpret_cast<uchar*>(warp.data);
			uchar* optr = reinterpret_cast<uchar*>(out.data);
			for (int i = 0; i < numberOfPixels; i++, fptr+=ch, optr+=ch) {
				if ((fptr[0] | fptr[1] | fptr[2]) > 0) {
					optr[0] = fptr[0];
					optr[1] = fptr[1];
					optr[2] = fptr[2];
				}
			}
			#else
			warpPerspective(image, out, perspTrans, Size(out.cols, out.rows));
			#endif
		}

		if (debug180)
			line(out, Point(out.cols/2, out.rows/2), Point(out.cols/2, out.rows), Scalar(0, 255, 0), 3);

		Mat presentation;
		resize(out, presentation, Size(768, 768));
		Mat tmp;
		flip(presentation, tmp, 1);
		presentation = tmp;
		hconcat(border, presentation, presentation);
		hconcat(presentation, border, presentation);


		// mark projector on original camera frame
		circle(*p.frame, anglesToPx(p.frame->rows, sourceAperture, vectorToAngles(projPos)), 10, Scalar(0, 0, 255), -1);

		Mat tmp1, tmp2, tmp3;
		double compression = 0.5;
		resize(out, tmp1, Size(), compression, compression);
		resize(*p.frame, tmp2, Size(), compression, compression);
		//resize(skinframe, tmp3, Size(), compression, compression);
		
		imshow("track", tmp1);
		imshow("orig", tmp2);
		imshow("Fullscreen ws5", presentation);
		//imshow("skin", tmp3);
		//imshow("converted", *equi);



		end_time = high_resolution_clock::now();
		time_span = end_time - start_time;
		// calculate remaining time to sleep in order to match camera's fps
		sleep = frame_duration.count() - time_span.count() - 1; // substract 1 so we are always ahead of the camera
		//printf("sleep = %d ms\n", sleep);
		if (is_first_frame) {
			sleep = 1;
			is_first_frame = false;
		} else if (sleep <= 0) {
			//printf("too many frames expected --------------------  %d ms\n", sleep);
			sleep = 1;
		}
		int val = waitKey(sleep);
		if (val > 0) {
			bool keyIsHeld = val == lastFrameKey;
			printf("key = %d\n", val);
			switch (val) {
			case 'd':
				debug180 = !debug180;
				break;
			case 82: // up
				targetAperture += (1+3*keyIsHeld)*CV_PI/180;
				cout << "new target aperture size: " << targetAperture*180/CV_PI << endl;
				break;
			case 84: // down
				targetAperture -= (1+3*keyIsHeld)*CV_PI/180;
				cout << "new target aperture size: " << targetAperture*180/CV_PI << endl;
				break;
			case 151: // numpad up w/o numlock
			case 184: { // numpad up
				Point2d angles = vectorToAngles(projPos);
				angles.y += projectorAngularCorrectionStep + projectorAngularCorrectionStep*3*keyIsHeld;
				projPos = anglesToVector(angles, norm(projPos));
				cout << "new projector position: " << projPos << endl;
				break;
			}
			case 157: // numpad center w/o numlock
			case 153: // numpad down w/o numlock
			case 181: // numpad center
			case 178: { // numpad down
				Point2d angles = vectorToAngles(projPos);
				angles.y -= projectorAngularCorrectionStep + projectorAngularCorrectionStep*3*keyIsHeld;
				projPos = anglesToVector(angles, norm(projPos));
				cout << "new projector position: " << projPos << endl;
				break;
			} case 150:
			case 180: { // numpad left
				Point2d angles = vectorToAngles(projPos);
				angles.x += projectorAngularCorrectionStep + projectorAngularCorrectionStep*3*keyIsHeld;
				projPos = anglesToVector(angles, norm(projPos));
				cout << "new projector position: " << projPos << endl;
				break;
			} case 152:
			case 182: { // numpad right
				Point2d angles = vectorToAngles(projPos);
				angles.x -= projectorAngularCorrectionStep + projectorAngularCorrectionStep*3*keyIsHeld;
				projPos = anglesToVector(angles, norm(projPos));
				cout << "new projector position: " << projPos << endl;
				break;
			} case 171: { // numpad +
				projPos += projPos / norm(projPos) * (0.1 + 0.1*3*keyIsHeld);
				cout << "new projector position: " << projPos << endl;
				break;
			} case 173: { // numpad -
				projPos -= projPos / norm(projPos) * (0.1 + 0.1*3*keyIsHeld);
				cout << "new projector position: " << projPos << endl;
				break;
			} case 27:  // esc
			case 'q':
				cap.release();
				break;
			}
			// force sleeping 1ms regardless of key press USING waitKey
			// (required if we press and hold a key while forcing opencv to redraw imshow^^)
			start_time = high_resolution_clock::now();
			do {
				waitKey(1);
				end_time = high_resolution_clock::now();
				time_span = end_time - start_time;
			} while (time_span.count() < 1);
		}
		lastFrameKey = val;
	}
//video.release();
	// the camera will be deinitialized automatically in VideoCapture destructor
	t1.join();
	return 0;
}
