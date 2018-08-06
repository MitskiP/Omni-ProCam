#include <ctime>
#include <ratio>
#include <chrono>
#include <thread>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ccalib/omnidir.hpp"
#include <algorithm>
#include <chrono>
#include <memory>

#include <opencv2/aruco.hpp>

#include "Helper.h"
#include "Sheet.h"

using namespace cv;
using namespace cv::aruco;
using namespace std;
using namespace std::chrono;


// output projector dimension
#define OHEIGHT 1050
#define OWIDTH 1400
#define OOFFSET ((OWIDTH-OHEIGHT)/2)


int main(int argc, char **argv) {

	int camID = 1;
	int waitTime = 1;
	VideoCapture inputVideo;
	if (argc == 2) {
		// open video file
		inputVideo.open(string(argv[1]));
		waitTime = 33;
	} else {
		// open Kodak POXPRO SP360 4k (235° fisheye camera)
		inputVideo.open(camID);
		inputVideo.set(CAP_PROP_FRAME_HEIGHT, 1440);
		inputVideo.set(CAP_PROP_FRAME_WIDTH, 1440);
	}

	// set dictionary to detect markers and setup sheets
	Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_4X4_250);
	vector<Sheet> sheets;
	/*
	 * Sheet(int markerID, Size2f sheetSize, float markerSize, Point2f markerTopLeft, Point2f drawingTopLeft, Point2f drawingBottomRight,
	 * string mediaFileStr, bool transpose, int framesPerProjFrame)
	 * markerID				- id of marker in dictionary
	 * sheetSize			- size of total frame which includes marker
	 * markerSize			- length of a side of a marker
	 * markerTopLeft		- position of top left corner of marker on sheet (origin @ top left of sheet)
	 * drawingTopLeft		- position of top left corner of drawing area
	 * drawingBottomRight	- position of bottom right corner of drawing area
	 * mediaFileStr			- media file to project on drawing area
	 * transpose			- whether to transpose frame of media before porjection
	 * framesPerProjFrame	- seek n frames for next projection
	 * if mediaFileStr is an image, set framesPerProjFrame to 0 !!!
	 */
	//sheets.push_back(Sheet(0, Size2f(29.7, 21), 4.4, Point2f(0.9, 0.9), Point2f(5.3, 5.3), Point2f(29.7, 21), "frame.png", false, 0));
	sheets.push_back(Sheet(0, Size2f(29.7, 21), 4.4, Point2f(0.9, 0.9), Point2f(5.3, 5.3), Point2f(29.7, 21), "res/parrot_250fps.mkv", false, 2));
	sheets.push_back(Sheet(1, Size2f(21, 29.7), 8.8, Point2f(0.9, 0.9), Point2f(0.9, 8.8+0.9), Point2f(21, 29.7), "res/penguin_250fps.mp4", false, 2));
	sheets.push_back(Sheet(2, Size2f(21, 29.7), 8.8, Point2f(0.9, 0.9), Point2f(0.9, 0.9), Point2f(21, 29.7), "res/rabbit_250fps.mp4", true, 2));
	sheets.push_back(Sheet(3, Size2f(21, 29.7), 8.8, Point2f(0.9, 0.9), Point2f(0.9, 8.8+0.9), Point2f(21, 29.7), "res/parrot_250fps.mkv", false, 2));
	sheets.push_back(Sheet(4, Size2f(42, 60), 8.8, Point2f(0.9, 0.9), Point2f(0.9, 0.9), Point2f(42, 60), "res/shio.png", false, 0));
	sheets.push_back(Sheet(5, Size2f(21, 29.7), 8.8, Point2f(0.9, 0.9), Point2f(0.9, 0.9), Point2f(21, 29.7), "res/john.jpg", false, 0));
	sheets.push_back(Sheet(6, Size2f(21, 29.7), 8.8, Point2f(0.9, 0.9), Point2f(0.9, 0.9), Point2f(21, 29.7), "res/maeda.jpg", false, 0));
	//sheets.push_back(Sheet(1, Size2f(29.7, 21), 8.8, Point2f(0.9, 0.9), Point2f(8.8+0.9, 8.8+0.9), Point2f(29.7, 21), "penguin.mp4", false, 2));
	
	// read camera parameters
	Mat cameraMatrix, distCoeffs;
	double xi;
	if (!readCameraParameters("res/omni_calibration.xml", cameraMatrix, distCoeffs))
		return -1;
	FileStorage fs("res/omni_calibration.xml", FileStorage::READ);
	fs["xi"] >> xi;

	cout << cameraMatrix << endl;
	cout << distCoeffs << endl;

	// set undistortion parameters
	Size new_size(1200, 1200);
	Matx33f Knew = Matx33f(new_size.width/8, 0, new_size.width/2,
		               0, new_size.height/8, new_size.height/2,
		               0, 0, 1);

	// set real world parameters
	Vec3d projPos(0, -7.49213, 18.5437); // projector position relative to camera
	double targetAperture = 140*CV_PI/180;

	// runtime configs and variables
	double projectorAngularCorrectionStep = 1*CV_PI/180;
	bool debug180 = true;
	int lastFrameKey = -1;

	Mat image, undistort;
	// border to add left and right to center projection image
	Mat border = Mat(OHEIGHT, OOFFSET, CV_8UC3, Scalar(0, 0, 0));
	for (long frameNo = 0; inputVideo.grab(); frameNo++) {

		inputVideo.retrieve(image);
		Mat out(image.rows, image.cols, image.type(), Scalar(0, 0, 0));

		omnidir::undistortImage(image, undistort, cameraMatrix, distCoeffs, xi, omnidir::RECTIFY_PERSPECTIVE, Knew, new_size);
		//omnidir::undistortImage(image, image, cameraMatrix, distCoeffs, xi, omnidir::RECTIFY_LONGLATI, Knew, new_size);
		//flip(image, image, 0);

		vector<int> idsO, idsU; // O = original, U = undistort
		vector<vector<Point2f> > cornersO, cornersU;
		detectMarkers(undistort, dictionary, cornersU, idsU);
		
		vector<int> sheetID(idsU.size());
		// filter only markers we currently use
		for (size_t i = 0; i < idsU.size(); i++) {
			bool remove = true;
			for (size_t k = 0; k < sheets.size(); k++) {
				if (idsU[i] == sheets[k].id) {
					remove = false;
					sheetID[i] = k;
					break;
				}
			}
			if (remove) {
				idsU.erase(idsU.begin() + i);
				cornersU.erase(cornersU.begin() + i);
				sheetID.erase(sheetID.begin() + i);
				i--;
			}
		}
		
		// if at least one marker detected
		if (idsU.size() > 0) {
			// detect now in original image
			detectMarkers(image, dictionary, cornersO, idsO);
			
			// calculate intersection of detected markers in original AND undistorted
			for (size_t i = 0; i < idsU.size(); i++) {
				if(find(idsO.begin(), idsO.end(), idsU[i]) == idsO.end()) {
					idsU.erase(idsU.begin() + i);
					cornersU.erase(cornersU.begin() + i);
					sheetID.erase(sheetID.begin() + i);
					i--;
				}
			}
			for (size_t i = 0; i < idsO.size(); i++) {
				if(find(idsU.begin(), idsU.end(), idsO[i]) == idsU.end()) {
					idsO.erase(idsO.begin() + i);
					cornersO.erase(cornersO.begin() + i);
					i--;
				}
			}

			// draw over QR code for debugging
			for (size_t m = 0; m < idsO.size(); m++) {
				Point pts[] = { cornersO[m][0], cornersO[m][1], cornersO[m][2], cornersO[m][3] };
				fillConvexPoly(image, pts, sizeof(pts)/sizeof(pts[0]), Scalar(0, 255, 255));
			}
			// correct marker corners position according to camera/projector position difference
			vector<Vec3d> rvecs, tvecs;
			Mat d;
			estimatePoseSingleMarkers(cornersU, 1.0, Knew, d, rvecs, tvecs);
			vector<double> distances(idsU.size());
			for (size_t k = 0; k < idsU.size(); k++) {
				//drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10.0);
			//	drawAxis(image, Knew, d, rvecs[i], tvecs[i], 10.0);

				// calculate distance to each marker
				distances[k] = norm(tvecs[k]) * sheets[sheetID[k]].mSize;
				cout << distances[k] << endl;
				
				// 0 3
				// 1 2
				// apply correction to each corner
				for (size_t i = 0; i < cornersO[k].size(); i++) {
					Point2d angles = pxToAngles(image.rows, 235*CV_PI/180, cornersO[k][i].x, cornersO[k][i].y);
					Vec3d vec = anglesToVector(angles, distances[k]);
					vec += -projPos;
					angles = vectorToAngles(vec);
					cornersO[k][i] = anglesToPx(image.rows, 235*CV_PI/180, angles);

					//sheets[sheetID[k]].realWorld[i] = vec;
					//sheets[sheetID[k]].projection[i] = cornersO[k][i];
					//sheets[sheetID[k]].updatedFrame = frameNo;

					#define HISTORY_SIZE 3
					sheets[sheetID[k]].realWorld[i] = (sheets[sheetID[k]].realWorld[i]*(HISTORY_SIZE-1) + vec) / HISTORY_SIZE;
					sheets[sheetID[k]].projection[i].x = (sheets[sheetID[k]].projection[i].x*(HISTORY_SIZE-1) + cornersO[k][i].x) / HISTORY_SIZE;
					sheets[sheetID[k]].projection[i].y = (sheets[sheetID[k]].projection[i].y*(HISTORY_SIZE-1) + cornersO[k][i].y) / HISTORY_SIZE;
					sheets[sheetID[k]].updatedFrame = frameNo;
				}

				// calculate perspective transformation matrix for marker
				//Point2f pts3[] = { cornersO[k][0], cornersO[k][1], cornersO[k][3], cornersO[k][2] }; // target points
				Point2f pts3[] = { sheets[sheetID[k]].projection[0], sheets[sheetID[k]].projection[1], sheets[sheetID[k]].projection[3], sheets[sheetID[k]].projection[2] }; // target points
				Size content = sheets[sheetID[k]].getContentSize();
				float h = content.width / (sheets[sheetID[k]].br.x - sheets[sheetID[k]].tl.x); // calculate pixels (on content) per marker size
				float v = content.height / (sheets[sheetID[k]].br.y - sheets[sheetID[k]].tl.y);
				Point2f tl = Point2f( (0 - sheets[sheetID[k]].tl.x) * h, (0 - sheets[sheetID[k]].tl.y) * v);
				Point2f br = Point2f( (1.0f - sheets[sheetID[k]].tl.x) * h, (1.0f - sheets[sheetID[k]].tl.y) * v);
				Point2f tr = Point2f(br.x, tl.y);
				Point2f bl = Point2f(tl.x, br.y);
				Point2f imgPts[] = { tl, tr, bl, br }; // source points of marker according to content
				sheets[sheetID[k]].perspTrans = getPerspectiveTransform(imgPts, pts3);
			}
		}

		// draw tracked sheets
#define DBG_CONTENT
		for (size_t k = 0; k < sheets.size(); k++) {
			if (frameNo - sheets[k].updatedFrame > 30) // ~15 fps
				continue;
			Mat content = sheets[k].nextFrame();
			Mat warp;
#ifndef DBG_CONTENT
			warpPerspective(content, image, sheets[k].perspTrans, Size(image.cols, image.rows));
			warpPerspective(content, out, sheets[k].perspTrans, Size(image.cols, image.rows));
#else
			warpPerspective(content, warp, sheets[k].perspTrans, Size(image.cols, image.rows));
			int numberOfPixels = warp.rows * warp.cols;
			int ch = warp.channels();
			uchar* fptr = reinterpret_cast<uchar*>(warp.data);
			uchar* optr = reinterpret_cast<uchar*>(image.data);
			uchar* optr2 = reinterpret_cast<uchar*>(out.data);
			for (int i = 0; i < numberOfPixels; i++, fptr+=ch, optr+=ch, optr2+=ch) {
				if ((fptr[0] | fptr[1] | fptr[2]) > 0) {
					optr[0] = fptr[0];
					optr[1] = fptr[1];
					optr[2] = fptr[2];
					optr2[0] = fptr[0];
					optr2[1] = fptr[1];
					optr2[2] = fptr[2];
				}
			}
#endif
		}

		// draw projector dot
		circle(image, anglesToPx(image.rows, 235*CV_PI/180, vectorToAngles(projPos)), 10, Scalar(0, 0, 255), -1);
		// visualize target aperture size
		circle(image, Point2f(image.cols/2, image.rows/2), image.cols/2*targetAperture/(235*CV_PI/180), Scalar(0, 0, 255), 2);

		// cut out to targetAperture size
		Rect roi;
		roi.width = out.cols * targetAperture / (235.0f*CV_PI/180);
		roi.height = out.rows * targetAperture / (235.0f*CV_PI/180);
		roi.x = (out.cols - roi.width) / 2;
		roi.y = (out.rows - roi.height) / 2;
		out = out(roi);

		// draw 0° line for calibration
		if (debug180)
			line(out, Point(out.cols/2, out.rows/2), Point(out.cols/2, out.rows), Scalar(0, 255, 0), 3);

		// fit presentation to output projector resolution
		Mat presentation;
		resize(out, presentation, Size(OHEIGHT, OHEIGHT));
		flip(presentation, presentation, 1);
		hconcat(border, presentation, presentation);
		hconcat(presentation, border, presentation);

		// show images
		resize(image, image, Size(), 0.5, 0.5);
		imshow("floating", image);
		imshow("Fullscreen ws5", presentation);


		// handle keyboard inputs
		int val = waitKey(waitTime);
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
				inputVideo.release();
				break;
			}
		}
		lastFrameKey = val;
	}
}
