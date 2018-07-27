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

#include "SharedQueue.h"
#include "SharedVideoCapture.h"
#include "BetterBlobDetector.h"

#include "Helper.h"
#include "Sheet.h"

using namespace cv;
using namespace cv::aruco;
using namespace std;
using namespace std::chrono;


#define OHEIGHT 1050
#define OWIDTH 1400
#define OOFFSET ((OWIDTH-OHEIGHT)/2)


int main(int argc, char **argv) {

	int camID = 1;
	int waitTime = 1;
	VideoCapture inputVideo;
	if (argc == 2) {
		inputVideo.open(string(argv[1]));
		waitTime = 33;
	} else {
		inputVideo.open(camID);
		inputVideo.set(CAP_PROP_FRAME_HEIGHT, 1440);
		inputVideo.set(CAP_PROP_FRAME_WIDTH, 1440);
	}

	// set dictionary to detect and setup sheets
	Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_4X4_250);
	vector<Sheet> sheets;
	sheets.push_back(Sheet(0, Size2f(29.7, 21), 4.4, Point2f(0.9, 0.9), Point2f(5.3, 5.3), Point2f(29.7, 21), "frame.png", false, 1));
	//sheets.push_back(Sheet(1, Size2f(29.7, 21), 8.8, Point2f(0.9, 0.9), Point2f(8.8+0.9, 8.8+0.9), Point2f(29.7, 21), "penguin.mp4", false, 2));
	sheets.push_back(Sheet(1, Size2f(21, 29.7), 8.8, Point2f(0.9, 0.9), Point2f(0.9, 8.8+0.9), Point2f(21, 29.7), "penguin.mp4", false, 2));
	//sheets.push_back(Sheet(3, Size2f(29.7, 21), 4.4, Point2f(5.3, 5.3), Point2f(29.7, 21)));
	
	// read camera parameters
	Mat cameraMatrix, distCoeffs;
	double xi;
	if (camID == 0) {
		if (!readCameraParameters("../calibration/webcam_out.xml", cameraMatrix, distCoeffs))
			return -1;
	} else {
		//if (!readCameraParameters("../calibration/out_camera_data.xml", cameraMatrix, distCoeffs))
		//if (!readCameraParameters("../calibration/perspective_out.xml", cameraMatrix, distCoeffs))
		//if (!readCameraParameters("../calibration/perspective_out.xml", cameraMatrix, distCoeffs))
		if (!readCameraParameters("../testing/omnidirectional_calibration/samples/omni_out.xml", cameraMatrix, distCoeffs))
			return -1;

		FileStorage fs("../testing/omnidirectional_calibration/samples/omni_out.xml", FileStorage::READ);
		fs["xi"] >> xi;
	}
	cout << cameraMatrix << endl;
	cout << distCoeffs << endl;

//Size new_size(1000, 1000);
Size new_size(1200, 1200);
//Matx33f Knew = Matx33f(new_size.width/4, 0, new_size.width/2,
//                   0, new_size.height/4, new_size.height/2,
//                   0, 0, 1);
Matx33f Knew = Matx33f(new_size.width/8, 0, new_size.width/2,
                   0, new_size.height/8, new_size.height/2,
                   0, 0, 1);
Size new_sizeLL(1200, 1200);
Matx33f KnewLL = Matx33f(new_sizeLL.width/(CV_PI), 0, 0,
                   0, new_sizeLL.height/(CV_PI), 0,
                   0, 0, 1);
//Knew = KnewLL;
//new_size = new_sizeLL;


//	int longitude = 0, latitude = -45;
//	int ha = 90, va = 90;
	
	
	Vec3d projPos(0, -7.49213, 18.5437);
	double targetAperture = 140*CV_PI/180;
	bool debug180 = true;
	double projectorAngularCorrectionStep = 1*CV_PI/180;

	int lastFrameKey = -1;


	Mat image;
	Mat border = Mat(OHEIGHT, OOFFSET, CV_8UC3, Scalar(0, 0, 0));
	for (long frameNo = 0; inputVideo.grab(); frameNo++) {

		inputVideo.retrieve(image);
		Mat undistort;
		Mat out(image.rows, image.cols, image.type(), Scalar(0, 0, 0));

		omnidir::undistortImage(image, undistort, cameraMatrix, distCoeffs, xi, omnidir::RECTIFY_PERSPECTIVE, Knew, new_size);
		//omnidir::undistortImage(image, image, cameraMatrix, distCoeffs, xi, omnidir::RECTIFY_LONGLATI, Knew, new_size);
		//flip(image, image, 0);

		vector<int> idsO, idsU;
		vector<vector<Point2f> > cornersO, cornersU;
		detectMarkers(undistort, dictionary, cornersU, idsU);
		
		vector<int> sheetID(idsU.size());
		// filter for only the markers we have
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
			
			// calculate intersection
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


			// draw over QR code
			for (size_t m = 0; m < idsO.size(); m++) {
				Point pts[] = { cornersO[m][0], cornersO[m][1], cornersO[m][2], cornersO[m][3] };
				fillConvexPoly(image, pts, sizeof(pts)/sizeof(pts[0]), Scalar(0, 255, 255));
				//fillConvexPoly(out, pts, sizeof(pts)/sizeof(pts[0]), Scalar(0, 255, 255));
//circle(image, cornersO[m][0], 10, Scalar(0, 0, 255), -1);
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
				
/*				// 0 3
				// 1 2
				// calculate effective drawing durface
				Point2f hdiff = (cornersO[k][3] - cornersO[k][0] + cornersO[k][2] - cornersO[k][1]) / 2;
				Point2f vdiff = (cornersO[k][1] - cornersO[k][0] + cornersO[k][2] - cornersO[k][3]) / 2;
				cornersO[k][2] = cornersO[k][0] + hdiff * sheets[sheetID[k]].br.x + vdiff * sheets[sheetID[k]].br.y;
				cornersO[k][1] = cornersO[k][0] + hdiff * sheets[sheetID[k]].tl.x + vdiff * sheets[sheetID[k]].br.y;
				cornersO[k][3] = cornersO[k][0] + hdiff * sheets[sheetID[k]].br.x + vdiff * sheets[sheetID[k]].tl.y;
				cornersO[k][0] = cornersO[k][0] + hdiff * sheets[sheetID[k]].tl.x + vdiff * sheets[sheetID[k]].tl.y;

				// draw effective area before correctoin
				Point pts[] = { cornersO[k][0], cornersO[k][1], cornersO[k][2], cornersO[k][3] };
				fillConvexPoly(image, pts, sizeof(pts)/sizeof(pts[0]), Scalar(255, 0, 0));
				fillConvexPoly(out, pts, sizeof(pts)/sizeof(pts[0]), Scalar(255, 0, 0));
*/				
				// apply correction to each corner
				for (size_t i = 0; i < cornersO[k].size(); i++) {
					Point2d angles = pxToAngles(image.rows, 235*CV_PI/180, cornersO[k][i].x, cornersO[k][i].y);
					Vec3d vec = anglesToVector(angles, distances[k]);
					vec += -projPos;
					angles = vectorToAngles(vec);
					cornersO[k][i] = anglesToPx(image.rows, 235*CV_PI/180, angles);

					sheets[sheetID[k]].realWorld[i] = vec;
					sheets[sheetID[k]].projection[i] = cornersO[k][i];
					sheets[sheetID[k]].updatedFrame = frameNo;
				}

				// calculate perspective matrix for marker
				Point2f pts3[] = { cornersO[k][0], cornersO[k][1], cornersO[k][3], cornersO[k][2] }; // target points
				//Point2f pts3[] = { sheets[sheetID[k]].projection[0], sheets[sheetID[k]].projection[1], sheets[sheetID[k]].projection[3], sheets[sheetID[k]].projection[2] }; // target points
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

		circle(image, anglesToPx(image.rows, 235*CV_PI/180, vectorToAngles(projPos)), 10, Scalar(0, 0, 255), -1);
		circle(image, Point2f(image.cols/2, image.rows/2), image.cols/2*targetAperture/(235*CV_PI/180), Scalar(0, 0, 255), 2);

		Rect roi;
		roi.width = out.cols * targetAperture / (235.0f*CV_PI/180);
		roi.height = out.rows * targetAperture / (235.0f*CV_PI/180);
		roi.x = (out.cols - roi.width) / 2;
		roi.y = (out.rows - roi.height) / 2;
		out = out(roi);

		if (debug180)
			line(out, Point(out.cols/2, out.rows/2), Point(out.cols/2, out.rows), Scalar(0, 255, 0), 3);

		Mat presentation;
		resize(out, presentation, Size(OHEIGHT, OHEIGHT));
		flip(presentation, presentation, 1);
		hconcat(border, presentation, presentation);
		hconcat(presentation, border, presentation);

		resize(image, image, Size(), 0.5, 0.5);
		imshow("floating", image);
		imshow("Fullscreen ws5", presentation);

/*
		if (ids.size() == 1 || ids.size() == 4) {
			double avgLongitude = pxToAngles(image.cols, 235*CV_PI/180, corners[0][0]).x;
			if (ids.size() == 4) {
				double l1 = pxToAngles(image.cols, 235*CV_PI/180, corners[0][0]).x;
				double l2 = pxToAngles(image.cols, 235*CV_PI/180, corners[1][0]).x;
				double l3 = pxToAngles(image.cols, 235*CV_PI/180, corners[2][0]).x;
				double l4 = pxToAngles(image.cols, 235*CV_PI/180, corners[3][0]).x;
				avgLongitude = avgAngle( avgAngle(l1, l2), avgAngle(l3, l4) );
			}

			Mat rotMat = genRotMat(Vec3d(latitude*CV_PI/180, avgLongitude, 0*CV_PI/180));
			Mat rotMatI = rotMat.inv();
			Mat perspective = toPerspective(image, 235*CV_PI/180, rotMat, Point2d(ha*CV_PI/180, va*CV_PI/180));
			
			
			float radius = image.cols / (235*CV_PI/180);
			for (size_t i = 0; i < corners.size(); i++) {
				for (size_t k = 0; k < corners[i].size(); k++) {
					Point2d angles = pxToAngles(image.cols, 235*CV_PI/180, corners[i][k]);

					Vec3d v = anglesToVector(angles, 1);
					Mat unrotated = rotMatI * Mat(v);
					Vec3d unrotatedV = Vec3d(unrotated.at<double>(0, 0), unrotated.at<double>(1, 0), unrotated.at<double>(2, 0));
					angles = vectorToAngles(unrotatedV);

					double len = radius / cos(angles.x) / cos(angles.y);
					//Vec3d vec = anglesToVector(angles, len);
					Vec3d vec = unrotatedV * len;
					float x = -vec[0] + perspective.cols/2;
					float y = -vec[1] + perspective.rows/2;

					corners[i][k] = Point2f(x, y);
				}
			}
//			drawDetectedMarkers(perspective, corners, ids);
//			drawDetectedMarkers(imageCopy, corners, ids);

//			detectMarkers(perspective, dictionary, corners, ids);
			drawDetectedMarkers(perspective, corners, ids);

			//Mat cameraMatrix, distCoeffs;
			vector<Vec3d> rvecs, tvecs;
			//estimatePoseSingleMarkers(corners, 4.3, cameraMatrix, distCoeffs, rvecs, tvecs);
			estimatePoseSingleMarkers(corners, 7.0, cameraMatrix, distCoeffs, rvecs, tvecs);
			// draw axis for each marker
			for(size_t i=0; i<ids.size(); i++) {
				//rvecs[i] = Vec3d(0, 0, 0);
				//tvecs[i][0] += 10;
				//drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10.0);
				drawAxis(perspective, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10.0);
				//cout << tvecs[i] << endl;
				cout << norm(tvecs[i]) << endl;
			}
			imshow("perspective", perspective);
		}
*/


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









