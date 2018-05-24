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
#include "SphereWorld.h"
#include "FisheyeFeeder.h"
#include "EquirectRetriever.h"
#include "FisheyeRetriever.h"


using namespace cv;
using namespace std;
using namespace std::chrono;


Mat origFile;
Mat rotationMatrix;

/*
Point2d PXtoLatLong(Point pos) {
	double ex = (double)(pos.x - er.frame.cols/2) / (er.frame.cols/2);
	double ey = (double)(pos.y - er.frame.rows/2) / (er.frame.rows/2);
	double longitude = ex * CV_PI;
	double latitude = ey * CV_PI/2;
	return Point2d(longitude, latitude);
}
*/
void outCallBack(int event, int x, int y, int flags, void* userdata) {
	if ( event == EVENT_LBUTTONDOWN ) {
	//	Point2d r = PXtoLatLong(Point2d(x, y));
	//	Mat rot = fe.genRotMat(Vec3d(-r.y, r.x, 0));
		//rot = rot * er.rotationMatrix;
	//	rotationMatrix = er.rotationMatrix * rot;
	} else if ( event == EVENT_MOUSEWHEEL ) {
		double r = getMouseWheelDelta(flags) > 0 ? 10*CV_PI/180 : -10*CV_PI/180;
	//	Mat rot = fe.genRotMat(Vec3d(0, 0, r));
		//rot = rot * er.rotationMatrix;
	//	rotationMatrix = er.rotationMatrix * rot;
	} else if ( event == EVENT_LBUTTONUP ) {
	} else if ( event == EVENT_MOUSEMOVE ) {
	}
}

class Page {
public:
	Page(Mat m, high_resolution_clock::time_point t) { frame = m; time = t; }
	Mat frame;
	high_resolution_clock::time_point time;
};

bool localFile = false;
SharedVideoCapture cap(1); // open the default camera
SharedQueue<Page> frames;

void cameraThread() {
	Mat frame;
	while (cap.isOpened()) {
		cap.read(frame);
		if (frame.empty())
			cap.release();
		else
			frames.push_back(Page(frame.clone(), high_resolution_clock::now()));
		if (localFile)
			this_thread::sleep_for(std::chrono::milliseconds(66));
			//waitKey(66); // 66.66ms = 15 fps
	}
	cap.release();
}


int main(int argc, char** argv) {
	
	// https://www.learnopencv.com/read-write-and-display-a-video-using-opencv-cpp-python/
	if (argc == 2) {
		cap.open(string(argv[1]));
		localFile = true;
	}
	if(!cap.isOpened())  // check if we succeeded
		return -1;

	namedWindow("converted", 1);
	setMouseCallback("converted", outCallBack, NULL);

	//cap.set(CAP_PROP_FRAME_HEIGHT, 1440);
	//cap.set(CAP_PROP_FRAME_WIDTH, 1440);
	cap.set(CAP_PROP_FRAME_HEIGHT, 1200);
	cap.set(CAP_PROP_FRAME_WIDTH, 1200);
	int w = cap.get(CAP_PROP_FRAME_WIDTH);
	int h = cap.get(CAP_PROP_FRAME_HEIGHT);



	// prepare
	int radius = w/2;
	FisheyeFeeder *ff = new FisheyeFeeder(235*CV_PI/180, Point2d(0, -CV_PI/2), radius);
	SphereWorld *sworld = new SphereWorld(radius);
	ff->connect(sworld);
	EquirectRetriever *er = new EquirectRetriever(2*CV_PI, CV_PI, 1.0);
	er->connect(sworld);
	//er->setRotation(er->genRotMat(Vec3d(CV_PI/2, 0, 0)));
	er->setRotation(er->genRotMat(Vec3d(0, 0, 0)));
	FisheyeRetriever *fr = new FisheyeRetriever(235*CV_PI/180, 1.0);
	fr->connect(sworld);
	fr->setRotation(fr->genRotMat(Vec3d(CV_PI/2, 0, 0)));
cout << "begin math" << endl;
high_resolution_clock::time_point a1 = high_resolution_clock::now();
	er->invalidateMaths();
high_resolution_clock::time_point a2 = high_resolution_clock::now();
cout << "elapsed " << ((duration<double, std::milli>)(a2-a1)).count() << endl;
	fr->invalidateMaths();
#ifdef DEBUG
exit(0);
#endif

//	ff.setAnchor(Point2d(0, -CV_PI));
//	ff.setApertureSize();
//	ff.invalidateMaths();
	Mat display;
	//namedWindow("edges",1);



	// prepare required variables for the main loop
	bool is_first_frame = true;
	int sleep;
	high_resolution_clock::time_point start_time;
	high_resolution_clock::time_point end_time;
	duration<double, std::milli> time_span;
	high_resolution_clock::time_point last_frame_time = high_resolution_clock::now();

	// start camera thread
    thread t1(cameraThread);

	while (cap.isOpened()) {

		if (frames.size() == 0) {
			//printf("waiting for new frame\n");
			continue;
		}

		// start timer and read next frame with timestamp
		start_time = high_resolution_clock::now();
		if (frames.size() > 1) {
			printf("SKIPPING %d frames\n", frames.size()-1);
			while (frames.size() > 2)
				frames.pop_front();
			last_frame_time = frames.front().time;
			frames.pop_front();
		}
		Page p = frames.front();
		frames.pop_front();
		duration<double, std::milli> frame_duration = p.time - last_frame_time;
		last_frame_time = p.time;

		// prepare frame
		shared_ptr<Mat> frame = make_shared<Mat>();
		resize(p.frame, *frame, Size(), 1.0, 1.0);

		ff->updateFrame(frame);
		shared_ptr<Mat> equi = er->getFrame();
		shared_ptr<Mat> fish = fr->getFrame();
		//fe.frame = frame;
		//er = fe.toEquirectangular(rotationMatrix);

		Mat tmp1, tmp2, tmp3;
		double compression = 0.25;
		resize(*frame, tmp1, Size(), compression, compression);
		resize(*equi, tmp2, Size(), compression, compression);
		resize(*fish, tmp3, Size(), compression, compression);

		imshow("orig", tmp1);
		imshow("converted", tmp2);
		//imshow("fish", tmp3);
		//imshow("orig", *frame);
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
			printf("too many frames expected ----------------------\n");
			printf("we are behind by %d ms\n", sleep);
			sleep = 1;
		}
		int val = waitKey(sleep);
		if (val > 0) {
			printf("key = %d\n", val);
			switch (val) {
			case 27:  // esc
			case 'q':
				cap.release();
				break;
			}
		}
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	t1.join();
	delete fr;
	delete er;
	delete ff;
	delete sworld;
	return 0;
}
