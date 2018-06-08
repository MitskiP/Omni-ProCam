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
			cap.release();
		else
			frames.push_back(Page(frame, high_resolution_clock::now()));
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

//	namedWindow("converted", 1);
//	setMouseCallback("converted", outCallBack, NULL);

	//cap.set(CAP_PROP_FRAME_HEIGHT, 1440);
	//cap.set(CAP_PROP_FRAME_WIDTH, 1440);
	cap.set(CAP_PROP_FRAME_HEIGHT, 1200);
	cap.set(CAP_PROP_FRAME_WIDTH, 1200);

	/*cap.set(CV_CAP_PROP_EXPOSURE, 0.5);
	cap.set(CV_CAP_PROP_GAIN, 0.5);
	cout << cap.get(CAP_PROP_EXPOSURE) << ", " << cap.get(CAP_PROP_GAIN) << endl;*/

	int w = cap.get(CAP_PROP_FRAME_WIDTH);
	int h = cap.get(CAP_PROP_FRAME_HEIGHT);


	cout << w << ", " << h << endl;


	// prepare
	//int radius = w/2;
	double radius = 768/CV_PI; // surface: 768 pixels, 180°
	FisheyeFeeder *ff = new FisheyeFeeder(235*CV_PI/180, Point2d(0, -CV_PI/2), w/2);
	SphereWorld *sworld = new SphereWorld(radius);
	ff->connect(sworld);
	EquirectRetriever *er = new EquirectRetriever(2*CV_PI, CV_PI, 1.0);
	er->connect(sworld);
	//er->setRotation(er->genRotMat(Vec3d(CV_PI/2, 0, 0)));
	er->setRotation(er->genRotMat(Vec3d(0, 0, 0)));
	FisheyeRetriever *fr = new FisheyeRetriever(180*CV_PI/180, 1.0);
	fr->connect(sworld);
	fr->setRotation(fr->genRotMat(Vec3d(CV_PI/2, 0, 0)));
cout << "begin math" << endl;
high_resolution_clock::time_point a1 = high_resolution_clock::now();
//	er->invalidateMaths();
high_resolution_clock::time_point a2 = high_resolution_clock::now();
cout << "elapsed " << ((duration<double, std::milli>)(a2-a1)).count() << endl;
a1 = high_resolution_clock::now();
	fr->invalidateMaths();
a2 = high_resolution_clock::now();
cout << "elapsed " << ((duration<double, std::milli>)(a2-a1)).count() << endl;
#ifdef DEBUG
exit(0);
#endif

	MarkerTracker mt;

//	ff.setAnchor(Point2d(0, -CV_PI));
//	ff.setApertureSize();
//	ff.invalidateMaths();
	Mat border = Mat(768, 128, DEFINED_CV_TYPE);
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

//bool initvideo = true;
//VideoWriter video;
	while (cap.isOpened() || frames.size() > 0) {

		if (frames.size() == 0) {
			//printf("waiting for new frame\n");
			continue;
		}
//cout << "frames to process: " << frames.size() << endl;
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

		mt.update(p.frame);
		// prepare frame
		
		Mat skinframe = mt.getSkinFrame();
		Mat components = mt.getConnectedComponentsFrame();

//		shared_ptr<Mat> equi = er->getFrame();
/*if (initvideo) {
video = VideoWriter("outcpp.avi",CV_FOURCC('M','J','P','G'),15, Size(equi->cols, equi->rows));
initvideo = false;
}
video.write(*equi);*/

		//ff->updateFrame(components);
		ff->updateFrame(*p.frame);
		shared_ptr<Mat> fish = fr->getFrame();
		hconcat(border, *fish, *fish);
		hconcat(*fish, border, *fish);




/*

//Size patternsize(10,7); //number of centers
Size patternsize(4, 11); //number of centers
Mat gray; //source image
cvtColor(*p.frame, gray, CV_BGR2GRAY);
vector<Point2f> centers; //this will be filled by the detected centers

SimpleBlobDetector::Params params;
//params.maxArea = 10e4;
//params.minArea = 10;
//params.minDistBetweenBlobs = 5;
//params.maxArea = 10e4;
//params.minArea = 3;
//params.minDistBetweenBlobs = 2;
Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);

bool patternfound = findCirclesGrid(gray, patternsize, centers, CALIB_CB_ASYMMETRIC_GRID, blobDetector);


//cout << "found: " << patternfound << endl;
drawChessboardCorners(*p.frame, patternsize, Mat(centers), patternfound);

/*

		Size patternsize(10, 7); //interior number of corners
vector<Point2f> corners; //this will be filled by the detected corners

//CALIB_CB_FAST_CHECK saves a lot of time on images
//that do not contain any chessboard corners
bool patternfound = findChessboardCorners(gray, patternsize, corners,
	CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
	+ CALIB_CB_FAST_CHECK);

//if(patternfound)
//	cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
//		TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

drawChessboardCorners(*p.frame, patternsize, Mat(corners), patternfound);
*/





		Mat tmp1, tmp2, tmp3;
		double compression = 0.5;
		resize(*p.frame, tmp1, Size(), compression, compression);
		resize(components, tmp2, Size(), compression, compression);
		resize(skinframe, tmp3, Size(), compression, compression);
		
		imshow("orig", tmp1);
		imshow("converted", tmp2);
		imshow("Fullscreen ws5", *fish);
		imshow("skin", tmp3);
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
//video.release();
	// the camera will be deinitialized automatically in VideoCapture destructor
	t1.join();
	delete fr;
	delete er;
	delete ff;
	delete sworld;
	return 0;
}
