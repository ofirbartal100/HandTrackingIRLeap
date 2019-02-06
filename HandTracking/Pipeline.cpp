#include <filesystem>
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#define FREQUENCY_OF_ROBUST_TRACKER 10
#define HIGHFPS 500

using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

int main(int argc, char **argv)
{
	// List of tracker types in OpenCV 3.4.1
	string trackerWorkingTypes[5] = { "KCF", "TLD","MEDIANFLOW", "MOSSE", "CSRT" };
	string trackerNotWorkingTypes[8] = { "BOOSTING", "MIL", "GOTURN" };
	// vector <string> trackerTypes(types, std::end(types));

	// Create a tracker
	string trackerType = trackerWorkingTypes[2];

	Ptr<Tracker> tracker;


	if (trackerType == "BOOSTING")
	{
		TrackerBoosting::Params params;
		tracker = TrackerBoosting::create(params);
	}
	if (trackerType == "MIL")
	{
		TrackerMIL::Params params;
		params.featureSetNumFeatures = 250;
		params.samplerTrackMaxNegNum = 100;
		params.samplerTrackMaxPosNum = 100000;
		params.samplerTrackInRadius = 4;
		params.samplerSearchWinSize = 65;
		params.samplerInitMaxNegNum = 25;
		params.samplerInitInRadius = 3;
		tracker = TrackerMIL::create(params);
	}
	if (trackerType == "KCF")
		tracker = TrackerKCF::create();
	if (trackerType == "TLD")
		tracker = TrackerTLD::create();
	if (trackerType == "MEDIANFLOW")
		tracker = TrackerMedianFlow::create();
	if (trackerType == "GOTURN")
		tracker = TrackerGOTURN::create();
	if (trackerType == "MOSSE")
		tracker = TrackerMOSSE::create();
	if (trackerType == "CSRT")
		tracker = TrackerCSRT::create();


	// Read video
	const string filename = "C:\\Users\\ofir\\Desktop\\temp\\ofir_handTracking.mp4";
	VideoCapture video;

	video.open(filename);
	// Exit if video is not opened
	if (!video.isOpened())
	{
		cout << "Could not read video file" << endl;
		return 1;
	}

	// Read first frame 
	Mat frame, gray_frame;
	bool ok = video.read(frame);
	cvtColor(frame, gray_frame, CV_BGR2GRAY);
	int frameType = frame.type();
	int types[] = { CV_8UC2, CV_8UC3 ,CV_8UC4,CV_8SC2,CV_8SC3,CV_8SC4,CV_16UC2,CV_16UC3,CV_16UC4,CV_16SC2,CV_16SC3,CV_16SC4, CV_32SC2,CV_32SC3,CV_32SC4,CV_32FC2,CV_32FC3, CV_32FC4, CV_64FC2, CV_64FC3, CV_64FC4,CV_16FC2, CV_16FC3,CV_16FC4 };

	// Define initial bounding box 
	Rect2d bbox(287, 23, 86, 320);

	// Uncomment the line below to select a different bounding box 
	bbox = selectROI(frame, false);
	// Display bounding box. 
	rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
	imshow("Tracking", frame);
	tracker->init(frame, bbox);
	//tracker->init(gray_frame, bbox);

	int updateFromRobustTracker = HIGHFPS / FREQUENCY_OF_ROBUST_TRACKER;
	while (video.read(frame))
	{

		if (updateFromRobustTracker == 0)
		{
			// Uncomment the line below to select a different bounding box 
			bbox = selectROI(frame, false);
			tracker.release();
			tracker = TrackerMedianFlow::create();
			tracker->init(frame, bbox);
			updateFromRobustTracker = HIGHFPS / FREQUENCY_OF_ROBUST_TRACKER;
		}
		updateFromRobustTracker--;

		// Start timer
		double timer = (double)getTickCount();

		// Update the tracking result
		bool ok = tracker->update(frame, bbox);
		//bool ok = tracker->update(gray_frame, bbox);

		// Calculate Frames per second (FPS)
		float fps = getTickFrequency() / ((double)getTickCount() - timer);

		if (ok)
		{
			// Tracking success : Draw the tracked object
			rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
		}
		else
		{
			// Tracking failure detected.
			putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
		}

		// Display tracker type on frame
		putText(frame, trackerType + " Tracker", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

		// Display FPS on frame
		putText(frame, "FPS : " + SSTR(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

		// Display frame.
		imshow("Tracking", frame);

		// Exit if ESC pressed.
		int k = waitKey(1);
		if (k == 27)
		{
			break;
		}

	}
}