#include <filesystem>
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#define FREQUENCY_OF_ROBUST_TRACKER 10
#define HIGHFPS 500
#define DEBUG true

using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()


Rect2d getNeighborsROI(const Rect2d& rect, const Mat& mat, float neighboring_scale);

void UpdateROI(float neighboringScale, Mat frame, Rect2d bbox);

int main(int argc, char **argv)
{

#pragma region Tracker Initializations


	// List of tracker types in OpenCV 3.4.1
	string trackerWorkingTypes[5] = { "KCF", "TLD","MEDIANFLOW", "MOSSE", "CSRT" };
	string trackerNotWorkingTypes[8] = { "BOOSTING", "MIL", "GOTURN" };

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


	//ROI Weights Initialization
	float neighboringScale = 0.05;

#pragma endregion

#pragma region Capture Initializations
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
#pragma endregion

#pragma region BoundingBox Initialization
	// Read first frame 
	Mat frame, gray_frame;
	bool ok = video.read(frame);
	cvtColor(frame, gray_frame, CV_BGR2GRAY);

	// select a bounding box 
	Rect2d bbox = selectROI(frame, false);
#pragma endregion

#pragma region Update ROI
	UpdateROI(neighboringScale, frame, bbox);
#pragma endregion

#pragma region Show Image
	if (DEBUG)
	{
		// Display bounding box. 
		rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
	}

	imshow("Tracking", frame);
#pragma endregion

#pragma region Reading Loop
	int updateFromRobustTracker = HIGHFPS / FREQUENCY_OF_ROBUST_TRACKER;
	tracker->init(frame, bbox);

	while (video.read(frame))
	{
		if (updateFromRobustTracker == 0)
		{
#pragma region Robust Tracker Update
			//select a bounding box 
			bbox = selectROI(frame, false);
			tracker.release();
			tracker = TrackerMedianFlow::create();
			tracker->init(frame, bbox);
			updateFromRobustTracker = HIGHFPS / FREQUENCY_OF_ROBUST_TRACKER;
#pragma endregion
		}
		updateFromRobustTracker--;

#pragma region Fast Tracker Update
		// Start timer
		double timer = (double)getTickCount();

		// Update the tracking result
		bool ok = tracker->update(frame, bbox);

#pragma region Update ROI
		UpdateROI(neighboringScale, frame, bbox);
#pragma endregion

		// Calculate Frames per second (FPS)
		float fps = getTickFrequency() / ((double)getTickCount() - timer);
#pragma endregion


#pragma region Show Image
		if (DEBUG)
		{
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
		}

		// Display frame.
		imshow("Tracking", frame);
#pragma endregion

		// Exit if ESC pressed.
		int k = waitKey(1);
		if (k == 27)
		{
			break;
		}

	}
#pragma endregion

}

Rect2d getNeighborsROI(const Rect2d& rect, const Mat& mat, float neighboring_scale)
{
	int width = rect.width * (1 + neighboring_scale);
	int height = rect.height * (1 + neighboring_scale);
	if (rect.x + rect.width * (1 + neighboring_scale) >= mat.cols)
	{
		width = mat.cols - rect.x - 1;
	}
	if (rect.y + rect.height * (1 + neighboring_scale) >= mat.rows)
	{
		height = mat.rows - rect.y - 1;
	}

	return Rect2d(rect.x, rect.y, width, height);
}


void UpdateROI(float neighboringScale, Mat frame, Rect2d bbox)
{
	//update search ROI
	Rect2d neighboringROI = getNeighborsROI(bbox, frame, neighboringScale);
	//extract the search roi out of the frame
	const Mat ROIref = frame(neighboringROI);
	Mat ROI;
	ROIref.copyTo(ROI);
	imshow("ROI", ROI);
}