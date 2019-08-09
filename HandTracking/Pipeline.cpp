#include <filesystem>
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/objdetect.hpp>
#include <ctime>

// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

#define FREQUENCY_OF_ROBUST_TRACKER 10
#define HIGHFPS 500
#define DEBUG false

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

using namespace cv;
using namespace std;
using namespace Pylon;

Rect2d getNeighborsROI(const Rect2d& rect, const Mat& mat, float neighboring_scale);

Mat UpdateROI(float neighboringScale, Mat frame, Rect2d bbox);


DWORD WINAPI myThread(LPVOID lpParameter)
{
	unsigned int& myCounter = *((unsigned int*)lpParameter);
	while (myCounter < 0xFFFFFFFF) ++myCounter;
	return 0;
}

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

	//HaarCascade Detector
	/*CascadeClassifier handDetector;
	if (!handDetector.load("C:\\Users\\ofir\\Desktop\\temp\\palm_v4.xml")) {
		std::cout << "Error Loading Cascade" << endl;
		return 1;
	}
*/
#pragma endregion

#pragma region Capture Initializations
	PylonInitialize();

	// Create an instant camera object with the camera device found first.
	CBaslerUsbInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());

	// Print the model name of the camera.
	cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;


	// This smart pointer will receive the grab result data.
	CGrabResultPtr ptrGrabResult;
	
	camera.Open();
	camera.AcquisitionFrameRateEnable = true;
	camera.AcquisitionFrameRate = 500;




	
#pragma endregion

#pragma region BoundingBox Initialization
	// Read first frame 
	Mat frame;
	camera.GrabOne(5000, ptrGrabResult, TimeoutHandling_ThrowException);
	if (ptrGrabResult->GrabSucceeded())
	{
		frame = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8U, (uint8_t*)ptrGrabResult->GetBuffer());
	}
	// select a bounding box 
	//Rect2d bbox = selectROI(frame, false);
#pragma endregion

#pragma region Update ROI
	//UpdateROI(neighboringScale, frame, bbox);
#pragma endregion

#pragma region Show Image
	if (DEBUG)
	{
		// Display bounding box. 
//		rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
	}

	cv::imshow("Tracking", frame);
#pragma endregion


#pragma region Video Writer
	/*const string& outputName = "outputVideo.avi";
	Size frameSize(frame.cols, frame.rows);
	VideoWriter outputVideo;
	outputVideo.open(outputName, VideoWriter::fourcc('M', 'J', 'P', 'G'), 500, frameSize,false);
*/
#pragma endregion 

#pragma region Reading Loop
	int updateFromRobustTracker = HIGHFPS / FREQUENCY_OF_ROBUST_TRACKER;
	//tracker->init(frame, bbox);


	// The camera device is parameterized with a default configuration which
	// sets up free-running continuous acquisition.
	camera.StartGrabbing();
	int counter = 0;
	camera.ExposureTime = 1000;
	double rrrrr2 = camera.ExposureTime.GetValue();
	double rrrrr = camera.ResultingFrameRate.GetValue();
	// Camera.StopGrabbing() is called automatically by the RetrieveResult() method
	clock_t begin = clock();
	cout << "start";
	while (camera.IsGrabbing()&&counter++ < 10000)
	{

		// Wait for an image and then retrieve it. A timeout of 5000 ms is used.
		camera.RetrieveResult(500, ptrGrabResult, TimeoutHandling_ThrowException);
		// Image grabbed successfully?
		if (ptrGrabResult->GrabSucceeded())
		{
			frame = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8U, (uint8_t*)ptrGrabResult->GetBuffer());
			//#pragma region Robust Tracker Update Region
			//
			//
			//			if (updateFromRobustTracker == 0)
			//			{
			//#pragma region Robust Tracker Update
			//				//select a bounding box 
			//				bbox = selectROI(frame, false);
			//				tracker.release();
			//				tracker = TrackerMedianFlow::create();
			//				tracker->init(frame, bbox);
			//				updateFromRobustTracker = HIGHFPS / FREQUENCY_OF_ROBUST_TRACKER;
			//#pragma endregion
			//			}
			//			updateFromRobustTracker--;
			//
			//#pragma endregion
			//
			//#pragma region Fast Tracker Update
			//			// Start timer
			//			double timer = (double)getTickCount();
			//
			//			// Update the tracking result
			//			bool ok = tracker->update(frame, bbox);
			//
			//#pragma region Haar Cascade Detector
			//
			//			std::vector<Rect> hands;
			//			//update search ROI
			//			Rect2d neighboringROI = getNeighborsROI(bbox, frame, neighboringScale);
			//
			//			const Mat ROIref = frame(neighboringROI);
			//			Mat MaxROI;
			//			ROIref.copyTo(MaxROI);
			//
			//			cv::imshow("ROI", MaxROI);
			//
			//			Mat ROI_gray;
			//			Size minSize(int(MaxROI.cols*(1 - neighboringScale)), int(MaxROI.rows*(1 - neighboringScale)));
			//
			//			/*cv::cvtColor(MaxROI, ROI_gray, CV_BGR2GRAY);
			//			cv::equalizeHist(ROI_gray, ROI_gray);*/
			//			//handDetector.detectMultiScale(ROI_gray, hands, 1 + 0.5*neighboringScale, 2, 0, minSize);
			//			handDetector.detectMultiScale(MaxROI, hands, 1 + 0.5*neighboringScale, 2, 0, minSize);
			//			if (hands.size() > 0)
			//			{
			//				bbox.x = neighboringROI.x + hands[0].x;
			//				bbox.y = neighboringROI.y + hands[0].y;
			//				bbox.width = hands[0].width;
			//				bbox.height = hands[0].height;
			//			}
			//
			//#pragma endregion
			//
			//
			//			// Calculate Frames per second (FPS)
			//			float fps = getTickFrequency() / ((double)getTickCount() - timer);
			//#pragma endregion
			//

#pragma region Show Image
			//if (DEBUG)
			//{
			//	if (ok)
			//	{
			//		// Tracking success : Draw the tracked object
			//		rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
			//	}
			//	else
			//	{
			//		// Tracking failure detected.
			//		putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
			//	}

			//	// Display tracker type on frame
			//	putText(frame, trackerType + " Tracker", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

			//	// Display FPS on frame
			//	putText(frame, "FPS : " + SSTR(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
			//}

			// Save frame
			//outputVideo.write(frame);
			// Display frame.
			cv::imshow("Tracking", frame);
#pragma endregion

			// Exit if ESC pressed.
			int k = waitKey(1);
			if (k == 27)
			{
				break;
			}

		}
		else
		{
			cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
		}

	}

	clock_t end = clock();
	cout << "done in:" << double(end - begin)/CLOCKS_PER_SEC;
#pragma endregion
	//outputVideo.release();

}

Rect2d getNeighborsROI(const Rect2d& rect, const Mat& mat, float neighboring_scale)
{
	int width = rect.width * (1 + neighboring_scale);
	int height = rect.height * (1 + neighboring_scale);
	int x = rect.x - (width - rect.width) / 2;
	int y = rect.y - (height - rect.height) / 2;

	if (x < 0) x = 0;
	if (y < 0) y = 0;

	if (x + width >= mat.cols)
	{
		width = mat.cols - x - 1;
	}
	if (y + height >= mat.rows)
	{
		height = mat.rows - y - 1;
	}

	return Rect2d(x, y, width, height);
}


Mat UpdateROI(float neighboringScale, Mat frame, Rect2d bbox)
{
	//update search ROI
	Rect2d neighboringROI = getNeighborsROI(bbox, frame, neighboringScale);
	//extract the search roi out of the frame
	const Mat ROIref = frame(neighboringROI);
	Mat ROI;
	ROIref.copyTo(ROI);
	cv::imshow("ROI", ROI);
	return ROI;
}