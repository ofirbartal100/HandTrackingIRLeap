#pragma once
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <thread>


extern "C" {
#include <LeapC.h>
#include <ExampleConnection.h>
}

using namespace std;


class LeapMotion
{
private:
	int64_t lastFrameID = 0; //The last frame received

	LEAP_TRACKING_EVENT *frame;

	cv::Point3f LeapVToVec3d(LEAP_VECTOR lv)
	{
		return cv::Point3f(lv.x, lv.y, lv.z);
	}

	void grabLoop()
	{
		while (IsGrabbing)
		{
			UpdateFrame();
			Sleep(30);//1000 / 30
		}
	}

	thread* grabbing_thread;
public:
	bool IsGrabbing;

	LeapMotion() {}
	~LeapMotion() {	}

	//Blocking
	void Connect()
	{
		//leap connection
		OpenConnection();
		while (!IsConnected)
			millisleep(100); //wait a bit to let the connection complete

		printf("Connected.");
		LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
		if (deviceProps)
			printf("Using device %s.\n", deviceProps->serial);

		UpdateFrame();
	}

	bool UpdateFrame()
	{
		frame = GetFrame();
		if (frame && (frame->tracking_frame_id > lastFrameID)) {
			lastFrameID = frame->tracking_frame_id;
			return frame->nHands > 0;
		}
		return false;
	}

	void StartGrabbing()
	{
		IsGrabbing = true;
		grabbing_thread = new thread(&LeapMotion::grabLoop, this);
	}

	void StopGrabbing()
	{
		if (IsGrabbing)
		{
			IsGrabbing = false;
			grabbing_thread->join();
			if (grabbing_thread)
			{
				delete grabbing_thread;
				//cout << "delete thread* grabbing_thread\n";
			}
		}
	}

	vector<cv::Point3f> GetJoints()
	{
		vector<cv::Point3f> joints;
		if (frame->nHands > 0)
		{
			LEAP_HAND* hand = &frame->pHands[0];

            auto arm = LeapVToVec3d(hand->arm.next_joint);
            joints.push_back(arm);

            joints.push_back(LeapVToVec3d(hand->thumb.metacarpal.next_joint));
            joints.push_back(LeapVToVec3d(hand->thumb.proximal.next_joint));
			joints.push_back(LeapVToVec3d(hand->thumb.intermediate.next_joint));
			joints.push_back(LeapVToVec3d(hand->thumb.distal.next_joint));

			joints.push_back(LeapVToVec3d(hand->index.metacarpal.next_joint));
			joints.push_back(LeapVToVec3d(hand->index.proximal.next_joint));
			joints.push_back(LeapVToVec3d(hand->index.intermediate.next_joint));
			joints.push_back(LeapVToVec3d(hand->index.distal.next_joint));

			joints.push_back(LeapVToVec3d(hand->middle.metacarpal.next_joint));
			joints.push_back(LeapVToVec3d(hand->middle.proximal.next_joint));
			joints.push_back(LeapVToVec3d(hand->middle.intermediate.next_joint));
			joints.push_back(LeapVToVec3d(hand->middle.distal.next_joint));

			joints.push_back(LeapVToVec3d(hand->ring.metacarpal.next_joint));
			joints.push_back(LeapVToVec3d(hand->ring.proximal.next_joint));
			joints.push_back(LeapVToVec3d(hand->ring.intermediate.next_joint));
			joints.push_back(LeapVToVec3d(hand->ring.distal.next_joint));

			joints.push_back(LeapVToVec3d(hand->pinky.metacarpal.next_joint));
			joints.push_back(LeapVToVec3d(hand->pinky.proximal.next_joint));
			joints.push_back(LeapVToVec3d(hand->pinky.intermediate.next_joint));
			joints.push_back(LeapVToVec3d(hand->pinky.distal.next_joint));
		}
		return joints;
	}

};