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

	cv::Vec3d LeapVToVec3d(LEAP_VECTOR lv)
	{
		return cv::Vec3d(lv.x, lv.y, lv.z);
	}
public:
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


	cv::Vec3d* GetJoints()
	{
		cv::Vec3d joints[21];
		if (frame->nHands > 0)
		{
			LEAP_HAND* hand = &frame->pHands[0];
			int c = 0;
			joints[c] = LeapVToVec3d(hand->palm.position);
			c++;

			for (int i = 0; i < 4; i++)
				joints[c + i] = LeapVToVec3d(hand->index.bones->next_joint);
			c += 4;

			for (int i = 0; i < 4; i++)
				joints[c + i] = LeapVToVec3d(hand->middle.bones->next_joint);
			c += 4;

			for (int i = 0; i < 4; i++)
				joints[c + i] = LeapVToVec3d(hand->ring.bones->next_joint);
			c += 4;

			for (int i = 0; i < 4; i++)
				joints[c + i] = LeapVToVec3d(hand->pinky.bones->next_joint);

			return joints;
		}
		return nullptr;
	}

};