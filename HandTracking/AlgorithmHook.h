#pragma once
#include "LeapMotion.h"

class AlgorithmHook
{
public:
	virtual void Run(){};
};


class CalibrationAlgorithmHook : public AlgorithmHook
{
	LeapMotion* _leap;
	vector<cv::Vec2i> _anchors;
public:
	CalibrationAlgorithmHook(LeapMotion* leap,vector<cv::Vec2i> anchors)
	{
		_leap = leap;
		_anchors = anchors;
	}
	 
	virtual void Run()
	{
		//insert calibration algorithm here
		if (_leap->UpdateFrame()) {
			cv::Vec3d* res = _leap->GetJoints();
			cout << "leap[0]:" << res[0] << " _ anchores[0]:" << _anchors[0] << endl;
		}
	}
};
