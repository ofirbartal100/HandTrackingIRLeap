#pragma once
#include "LeapMotion.h"

#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/objdetect.hpp>
#include <opencv2/calib3d.hpp>
#include "LeapToImageMapper.h"

class AlgorithmHook
{
public:
    virtual void Run(DWORD key_dword) {};
};


class CalibrationAlgorithmHook : public AlgorithmHook
{
    LeapToImageMapper* _mapper;

public:
    CalibrationAlgorithmHook(LeapToImageMapper* mapper)
    {
        _mapper = mapper;

        std::cout << "Hook Map :" << endl;
        std::cout << "Move 10px Right : 6" << endl;
        std::cout << "Move 10px Down : 2" << endl;
        std::cout << "Move 10px Left : 4" << endl;
        std::cout << "Move 10px Up : 8" << endl;
        std::cout << "Register Current Points For Calibration : Space" << endl;
        std::cout << "Calculate Calibration : Enter" << endl;
    }

    virtual void Run(DWORD key_dword)
    {
        const cv::Point2f moveRight(10, 0);
        const cv::Point2f moveDown(0, 10);
        const cv::Point2f moveLeft(-10, 0);
        const cv::Point2f moveUp(0, -10);

        switch (key_dword)
        {
            //6 numpad right
        case 102:
            _mapper->MovePattern(moveRight);
            break;
            //2 numpad down
        case 98:
            _mapper->MovePattern(moveDown);
            break;
            //4 numpad left
        case 100:
            _mapper->MovePattern(moveLeft);
            break;
            //8 numpad up
        case 104:
            _mapper->MovePattern(moveUp);
            break;
        case VK_SPACE:
            _mapper->RegisterPoints();
            break;
            //enter
        case 13:
            _mapper->Calibrate();
            break;
        }

    }
};

