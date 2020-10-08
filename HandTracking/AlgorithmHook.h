#pragma once
#ifndef __AHOOK_H__
#define __AHOOK_H__

#include "LeapToImageMapper.h"
#include "UserInputHandler.h"
#include "AnnotatedVideoSaver.h"
#include "VideoShower.h"

class AlgorithmHook
{
public:
    virtual int Run(DWORD key_dword) { return 0; };
    virtual void Description() {};
};


class CalibrationAlgorithmHook : public AlgorithmHook
{
    LeapToImageMapper* _mapper;

public:
    CalibrationAlgorithmHook(LeapToImageMapper* mapper)
    {
        _mapper = mapper;
    }

    virtual void Description()
    {
        std::cout << "Hook Map :" << endl;
        /*std::cout << "Move 10px Right : 6" << endl;
        std::cout << "Move 10px Down : 2" << endl;
        std::cout << "Move 10px Left : 4" << endl;
        std::cout << "Move 10px Up : 8" << endl;
        std::cout << "Scale X Up : U" << endl;
        std::cout << "Scale X Down : J" << endl;
        std::cout << "Scale Y Up : I" << endl;
        std::cout << "Scale Y Down : K" << endl;
        std::cout << "Rotate CW : O" << endl;
        std::cout << "Rotate CCW : L" << endl;*/
        std::cout << "Annotate Snapshot : S" << endl;
        std::cout << "Calculate Calibration : Enter" << endl;
    }

    virtual int Run(DWORD key_dword)
    {
        const cv::Point2f moveRight(10, 0);
        const cv::Point2f moveDown(0, 10);
        const cv::Point2f moveLeft(-10, 0);
        const cv::Point2f moveUp(0, -10);


        switch (key_dword)
        {
        //    //6 numpad right
        //case 102:
        //    _mapper->TransformPattern(0, 1, 1, 10, 0);
        //    //_mapper->MovePattern(moveRight);
        //    break;
        //    //2 numpad down
        //case 98:
        //    _mapper->TransformPattern(0, 1, 1, 0, 10);
        //    //_mapper->MovePattern(moveDown);
        //    break;
        //    //4 numpad left
        //case 100:
        //    _mapper->TransformPattern(0, 1, 1, -10, 0);
        //    //_mapper->MovePattern(moveLeft);
        //    break;
        //    //8 numpad up
        //case 104:
        //    _mapper->TransformPattern(0, 1, 1, 0, -10);
        //    //_mapper->MovePattern(moveUp);
        //    break;
        //    //I scale y up
        //case 73:
        //    _mapper->TransformPattern(0, 1, 1.2, 0, 0);
        //    break;
        //    //J scale x down
        //case 74:
        //    _mapper->TransformPattern(0, 1.0 / 1.2, 1, 0, 0);
        //    break;
        //    //U scale x up
        //case 85:
        //    _mapper->TransformPattern(0, 1.2, 1, 0, 0);
        //    break;
        //    //K scale y down
        //case 75:
        //    _mapper->TransformPattern(0, 1, 1.0 / 1.2, 0, 0);
        //    break;
        //    //O rotate cw
        //case 79:
        //    _mapper->TransformPattern(10, 1, 1, 0, 0);
        //    break;
        //    //L rotate ccw
        //case 76:
        //    _mapper->TransformPattern(-10, 1, 1, 0, 0);
        //    break;
        //case VK_SPACE:
        //    _mapper->RegisterPoints();
        //    cout << "Registered Points" << endl;
        //    break;
        //    //enter
        case 13:
            _mapper->Calibrate();
            return 1; // end calibration stage and go back to main menu
            break;
            //S
        case 83:
            _mapper->SnapShot();
            break;
        }
        return 0;
    }

};


class RecordAlgorithmHook : public AlgorithmHook
{
    AnnotatedVideoSaver* avs;
public:
    RecordAlgorithmHook(AnnotatedVideoSaver* saver)
    {
        avs = saver;
    }

    virtual void Description()
    {
        std::cout << "Record Menu :" << endl;
        std::cout << "For Start Recording: R" << endl;
        std::cout << "For Stop Recording: T" << endl;
    }

    virtual int Run(DWORD key_dword)
    {

        switch (key_dword)
        {
            //R letter
        case 82:
            avs->Start("D:\\TAU\\Research\\AnnotatedVideos\\");
            break;

            //T letter
        case 84:
            avs->Close();
            break;
        }
        return 0;
    }
};


class BaseAlgorithmHook : public AlgorithmHook
{
    UserInputHandler* uih;
    CalibrationAlgorithmHook* cah;
    RecordAlgorithmHook* rah;
public:
    BaseAlgorithmHook(UserInputHandler* a, CalibrationAlgorithmHook* c, RecordAlgorithmHook* r)
    {
        uih = a;
        cah = c;
        rah = r;
    }

    virtual void Description()
    {
        std::cout << "Main Menu :" << endl;
        std::cout << "For Calibration Stage: C" << endl;
        std::cout << "For Recording Stage: R" << endl;
    }

    virtual int Run(DWORD key_dword)
    {

        switch (key_dword)
        {
            //C letter
        case 67:
            uih->dynamicAlgoHook = cah;
            uih->dynamicAlgoHook->Description();
            break;
            //R letter
        case 82:
            uih->dynamicAlgoHook = rah;
            uih->dynamicAlgoHook->Description();
            break;
        }
        return 0;
    }
};

#endif