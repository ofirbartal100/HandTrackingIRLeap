#pragma once
#ifndef __AHOOK_H__
#define __AHOOK_H__

#include "LeapToImageMapper.h"
#include "UserInputHandler.h"

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
        std::cout << "Move 10px Right : 6" << endl;
        std::cout << "Move 10px Down : 2" << endl;
        std::cout << "Move 10px Left : 4" << endl;
        std::cout << "Move 10px Up : 8" << endl;
        std::cout << "Register Current Points For Calibration : Space" << endl;
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
            return 1; // end calibration stage and go back to main menu
            break;
        }
        return 0;
    }
};


class RecordAlgorithmHook : public AlgorithmHook
{
public:
    RecordAlgorithmHook()
    {
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
            //_mapper->StartRecording();
            break;

            //T letter
        case 84:
            //_mapper->StopRecording();
            break;
        }
        return 0;
    }
};


class BaseAlgorithmHook : public AlgorithmHook
{
    UserInputHandler* uih;
    CalibrationAlgorithmHook* cah;
public:
    BaseAlgorithmHook(UserInputHandler* a, CalibrationAlgorithmHook* c)
    {
        uih = a;
        cah = c;
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
        //    //R letter
        //case 82:
        //    _mapper->ToggleRecord();
        }
        return 0;
    }
};

#endif