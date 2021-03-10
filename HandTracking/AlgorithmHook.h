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
        std::cout << "Rotate CW : O" << endl;*/
        std::cout << "Load Last Calibration : L" << endl;
        std::cout << "Annotate Snapshot : S" << endl;
        std::cout << "Calculate Calibration : Enter" << endl;
    }

    virtual int Run(DWORD key_dword)
    {
        switch (key_dword)
        {
        case 13:
            _mapper->Calibrate();
            return 1; // end calibration stage and go back to main menu
            break;
            //S
        case 83:
            _mapper->SnapShot();
            break;
            // L
        case 76:
            _mapper->LoadPoints();
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
    int _dynamicAlgoHooksLength;
    AlgorithmHook** _dynamicAlgoHooks;
    DWORD* _casesValues;
public:

    BaseAlgorithmHook(UserInputHandler* a,int dynamicAlgoHooksLength,  AlgorithmHook** dynamicAlgoHooks, DWORD* casesValues)
    {
        uih = a;
        _dynamicAlgoHooksLength = dynamicAlgoHooksLength;
        _dynamicAlgoHooks = dynamicAlgoHooks;
        _casesValues = casesValues;
    }

    virtual void Description()
    {
        std::cout << "Main Menu :" << endl;
        std::cout << "For Calibration Stage: C" << endl;
        std::cout << "For Recording Stage: R" << endl;
    }

    virtual int Run(DWORD key_dword)
    {
        for(int i =0;i< _dynamicAlgoHooksLength; i++)
        {
            if(_casesValues[i] == key_dword)
            {
                uih->dynamicAlgoHook = _dynamicAlgoHooks[i];
                uih->dynamicAlgoHook->Description();
                break;
            }
        }
        return 0;
    }
};

#endif