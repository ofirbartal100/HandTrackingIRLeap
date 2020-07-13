#pragma once
#ifndef __USER_INPUT_HANDLER_H__
#define __USER_INPUT_HANDLER_H__

#include <Windows.h>
class AlgorithmHook;

class UserInputHandler
{
public:
	static UserInputHandler* getInstance();
    static LRESULT CALLBACK CBTProc_S(int nCode, WPARAM wParam, LPARAM lParam);
    virtual void AnalyseWindow(HWND hWnd);
    void Go();
    void End();
    LRESULT CBTProc(int nCode, WPARAM wParam, LPARAM lParam);
	AlgorithmHook* dynamicAlgoHook;
	AlgorithmHook* baseAlgoHook;
    virtual ~UserInputHandler();
private:
	HHOOK m_hook;
	static UserInputHandler*  instance;
	UserInputHandler();
};

#endif