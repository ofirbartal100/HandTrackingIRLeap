#pragma once
#include <Windows.h>
#include "AlgorithmHook.h"

class UserInputHandler
{
public:
	static UserInputHandler* getInstance();
	static LRESULT CALLBACK CBTProc_S(int nCode, WPARAM wParam, LPARAM lParam)
	{
		return instance->CBTProc(nCode, wParam, lParam);
	}
	virtual void AnalyseWindow(HWND hWnd) {}
	void Go()
	{
		m_hook = SetWindowsHookEx(WH_KEYBOARD_LL, (HOOKPROC)
			UserInputHandler::CBTProc_S, NULL, 0);
	}
	void End()
	{
		if (m_hook) UnhookWindowsHookEx(m_hook);
	}

	LRESULT CBTProc(int nCode, WPARAM wParam, LPARAM lParam)
	{
		if (nCode >= 0)
		{
			// the action is valid: HC_ACTION.
			if (wParam == WM_KEYDOWN)
			{
				// lParam is the pointer to the struct containing the data needed, so cast and assign it to kdbStruct.
				KBDLLHOOKSTRUCT kbdStruct = *((KBDLLHOOKSTRUCT*)lParam);
				algoHook->Run(kbdStruct.vkCode);

			}
		}

		// call the next hook in the hook chain. This is nessecary or your hook chain will break and the hook stops
		return CallNextHookEx(m_hook, nCode, wParam, lParam);
	}
	AlgorithmHook* algoHook;

	virtual ~UserInputHandler()
	{
		if(algoHook)
		{
			delete algoHook;
			cout << "AlgorithmHook* algoHook deleted\n";
		}
		if(instance)
		{
			delete instance;
			cout << "UserInputHandler* instance deleted";
		}
	}
private:
	HHOOK m_hook;
	static UserInputHandler*  instance;
	UserInputHandler();
};

/* Null, because instance will be initialized on demand. */
UserInputHandler* UserInputHandler::instance = 0;

UserInputHandler* UserInputHandler::getInstance()
{
	if (instance == 0)
	{
		instance = new UserInputHandler();
	}

	return instance;
}

UserInputHandler::UserInputHandler()
{
	algoHook = new AlgorithmHook();
}