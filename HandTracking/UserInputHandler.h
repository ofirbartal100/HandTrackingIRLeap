#pragma once
#include <Windows.h>
#include "AlgorithmHook.h"

//
//class UserInputHandler
//{
//	// variable to store the HANDLE to the hook. Don't declare it anywhere else then globally
//	// or you will get problems since every function uses this variable.
//	static HHOOK _hook;
//
//	// This struct contains the data received by the hook callback. As you see in the callback function
//	// it contains the thing you will need: vkCode = virtual key code.
//	static KBDLLHOOKSTRUCT kbdStruct;
//
//	// This is the callback function. Consider it the event that is raised when, in this case, 
//	// a key is pressed.
//	static LRESULT __stdcall HookCallback(int nCode, WPARAM wParam, LPARAM lParam)
//	{
//		if (nCode >= 0)
//		{
//			// the action is valid: HC_ACTION.
//			if (wParam == WM_KEYDOWN)
//			{
//				// lParam is the pointer to the struct containing the data needed, so cast and assign it to kdbStruct.
//				kbdStruct = *((KBDLLHOOKSTRUCT*)lParam);
//				// a key (non-system) is pressed.
//				if (kbdStruct.vkCode == VK_SPACE)
//				{
//					// F1 is pressed!
//					MessageBox(NULL, "Space is pressed!", "key pressed", MB_ICONINFORMATION);
//				}
//			}
//		}
//
//		// call the next hook in the hook chain. This is nessecary or your hook chain will break and the hook stops
//		return CallNextHookEx(_hook, nCode, wParam, lParam);
//	}
//public:
//
//	void SetHook()
//	{
//		// Set the hook and set it to use the callback function above
//		// WH_KEYBOARD_LL means it will set a low level keyboard hook. More information about it at MSDN.
//		// The last 2 parameters are NULL, 0 because the callback function is in the same thread and window as the
//		// function that sets and releases the hook. If you create a hack you will not need the callback function 
//		// in another place then your own code file anyway. Read more about it at MSDN.
//		if (!(_hook = SetWindowsHookEx(WH_KEYBOARD_LL, HookCallback, NULL, 0)))
//		{
//			MessageBox(NULL, "Failed to install hook!", "Error", MB_ICONERROR);
//		}
//	}
//
//	void ReleaseHook()
//	{
//		UnhookWindowsHookEx(_hook);
//	}
//};

class CMyClass
{
public:
	static CMyClass* getInstance();
	static LRESULT CALLBACK CBTProc_S(int nCode, WPARAM wParam, LPARAM lParam)
	{
		return instance->CBTProc(nCode, wParam, lParam);
	}
	virtual void AnalyseWindow(HWND hWnd) {}
	void Go()
	{
		/*HWND hwnd = ::FindWindow("AGame", 0);
		m_hook = SetWindowsHookEx(WH_KEYBOARD_LL, (HOOKPROC)
			CMyClass::CBTProc_S, NULL, GetWindowThreadProcessId(hwnd, NULL));*/
		m_hook = SetWindowsHookEx(WH_KEYBOARD_LL, (HOOKPROC)
			CMyClass::CBTProc_S, NULL, 0);
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
				// a key (non-system) is pressed.
				if (kbdStruct.vkCode == VK_SPACE)
				{
					cout << "Doing Somthing\n";
					algoHook->Run();
					// F1 is pressed!
					//MessageBox(NULL, "Space is pressed!", "key pressed", MB_ICONINFORMATION);
				}
			}
		}

		// call the next hook in the hook chain. This is nessecary or your hook chain will break and the hook stops
		return CallNextHookEx(m_hook, nCode, wParam, lParam);
	}
	AlgorithmHook* algoHook;
private:
	HHOOK m_hook;
	static CMyClass*  instance;
	CMyClass();
};

/* Null, because instance will be initialized on demand. */
CMyClass* CMyClass::instance = 0;

CMyClass* CMyClass::getInstance()
{
	if (instance == 0)
	{
		instance = new CMyClass();
	}

	return instance;
}

CMyClass::CMyClass()
{
	algoHook = new AlgorithmHook();
}