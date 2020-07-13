#include "UserInputHandler.h"
#include "AlgorithmHook.h"

LRESULT CALLBACK UserInputHandler::CBTProc_S(int nCode, WPARAM wParam, LPARAM lParam)
{
    return instance->CBTProc(nCode, wParam, lParam);
}

void UserInputHandler::AnalyseWindow(HWND hWnd) {}

void UserInputHandler::Go()
{
    m_hook = SetWindowsHookEx(WH_KEYBOARD_LL, (HOOKPROC)
        UserInputHandler::CBTProc_S, NULL, 0);
    dynamicAlgoHook->Description();
}

void UserInputHandler::End()
{
    if (m_hook) UnhookWindowsHookEx(m_hook);
}

LRESULT UserInputHandler::CBTProc(int nCode, WPARAM wParam, LPARAM lParam)
{
    if (nCode >= 0)
    {
        // the action is valid: HC_ACTION.
        if (wParam == WM_KEYDOWN)
        {
            // lParam is the pointer to the struct containing the data needed, so cast and assign it to kdbStruct.
            KBDLLHOOKSTRUCT kbdStruct = *((KBDLLHOOKSTRUCT*)lParam);
            int res = dynamicAlgoHook->Run(kbdStruct.vkCode);
            if (res == 1)
            {
                dynamicAlgoHook = baseAlgoHook;
                dynamicAlgoHook->Description();
            }
        }
    }

    // call the next hook in the hook chain. This is nessecary or your hook chain will break and the hook stops
    return CallNextHookEx(m_hook, nCode, wParam, lParam);
}

UserInputHandler::~UserInputHandler()
{
    if (dynamicAlgoHook)
    {
        delete dynamicAlgoHook;
        cout << "AlgorithmHook* algoHook deleted\n";
    }
    if (baseAlgoHook)
    {
        delete baseAlgoHook;
        cout << "AlgorithmHook* algoHook deleted\n";
    }
    if (instance)
    {
        delete instance;
        cout << "UserInputHandler* instance deleted";
    }
}


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
    baseAlgoHook = new AlgorithmHook();
    dynamicAlgoHook = new AlgorithmHook();
}