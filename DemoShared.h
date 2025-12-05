#pragma once
#define NOMINMAX
#include <windows.h>

#include "WMX3Api.h"
#include "CoreMotionApi.h"

namespace wmx3Api {
    class WMX3Api;
    class CoreMotion;
}

extern wmx3Api::WMX3Api g_wmx;
extern wmx3Api::CoreMotion g_cm;
extern bool g_deviceOpened;
extern bool g_commStarted;

// 어떤 공용 헤더 (DemoShared.h 같은 데)
extern bool g_diStable[8];
// 다른 cpp 에서 가져다 쓸 상수 선언
extern const DWORD kOutputPulseMs;

// 메인에 정의된 유틸 함수들
bool EnsureServoOn(int axis);
bool EnsurePosModeNoStop(int axis);
int  TimeMsToAcc(double vel_cnt_per_s, double t_ms);

// 주행/정지 API
bool StartAbsMoveWithProfile(int axis, long long target, double vpps, double tAcc, double tDec);
void StopAxis(int axis);

bool IsAxis0AtConveyorBarcode();
bool IsAxis0AtWorkstationBarcode();
bool IsAxis2Up();
bool IsAxis2Workdown();
bool IsAxis2Conveyordown();
bool IsGripperOpen();
bool IsGripperOpenAndIdle();
bool IsGripperClosed();
bool IsGripperClosedAndIdle();
bool HasBox();
bool NoBox();


void DoClose_Compat(HWND hWnd);
void DoOpen_Compat(HWND hWnd);
void DoStopAll(HWND hWnd);

void StartDemoHomeWithBox();
void StartDemoHomeWithoutBox();
void StartDemoLoad();
void StartDemoUnload();


// 데모/외부 윈도우 진입점
void ShowDemoControlWindow(HWND hParent);
extern "C" int RunGPIOWindowExternal(HINSTANCE hInst);

// =======================================
// 상태 관리 (기존 Demo 유지)
// =======================================
enum class TaskId : int {
    GoWorkstation = 0,
    GoConveyor,
    LiftUp,
    WorkDown,
    ConveyorDown,
    GripOpen,
    GripClose,
    GripServoOff,
    DemoHomeWithBox,
    DemoWorkWithBox,
    DemoHomeWithoutBox,
    DemoWorkWithoutBox,
    DemoLoad,
    DemoUnload,
    COUNT
};
enum class TaskState : int { Idle = 0, Running, Done, Failed, Stopped };

// 이 함수들은 inline으로 두거나, 선언만 하고 .cpp에 정의해도 됨
const TCHAR* TaskName(TaskId id);
const TCHAR* TaskStateStr(TaskState s);
struct TaskStatus {
    std::atomic<TaskState> state{ TaskState::Idle };
    std::atomic<DWORD>     lastChangeTick{ 0 };
};

extern TaskStatus g_taskStatus[(int)TaskId::COUNT];
extern HWND g_hStatusStatics[(int)TaskId::COUNT];
extern HWND g_hDemoWnd;

TaskState GetTaskState(TaskId id);

void SetTaskState(TaskId id, TaskState st);
void ResetAllTaskStates();


bool WaitAllAxesStopped(double velEps, DWORD timeoutMs);
bool WaitUntil(bool (*pred)(), DWORD timeoutMs, DWORD pollMs = 20);
bool WaitTaskFinished(TaskId id, DWORD timeoutMs, DWORD pollMs = 20);