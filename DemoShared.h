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

// 메인에 정의된 유틸 함수들
bool EnsureServoOn(int axis);
bool EnsurePosModeNoStop(int axis);
int  TimeMsToAcc(double vel_cnt_per_s, double t_ms);

// 주행/정지 API
bool StartAbsMoveWithProfile(int axis, long long target, double vpps, double tAcc, double tDec);
void StopAxis(int axis);

// 데모/외부 윈도우 진입점
void ShowDemoControlWindow(HWND hParent);
extern "C" int RunGPIOWindowExternal(HINSTANCE hInst);