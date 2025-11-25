#pragma once
#pragma once
#include <windows.h>
#include "WMX3Api.h"
#include "CoreMotionApi.h"

// 외부에서 호출: Map 창 생성
HWND CreateMapWindow(HWND hParent, wmx3Api::WMX3Api* pWmx, wmx3Api::CoreMotion* pCm);

// 메인과 Map 간의 메시지 상수는 메인 쪽에서 정의됨
// extern 등은 필요 없고, 메인에서 include 순서로 해결