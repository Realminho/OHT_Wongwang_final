// ===============================
// WMX3 Practice + Sync Group UI (Restored + Fixed) + ECAT 0x6063/0x603F Monitor
// + Alt Target helper + Error Manual JSON Viewer (Fastech/Welcon)
// - Added: Auto column width by content + multi-keyword search
// - Added: E-Stop toggle buttons and status on Main and Sync windows
// - Modified: TCP Serial Monitor moved to a separate window with "Serial Monitor..." button
// - Modified by request: Replace Demo0/1/2 buttons with Demo + GPIO buttons
// Demo: opens external DemoControl window (ShowDemoControlWindow)
// GPIO: opens external GPIO window (RunGPIOWindowExternal)
// - Fixed: desyncDec member removed from Sync::SyncGroup (use Config::SyncParam.masterDesyncDec/slaveDesyncDec)
// - Added: Motion Log/Scope windows and background logger thread
// ===============================
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include <commctrl.h>
#include <tchar.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>
#include <functional>
#include <fstream>
#include <iterator>
#include <cwctype>
#include <locale>
#include <direct.h>
#include <cwchar>
#include <shlwapi.h>

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "IOApi.h"
#include "EcApi.h" // EtherCAT API
#include "LogApi.h"
#include "DemoShared.h"

#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "Shlwapi.lib")

// MapView.h optional include
#ifdef __has_include
# if __has_include("MapView.h")
# include "MapView.h"
# if !defined(MAPVIEW_CREATE_DECLARED)
// #define MAPVIEW_CREATE_DECLARED 1
# endif
# endif
#endif

using namespace wmx3Api;
using namespace ecApi;

// ===== External launchers declared by user-provided modules =====
void ShowDemoControlWindow(HWND hParent, bool minimized = false);
extern "C" int RunGPIOWindowExternal(HINSTANCE hInst);

// ==== DemoControl에서 제공하는 GPIO/그리퍼/상태 함수 extern ====
// CHANGED: 아래 extern들은 democontrol 모듈의 함수를 창 없이도 호출하기 위해 필요
extern void ToggleDO_HW(int pin, bool turnOn, HWND hWnd);
extern void DoGripServoOff_Compat(HWND hWnd);
extern bool EnumerateGPIO();
extern bool PickBank_DI0_7_DO8_15();
extern bool EnsureDO8to15AsOutput_BankFirst();
extern void RefreshLevels(HWND hWnd);
static int g_gpioBank = -1;

extern bool IsGripperOpen();
extern bool IsGripperOpenAndIdle();
extern bool IsGripperClosed();
extern bool IsGripperClosedAndIdle();
extern bool g_diStable[8]; // DI0(Motioning), DI1(Catched) 등 디바운스 결과 사용

// ============ 상태 조회 헬퍼(예시) ============
static bool IsGripperAlreadyOpen()
{
	// IsGripperOpenAndIdle가 충분하면 그것으로 대체 가능
	return IsGripperOpenAndIdle();
}

static bool IsGripperAlreadyClosed()
{
	return IsGripperClosedAndIdle();
}

extern struct ApproachProfile {
	double vpps = 1000.0;
	double accMs = 80.0;
	double decMs = 10.0;
};

static void SendSimpleAck(SOCKET s, unsigned char reqOpCode);

extern void StartMoveWithApproach(int axis, long long target, TaskId task,
	double mainVpps, double mainAccMs, double mainDecMs,
	double posEps, double velEps, DWORD timeoutMs,
	double approachEps, const ApproachProfile& ap);

// ===================== 사용자 설정 =====================
static TCHAR g_installPath[] = TEXT("C:\\Program Files\\SoftServo\\WMX3");
static UINT POLL_MS = 100;
static const int vel_idle_threshold = 5;
static const long long inpos_tol_counts = 5;
static const DWORD idle_wait_poll_ms = 10;

static const int TRAVEL_AXIS1_SIGN = -1;

static const long long ZONE_PULSES = 50000;
static const int TARGET_ZONE_IDX = 5;
static const long long TARGET_ZONE5 = TARGET_ZONE_IDX * ZONE_PULSES;

static const long long HOIST_DOWN_PULSES = 100000;
static const long long GRIPPER_CLOSE_PULSE = 10000;

// ===================== 센서 기반 주행 설정(Demo2) =====================
static const int IO_ADDR = 14;
static const int IO_BIT = 0;
static const bool SENSOR_ACTIVE_HIGH = true;
static const DWORD SENSOR_POLL_MS = 5;
static const DWORD SENSOR_DEBOUNCE_MS = 5;
static const int FORWARD_TARGET_COUNTS = 5;
static const int BACKWARD_TARGET_COUNTS = 5;

// ======================================================

// 로그/스코프 설정
const DWORD LOG_POLL_MS = 50;

// 축별 명령 수신/완료 카운터
std::atomic<unsigned long> g_cmdDoneCount[4] = { 0,0,0,0 };

std::atomic<bool> g_forceGripCodeZero{ false };

std::atomic<bool> g_gripHoldCodeZero{ false };  // true면 무조건 0x00
std::atomic<bool> g_gripAwaitDone{ false };     // 명령 후 완료대기
std::atomic<bool> g_gripSawBusy{ false };       // busy를 한번이라도 봤는지

// Stop(0x29) 이후 Reset(0xFD) 전까지 명령 차단 라치
std::atomic_bool g_stop29Latched{ false };



// 명령 추적
struct AxisCommandInfo {
	std::atomic<long long> target{ 0 };
	std::atomic<int> axis{ -1 };
	std::atomic<int> vel{ 0 };
	std::atomic<int> acc{ 0 };
	std::atomic<int> dec{ 0 };
	std::atomic<bool> active{ false };
	std::atomic<ULONGLONG> startTick{ 0 };
	std::atomic<ULONGLONG> endTick{ 0 };
};
AxisCommandInfo g_axisCmdInfo[4];

// CHANGED: Per-axis sampling enable flag and per-command row index for possible future logic
std::atomic<bool> g_axisLogEnabled[4] = { false,false,false,false };
std::atomic<unsigned long> g_axisLogRowIdx[4] = { 0,0,0,0 };

// WMX3 전역
WMX3Api g_wmx;
CoreMotion g_cm(&g_wmx);
Home g_home(&g_cm);
bool g_deviceOpened = false;
bool g_commStarted = false;

// Log 클래스 전역
Log g_log(&g_wmx);

// 재사용 상태 버퍼
static CoreMotionStatus g_status{};

static void StopMultiJog();
static void StopJogIfActive();

extern void GO_Conveyor();     // Conveyor 버튼이 눌렸을 때 실행되는 함수
extern void Go_Workstation();  // Workstation 버튼이 눌렸을 때 실행되는 함수
extern void ConveyorDown();    // Conveyor Down 버튼
extern void WorkDown();        // Work Down 버튼
extern void DoUp();            // Up 버튼

extern void DoClose_Compat(HWND hWnd); // Close 버튼
extern void DoOpen_Compat(HWND hWnd); // Open 버튼
extern void DoStopAll(HWND hWnd); // Stop All 버튼

// ===== CHANGED: 그립 동작 중인지 여부 플래그 =====
// democontrol 창 없이 TCP로 Open/Close를 수행할 때, 움직이는 동안은 0x00 상태를 내기 위함
static std::atomic<bool> g_gripBusy{ false };

// ===== STO 펄스 자동 OFF를 위한 예약 =====
// oht main에서도 AutoInitIoAndRefresh 시점에 ToggleDO_HW(10,true) 후 자동 OFF를 처리
static std::atomic<bool> g_ohtStoPulsePendingOff{ false };
static std::atomic<DWORD> g_ohtStoPulseOnTick{ 0 };
static const DWORD kOhtStoPulseMs = 100;

// ------------------ Serial Monitor Window ------------------
static HWND g_hSerialWnd = nullptr;

// WndProc 먼저 선언
LRESULT CALLBACK SerialWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);


// ===================== NEW: Manual/Auto Mode =====================
static std::atomic<bool> g_autoMode{ false }; // false=Manual, true=Auto

// Helper to check/guard manual-only commands
static bool IsManualAllowed(HWND hWnd) {
	if (g_autoMode.load()) {
		MessageBox(hWnd, TEXT("현재 자동 모드입니다. 수동 동작은 무시됩니다."), TEXT("모드"), MB_ICONWARNING);
		return false;
	}
	return true;
}
// Helper to check/guard auto-only commands (TCP etc.)
static bool IsAutoAllowed() {
	return g_autoMode.load();
}

// Ensure Serial Monitor window exists and is a top-level independent window
static void EnsureSerialWindowTopLevel(HWND hMain)
{
	if (g_hSerialWnd && IsWindow(g_hSerialWnd))
		return;

	WNDCLASS wc{};
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = SerialWndProc; // 람다 대신 일반 함수 포인터
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = (HINSTANCE)GetWindowLongPtr(hMain, GWLP_HINSTANCE);
	wc.hIcon = nullptr;
	wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
	wc.lpszMenuName = nullptr;
	wc.lpszClassName = TEXT("WMX3SerialWnd");

	RegisterClass(&wc);

	// Create as top-level independent window (no parent), ensure it shows on taskbar
	g_hSerialWnd = CreateWindowEx(
		WS_EX_APPWINDOW,
		TEXT("WMX3SerialWnd"),
		TEXT("Serial Monitor (TCP)"),
		WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_SIZEBOX,
		CW_USEDEFAULT, CW_USEDEFAULT, 800, 400,
		nullptr,           // NO parent
		nullptr,           // menu
		wc.hInstance,
		nullptr);

	if (g_hSerialWnd) {
		ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
		UpdateWindow(g_hSerialWnd);
		BringWindowToTop(g_hSerialWnd);
		SetForegroundWindow(g_hSerialWnd);
		SetWindowPos(g_hSerialWnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE);
		SetWindowPos(g_hSerialWnd, HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE);
	}
}

static void ApplyModeUI(HWND hMain) {
	// 수동 모드: 메인 GUI 전면, SerialMonitor 숨기거나 최소화
	// 자동 모드: 메인 GUI 최소화, SerialMonitor(있으면) 전면
	if (!hMain) return;
	if (g_autoMode.load()) {
		ShowWindow(hMain, SW_MINIMIZE);
		EnsureSerialWindowTopLevel(hMain);
		if (IsWindow(g_hSerialWnd)) {
			ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
			BringWindowToTop(g_hSerialWnd);
			SetForegroundWindow(g_hSerialWnd);
		}
	}
	else {
		ShowWindow(hMain, SW_SHOWNORMAL);
		SetForegroundWindow(hMain);
		if (IsWindow(g_hSerialWnd)) {
			// 자동 모드 해제 시 Serial을 굳이 닫지 않고 그대로 유지. 필요하면 최소화.
			ShowWindow(g_hSerialWnd, SW_MINIMIZE);
		}
	}
}
static void SwitchToManual(HWND hMain) {
	// Stop any jog
	StopMultiJog();
	StopJogIfActive();
	// Optionally stop all motion?
	for (int a = 0; a < 4; ++a) StopAxis(a); // 선택사항

	g_autoMode = false;
	ApplyModeUI(hMain);
}
static void SwitchToAuto(HWND hMain) {
	// Stop jog
	StopMultiJog();
	StopJogIfActive();
	// Optionally stop all motion?
	for (int a = 0; a < 4; ++a) StopAxis(a); // 선택사항

	g_autoMode = true;
	ApplyModeUI(hMain);
}

static void ShowErrMsgBox(const TCHAR* title, long err, WMX3Api& api) {
	char bufA[256] = {};
	api.ErrorToString(err, bufA, (unsigned)sizeof(bufA));
#ifdef UNICODE
	wchar_t wbuf[256]{}; MultiByteToWideChar(CP_ACP, 0, bufA, -1, wbuf, 256);
	TCHAR msg[512] = {};
	_stprintf_s(msg, TEXT("%s\r\nerr=%ld (%s)"), title, err, wbuf);
#else
	TCHAR msg[512] = {};
	_stprintf_s(msg, TEXT("%s\r\nerr=%ld (%hs)"), title, err, bufA);
#endif
	MessageBox(nullptr, msg, TEXT("WMX3 Error"), MB_OK | MB_ICONERROR);
}

int TimeMsToAcc(double vel_cnt_per_s, double t_ms) {
	if (t_ms <= 0) t_ms = 1;
	double acc = vel_cnt_per_s / (t_ms / 1000.0);
	if (acc < 1) acc = 1;
	if (acc > 200000000) acc = 200000000;
	return (int)acc;
}
bool EnsureServoOn(int axis) {
	g_cm.GetStatus(&g_status);
	if (!g_status.axesStatus[axis].servoOn) {
		long e = g_cm.axisControl->SetServoOn(axis, 1);
		if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("Servo ON 실패"), e, g_wmx); return false; }
		Sleep(20);
	}
	return true;
}
// 급정지(QuickStop Dec 파라미터 사용)로 축 정지
void StopAxis(int axis)
{
	if (!g_commStarted) return;

	// 1) 모션 쪽은 Quick Stop 사용
	//    - WMX3APIFUNC ExecQuickStop(int axis)
	//    - QuickStop Dec 파라미터로 감속 → 급정지
	long e = g_cm.motion->ExecQuickStop(axis);

	// 필요하면 실패 시 일반 Stop 으로 폴백
	if (e != ErrorCode::None) {
		g_cm.motion->Stop(axis);
	}

	// 2) 속도/토크 명령도 모두 끊어준다 (이전 코드 유지)
	g_cm.velocity->Stop(axis);
	if (g_cm.torque) {
		g_cm.torque->StopTrq(axis);
	}
}

static bool EnterPosMode(int axis) {
	long e = g_cm.axisControl->SetAxisCommandMode(axis, AxisCommandMode::Position);
	if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("SetAxisCommandMode(Position) 실패"), e, g_wmx); return false; }
	return true;
}
bool EnsurePosModeNoStop(int axis) {
	g_cm.GetStatus(&g_status);
	if (g_status.axesStatus[axis].axisCommandModeFeedback == AxisCommandMode::Position) return true;
	return EnterPosMode(axis);
}

// ====================== NEW: Limit/Home sensors for Axis2 ======================
// Limit sensor: IO_ADDR=8, IO_BIT=1 (Active High)
// Home sensor:  IO_ADDR=8, IO_BIT=2 (Active High)
// EtherCAT IO 보드 주소/비트
static const int AX0_LIMIT_L_ADDR = 0;
static const int AX0_LIMIT_L_BIT = 0;
static const int AX0_LIMIT_R_ADDR = 0;
static const int AX0_LIMIT_R_BIT = 1;

static const int AX2_LIMIT_ADDR = 8;
static const int AX2_LIMIT_BIT = 1;
static const int AX2_HOME_ADDR = 8;
static const int AX2_HOME_BIT = 2;

// 센서 활성 레벨 (필요시 LOW → HIGH로 수정)
static const bool AX2_LIMIT_ACTIVE_HIGH = true;
static const bool AX2_HOME_ACTIVE_HIGH = true;
static const bool AX0_LIMIT_L_ACTIVE_HIGH = true;
static const bool AX0_LIMIT_R_ACTIVE_HIGH = true;

// 폴링/디바운스 시간
static const DWORD AX2_SENSOR_POLL_MS = 5;
static const DWORD AX2_SENSOR_DEBOUNCE_MS = 5;

// Axis2 센서 상태 (GUI 표시용)
static std::atomic<bool> g_ax2LimitOn{ false };   // 현재 Limit 입력 ON 여부
static std::atomic<bool> g_ax2HomeOn{ false };    // 현재 Home 입력 ON 여부

// Axis2 Limit/Home 제어 플래그
static std::atomic<bool>  g_ax2LimitLatched{ false };      // Limit 최초 인식 래치
static std::atomic<bool>  g_ax2LimitBlocking{ false };     // Limit 중 -방향 명령 차단
static std::atomic<bool>  g_ax2StopIssuedOnLimit{ false }; // Limit에서 Stop 명령 1회 발행
static std::atomic<bool>  g_ax2HomingStarted{ false };     // Limit 후 Home 시작 여부

static std::atomic<bool>  g_ax2HomeDebounceOn{ false };    // Home ON 디바운스 완료
static std::atomic<DWORD> g_ax2HomeLastTick{ 0 };          // Home 디바운스용 시각
static std::atomic<bool>  g_ax2HomeRampIssued{ false };    // Home ON 시 속도 2000 전환 명령 1회 발행

static std::atomic<DWORD> g_ax2LimitIdleTime{ 0 };   // Idle 최초 감지 시각

static std::atomic<bool>  g_ax2ServoReady{ false };    // axis2 servo on 준비 완료 여부
static std::atomic<DWORD> g_ax2ServoOnTime{ 0 };       // axis2 servoOn 감지 시각
static const DWORD AX2_SENSOR_ENABLE_DELAY_MS = 1500;  // 1초 지연

// ================= Axis0 Left/Right Limit 상태/블록 플래그 =================
// AX0 쪽은 센서 읽기 결과를 "free = true, limit 감지 = false" 로 사용
static std::atomic<bool> g_ax0LimitLFree{ true };          // true = 정상, false = L 리밋 감지
static std::atomic<bool> g_ax0LimitRFree{ true };          // true = 정상, false = R 리밋 감지

static std::atomic<bool> g_ax0LimitLLatched{ false };      // L 리밋 최초 인식 래치
static std::atomic<bool> g_ax0LimitRLatched{ false };      // R 리밋 최초 인식 래치

// L 리밋 ON → +방향 블록, R 리밋 ON → -방향 블록
static std::atomic<bool> g_ax0BlockPlus{ false };          // Axis0 + 방향 명령 차단
static std::atomic<bool> g_ax0BlockMinus{ false };         // Axis0 - 방향 명령 차단


static bool IsAxis2ServoOn()
{
	g_cm.GetStatus(&g_status);
	return g_status.axesStatus[2].servoOn;
}


// 리밋/홈 센서 읽기
static bool ReadInputBit(int addr, int bit, bool activeHigh) {
	if (!g_commStarted) return false;
	Io io(&g_wmx);
	unsigned char v = 0;
	long e = io.GetInBitEx(addr, bit, &v);
	if (e != ErrorCode::None) return false; // 실패 시 OFF 취급
	bool onRaw = (v != 0);
	return activeHigh ? onRaw : !onRaw;
}

static bool Axis2IsIdle()
{
	g_cm.GetStatus(&g_status);
	int av = (int)std::lround(g_status.axesStatus[2].actualVelocity);
	long long perr =
		(long long)g_status.axesStatus[2].posCmd -
		(long long)g_status.axesStatus[2].actualPos;

	return (std::abs(av) <= vel_idle_threshold) &&
		(std::llabs(perr) <= inpos_tol_counts);
}

// Axis2 현재 이동 방향 추정 (+1 / -1 / 0)
static int Axis2CurrentDir()
{
	g_cm.GetStatus(&g_status);
	double vcmd = g_status.axesStatus[2].velocityCmd;
	if (vcmd > vel_idle_threshold) return +1;
	if (vcmd < -vel_idle_threshold) return -1;
	return 0;
}

// Axis2 감속/저속 전환: 홈센서 ON 때 빠르게 감속하여 1000pps 수준으로 낮춤
static void Axis2HomeSoftDecelTo500() {
	if (!g_commStarted) return;

	// 홈센서 대응은 "움직이고 있을 때만" 수행. 정지 상태면 무시
	if (Axis2IsIdle()) return;

	if (!EnsureServoOn(2) || !EnsurePosModeNoStop(2)) return;

	// 현재 위치/속도
	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[2].actualPos;

	int dir = Axis2CurrentDir();
	if (dir == 0) dir = +1; // 정지에 가까우면 +쪽으로 소폭

	// 가까운 소타겟으로 감속: 2만 펄스 앞(또는 뒤) 지점
	long long smallStep = 5000 * dir;
	long long softTarget = cur + smallStep;

	// 감속을 빠르게: Dec 시간을 짧게(예: 30ms), 목표속도 5000pps로 전환
	double newVel = 500.0; // 요청사항
	double accMs = 80.0;
	double decMs = 10.0;

	// 1단계: 가까운 점까지 빠르게 감속
	{
		Motion::PosCommand pc{};
		pc.axis = 2;
		pc.target = softTarget;
		pc.profile.type = ProfileType::SCurve;
		pc.profile.velocity = (int)std::lround(newVel);
		pc.profile.acc = TimeMsToAcc(pc.profile.velocity, accMs);
		pc.profile.dec = TimeMsToAcc(pc.profile.velocity, decMs);
		g_cm.motion->StartPos(&pc);
	}
	g_ax2HomeRampIssued = true;
}

// Axis2 Limit 감지 → Stop → Idle되면 home 시작
static void Axis2HandleLimitOnceAndHome() {
	if (!g_commStarted) return;
	if (!g_ax2LimitLatched.load()) return;

	// Stop은 1회만
	if (!g_ax2StopIssuedOnLimit.exchange(true)) {
		StopAxis(2);
	}

	// 아직 Idle이 아니라면 홈 타이머 초기화 후 대기
	if (!Axis2IsIdle()) {
		g_ax2LimitIdleTime = 0;   // Idle 감지 시간 리셋
		return;
	}

	// Idle 상태 도달
	if (g_ax2LimitIdleTime.load() == 0) {
		// Idle 처음 감지된 순간 타임스탬프 저장
		g_ax2LimitIdleTime = GetTickCount();
		return;
	}

	// Idle 유지 시간 확인 — 2000ms(2초) 지나야 Home 시작
	DWORD elapsed = GetTickCount() - g_ax2LimitIdleTime.load();
	if (elapsed < 2000) {
		return; // 2초 대기 중
	}

	// 3초 지났고, 아직 Home 시작 안 했으면 Home 시작
	if (!g_ax2HomingStarted.load()) {
		if (EnsureServoOn(2) && EnsurePosModeNoStop(2)) {
			g_home.StartHome(2);      // Home → pos=0 리셋
			g_ax2HomingStarted = true;
		}
	}
}

// Axis2 리밋 상태에서 -방향 명령 금지 확인
static bool Axis2IsMinusCommandBlocked(long long currentPos, long long targetPos, long long stepOrSign) {
	// 리밋 블로킹 상태가 아닐 때 허용
	if (!g_ax2LimitBlocking.load()) return false;

	// 조그: stepOrSign이 +1/-1 로 들어온다고 보고 음수면 차단
	if (stepOrSign == +1 || stepOrSign == -1) {
		return (stepOrSign < 0);
	}
	// 절대/상대: 타깃이 현재 위치보다 작은 방향이면 음(-) 방향
	long long delta = targetPos - currentPos;
	return (delta < 0);
}

// Axis2 마이너스 금지 시 경고
static void Axis2ShowMinusBlockedWarning(HWND hWnd) {
	MessageBox(hWnd, TEXT("Axis2: Limit 센서 ON 상태입니다.\r\n-방향 명령은 허용되지 않습니다."), TEXT("Axis2 보호"), MB_ICONWARNING | MB_OK);
}

// ---------------------------------------------------------------------
// Axis0 Limit 블록 검사 + 경고 (L=+ 방향 차단, R=- 방향 차단)
// ---------------------------------------------------------------------
static bool Axis0IsCommandBlocked(long long currentPos,
	long long targetPos,
	long long stepOrSign)
{
	bool blockPlus = g_ax0BlockPlus.load();
	bool blockMinus = g_ax0BlockMinus.load();

	if (!blockPlus && !blockMinus) return false;

	// 조그 명령: stepOrSign = +1 / -1
	if (stepOrSign == +1 || stepOrSign == -1) {
		if (stepOrSign > 0 && blockPlus)  return true; // + 방향 조그 차단
		if (stepOrSign < 0 && blockMinus) return true; // - 방향 조그 차단
		return false;
	}

	// 일반 이동: targetPos 기준으로 방향 판단
	long long delta = targetPos - currentPos;
	if (delta > 0 && blockPlus)  return true; // + 방향 이동 차단
	if (delta < 0 && blockMinus) return true; // - 방향 이동 차단

	return false;
}

static void Axis0ShowBlockedWarning(HWND hWnd, int dirSign)
{
	if (dirSign > 0) {
		MessageBox(hWnd,
			TEXT("Axis0: Left Limit 센서 ON 상태입니다.\r\n+ 방향 명령은 허용되지 않습니다."),
			TEXT("Axis0 보호"),
			MB_ICONWARNING | MB_OK);
	}
	else if (dirSign < 0) {
		MessageBox(hWnd,
			TEXT("Axis0: Right Limit 센서 ON 상태입니다.\r\n- 방향 명령은 허용되지 않습니다."),
			TEXT("Axis0 보호"),
			MB_ICONWARNING | MB_OK);
	}
}

// TCP Server globals
static std::atomic<bool> g_tcpRunning{ false };
static std::thread g_tcpThread;
static SOCKET g_listenSock = INVALID_SOCKET;
static SOCKET g_clientSock = INVALID_SOCKET;
static HWND g_hMainWnd = nullptr;
static HWND g_hTcpLogList = nullptr;
static wchar_t g_tcpBindIp[64] = L"0.0.0.0";
static int g_tcpBindPort = 9100;

#define WM_APP_TCP_LOG (WM_APP + 101)
#define WM_APP_TCP_STATE (WM_APP + 102)
#define WM_APP_SHOW_DEMO_MIN (WM_APP + 103)   // ★ 추가
#define WM_APP_MAP_GOTO (WM_APP + 201)

struct MapGotoParam { long long target; };

// E-Stop 상태
static std::atomic<bool> g_estopActive{ false };

// ECAT 전역
static Ecat g_ecat(&g_wmx);

// Jog 상태
static int g_jogActiveAxis = -1;
static int g_jogActiveSign = 0;
static bool g_multiJogActive = false;
static int g_multiJogSign = 0;
static bool g_multiJogAxisActive[4] = {};

static int g_lastCmdVel[4] = { 0,0,0,0 };
static int g_lastCmdTrq[4] = { 0,0,0,0 };

// Demo 상태
static std::atomic<bool> g_demoRunning{ false };
static std::thread g_demoThread;

// ==== Demo1(펄스) zone 표시 ====
static std::atomic<int> g_zoneDisplay{ 0 };
static std::atomic<bool> g_zoneInit{ false };

// ==== Demo2(센서) zone 카운터 ====
static std::atomic<int> g_sensorZoneCount{ 0 };

// Demo 스테이지/종류
enum class DemoStage {
	Idle,
	StartAtZero,
	TravelingTo5,
	At5,
	HoistDown,
	GripClose,
	HoistUp,
	TravelingTo0,
	At0Return,
	HoistDown2,
	GripOpen,
	HoistUp2,
	Complete
};
static std::atomic<DemoStage> g_demoStage{ DemoStage::Idle };

enum class DemoKind { None, PulseZones, SensorZones, Demo0 };
static std::atomic<DemoKind> g_demoKind{ DemoKind::None };

// ==== Sync 창 전역 핸들
static HWND g_hSyncWnd = nullptr;

// -------------- Error Manual JSON Viewer --------------
static const TCHAR* kFastechJson = TEXT("fastech_errors.json");
static const TCHAR* kWelconJson = TEXT("welcon_errors.json");

enum : int {
	ID_ERR_TIMER = 40000,
	ID_ERR_LIST = 40001,
	ID_ERR_SEARCH_EDIT = 40002,
	ID_ERR_SEARCH_BTN = 40003,
	ID_ERR_SRC_LABEL = 40004,
	ID_ERR_SHOW_ALL = 40005
};

struct ErrItem {
	std::wstring codeHex;
	std::wstring vendorCode;
	std::wstring name;
	std::wstring category;
	std::vector<std::wstring> causes;
	std::vector<std::wstring> actions;
};
struct ErrManual {
	std::wstring sourceName;
	std::vector<ErrItem> items;
};

static std::wstring s2ws_utf8(const std::string& a)
{
	if (a.empty()) return L"";
	int n = MultiByteToWideChar(CP_UTF8, 0, a.c_str(), (int)a.size(), nullptr, 0);
	if (n <= 0) return L"";
	std::wstring w; w.resize(n);
	MultiByteToWideChar(CP_UTF8, 0, a.c_str(), (int)a.size(), &w[0], n);
	return w;
}

static std::wstring trim(const std::wstring& w) { size_t a = 0, b = w.size(); while (a < b && iswspace((wint_t)w[a])) ++a; while (b > a && iswspace((wint_t)w[b - 1])) --b; return w.substr(a, b - a); }
static bool ieq(const std::wstring& a, const std::wstring& b) { if (a.size() != b.size()) return false; for (size_t i = 0; i < a.size(); ++i) { if (towlower((wint_t)a[i]) != towlower((wint_t)b[i])) return false; } return true; }
static bool icontains(const std::wstring& a, const std::wstring& pat) { std::wstring A = a, B = pat; std::transform(A.begin(), A.end(), A.begin(), ::towlower); std::transform(B.begin(), B.end(), B.begin(), ::towlower); return A.find(B) != std::wstring::npos; }

static std::wstring NormalizeCode4(const std::wstring& in)
{
	std::wstring s = trim(in);
	if (s.empty()) return L"";
	std::wstring u = s;
	std::transform(u.begin(), u.end(), u.begin(), ::towupper);
	if (u.size() >= 2 && u[0] == L'0' && u[1] == L'X') u = u.substr(2);
	if (!u.empty() && u.back() == L'H') u.pop_back();
	std::wstring hex;
	for (wchar_t c : u) {
		if ((c >= L'0' && c <= L'9') || (c >= L'A' && c <= L'F')) hex.push_back(c);
	}
	if (hex.size() > 4) hex = hex.substr(hex.size() - 4);
	while (hex.size() < 4) hex = L"0" + hex;
	return hex;
}

static std::wstring ItemCodeKey4(const ErrItem& e)
{
	std::wstring k = NormalizeCode4(e.codeHex);
	if (!k.empty() && k != L"0000") return k;
	std::wstring v;
	for (wchar_t c : e.vendorCode) {
		if ((c >= L'0' && c <= L'9') || (c >= L'A' && c <= L'F') || (c >= L'a' && c <= L'f') || c == L'x' || c == L'X' || c == L'h' || c == L'H')
			v.push_back(c);
	}
	return NormalizeCode4(v);
}

static bool ReadFileAllUtf8(const wchar_t* path, std::string& outUtf8)
{
	outUtf8.clear();
	FILE* fp = nullptr;
	if (_wfopen_s(&fp, path, L"rb") != 0 || !fp) return false;
	fseek(fp, 0, SEEK_END);
	long sz = ftell(fp);
	if (sz < 0) { fclose(fp); return false; }
	fseek(fp, 0, SEEK_SET);
	outUtf8.resize((size_t)sz);
	if (sz > 0) {
		size_t rd = fread(&outUtf8[0], 1, (size_t)sz, fp);
		outUtf8.resize(rd);
	}
	fclose(fp);
	if (outUtf8.size() >= 3 && (unsigned char)outUtf8[0] == 0xEF && (unsigned char)outUtf8[1] == 0xBB && (unsigned char)outUtf8[2] == 0xBF) {
		outUtf8.erase(0, 3);
	}
	return true;
}

static bool LoadManualJson(const wchar_t* path, ErrManual& out, const wchar_t* sourceName)
{
	out.items.clear(); out.sourceName = sourceName ? sourceName : L"";
	std::string txt8;
	if (!ReadFileAllUtf8(path, txt8)) return false;
	auto skipws = [&](size_t& i) { while (i < txt8.size()) { char c = txt8[i]; if (c == ' ' || c == '\r' || c == '\n' || c == '\t') ++i; else break; } };
	size_t i = 0;
	skipws(i); if (i >= txt8.size() || txt8[i] != '[') return false; ++i;
	while (true) {
		skipws(i); if (i >= txt8.size()) break;
		if (txt8[i] == ']') { ++i; break; }
		if (txt8[i] == ',') { ++i; continue; }
		if (txt8[i] != '{') return false; ++i;
		ErrItem it{};
		while (true) {
			skipws(i); if (i >= txt8.size()) return false;
			if (txt8[i] == '}') { ++i; break; }
			if (txt8[i] == ',') { ++i; continue; }
			if (txt8[i] != '"') return false;
			++i; size_t ks = i; while (i < txt8.size() && txt8[i] != '"') ++i; if (i >= txt8.size()) return false;
			std::string key = txt8.substr(ks, i - ks); ++i;
			skipws(i); if (i >= txt8.size() || txt8[i] != ':') return false; ++i; skipws(i);
			if (i < txt8.size() && txt8[i] == '"') {
				++i; size_t vs = i; while (i < txt8.size() && txt8[i] != '"') ++i; if (i >= txt8.size()) return false;
				std::string val = txt8.substr(vs, i - vs); ++i;
				std::wstring wv = s2ws_utf8(val);
				if (key == "codeHex") it.codeHex = wv;
				else if (key == "vendorCode") it.vendorCode = wv;
				else if (key == "name") it.name = wv;
				else if (key == "category") it.category = wv;
			}
			else if (i < txt8.size() && txt8[i] == '[') {
				++i; std::vector<std::wstring> arr;
				while (true) {
					skipws(i); if (i >= txt8.size()) return false;
					if (txt8[i] == ']') { ++i; break; }
					if (txt8[i] == ',') { ++i; continue; }
					if (txt8[i] != '"') return false;
					++i; size_t vs = i; while (i < txt8.size() && txt8[i] != '"') ++i; if (i >= txt8.size()) return false;
					std::string val = txt8.substr(vs, i - vs); ++i;
					arr.push_back(s2ws_utf8(val));
				}
				if (key == "causes") it.causes = std::move(arr);
				else if (key == "actions") it.actions = std::move(arr);
			}
			else {
				return false;
			}
		}
		out.items.push_back(std::move(it));
		skipws(i);
		if (i < txt8.size() && txt8[i] == ',') { ++i; continue; }
		if (i < txt8.size() && txt8[i] == ']') { ++i; break; }
	}
	return !out.items.empty();
}

static std::vector<ErrItem> FilterByCode(const std::vector<ErrItem>& src, const std::wstring& q)
{
	std::vector<ErrItem> out;
	std::wstring qq = trim(q);
	if (qq.empty()) { out = src; return out; }
	for (const auto& e : src) {
		if (icontains(e.codeHex, qq) || icontains(e.vendorCode, qq)) out.push_back(e);
	}
	return out;
}

static std::vector<ErrItem> FilterByCodeExact(const std::vector<ErrItem>& src, const std::wstring& q)
{
	std::vector<ErrItem> out;
	std::wstring key = NormalizeCode4(q);
	if (key.empty()) return out;
	for (const auto& e : src) {
		if (ItemCodeKey4(e) == key) out.push_back(e);
	}
	return out;
}

static std::vector<ErrItem> FilterMultiKeywords(const std::vector<ErrItem>& src, const std::wstring& q)
{
	std::vector<ErrItem> result;
	std::wstring s = q;
	std::vector<std::wstring> tokens;
	std::wstring cur;
	for (wchar_t c : s) {
		if (c == L',' || iswspace(c)) {
			if (!trim(cur).empty()) { tokens.push_back(trim(cur)); cur.clear(); }
		}
		else {
			cur.push_back(c);
		}
	}
	if (!trim(cur).empty()) tokens.push_back(trim(cur));
	if (tokens.empty()) return src;

	auto appendUnique = [&](const ErrItem& e) {
		std::wstring key = ItemCodeKey4(e);
		for (const auto& x : result) {
			if (ItemCodeKey4(x) == key && x.name == e.name) return;
		}
		result.push_back(e);
		};

	for (const auto& tok : tokens) {
		std::wstring norm = NormalizeCode4(tok);
		if (!norm.empty()) {
			for (const auto& e : src) if (ItemCodeKey4(e) == norm) appendUnique(e);
		}
		else {
			for (const auto& e : src) {
				if (icontains(e.codeHex, tok) || icontains(e.vendorCode, tok) || icontains(e.name, tok) || icontains(e.category, tok))
					appendUnique(e);
			}
		}
	}
	return result;
}

// Error Manual Window
static LRESULT CALLBACK ErrWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	static ErrManual manual{};
	static std::vector<ErrItem> view{};
	static HWND hList = nullptr;
	static HWND hEdt = nullptr;
	static HWND hBtnSearch = nullptr;
	static HWND hBtnShowAll = nullptr;
	static HWND hSrcStatic = nullptr;
	static HWND hSrcLabel = nullptr;

	auto autosize_columns_by_content = [&]() {
		if (!hList) return;
		HFONT hFont = (HFONT)SendMessage(hList, WM_GETFONT, 0, 0);
		HDC hdc = GetDC(hList);
		HFONT hOld = (HFONT)SelectObject(hdc, hFont);

		const wchar_t* headers[6] = { L"Code", L"Vendor", L"Name", L"Category", L"Causes", L"Actions" };
		int maxW[6] = {};
		SIZE sz{};

		auto measure = [&](const std::wstring& s)->int {
			if (s.empty()) {
				GetTextExtentPoint32W(hdc, L"-", 1, &sz);
				return sz.cx + 16;
			}
			GetTextExtentPoint32W(hdc, s.c_str(), (int)s.size(), &sz);
			return sz.cx + 16;
			};

		for (int c = 0; c < 6; ++c) {
			maxW[c] = measure(headers[c]);
		}

		for (const auto& e : view) {
			std::wstring key4 = NormalizeCode4(e.codeHex);
			std::wstring codeDisp = key4.empty() ? trim(e.codeHex) : (L"0x" + key4);
			if (codeDisp.empty()) codeDisp = L"-";
			maxW[0] = std::max(maxW[0], measure(codeDisp));
			maxW[1] = std::max(maxW[1], measure(e.vendorCode));
			maxW[2] = std::max(maxW[2], measure(e.name));
			maxW[3] = std::max(maxW[3], measure(e.category));
			std::wstring cs;
			for (size_t k = 0; k < e.causes.size(); ++k) { if (k) cs += L"; "; cs += e.causes[k]; }
			std::wstring as;
			for (size_t k = 0; k < e.actions.size(); ++k) { if (k) as += L"; "; as += e.actions[k]; }
			maxW[4] = std::max(maxW[4], measure(cs));
			maxW[5] = std::max(maxW[5], measure(as));
		}

		RECT rList{}; GetWindowRect(hList, &rList);
		MapWindowPoints(nullptr, hWnd, (LPPOINT)&rList, 2);
		int listW = rList.right - rList.left;

		const int minW[6] = { 70, 70, 120, 90, 160, 160 };
		const int maxTotal = std::max(400, listW - 4);

		int sum = 0;
		for (int c = 0; c < 6; ++c) {
			maxW[c] = std::max(maxW[c], minW[c]);
			sum += maxW[c];
		}
		if (sum > maxTotal && sum > 0) {
			double scale = (double)maxTotal / (double)sum;
			for (int c = 0; c < 6; ++c) {
				maxW[c] = std::max(minW[c], (int)std::floor(maxW[c] * scale));
			}
		}

		for (int c = 0; c < 6; ++c) {
			ListView_SetColumnWidth(hList, c, maxW[c]);
		}

		SelectObject(hdc, hOld);
		ReleaseDC(hList, hdc);
		};

	auto refill = [&]() {
		if (!hList) return;
		ListView_DeleteAllItems(hList);

		for (int i = 0; i < (int)view.size(); ++i) {
			const auto& e = view[i];

			std::wstring key4 = NormalizeCode4(e.codeHex);
			std::wstring codeDisp = key4.empty() ? trim(e.codeHex) : (L"0x" + key4);
			if (codeDisp.empty()) codeDisp = L"-";

			LVITEMW it{}; it.mask = LVIF_TEXT; it.iItem = i; it.iSubItem = 0;
			it.pszText = (LPWSTR)codeDisp.c_str();
			int idx = ListView_InsertItem(hList, &it);

			ListView_SetItemText(hList, idx, 1, (LPWSTR)e.vendorCode.c_str());
			ListView_SetItemText(hList, idx, 2, (LPWSTR)e.name.c_str());
			ListView_SetItemText(hList, idx, 3, (LPWSTR)e.category.c_str());

			std::wstring cs;
			for (size_t k = 0; k < e.causes.size(); ++k) { if (k) cs += L"; "; cs += e.causes[k]; }
			ListView_SetItemText(hList, idx, 4, (LPWSTR)cs.c_str());

			std::wstring as;
			for (size_t k = 0; k < e.actions.size(); ++k) { if (k) as += L"; "; as += e.actions[k]; }
			ListView_SetItemText(hList, idx, 5, (LPWSTR)as.c_str());
		}

		autosize_columns_by_content();
		};

	auto rebuildColumnsIfEmpty = [&]() {
		if (!hList) return;
		if (ListView_GetColumnWidth(hList, 0) == 0) {
			LVCOLUMNW c{};
			c.mask = LVCF_TEXT | LVCF_WIDTH | LVCF_SUBITEM;
			c.pszText = (LPWSTR)L"Code"; c.cx = 90; ListView_InsertColumn(hList, 0, &c);
			c.pszText = (LPWSTR)L"Vendor"; c.cx = 80; c.iSubItem = 1; ListView_InsertColumn(hList, 1, &c);
			c.pszText = (LPWSTR)L"Name"; c.cx = 170; c.iSubItem = 2; ListView_InsertColumn(hList, 2, &c);
			c.pszText = (LPWSTR)L"Category"; c.cx = 90; c.iSubItem = 3; ListView_InsertColumn(hList, 3, &c);
			c.pszText = (LPWSTR)L"Causes"; c.cx = 240; c.iSubItem = 4; ListView_InsertColumn(hList, 4, &c);
			c.pszText = (LPWSTR)L"Actions"; c.cx = 240; c.iSubItem = 5; ListView_InsertColumn(hList, 5, &c);
		}
		};

	auto do_search = [&]() {
		if (!hEdt) return;
		wchar_t q[1024]{}; GetWindowTextW(hEdt, q, 1000);
		std::wstring qq = q;
		auto filtered = FilterMultiKeywords(manual.items, qq);
		view = std::move(filtered);
		refill();
		if (view.empty()) {
			MessageBoxW(hWnd, L"일치하는 에러 코드가 없습니다.\n예) FF04, ff04, 0xFF04, FF04h\n여러 개: FF04 FF01 또는 FF04,FF01", L"검색 결과", MB_ICONINFORMATION);
		}
		};

	switch (msg) {
	case WM_CREATE:
	{
		auto pcs = (CREATESTRUCT*)lParam;
		const wchar_t** pp = (const wchar_t**)pcs->lpCreateParams;
		const wchar_t* jsonPath = pp[0];
		const wchar_t* source = pp[1];

		CreateWindowW(L"STATIC", L"Source:", WS_CHILD | WS_VISIBLE, 10, 10, 60, 22, hWnd, (HMENU)ID_ERR_SRC_LABEL, 0, 0);
		hSrcStatic = CreateWindowW(L"STATIC", source, WS_CHILD | WS_VISIBLE, 70, 10, 200, 22, hWnd, 0, 0, 0);

		CreateWindowW(L"STATIC", L"Error Search:", WS_CHILD | WS_VISIBLE, 300, 10, 100, 22, hWnd, 0, 0, 0);
		hEdt = CreateWindowW(L"EDIT", L"", WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL, 400, 8, 260, 24, hWnd, (HMENU)ID_ERR_SEARCH_EDIT, 0, 0);
		hBtnSearch = CreateWindowW(L"BUTTON", L"Search", WS_CHILD | WS_VISIBLE, 670, 8, 70, 24, hWnd, (HMENU)ID_ERR_SEARCH_BTN, 0, 0);
		hBtnShowAll = CreateWindowW(L"BUTTON", L"전체 에러코드", WS_CHILD | WS_VISIBLE, 745, 8, 110, 24, hWnd, (HMENU)ID_ERR_SHOW_ALL, 0, 0);

		hList = CreateWindowW(WC_LISTVIEWW, L"", WS_CHILD | WS_VISIBLE | WS_BORDER | LVS_REPORT | LVS_SINGLESEL,
			10, 40, 920, 520, hWnd, (HMENU)ID_ERR_LIST, 0, 0);
		ListView_SetExtendedListViewStyle(hList, LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);
		rebuildColumnsIfEmpty();

		if (!LoadManualJson(jsonPath, manual, source)) {
			MessageBoxW(hWnd, L"JSON 파일을 읽을 수 없습니다.\r\n실행파일 폴더에 배치했는지 확인하세요.", L"Error Manual", MB_ICONWARNING);
		}
		view = manual.items;

		refill();

		SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)new std::function<void()>([&]() {
			do_search();
			}));

		MONITORINFO mi{ sizeof(MONITORINFO) };
		HMONITOR hMon = MonitorFromWindow(hWnd, MONITOR_DEFAULTTONEAREST);
		if (GetMonitorInfo(hMon, &mi)) {
			int W = mi.rcWork.right - mi.rcWork.left;
			int H = mi.rcWork.bottom - mi.rcWork.top;
			SetWindowPos(hWnd, nullptr, mi.rcWork.left, mi.rcWork.top, W, H, SWP_SHOWWINDOW);
			ShowWindow(hWnd, SW_MAXIMIZE);
		}
		else {
			ShowWindow(hWnd, SW_MAXIMIZE);
		}
		PostMessage(hWnd, WM_SIZE, 0, 0);
		return 0;
	}
	case WM_COMMAND:
		if (LOWORD(wParam) == ID_ERR_SEARCH_BTN) {
			auto pf = (std::function<void()>*)GetWindowLongPtr(hWnd, GWLP_USERDATA);
			if (pf) (*pf)();
			return 0;
		}
		if (LOWORD(wParam) == ID_ERR_SHOW_ALL) {
			view = manual.items;
			refill();
			return 0;
		}
		return 0;

	case WM_KEYDOWN:
		if (wParam == VK_RETURN) {
			auto pf = (std::function<void()>*)GetWindowLongPtr(hWnd, GWLP_USERDATA);
			if (pf) (*pf)();
			return 0;
		}
		break;

	case WM_SIZE:
	{
		RECT rc{}; GetClientRect(hWnd, &rc);
		int cx = rc.right - rc.left;
		int cy = rc.bottom - rc.top;
		int pad = 10;

		int topY = 8;
		int rowH = 24;

		if (hSrcStatic) SetWindowPos(hSrcStatic, nullptr, pad + 60, topY, 200, rowH, SWP_NOZORDER);

		int edtMaxW = std::max(260, cx / 3);
		int edtW = std::min(600, edtMaxW);

		if (hEdt) SetWindowPos(hEdt, nullptr, 400, topY, edtW, rowH, SWP_NOZORDER);
		if (hBtnSearch) SetWindowPos(hBtnSearch, nullptr, 400 + edtW + 10, topY, 70, rowH, SWP_NOZORDER);
		if (hBtnShowAll) SetWindowPos(hBtnShowAll, nullptr, 400 + edtW + 10 + 75, topY, 110, rowH, SWP_NOZORDER);

		int listX = pad;
		int listY = topY + rowH + 8;
		int listW = cx - pad * 2;
		int listH = cy - listY - pad;
		if (hList) SetWindowPos(hList, nullptr, listX, listY, listW, listH, SWP_NOZORDER);

		refill();
		return 0;
	}

	case WM_DESTROY:
		if (auto p = (std::function<void()>*)GetWindowLongPtr(hWnd, GWLP_USERDATA)) { delete p; SetWindowLongPtr(hWnd, GWLP_USERDATA, 0); }
		return 0;
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}

static void ShowErrorManual(HWND hParent, bool fastech)
{
	const wchar_t* params[2];
	params[0] = fastech ? kFastechJson : kWelconJson;
	params[1] = fastech ? L"Fastech" : L"Welcon";

	WNDCLASSW wc{}; wc.lpszClassName = fastech ? L"ErrManFastech" : L"ErrManWelcon";
	wc.lpfnWndProc = ErrWndProc; wc.hInstance = (HINSTANCE)GetWindowLongPtr(hParent, GWLP_HINSTANCE);
	wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
	RegisterClassW(&wc);

	RECT wr{};
	SystemParametersInfo(SPI_GETWORKAREA, 0, &wr, 0);
	int W = std::max(960, static_cast<int>(wr.right - wr.left));
	int H = std::max(620, static_cast<int>(wr.bottom - wr.top));

	HWND w = CreateWindowW(wc.lpszClassName, fastech ? L"Fastech Error Manual" : L"Welcon Error Manual",
		WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_SIZEBOX,
		wr.left, wr.top, W, H, hParent, nullptr, wc.hInstance, (LPVOID)params);
	ShowWindow(w, SW_SHOWNORMAL);
}

// Forward decl
static bool IsAxisChecked(HWND hWnd, int axis);
static void SetAxisChecked(HWND hWnd, int axis, bool checked);
static void UpdateSelectedAxesTextOnDemand(HWND hWnd);
static void DoAbsMoveAxis(HWND hWnd, int axis);
static void DoRelMoveAxis(HWND hWnd, int axis, int dir);
static void DoMultiAbs(HWND hWnd);
static void DoMultiRel(HWND hWnd);
static void DoMultiAlarmReset(HWND hWnd);

// Demo helpers forward
static bool EnsureStartAtZero(HWND hWnd);
static bool Travel_0_to_5_Pulse(HWND hWnd);
static bool Travel_5_to_0_Pulse(HWND hWnd);
static bool Travel_0_to_5_Sensor(HWND hWnd, int targetCounts, DWORD timeout_ms);
static bool Travel_5_to_0_Sensor(HWND hWnd, int targetCounts, DWORD timeout_ms);
static bool Hoist_Down_100k(HWND hWnd);
static bool Hoist_Up_100k_to_Zero(HWND hWnd);
static bool Gripper_Close_10k(HWND hWnd);
static bool Gripper_Open_to_Zero(HWND hWnd);
static void DemoPulseProc(HWND hWnd);
static void DemoSensorProc(HWND hWnd);
static void Demo0Proc(HWND hWnd);
static void DisableAllEnabledSyncGroups();

// ------------------ 유틸 ------------------
static double GetDlgDouble(HWND h, int id, double def) {
	wchar_t buf[64] = {};
	HWND he = GetDlgItem(h, id);
	if (!he) return def;
	GetWindowTextW(he, buf, (int)std::size(buf));
	if (buf[0] == 0) return def;
	return _wtof(buf);
}
static int GetDlgInt(HWND h, int id, int def) {
	wchar_t buf[64] = {};
	HWND he = GetDlgItem(h, id);
	if (!he) return def;
	GetWindowTextW(he, buf, (int)std::size(buf));
	if (buf[0] == 0) return def;
	return _wtoi(buf);
}
static void SetDlgDouble(HWND h, int id, double v) {
	wchar_t buf[64]; _snwprintf_s(buf, _TRUNCATE, L"%.6f", v);
	HWND he = GetDlgItem(h, id);
	if (he) SetWindowTextW(he, buf);
}
static void SetDlgInt(HWND h, int id, int v) {
	wchar_t buf[64]; _snwprintf_s(buf, _TRUNCATE, L"%d", v);
	HWND he = GetDlgItem(h, id);
	if (he) SetWindowTextW(he, buf);
}




// ------------------ ECAT 0x6063/0x603F 읽기 함수 ------------------
const int kAxisSlaveId[4] = { 0, 1, 2, 3 };

static const unsigned short kIdx6063 = 0x6063;
static const unsigned char kSubIdx6063 = 0x00;

static const unsigned short kIdx603F = 0x603F;
static const unsigned char kSubIdx603F = 0x00;

const unsigned short kIdxActualCurrent = 0x2181;
const unsigned char kSubIdxActualCurrent = 0x00;

const unsigned short kIdxTorqueActual = 0x6077;
const unsigned char kSubIdxTorqueActual = 0x00;

bool ReadAxis_TxPDO_6063(int slaveId, int& outVal) {
	if (!g_deviceOpened || !g_commStarted) return false;
	unsigned char buf[8] = {};
	unsigned int actual = 0;
	long e = g_ecat.PdoRead(slaveId, kIdx6063, kSubIdx6063, (unsigned int)sizeof(buf), buf, &actual);
	if (e != ErrorCode::None) return false;
	if (actual < 4) return false;
	int32_t v = (int32_t)((uint32_t)buf[0]
		| ((uint32_t)buf[1] << 8)
		| ((uint32_t)buf[2] << 16)
		| ((uint32_t)buf[3] << 24));
	outVal = (int)v;
	return true;
}

static bool ReadAxis_TxPDO_603F(int slaveId, int& outVal) {
	if (!g_deviceOpened || !g_commStarted) return false;
	unsigned char buf[8] = {};
	unsigned int actual = 0;
	long e = g_ecat.PdoRead(slaveId, kIdx603F, kSubIdx603F, (unsigned int)sizeof(buf), buf, &actual);
	if (e != ErrorCode::None) return false;
	if (actual >= 4) {
		int32_t v = (int32_t)((uint32_t)buf[0]
			| ((uint32_t)buf[1] << 8)
			| ((uint32_t)buf[2] << 16)
			| ((uint32_t)buf[3] << 24));
		outVal = (int)v;
		return true;
	}
	else if (actual >= 2) {
		uint16_t v16 = (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
		outVal = (int)(uint32_t)v16;
		return true;
	}
	return false;
}

//bool ReadAxis_TxPDO_2181_ActualCurrent(int slaveId, int& outVal) {
//	if (!g_deviceOpened || !g_commStarted) return false;
//	unsigned char buf[8] = {};
//	unsigned int actual = 0;
//	long e = g_ecat.PdoRead(slaveId, kIdxActualCurrent, kSubIdxActualCurrent, (unsigned int)sizeof(buf), buf, &actual);
//	if (e != ErrorCode::None) return false;
//	if (actual < 4) return false;
//	int32_t v = (int32_t)((uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24));
//	outVal = (int)v;
//	return true;
//}
//
//bool ReadAxis_TxPDO_6077_TorqueActual(int slaveId, int& outVal) {
//	if (!g_deviceOpened || !g_commStarted) return false;
//	unsigned char buf[8] = {};
//	unsigned int actual = 0;
//	long e = g_ecat.PdoRead(slaveId, kIdxTorqueActual, kSubIdxTorqueActual, (unsigned int)sizeof(buf), buf, &actual);
//	if (e != ErrorCode::None) return false;
//	if (actual < 2) return false;
//	int16_t s16 = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
//	outVal = (int)s16;
//	return true;
//}

static bool ReadAxis0_TxPDO_6063(int& outVal) {
	return ReadAxis_TxPDO_6063(kAxisSlaveId[0], outVal);
}

// ------------------ IO / Sensor ------------------
static bool ReadSensorOn(Io& io) {
	unsigned char uc = 0;
	long e = io.GetInBitEx(IO_ADDR, IO_BIT, &uc);
	if (e != ErrorCode::None) return true;
	bool on_raw = (uc != 0);
	return SENSOR_ACTIVE_HIGH ? on_raw : !on_raw;
}

struct SensorEdgeMoveState {
	bool inited = false;
	bool lastStableOn = false;
	DWORD lastChangeTick = 0;
};
static SensorEdgeMoveState g_edgeMoveState;

static bool DebounceReadSensor(Io& io, bool& stableOn, DWORD& lastChangeTick) {
	bool cur = ReadSensorOn(io);
	DWORD now = GetTickCount();
	if (cur != stableOn) {
		if (now - lastChangeTick >= SENSOR_DEBOUNCE_MS) {
			stableOn = cur;
			lastChangeTick = now;
			return true;
		}
		return false;
	}
	return false;
}

// ======================== 인터락 보조 함수들 ========================

// 현재 위치(주행) 코드: Load=0x01, Unload=0x02, 그 외=0x00
inline unsigned char CalcPosTravelCode()
{
	int bc = 0;
	if (!ReadAxis0_TxPDO_6063(bc)) return 0x00;
	if (std::llabs((long long)bc - 476774) <= 10) return 0x01; // Load
	if (std::llabs((long long)bc - 491332) <= 10) return 0x02; // Unload
	return 0x00;
}

// Axis2 리밋 센서 현재 상태
inline bool IsAxis2LimitOn()
{
	return ReadInputBit(AX2_LIMIT_ADDR, AX2_LIMIT_BIT, AX2_LIMIT_ACTIVE_HIGH);
}

// Auto 모드일 때만 적용되는 인터락: Axis0(주행) 시작 가능 여부
static bool CanAxis0Move_Auto()
{
	// 요구: Axis2 리밋 센서 ON일 때만 Axis0 동작 가능
	return IsAxis2LimitOn();
}

// Auto 모드일 때만 적용되는 인터락: Axis2(상하) 시작 가능 여부
static bool CanAxis2Move_Auto()
{
	// 요구: 주행 위치가 Load/Unload 위치일 때만 Axis2 동작 가능
	unsigned char code = CalcPosTravelCode();
	return (code == 0x01 || code == 0x02);
}

static bool CanAxis2Move_Auto_RequireTravel(unsigned char requiredCode)
{
	unsigned char code = CalcPosTravelCode();
	if (code != requiredCode) {
		wchar_t s[128];
		swprintf_s(s, L"[INTERLOCK] Axis2 blocked: TravelCode=0x%02X, required=0x%02X",
			(unsigned)code, (unsigned)requiredCode);
		//AppendLog(s);
		return false;
	}
	return true;
}


// Auto 모드에서 Axis0/Axis2 인터락 메시지
static void ShowAxis0InterlockMsg()
{
	MessageBox(g_hMainWnd ? g_hMainWnd : nullptr,
		TEXT("인터락: Axis2 리밋 센서가 ON일 때만 주행축(Axis0) 동작 가능합니다."),
		TEXT("Interlock (Axis0)"), MB_ICONWARNING);
}
static void ShowAxis2InterlockMsg()
{
	MessageBox(g_hMainWnd ? g_hMainWnd : nullptr,
		TEXT("인터락: 주행부가 Load/Unload 위치에 있을 때만 상하축(Axis2) 동작 가능합니다."),
		TEXT("Interlock (Axis2)"), MB_ICONWARNING);
}

// Auto 모드에서 Axis0/Axis2의 이동/조그/상대/절대 명령에 대해 공통 검사
static bool CheckInterlockBeforeAxisCommand(int axis)
{
	if (!g_autoMode.load()) return true; // 인터락은 Auto 모드에서만

	if (axis == 0) {
		if (!CanAxis0Move_Auto()) { ShowAxis0InterlockMsg(); return false; }
	}
	if (axis == 2) {
		if (!CanAxis2Move_Auto()) { ShowAxis2InterlockMsg(); return false; }
	}
	return true;
}

// ===================================================================

static bool StartRelMoveWithProfile(int axis, long long delta, double vpps, double tAcc, double tDec) {

	// Axis0/Axis2 인터락
	//if (!CheckInterlockBeforeAxisCommand(axis)) return false;

	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[axis].actualPos;
	long long tgt = cur + delta;

	// Axis2 보호: 리밋 ON 시 -방향 금지
	if (axis == 2 && g_commStarted) {
		if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)((delta >= 0) ? +1 : -1))) {
			if (g_hMainWnd) Axis2ShowMinusBlockedWarning(g_hMainWnd);
			return false;
		}
	}

	// Axis0 보호: L/R 리밋에 따른 방향 차단
	if (axis == 0 && g_commStarted) {
		long long sign = (delta >= 0) ? +1 : -1;
		if (Axis0IsCommandBlocked(cur, tgt, sign)) {
			if (g_hMainWnd) Axis0ShowBlockedWarning(g_hMainWnd, (int)sign);
			return false;
		}
	}

	Motion::PosCommand pc; pc.axis = axis; pc.target = tgt;
	pc.profile.type = ProfileType::SCurve;
	pc.profile.velocity = (int)std::lround(vpps);
	pc.profile.acc = TimeMsToAcc(vpps, tAcc);
	pc.profile.dec = TimeMsToAcc(vpps, tDec);
	long e = g_cm.motion->StartPos(&pc);
	if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("StartPos(Rel) 실패"), e, g_wmx); return false; }

	// CHANGED: Enable logging for this axis and mark new command
	g_axisCmdInfo[axis].axis = axis;
	g_axisCmdInfo[axis].target = tgt;
	g_axisCmdInfo[axis].vel = pc.profile.velocity;
	g_axisCmdInfo[axis].acc = pc.profile.acc;
	g_axisCmdInfo[axis].dec = pc.profile.dec;
	g_axisCmdInfo[axis].startTick = GetTickCount64();
	g_axisCmdInfo[axis].endTick = 0;
	g_axisCmdInfo[axis].active = true;
	g_axisLogEnabled[axis] = true;
	g_axisLogRowIdx[axis] = 0;

	return true;
}

// ------------------ 컨트롤 ID ------------------
inline int ID_EDIT_POS_A(int axis) { return 1000 + axis; }
inline int ID_EDIT_VEL_A(int axis) { return 1100 + axis; }
inline int ID_EDIT_ACCT_A(int axis) { return 1200 + axis; }
inline int ID_EDIT_DECT_A(int axis) { return 1300 + axis; }
inline int ID_BTN_SVON_A(int axis) { return 2000 + axis; }
inline int ID_BTN_SVOFF_A(int axis) { return 2100 + axis; }
inline int ID_BTN_HOME_A(int axis) { return 2200 + axis; }
inline int ID_BTN_ABS_A(int axis) { return 2300 + axis; }
inline int ID_BTN_REL_A(int axis) { return 2400 + axis; }
inline int ID_BTN_JOGP_A(int axis) { return 2500 + axis; }
inline int ID_BTN_JOGM_A(int axis) { return 2600 + axis; }
inline int ID_BTN_STOP_A(int axis) { return 2700 + axis; }

inline int ID_EDIT_ALTTGT_A(int axis) { return 3000 + axis; }
inline int ID_BTN_APPLY_ALT_A(int axis) { return 3100 + axis; }

#define ID_CHECK_AXIS_0 5000
#define ID_CHECK_AXIS_1 5001
#define ID_CHECK_AXIS_2 5002
#define ID_CHECK_AXIS_3 5003

#define ID_BTN_MULTI_ABS 6000
#define ID_BTN_MULTI_REL 6001
#define ID_BTN_MULTI_STOP 6002
#define ID_BTN_MULTI_JOGP 6003
#define ID_BTN_MULTI_JOGM 6004
#define ID_BTN_MULTI_SVON 6005
#define ID_BTN_MULTI_SVOFF 6006
#define ID_BTN_SELECT_ALL 6007
#define ID_BTN_CLEAR_ALL 6008
#define ID_BTN_MULTI_ALARM_RST 6009
#define ID_BTN_MULTI_HOME 6010

#define ID_BTN_CREATE_DEVICE 7000
#define ID_BTN_START_COMM 7001
#define ID_BTN_DEMO 7002
#define ID_BTN_DEMO2 7003
#define ID_BTN_DEMO0 7004

#define ID_BTN_SYNC_WINDOW 7100

#define ID_BTN_FASTECH_MANUAL 7200
#define ID_BTN_WELCON_MANUAL 7201

#define ID_BTN_ESTOP_TOGGLE 7250
#define ID_TXT_ESTOP_STATE 7251

// NEW: Manual/Auto mode buttons and label
#define ID_BTN_MODE_MANUAL 7260
#define ID_BTN_MODE_AUTO   7261
#define ID_TXT_MODE_STATE  7262

inline int ID_TXT_STATUS(int axis, int col) { return 8000 + axis * 30 + col; }
#define ID_TXT_SELECTED_AXES 9000
#define ID_TXT_ZONE_ANNOUNCE 9100
#define ID_TXT_MOTION_ANNOUNCE 9101

#define ID_TIMER 1

#define ID_TXT_ECAT_6063 9150
#define ID_TXT_ECAT_603F 9151

// Axis2 Limit / Home 상태 표시용 텍스트
#define ID_TXT_AX2_LIMIT  9152
#define ID_TXT_AX2_HOME   9153
#define ID_TXT_AX0_LIMIT_L  9154
#define ID_TXT_AX0_LIMIT_R  9155

// ==== Serial Monitor Window (separate) ====
#define ID_EDIT_TCP_IP 9300
#define ID_EDIT_TCP_PORT 9301
#define ID_BTN_TCP_START 9302
#define ID_BTN_TCP_STOP 9303
#define ID_LIST_TCP_LOG 9304
#define ID_BTN_SERIAL_WINDOW 9400
#define ID_TXT_TCP_STATE 9450
#define ID_BTN_LOG_WINDOW 9600
#define ID_BTN_SCOPE_WINDOW 9601

// ==== Sync 창 컨트롤 ID ====
enum : int {
	ID_SYNC_TIMER = 10001,
	ID_SYNC_GROUP_COMBO = 10002,

	ID_SYNC_PARAM_ENABLE = 10010,
	ID_SYNC_PARAM_DISABLE = 10011,
	ID_SYNC_PARAM_REFRESH = 10012,

	ID_SYNC_PARAM_SERVO_ONOFF = 10100,
	ID_SYNC_PARAM_STARTUP = 10101,
	ID_SYNC_PARAM_DESYNC = 10102, // placeholder label
	ID_SYNC_PARAM_CYCLE_RATIO = 10103,
	ID_SYNC_PARAM_MAX_CATCH_UP = 10104,
	ID_SYNC_PARAM_CATCHUP_VEL = 10105,
	ID_SYNC_PARAM_CATCHUP_ACC = 10106,
	ID_SYNC_PARAM_TOLERANCE = 10107,

	// NOTE: no grp.desyncDec in SyncGroup. Keep an edit box to set Config::SyncParam.masterDesyncDec / slaveDesyncDec
	ID_SYNC_PARAM_MASTER_DESYNC_DEC = 10108,
	ID_SYNC_PARAM_SLAVE_DESYNC_DEC = 10109,

	ID_SYNC_PARAM_USE_MASTER_FB = 10110,
	ID_SYNC_PARAM_AMP_ERR_SVON = 10111,

	ID_SYNC_RAD_MASTER = 10200,
	ID_SYNC_RAD_SLAVE = 10201,
	ID_SYNC_AXIS_SELECT = 10202,
	ID_SYNC_POS_CMD = 10203,
	ID_SYNC_POS_ACT = 10204,
	ID_SYNC_OPSTATE = 10205,
	ID_SYNC_BTN_SVON = 10206,
	ID_SYNC_BTN_HOME = 10207,
	ID_SYNC_BTN_STOP = 10208,
	ID_SYNC_BTN_ALARM_RST = 10209,

	ID_SYNC_JOG_SPEED = 10300,
	ID_SYNC_ACC = 10301,
	ID_SYNC_DEC = 10302,
	ID_SYNC_JERK = 10303,
	ID_SYNC_CMD_VEL = 10304,
	ID_SYNC_ACT_VEL = 10305,
	ID_SYNC_BTN_JOG_CCW = 10306,
	ID_SYNC_BTN_JOG_CW = 10307,
	ID_SYNC_ABS_POS = 10308,
	ID_SYNC_BTN_ABS = 10309,
	ID_SYNC_REL_STEP = 10310,
	ID_SYNC_BTN_REL_P = 10311,
	ID_SYNC_BTN_REL_M = 10312,

	ID_SYNC_ALT_TARGET = 10350,
	ID_SYNC_BTN_APPLY_ALT = 10351,
	ID_SYNC_ECAT_6063 = 10352,
	ID_SYNC_ECAT_603F = 10353,

	ID_SYNC_BTN_FASTECH_MANUAL = 10360,
	ID_SYNC_BTN_WELCON_MANUAL = 10361,

	ID_SYNC_STATE_ENABLED = 10400,
	ID_SYNC_STATE_HOMEDONE = 10401,
	ID_SYNC_ALL_SERVO_ON = 10402,
	ID_SYNC_ALL_SERVO_OFF = 10403,
	ID_SYNC_GROUP_HOME = 10404,
	ID_SYNC_GROUP_CLEAR = 10405,

	ID_SYNC_AXIS_LIST = 10450,

	ID_SYNC_MASTER_AXIS_BTN_BASE = 12000,
	ID_SYNC_SLAVE_AXIS_CHK_BASE = 13000,

	ID_SYNC_BTN_ESTOP_TOGGLE = 14000,
	ID_SYNC_TXT_ESTOP_STATE = 14001
};

// ======================================================

// UI 헬퍼
struct FieldPos { int endX; };
static FieldPos TightLabeledEdit2(HWND parent, int x, int y,
	const TCHAR* lab, int labelEditGap,
	int editW, int cid, const TCHAR* init,
	const TCHAR* unit = nullptr, int editUnitGap = 10) {
	HDC hdc = GetDC(parent);
	HFONT hFont = (HFONT)SendMessage(parent, WM_GETFONT, 0, 0);
	HFONT hOld = (HFONT)SelectObject(hdc, hFont);
	RECT rc = { 0,0,0,0 };
#ifdef UNICODE
	DrawTextW(hdc, lab, -1, &rc, DT_CALCRECT);
#else
	DrawText(hdc, lab, -1, &rc, DT_CALCRECT);
#endif
	int lw = rc.right - rc.left; if (lw < 30) lw = 30;
	SelectObject(hdc, hOld); ReleaseDC(parent, hdc);

	CreateWindow(TEXT("STATIC"), lab, WS_CHILD | WS_VISIBLE, x, y, lw, 20, parent, nullptr, nullptr, nullptr);
	int ex = x + lw + labelEditGap;
	CreateWindow(TEXT("EDIT"), init, WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
		ex, y - 2, editW, 24, parent, (HMENU)(INT_PTR)cid, nullptr, nullptr);
	int endX = ex + editW;

	if (unit && unit[0]) {
		CreateWindow(TEXT("STATIC"), unit, WS_CHILD | WS_VISIBLE, endX + editUnitGap, y, 70, 20, parent, nullptr, nullptr, nullptr);
		endX += editUnitGap + 70;
	}
	return { endX };
}

// ------------------ 조그 ------------------
struct JogBtnCtx { int axis; int sign; bool isMulti; };

static bool StartJog(HWND hWnd, int axis, int sign) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return false; }

	//// Auto 모드 인터락: Axis0/2 조그 시작 전 검사
	//if (g_autoMode.load()) {
	//	if (!CheckInterlockBeforeAxisCommand(axis)) return false;
	//}

	if (axis == 2) {
		// Axis2는 Limit ON 시 -방향 차단
		g_cm.GetStatus(&g_status);
		long long curPos = (long long)g_status.axesStatus[2].actualPos;
		if (Axis2IsMinusCommandBlocked(curPos, curPos, sign)) {
			Axis2ShowMinusBlockedWarning(hWnd);
			return false;
		}
	}

	if (axis == 0) {
		// Axis0: L/R 리밋 방향 차단
		g_cm.GetStatus(&g_status);
		long long curPos = (long long)g_status.axesStatus[0].actualPos;
		if (Axis0IsCommandBlocked(curPos, curPos, (long long)sign)) {
			Axis0ShowBlockedWarning(hWnd, sign);
			return false;
		}
	}

	//DisableAllEnabledSyncGroups();
	if (!EnsureServoOn(axis)) return false;
	if (!EnsurePosModeNoStop(axis)) return false;

	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[axis].actualPos;

	double vpps = GetDlgDouble(hWnd, ID_EDIT_VEL_A(axis), 10000.0);
	double tAcc = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(axis), 100.0);
	double tDec = GetDlgDouble(hWnd, ID_EDIT_DECT_A(axis), 100.0);

	Motion::PosCommand pc;
	pc.axis = axis;
	pc.target = cur + (long long)(sign * 1000000000LL);
	pc.profile.type = ProfileType::SCurve;
	pc.profile.velocity = (int)std::lround(vpps);
	pc.profile.acc = TimeMsToAcc(vpps, tAcc);
	pc.profile.dec = TimeMsToAcc(vpps, tDec);

	long e = g_cm.motion->StartPos(&pc);
	if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("StartPos(JOG) 실패"), e, g_wmx); return false; }

	// 로그 트래킹 및 enable
	g_axisCmdInfo[axis].axis = axis;
	g_axisCmdInfo[axis].target = pc.target;
	g_axisCmdInfo[axis].vel = pc.profile.velocity;
	g_axisCmdInfo[axis].acc = pc.profile.acc;
	g_axisCmdInfo[axis].dec = pc.profile.dec;
	g_axisCmdInfo[axis].startTick = GetTickCount64();
	g_axisCmdInfo[axis].endTick = 0;
	g_axisCmdInfo[axis].active = true;
	g_axisLogEnabled[axis] = true; // CHANGED
	g_axisLogRowIdx[axis] = 0; // CHANGED

	g_lastCmdVel[axis] = (int)std::lround(pc.profile.velocity * sign);
	g_jogActiveAxis = axis;
	g_jogActiveSign = sign;
	SetCapture(hWnd);
	return true;
}
static void StopJogIfActive() {
	if (g_jogActiveAxis >= 0) {
		g_cm.motion->Stop(g_jogActiveAxis);
		g_cm.velocity->Stop(g_jogActiveAxis);
		if (g_cm.torque) g_cm.torque->StopTrq(g_jogActiveAxis);
		g_jogActiveAxis = -1;
		g_jogActiveSign = 0;
	}
}
static bool StartMultiJog(HWND hWnd, int sign) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return false; }
	//DisableAllEnabledSyncGroups();
	bool any = false; for (int a = 0; a < 4; ++a) g_multiJogAxisActive[a] = false;

	g_cm.GetStatus(&g_status);
	for (int a = 0; a < 4; ++a) {
		if (!IsAxisChecked(hWnd, a)) continue;

		// Auto 모드 인터락: Axis0/2에만 적용
		if (g_autoMode.load()) {
			//if (!CheckInterlockBeforeAxisCommand(a)) continue;
		}

		// Axis2 보호: 리밋 ON시 -방향 JOG 차단
		if (a == 2) {
			long long cur = (long long)g_status.axesStatus[2].actualPos;
			long long tgt = cur + (long long)(sign * 1000000000LL);
			if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)sign)) {
				Axis2ShowMinusBlockedWarning(hWnd);
				continue;
			}
		}

		// Axis0 보호: L/R 리밋에 따른 방향 차단
		if (a == 0) {
			long long cur = (long long)g_status.axesStatus[0].actualPos;
			long long tgt = cur + (long long)(sign * 1000000000LL);
			if (Axis0IsCommandBlocked(cur, tgt, (long long)sign)) {
				Axis0ShowBlockedWarning(hWnd, sign);
				continue;
			}
		}

		if (!EnsureServoOn(a)) continue;
		if (!EnsurePosModeNoStop(a)) continue;

		long long cur = (long long)g_status.axesStatus[a].actualPos;
		double vpps = GetDlgDouble(hWnd, ID_EDIT_VEL_A(a), 10000.0);
		double tAcc = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(a), 100.0);
		double tDec = GetDlgDouble(hWnd, ID_EDIT_DECT_A(a), 100.0);

		Motion::PosCommand pc; pc.axis = a; pc.target = cur + (long long)(sign * 1000000000LL);
		pc.profile.type = ProfileType::SCurve; pc.profile.velocity = (int)std::lround(vpps);
		pc.profile.acc = TimeMsToAcc(vpps, tAcc); pc.profile.dec = TimeMsToAcc(vpps, tDec);
		long e = g_cm.motion->StartPos(&pc);
		if (e == ErrorCode::None) {
			g_lastCmdVel[a] = (int)std::lround((double)pc.profile.velocity * sign);
			g_multiJogAxisActive[a] = true; any = true;

			// enable logging per-axis
			g_axisCmdInfo[a].axis = a;
			g_axisCmdInfo[a].target = pc.target;
			g_axisCmdInfo[a].vel = pc.profile.velocity;
			g_axisCmdInfo[a].acc = pc.profile.acc;
			g_axisCmdInfo[a].dec = pc.profile.dec;
			g_axisCmdInfo[a].startTick = GetTickCount64();
			g_axisCmdInfo[a].endTick = 0;
			g_axisCmdInfo[a].active = true;
			g_axisLogEnabled[a] = true; // CHANGED
			g_axisLogRowIdx[a] = 0; // CHANGED
		}
		else ShowErrMsgBox(TEXT("StartPos(Multi Jog) 실패"), e, g_wmx);
	}
	if (any) { g_multiJogActive = true; g_multiJogSign = sign; SetCapture(hWnd); return true; }
	g_multiJogActive = false; return false;
}
static void StopMultiJog() {
	if (!g_multiJogActive) return;
	for (int a = 0; a < 4; ++a) if (g_multiJogAxisActive[a]) {
		g_cm.motion->Stop(a); g_cm.velocity->Stop(a); if (g_cm.torque) g_cm.torque->StopTrq(a); g_multiJogAxisActive[a] = false;
	}
	g_multiJogActive = false; g_multiJogSign = 0;
}

// ------------------ E-Stop 토글 공용 함수 ------------------
static void UpdateEStopUi(HWND hWnd, bool syncWindow)
{
	if (syncWindow) {
		if (HWND h = GetDlgItem(hWnd, ID_SYNC_TXT_ESTOP_STATE))
			SetWindowText(h, g_estopActive.load() ? TEXT("E-STOP ACTIVE") : TEXT("NORMAL"));
		if (HWND b = GetDlgItem(hWnd, ID_SYNC_BTN_ESTOP_TOGGLE))
			SetWindowText(b, g_estopActive.load() ? TEXT("비상정지해제") : TEXT("비상정지"));
	}
	else {
		if (HWND h = GetDlgItem(hWnd, ID_TXT_ESTOP_STATE))
			SetWindowText(h, g_estopActive.load() ? TEXT("E-STOP ACTIVE") : TEXT("NORMAL"));
		if (HWND b = GetDlgItem(hWnd, ID_BTN_ESTOP_TOGGLE))
			SetWindowText(b, g_estopActive.load() ? TEXT("비상정지해제") : TEXT("비상정지"));
	}
}

static void DoToggleEStop(HWND hWnd, bool syncWindow)
{
	if (!g_deviceOpened || !g_commStarted) {
		MessageBox(hWnd, TEXT("먼저 Device 생성 및 통신을 시작하세요."), TEXT("E-Stop"), MB_ICONWARNING);
		return;
	}
	if (!g_estopActive.load()) {
		long e = g_cm.ExecEStop(EStopLevel::Final);
		if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("ExecEStop 실패"), e, g_wmx); return; }
		g_estopActive = true;
	}
	else {
		long e = g_cm.ReleaseEStop();
		if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("ReleaseEStop 실패"), e, g_wmx); return; }
		g_estopActive = false;
	}
	UpdateEStopUi(hWnd, syncWindow);
}

bool StartAbsMoveWithProfile(int axis, long long target, double vpps, double tAcc, double tDec) {

	// Auto 모드 인터락: Axis0/2 이동 전 검사
	//if (!CheckInterlockBeforeAxisCommand(axis)) return false;

	// Axis2 보호 체크: 리밋 ON시 -방향 금지
	if (axis == 2 && g_commStarted) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[2].actualPos;
		if (Axis2IsMinusCommandBlocked(cur, target, 0)) {
			if (g_hMainWnd) Axis2ShowMinusBlockedWarning(g_hMainWnd);
			return false;
		}
	}

	// Axis0 보호 체크: L/R 리밋에 따른 방향 금지
	if (axis == 0 && g_commStarted) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[0].actualPos;
		long long delta = target - cur;
		long long sign = (delta > 0) ? +1 : (delta < 0 ? -1 : 0);
		if (sign != 0 && Axis0IsCommandBlocked(cur, target, sign)) {
			if (g_hMainWnd) Axis0ShowBlockedWarning(g_hMainWnd, (int)sign);
			return false;
		}
	}

	Motion::PosCommand pc; pc.axis = axis; pc.target = target;
	pc.profile.type = ProfileType::SCurve;
	pc.profile.velocity = (int)std::lround(vpps);
	pc.profile.acc = TimeMsToAcc(vpps, tAcc);
	pc.profile.dec = TimeMsToAcc(vpps, tDec);
	long e = g_cm.motion->StartPos(&pc);
	if (e != ErrorCode::None) { ShowErrMsgBox(TEXT("StartPos 실패"), e, g_wmx); return false; }

	// 고정된 시작 시간 및 active 설정 + CHANGED: enable per-axis sampling
	g_axisCmdInfo[axis].axis = axis;
	g_axisCmdInfo[axis].target = target;
	g_axisCmdInfo[axis].vel = pc.profile.velocity;
	g_axisCmdInfo[axis].acc = pc.profile.acc;
	g_axisCmdInfo[axis].dec = pc.profile.dec;
	g_axisCmdInfo[axis].startTick = GetTickCount64();
	g_axisCmdInfo[axis].endTick = 0;
	g_axisCmdInfo[axis].active = true;

	g_axisLogEnabled[axis] = true; // CHANGED: resume logging on new command
	g_axisLogRowIdx[axis] = 0; // reset row index for this command

	return true;
}
static bool StartRelMoveWithProfile_Generic(int axis, long long delta, double vpps, double tAcc, double tDec) {
	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[axis].actualPos;
	long long tgt = cur + delta;
	return StartAbsMoveWithProfile(axis, tgt, vpps, tAcc, tDec);
}

// ------------------ WMX3 init/shutdown ------------------
// ====== Init guards & retry helpers ======
static std::atomic<bool> g_initTried{ false };
static std::atomic<bool> g_commTried{ false };
static std::atomic<bool> g_autoStartDone{ false };
static std::atomic<bool> g_ioInitDone{ false };

static long CreateDeviceWithRetry(WMX3Api& api, const TCHAR* installPath, int maxRetry = 2, DWORD backoffMs = 300)
{
	long err = ErrorCode::None;
	for (int i = 0; i <= maxRetry; ++i) {
		err = api.CreateDevice(installPath, DeviceType::DeviceTypeNormal);
		if (err == ErrorCode::None) return err;
		// 에러 268 등 제어 채널 락 실패 시에는 CloseDevice 하고 백오프 후 재시도
		api.CloseDevice();
		Sleep(backoffMs * (i + 1));
	}
	return err;
}
static bool InitDevice() {
	// 이미 열려 있으면 OK
	if (g_deviceOpened) return true;

	// 중복 시도 방지
	bool expected = false;
	if (!g_initTried.compare_exchange_strong(expected, true)) {
		// 다른 경로에서 이미 시도 중이거나 완료됨
		// 현재 상태를 그대로 보고
		return g_deviceOpened;
	}

	// 재시도 포함하여 CreateDevice
	long e = CreateDeviceWithRetry(g_wmx, g_installPath, /*maxRetry*/2, /*backoffMs*/300);
	if (e != ErrorCode::None) {
		ShowErrMsgBox(TEXT("CreateDevice 실패"), e, g_wmx);
		g_initTried = false; // 다음에 다시 눌러볼 수 있게
		return false;
	}

	g_deviceOpened = true;
	return true;
}

// Helper: Set gear ratio for one axis (numerator/denominator)
static bool SetAxisGearRatio(int axis, double numerator, double denominator) {
	if (!g_commStarted) return false;
	long err = g_cm.config->SetGearRatio(axis, numerator, denominator);
	if (err != ErrorCode::None) {
		ShowErrMsgBox(TEXT("SetGearRatio 실패"), err, g_wmx);
		return false;
	}
	return true;
}

static bool StartComm() {
	if (!g_deviceOpened) {
		// 장치가 없으면 먼저 InitDevice
		if (!InitDevice()) return false;
	}

	if (g_commStarted) return true;

	bool expected = false;
	if (!g_commTried.compare_exchange_strong(expected, true)) {
		// 다른 경로에서 이미 시도 중이거나 완료됨
		return g_commStarted;
	}

	long e = g_wmx.StartCommunication(15000);
	if (e != ErrorCode::None) {
		ShowErrMsgBox(TEXT("StartCommunication 실패"), e, g_wmx);
		g_commTried = false; // 다음 시도 허용
		return false;
	}

	g_commStarted = true;
	g_estopActive = false;

	// 기어비 설정 등 초기 파라미터
	SetAxisGearRatio(0, 43000.0, 10000.0);
	SetAxisGearRatio(1, 43000.0, 10000.0);
	SetAxisGearRatio(2, 100000.0, 10000.0);

	// Axis2 sensor flags reset
	g_ax2LimitOn = false;
	g_ax2HomeOn = false;
	g_ax2LimitLatched = false;
	g_ax2LimitBlocking = false;
	g_ax2StopIssuedOnLimit = false;
	g_ax2HomingStarted = false;
	g_ax2HomeDebounceOn = false;
	g_ax2HomeLastTick = GetTickCount();
	g_ax2HomeRampIssued = false;

	return true;
}

static void ShutdownWMX() {
	StopJogIfActive();
	StopMultiJog();

	if (g_commStarted) {
		g_wmx.StopCommunication();
		g_commStarted = false;
		g_commTried = false;
	}

	if (g_deviceOpened) {
		g_wmx.CloseDevice();
		g_deviceOpened = false;
		g_initTried = false;
	}

	g_estopActive = false;
}

// ------------------ 주기 상태 전송(1초) 쓰레드 [NEW] ------------------
std::atomic<bool> g_periodicRun{ false };
std::thread g_periodicThread;
std::atomic<unsigned char> g_heartbeat{ 0 };

// Demo alarm: 1 byte (0x00 = none). Latched until cleared.
std::atomic<unsigned char> g_demoAlarm{ 0x00 };

// (선택) 첫 에러만 잡고 싶으면 이렇게 쓰는 함수
static inline void LatchDemoAlarm(unsigned char code)
{
	if (code == 0x00) return;
	unsigned char cur = g_demoAlarm.load(std::memory_order_relaxed);
	if (cur == 0x00) g_demoAlarm.store(code, std::memory_order_relaxed);
}

static bool IsAxisServoOn(int axis)
{
	if (!g_commStarted) return false;
	g_cm.GetStatus(&g_status);

	// ⚠️ 프로젝트의 status 구조체에 맞게 필드명만 확인해줘.
	// (servoOn / ampEnabled / driveEnabled 등일 수 있음)
	return (g_status.axesStatus[axis].servoOn != 0);
}

enum : unsigned char
{
	DEMO_ALM_NONE = 0x00,
	SERVO_ALM = 0x30,

	// ---- TRAVEL(0x2A): 0x40~0x4F
	DEMO_ALM_TRAVEL_INTERLOCK_FAIL = 0x40,
	DEMO_ALM_TRAVEL_BUSY = 0x41,
	DEMO_ALM_TRAVEL_SERVO1_OFF = 0x42, // ★ Axis1 ServoOn 요구
	DEMO_ALM_TRAVEL_INVALID_POS = 0x43,
	DEMO_ALM_TRAVEL_TO_CONVEYOR_TO = 0x44,
	DEMO_ALM_TRAVEL_TO_WORK_TO = 0x45,

	// ---- HOIST(0x2B): 0x50~0x5F
	DEMO_ALM_HOIST_INTERLOCK_FAIL = 0x50,
	DEMO_ALM_HOIST_BUSY = 0x51,
	DEMO_ALM_HOIST_SERVO2_OFF = 0x52, // ★ Axis2 ServoOn 요구
	DEMO_ALM_HOIST_INVALID_POS = 0x53,
	DEMO_ALM_HOIST_CONVEYORDOWN_TO = 0x54,
	DEMO_ALM_HOIST_WORKDOWN_TO = 0x55,
	DEMO_ALM_HOIST_UP_TO = 0x56,
};


// 운전 준비 완료 플래그 (주기 프레임에서 사용)
std::atomic<bool> g_driveReady{ false };

extern bool IsGripperOpen(); // 그립퍼 열림 상태 외부 참조
extern bool IsGripperOpenAndIdle(); // 그립퍼 열림 상태 외부 참조
extern bool IsGripperClosed(); // 그립퍼 닫힘 상태 외부 참조
extern bool IsGripperClosedAndIdle(); // 그립퍼 닫힘 상태 외부 참조
extern bool g_distable[8]; // 그립퍼 축 비활성화 플래그 외부 참조
extern bool WaitAllAxesStopped(double velEps, DWORD timeoutMs); // 외부 참조


static inline bool BetweenTol(long long v, long long center, long long tol) {
	return (std::llabs(v - center) <= tol);
}

// 현재위치(주행) 계산: Axis0의 6063 기준
// Load=01 (278000±10), Unload=02 (253381±10), 그 외 00
unsigned char CalcPosTravelCode(); // 위에서 정의됨
//static unsigned char CalcPosTravelCode()
//{
//	int bc = 0;
//	if (!ReadAxis0_TxPDO_6063(bc)) return 0x00;
//	if (BetweenTol(bc, 278000, 10)) return 0x01; // Load
//	if (BetweenTol(bc, 253381, 10)) return 0x02; // Unload
//	return 0x00;
//}

// 현재위치(상하) 계산: Axis2 actualPos
// Load=01 (60000±10), Unload=02 (57000±10), Up=03 (0±10), 그 외 00
static unsigned char CalcPosHoistCode()
{
	if (!g_commStarted) return 0x00;
	g_cm.GetStatus(&g_status);
	long long p = (long long)g_status.axesStatus[2].actualPos;
	if (BetweenTol(p, 51910, 10)) return 0x01;
	if (BetweenTol(p, 45000, 10)) return 0x02;
	if (BetweenTol(p, 0, 10))     return 0x03;
	return 0x00;
}

static std::atomic<DWORD> g_gripBusyStartTick{ 0 };

// “중복 명령 후 stuck”으로 판단할 시간(ms) - 필요에 맞게 조절
static constexpr DWORD GRIP_STUCK_REPORT_MS = 5000; // 예: 2초
extern std::atomic<unsigned char> g_doShadow[32];

std::atomic<bool> g_servo12Ready{ true };         // “명령 수신 가능” 게이트
std::atomic<bool> g_servo12MonRunning{ false };   // 중복 스레드 방지

inline void StartServo12ReadyMonitor()
{
	bool expected = false;
	if (!g_servo12MonRunning.compare_exchange_strong(expected, true,
		std::memory_order_acq_rel, std::memory_order_relaxed))
		return;

	std::thread([]() {
		using namespace std::chrono_literals;

		// 둘 다 Servo ON 될 때까지 계속 폴링
		while (true) {
			bool s1 = IsAxisServoOn(1);
			bool s2 = IsAxisServoOn(2);

			if (s1 && s2) {
				g_servo12Ready.store(true, std::memory_order_release);
				//AppendLog(L"[STATE] ServoGate OPEN: Axis1&2 Servo ON confirmed -> commands enabled");
				break;
			}

			// 너무 로그가 많으면 부담이라 200ms 정도 추천
			std::this_thread::sleep_for(200ms);
		}

		g_servo12MonRunning.store(false, std::memory_order_release);
		}).detach();
}

// 공통 게이트 체크(모션/그립 명령 앞단에서 사용)
inline bool BlockIfServoNotReady(unsigned char cmd)
{
	if (!g_servo12Ready.load(std::memory_order_acquire)) {
		wchar_t msg[128];
		swprintf_s(msg, L"[BLOCK] 0x%02X ignored: Axis1/2 Servo ON not confirmed yet", (unsigned)cmd);
		//AppendLog(msg);

		LatchDemoAlarm(SERVO_ALM);

		// PLC 대기 방지용 ACK (원치 않으면 제거 가능하지만, PLC가 멈추는 경우가 많음)
		SendSimpleAck(g_clientSock, cmd);
		return true; // “처리 완료(동작 없음)”
	}
	return false;
}

unsigned char CalcPosGripCode()
{
	// 진행중이면 0x00 (기본)
	bool busy = g_gripBusy.load(std::memory_order_relaxed) || g_diStable[0];

	// 1) HOLD가 켜져 있으면 기본은 무조건 0x00
	if (g_gripHoldCodeZero.load(std::memory_order_relaxed))
	{
		// 1-1) 명령 후 완료대기 중이면: busy->idle 전이를 기다린다
		if (g_gripAwaitDone.load(std::memory_order_relaxed))
		{
			if (busy) {
				g_gripSawBusy.store(true, std::memory_order_relaxed);
				return 0x00; // 동작 중 0x00 고정
			}

			// idle 상태
			if (g_gripSawBusy.load(std::memory_order_relaxed)) {
				// busy를 한번 봤고 이제 idle이면 "동작 완료"로 판단
				g_gripAwaitDone.store(false, std::memory_order_relaxed);
				g_gripSawBusy.store(false, std::memory_order_relaxed);
				g_gripHoldCodeZero.store(false, std::memory_order_relaxed); // 이제 자동 갱신 허용
				// 아래 기존 자동판정 로직으로 내려가서 0x01/0x02를 산출
			}
			else {
				// 명령은 받았는데 아직 busy가 한번도 안 켜짐(지연/실패/대기)
				return 0x00;
			}
		}
		else {
			// 1-2) Stop 이후, 명령이 오기 전(또는 Reset만 한 상태): 계속 0x00
			return 0x00;
		}
	}

	if (busy) {
		DWORD now = GetTickCount();
		DWORD t0 = g_gripBusyStartTick.load(std::memory_order_relaxed);
		if (t0 == 0) g_gripBusyStartTick.store(now, std::memory_order_relaxed);

		DWORD elapsed = now - g_gripBusyStartTick.load(std::memory_order_relaxed);

		// ★ 일정 시간 이상 진행중이면 DO 토글 상태로 강제 보고
		if (elapsed >= GRIP_STUCK_REPORT_MS) {
			bool do8 = (g_doShadow[8].load(std::memory_order_relaxed) != 0);
			bool do9 = (g_doShadow[9].load(std::memory_order_relaxed) != 0);

			if (do8 && !do9) return 0x01; // Open 의도
			if (do9 && !do8) return 0x02; // Close 의도
		}

		return 0x00;
	}

	// Busy 풀리면 타이머 리셋
	g_gripBusyStartTick.store(0, std::memory_order_relaxed);

	// ---- 이하 기존 자동 판정 로직 ----
	bool isOpen = IsGripperOpenAndIdle();
	bool isClose = IsGripperClosedAndIdle();
	bool openinit = IsGripperOpen();
	bool closeinit = IsGripperClosed();

	if ((isOpen && !isClose) || openinit)  return 0x01;
	if ((!isOpen && isClose) || closeinit) return 0x02;
	return 0x00;
}




// 알람코드(주행축): axis0/1 중 0이 아닌 603F 반환(우선 axis0)
static unsigned short GetTravelAlarm603F()
{
	int e0 = 0, e1 = 0;
	bool ok0 = ReadAxis_TxPDO_603F(kAxisSlaveId[0], e0);
	bool ok1 = ReadAxis_TxPDO_603F(kAxisSlaveId[1], e1);
	unsigned short v0 = ok0 ? (unsigned short)(e0 & 0xFFFF) : 0;
	unsigned short v1 = ok1 ? (unsigned short)(e1 & 0xFFFF) : 0;
	return v0 ? v0 : v1;
}

// 알람코드(상하축): axis2의 603F
static unsigned short GetHoistAlarm603F()
{
	int e2 = 0;
	bool ok2 = ReadAxis_TxPDO_603F(kAxisSlaveId[2], e2);
	return ok2 ? (unsigned short)(e2 & 0xFFFF) : 0;
}

static std::atomic<unsigned short> g_ohtMsgId{ 1 };

// 0x0001 ~ 0xFFFF 사용, 0x0000은 건너뜀
static unsigned short NextOhtMsgId()
{
	unsigned short cur = g_ohtMsgId.load(std::memory_order_relaxed);
	while (true) {
		unsigned short next = (cur == 0xFFFF) ? 1 : (unsigned short)(cur + 1);
		if (g_ohtMsgId.compare_exchange_weak(
			cur, next,
			std::memory_order_release,
			std::memory_order_relaxed))
		{
			return cur; // cur 값을 실제로 쓸 MsgID로 사용
		}
		// 실패하면 cur가 새 값으로 갱신되니 다시 루프 돌면서 재시도
	}
}

inline void ForceGripCodeZero(bool on)
{
	g_forceGripCodeZero.store(on, std::memory_order_relaxed);
	if (on) {
		g_gripBusyStartTick.store(0, std::memory_order_relaxed); // 타이머도 리셋(선택)
	}
}


static void SendPeriodicStateFrame(SOCKET s)
{
	if (s == INVALID_SOCKET) return;

	unsigned short msgId = NextOhtMsgId();

	unsigned char mode = g_autoMode.load() ? 0x01 : 0x00;
	unsigned char posTravel = CalcPosTravelCode();
	unsigned char posHoist = CalcPosHoistCode();
	unsigned char posGrip = CalcPosGripCode(); // 보류
	unsigned char demoAlm = g_demoAlarm.load(std::memory_order_relaxed); // ★ NEW (1 byte)
	unsigned short almTravel = GetTravelAlarm603F();
	unsigned short almHoist = GetHoistAlarm603F();
	unsigned char almGripL = 0x00, almGripH = 0x00; // 보류
	unsigned char hb = g_heartbeat.load();
	unsigned char driveReady = g_driveReady.load() ? 0x01 : 0x00;

	// [수정] 실제 페이로드 바이트 수 계산 (11바이트)
	// mode(1) + posTravel(1) + posHoist(1) + posGrip(1)
	// + almTravel(2) + almHoist(2) + almGrip(2) + hb(1)
	const unsigned char payload_len = 0x0D; // [수정] 0x09 -> 0x0B (11)

	// [수정] 프레임 총 길이: STX(1) + MsgID(2) + Op(1) + Len(1) + Payload(payload_len) + ETX(1)
	const size_t frame_capacity = 5 + payload_len + 1;

	// [수정] 고정 크기 대신 계산된 크기로 배열 확보
	unsigned char frame[5 + 0x0D + 1] = {}; // = 5 + 12 + 1 = 18 바이트

	// Header
	frame[0] = 0x02;       // STX
	frame[1] = 0x00; // MsgID High
	frame[2] = 0x00;        // MsgID Low
	frame[3] = 0xFE;       // Operation Code: State
	frame[4] = payload_len; // [수정] 0x09 -> payload_len

	// Payload
	size_t i = 5;
	frame[i++] = mode;
	frame[i++] = posTravel;
	frame[i++] = posHoist;
	frame[i++] = posGrip;
	
	// 알람코드(주행) 2 bytes (LSB, MSB)
	frame[i++] = (unsigned char)(almTravel & 0xFF);
	frame[i++] = (unsigned char)((almTravel >> 8) & 0xFF);
	// 알람코드(상하) 2 bytes
	frame[i++] = (unsigned char)(almHoist & 0xFF);
	frame[i++] = (unsigned char)((almHoist >> 8) & 0xFF);
	// 알람코드(그립) 2 bytes
	frame[i++] = almGripL;
	frame[i++] = almGripH;
	// Heartbeat (토글 대상 값)
	frame[i++] = hb;
	// 운전준비 완료 플래그
	frame[i++] = driveReady;
	// ★ NEW: demoAlarm 1 byte 먼저 추가
	frame[i++] = demoAlm;

	// ETX
	frame[i++] = 0x03;

	// [추가] 방어적 검사: i는 frame_capacity와 같아야 함
	// (개발 중 디버그 보조용, 릴리스에서는 제거 가능)
	// assert(i == frame_capacity);

	send(s, (const char*)frame, (int)i, 0);

	// 토글
	g_heartbeat = (unsigned char)(hb ? 0x00 : 0x01);
}

// msgId는 이제 필요 없음, OHT 자체 시퀀스로 보냄
static void SendSimpleAck(SOCKET s, unsigned char reqOpCode)
{
	if (s == INVALID_SOCKET) return;

	unsigned char f[7];
	unsigned short msgId = NextOhtMsgId();

	f[0] = 0x02;                                 // STX
	f[1] = 0x00; // MsgID High
	f[2] = 0x00;        // MsgID Low
	f[3] = 0xFF;                                 // ACK OpCode
	f[4] = 0x01;                                 // Payload Length = 1
	f[5] = reqOpCode;                            // Payload: 원 요청 OpCode
	f[6] = 0x03;                                 // ETX

	send(s, (const char*)f, 7, 0);
}


static void SendSimpleDone(SOCKET s, unsigned char reqOpCode)
{
	if (s == INVALID_SOCKET) return;

	unsigned char f[7];
	unsigned short msgId = NextOhtMsgId();

	f[0] = 0x02;                                 // STX
	f[1] = 0x00; // MsgID High
	f[2] = 0x00;        // MsgID Low
	f[3] = 0x81;                                 // DONE OpCode
	f[4] = 0x01;                                 // Payload Length = 1
	f[5] = reqOpCode;                            // Payload: 원 요청 OpCode
	f[6] = 0x03;                                 // ETX

	send(s, (const char*)f, 7, 0);
}



// ---- NEW: Human-readable op name ----
static const wchar_t* DecodeOpName(unsigned char op)
{
	switch (op) {
	case 0x21: return L"Load Request";
	case 0x22: return L"Unload Request";
	case 0x29: return L"Stop Motion Request";
	case 0x2A: return L"Travel Position Command";
	case 0x2B: return L"Hoist Position Command";
	case 0x2C: return L"Grip Position Command";
	case 0x2F: return L"Drive Ready Request";
	case 0xFD: return L"Reset Request";
	case 0xFE: return L"State Frame";
	case 0xFF: return L"ACK";
	case 0x81: return L"Done";
	case 0xF0: return L"Comm Open";
	default:   return L"Unknown";
	}
}

// ---- NEW: Append human readable log to Serial Monitor ----
static void AppendLog(const wchar_t* wmsg)
{
	size_t len = wcslen(wmsg);
	wchar_t* dup = (wchar_t*)malloc((len + 1) * sizeof(wchar_t));
	if (!dup) return;
	wcscpy_s(dup, len + 1, wmsg);
	PostMessage(g_hMainWnd ? g_hMainWnd : GetDesktopWindow(), WM_APP_TCP_LOG, 0, (LPARAM)dup);
}

static void LogFrameHuman(const unsigned char* f, int len, const wchar_t* prefix)
{
	if (!f || len <= 0) return;

	// Hex line
	wchar_t hexbuf[2048];
	int wi = swprintf_s(hexbuf, L"%s HEX (%d): ", prefix, len);
	for (int i = 0; i < len && wi < (int)_countof(hexbuf) - 4; ++i)
		wi += swprintf_s(hexbuf + wi, _countof(hexbuf) - wi, L"%02X ", (unsigned char)f[i]);
	AppendLog(hexbuf);

	// If basic frame 02 .... 03, decode op and length
	if (len >= 6 && f[0] == 0x02 && f[len - 1] == 0x03) {
		unsigned char op = f[3];
		unsigned char plen = f[4];
		wchar_t info[256];
		swprintf_s(info, L"%s Decoded: OP=0x%02X (%s), PayloadLen=%u", prefix, op, DecodeOpName(op), (unsigned)plen);
		AppendLog(info);
	}
}

// 이벤트 처리용 도우미: 예시 동작들
static void DoOhtAction_Load_Axis0_MoveTo50000()
{
	if (!g_commStarted) return;
	int ax = 0;
	EnsureServoOn(ax);
	EnsurePosModeNoStop(ax);
	// 절대 50000 이동, 프로파일은 간단 값
	StartAbsMoveWithProfile(ax, 50000, 10000, 100, 100);
	// 완료 대기
	DWORD t0 = GetTickCount();
	while (true) {
		g_cm.GetStatus(&g_status);
		long long ap = (long long)g_status.axesStatus[ax].actualPos;
		int av = (int)std::lround(g_status.axesStatus[ax].actualVelocity);
		if (std::llabs(ap - 50000) <= 5 && std::abs(av) <= 5) break;
		if (GetTickCount() - t0 > 30000) break; // timeout 30s
		Sleep(10);
	}
}

// 주행축: axis0, 절대 0 이동 (포지션 번호 2)
static void DoOhtAction_Load_Axis0_MoveTo0()
{
	if (!g_commStarted) return;
	int ax = 0;
	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;

	StartAbsMoveWithProfile(ax, 0, 10000, 100, 100);

	DWORD t0 = GetTickCount();
	while (true) {
		g_cm.GetStatus(&g_status);
		long long ap = (long long)g_status.axesStatus[ax].actualPos;
		int av = (int)std::lround(g_status.axesStatus[ax].actualVelocity);
		if (std::llabs(ap - 0) <= 5 && std::abs(av) <= 5) break;
		if (GetTickCount() - t0 > 30000) break;
		Sleep(10);
	}
}

// 상하축: axis2, Load 높이(60000)
static void DoOhtAction_Load()
{
	if (!g_commStarted) return;
	int ax = 2;
	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;

	StartAbsMoveWithProfile(ax, 51600, 5000, 100, 100);

	DWORD t0 = GetTickCount();
	while (true) {
		g_cm.GetStatus(&g_status);
		long long ap = (long long)g_status.axesStatus[ax].actualPos;
		int av = (int)std::lround(g_status.axesStatus[ax].actualVelocity);
		if (std::llabs(ap - 51600) <= 10 && std::abs(av) <= vel_idle_threshold) break;
		if (GetTickCount() - t0 > 20000) break;
		Sleep(10);
	}
}

// 상하축: axis2, Unload 높이(57000)
static void DoOhtAction_Unload()
{
	if (!g_commStarted) return;
	int ax = 2;
	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;

	StartAbsMoveWithProfile(ax, 57000, 10000, 100, 100);

	DWORD t0 = GetTickCount();
	while (true) {
		g_cm.GetStatus(&g_status);
		long long ap = (long long)g_status.axesStatus[ax].actualPos;
		int av = (int)std::lround(g_status.axesStatus[ax].actualVelocity);
		if (std::llabs(ap - 57000) <= 10 && std::abs(av) <= vel_idle_threshold) break;
		if (GetTickCount() - t0 > 20000) break;
		Sleep(10);
	}
}

// 상하축: axis2, Up 위치(0)
static void DoOhtAction_Up()
{
	if (!g_commStarted) return;
	int ax = 2;
	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;

	StartAbsMoveWithProfile(ax, 0, 10000, 100, 100);

	DWORD t0 = GetTickCount();
	while (true) {
		g_cm.GetStatus(&g_status);
		long long ap = (long long)g_status.axesStatus[ax].actualPos;
		int av = (int)std::lround(g_status.axesStatus[ax].actualVelocity);
		if (std::llabs(ap - 0) <= 10 && std::abs(av) <= vel_idle_threshold) break;
		if (GetTickCount() - t0 > 20000) break;
		Sleep(10);
	}
}

// 그립축: axis3, Open(0)
static void GripOpen()
{
	if (!g_commStarted) return;
	int ax = 3;
	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;

	StartAbsMoveWithProfile(ax, 0, 10000, 100, 100);

	DWORD t0 = GetTickCount();
	while (true) {
		g_cm.GetStatus(&g_status);
		long long ap = (long long)g_status.axesStatus[ax].actualPos;
		int av = (int)std::lround(g_status.axesStatus[ax].actualVelocity);
		if (std::llabs(ap - 0) <= 5 && std::abs(av) <= 5) break;
		if (GetTickCount() - t0 > 20000) break;
		Sleep(10);
	}
}

// 그립축: axis3, Close(10000)
static void GripClose()
{
	if (!g_commStarted) return;
	int ax = 3;
	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;

	StartAbsMoveWithProfile(ax, 10000, 10000, 100, 100);

	DWORD t0 = GetTickCount();
	while (true) {
		g_cm.GetStatus(&g_status);
		long long ap = (long long)g_status.axesStatus[ax].actualPos;
		int av = (int)std::lround(g_status.axesStatus[ax].actualVelocity);
		if (std::llabs(ap - 10000) <= 5 && std::abs(av) <= 5) break;
		if (GetTickCount() - t0 > 20000) break;
		Sleep(10);
	}
}

static void DoOhtAction_EStopAll() {
	// 예시: 전체 급정지
	for (int a = 0; a < 4; ++a) StopAxis(a);
}

// Stop 라치 동안 DO11 블링크 스레드 중복 실행 방지용
std::atomic<bool> g_do11BlinkRunning{ false };

// Stop 라치가 ON인 동안만 DO11을 깜빡이게 함
inline void StartDO11Blink_UntilReset()
{
	bool expected = false;
	if (!g_do11BlinkRunning.compare_exchange_strong(expected, true,
		std::memory_order_acq_rel, std::memory_order_relaxed))
	{
		// 이미 블링크 스레드가 돌고 있음
		return;
	}

	std::thread([]() {
		using namespace std::chrono_literals;

		bool on = false;

		// Stop29 latch가 ON인 동안만 계속 토글
		while (g_stop29Latched.load(std::memory_order_acquire)) {
			on = !on;
			ToggleDO_HW(11, on, nullptr);      // ★ ON/OFF 번갈아
			std::this_thread::sleep_for(250ms); // 주기(원하는대로 200~500ms 추천)
		}

		// Reset 들어와 latch OFF되면 DO11은 반드시 OFF로 정리
		ToggleDO_HW(11, false, nullptr);

		g_do11BlinkRunning.store(false, std::memory_order_release);
		}).detach();
}

// Periodic thread proc [NEW]
void PeriodicThreadProc()
{
	while (g_periodicRun.load()) {
		if (g_clientSock != INVALID_SOCKET) {
			SendPeriodicStateFrame(g_clientSock);
		}
		for (int i = 0; i < 10 && g_periodicRun.load(); ++i) Sleep(100); // 1초
	}
}

void PostTcpStateToMain(const wchar_t* msg)
{
	size_t len = wcslen(msg);
	wchar_t* dup = (wchar_t*)malloc((len + 1) * sizeof(wchar_t));
	if (!dup) return;
	wcscpy_s(dup, len + 1, msg);
	PostMessage(g_hMainWnd ? g_hMainWnd : GetDesktopWindow(), WM_APP_TCP_STATE, 0, (LPARAM)dup);
}

static void CloseClient()
{
	if (g_clientSock != INVALID_SOCKET) {
		closesocket(g_clientSock);
		g_clientSock = INVALID_SOCKET;
	}
}
static void CloseListen()
{
	if (g_listenSock != INVALID_SOCKET) {
		closesocket(g_listenSock);
		g_listenSock = INVALID_SOCKET;
	}
}

// 보조 대기 함수들 (질문 본문과 동일) — WaitUntil, WaitTaskFinished, WaitAllAxesStopped 등
extern bool WaitUntil(bool (*pred)(), DWORD timeoutMs, DWORD pollMs);
extern bool WaitAllAxesStopped(double velEps, DWORD timeoutMs);
extern bool WaitTaskFinished(TaskId id, DWORD timeoutMs, DWORD pollMs);

std::atomic<bool> g_motionBusy{ false };

// DONE(0x21, 0x22)에 대한 ACK 수신 여부
std::atomic<bool> g_ackLoadDone{ false };
std::atomic<bool> g_ackUnloadDone{ false };

// 0x2C 또는 0x2F 수신 시 공통으로 호출
inline void BeginGripOpReportGate()
{
	// Stop 이후 hold 상태에서만 의미가 있음 (그 외에도 안전하게 작동)
	g_gripHoldCodeZero.store(false, std::memory_order_relaxed); // 동작 중 0x00 유지
	g_gripAwaitDone.store(false, std::memory_order_relaxed); // 완료 기다림
	g_gripSawBusy.store(false, std::memory_order_relaxed); // 새 동작 시작
	g_gripBusyStartTick.store(0, std::memory_order_relaxed); // stuck 타이머 리셋
}


void TcpServerThreadProc()
{
	WSADATA wsa{};
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		AppendLog(L"[TCP] WSAStartup failed");
		PostTcpStateToMain(L"TCP: STOPPED");
		g_tcpRunning = false;
		return;
	}
	AppendLog(L"[TCP] WSAStartup OK");

	g_listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (g_listenSock == INVALID_SOCKET) {
		AppendLog(L"[TCP] socket() failed");
		WSACleanup();
		PostTcpStateToMain(L"TCP: STOPPED");
		g_tcpRunning = false;
		return;
	}

	int opt = 1;
	setsockopt(g_listenSock, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons((u_short)g_tcpBindPort);

	char ipA[64];
	WideCharToMultiByte(CP_ACP, 0, g_tcpBindIp, -1, ipA, sizeof(ipA), 0, 0);

	if (inet_pton(AF_INET, ipA, &addr.sin_addr) != 1) {
		AppendLog(L"[TCP] inet_pton() failed. Using default 0.0.0.0");
		inet_pton(AF_INET, "0.0.0.0", &addr.sin_addr);
	}

	if (bind(g_listenSock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
		AppendLog(L"[TCP] bind() failed (port in use?)");
		CloseListen();
		WSACleanup();
		PostTcpStateToMain(L"TCP: STOPPED");
		g_tcpRunning = false;
		return;
	}

	if (listen(g_listenSock, 1) == SOCKET_ERROR) {
		AppendLog(L"[TCP] listen() failed");
		CloseListen();
		WSACleanup();
		PostTcpStateToMain(L"TCP: STOPPED");
		g_tcpRunning = false;
		return;
	}

	{
		wchar_t msg[128];
		swprintf_s(msg, L"TCP: LISTEN %s:%d", g_tcpBindIp, g_tcpBindPort);
		PostTcpStateToMain(msg);
		swprintf_s(msg, L"[TCP] Listening on %s:%d", g_tcpBindIp, g_tcpBindPort);
		AppendLog(msg);
	}

	while (g_tcpRunning.load()) {
		sockaddr_in cli{};
		int clen = sizeof(cli);
		AppendLog(L"[TCP] Waiting for client...");
		g_clientSock = accept(g_listenSock, (sockaddr*)&cli, &clen);
		if (g_clientSock == INVALID_SOCKET) {
			if (!g_tcpRunning.load()) break;
			AppendLog(L"[TCP] accept() failed");
			continue;
		}
		AppendLog(L"[TCP] Client connected");
		PostTcpStateToMain(L"TCP: CLIENT CONNECTED");

		// 연결되자마자 주기프레임 1번 즉시 전송
		g_heartbeat = 0;
		g_driveReady.store(false, std::memory_order_relaxed); // ★ 운전준비 플
		SendPeriodicStateFrame(g_clientSock);

		// Periodic thread start
		g_periodicRun = true;
		g_heartbeat = 0;

		{
			SOCKET sockPeriodic = g_clientSock; // ★ 이 시점의 소켓을 캡쳐해서 사용 (안전)
			std::thread([sockPeriodic]() mutable {

				auto UpdateDriveReadyIfOk = []() {
					if (g_motionBusy.load(std::memory_order_relaxed)) return;
					// 이미 true면 더 볼 필요 없음
					if (g_driveReady.load(std::memory_order_relaxed)) return;

					unsigned char gripCode = CalcPosGripCode();
					unsigned char hoistCode = CalcPosHoistCode();

					// 조건 만족 시 driveReady ON
					if (gripCode != 0x00 && hoistCode == 0x03) {
						g_driveReady.store(true, std::memory_order_relaxed);
						AppendLog(L"[INFO] driveReady auto set -> 1 (grip!=0 && hoist==0x03)");
					}
					};

				while (g_periodicRun.load(std::memory_order_relaxed) && g_tcpRunning.load()) {
					if (sockPeriodic == INVALID_SOCKET) break;

					// 1) 상태 기반 driveReady 자동 갱신
					UpdateDriveReadyIfOk();

					// 2) 주기 프레임 전송
					SendPeriodicStateFrame(sockPeriodic);

					// 3) 주기(1초)
					::Sleep(1000);
				}

				}).detach();
		}
		// =========================================================

		char rbuf[512];
		std::string line;

		auto HandleCreateDevice = [&]() {
			AppendLog(L"[BIN] CreateDevice (legacy) ignored in new protocol");
			};

		// 이벤트 프레임 처리
		auto HandleEventFrame = [&](const unsigned char* f, int len) -> bool {
			if (len < 6) return false;
			if (f[0] != 0x02 || f[len - 1] != 0x03) return false;

			unsigned char op = f[3];
			unsigned char payLen = f[4];
			int expected = 5 + payLen + 1;
			if (expected != len) return false;

			// 수신 프레임 로그
			LogFrameHuman(f, len, L"[RX]");

			bool ok = false;

			switch (op) {
				// ---------------- LOAD ----------------
			case 0x21: // LOAD Request
			{
				AppendLog(L"[INFO] PLC -> PC : Load Request");


				if (g_stop29Latched.load(std::memory_order_acquire)) {
					AppendLog(L"[BLOCK] 0x21 ignored: Stop29 latch is ON (wait Reset 0xFD)");
					SendSimpleAck(g_clientSock, 0x21);
					return true;
				}

				// 이미 다른 모션/시퀀스 동작 중이면 거부
				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Load command ignored: motion already in progress");
					return false;
				}

				// ACK 먼저
				SendSimpleAck(g_clientSock, 0x21);
				AppendLog(L"[TX] Load Request ACK sent");

				SOCKET sockLoad = g_clientSock;
				std::thread([sockLoad]() {
					AppendLog(L"[ACT] Load Action Start");
					ToggleDO_HW(11, true, nullptr);
					StartDemoLoad(); // TaskId::DemoLoad 를 Running으로 세팅

					bool okLoad = WaitTaskFinished(TaskId::DemoLoad, 120000);
					if (!okLoad) {
						AppendLog(L"[WARN] Load sequence timeout or failed");
						ToggleDO_HW(11, false, nullptr);
						g_driveReady.store(false, std::memory_order_relaxed); // ★ NEW: 실패하면 0으로 리셋
						g_motionBusy.store(false);
						return;
						// 실패시 Done은 보내지 않음 (현재 정책)
					}
					else {
						ToggleDO_HW(11, false, nullptr);
						AppendLog(L"[ACT] Load action complete (OK)");
						// ACK 플래그 초기화
						g_ackLoadDone.store(false, std::memory_order_relaxed);
						SendSimpleDone(sockLoad, 0x21);
						AppendLog(L"[TX] Load Done sent");

						DWORD lastSend = GetTickCount();

						while (g_tcpRunning.load()) {
							// PLC에서 ACK(0xFF, payload=0x21)를 받으면 g_ackLoadDone = true
							if (g_ackLoadDone.load(std::memory_order_relaxed)) {
								AppendLog(L"[INFO] Load Done ACK received from PLC");
								break;
							}

							DWORD now = GetTickCount();
							if (now - lastSend >= 2000) {
								// 2초 동안 ACK를 못 받으면 Done 다시 전송
								AppendLog(L"[WARN] Load Done ACK not received, resending...");
								SendSimpleDone(sockLoad, 0x21);
								AppendLog(L"[TX] Load Done re-sent");
								lastSend = now;
								// g_ackLoadDone 는 ACK 올 때만 true로 바뀜
							}

							::Sleep(50);
						}
					}

					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- UNLOAD ----------------
			case 0x22: // UNLOAD Request
			{
				AppendLog(L"[INFO] PLC -> PC : Unload Request");

				if (g_stop29Latched.load(std::memory_order_acquire)) {
					AppendLog(L"[BLOCK] 0x22 ignored: Stop29 latch is ON (wait Reset 0xFD)");
					SendSimpleAck(g_clientSock, 0x22);
					return true;
				}

				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Unload command ignored: motion already in progress");
					return false;
				}

				SendSimpleAck(g_clientSock, 0x22);
				AppendLog(L"[TX] Unload Request ACK sent");

				SOCKET sockUnload = g_clientSock;
				std::thread([sockUnload]() {
					AppendLog(L"[ACT] Unload Action Start");
					ToggleDO_HW(11, true, nullptr);
					StartDemoUnload();

					bool okUnload = WaitTaskFinished(TaskId::DemoUnload, 60000);
					if (!okUnload) {
						AppendLog(L"[WARN] Unload sequence timeout or failed");
						ToggleDO_HW(11, false, nullptr);
						g_driveReady.store(false, std::memory_order_relaxed); // ★ NEW: 실패하면 0으로 리셋
						g_motionBusy.store(false);
						return;
					}
					else {
						ToggleDO_HW(11, false, nullptr);
						AppendLog(L"[ACT] Unload action complete (OK)");
						g_ackUnloadDone.store(false, std::memory_order_relaxed);
						SendSimpleDone(sockUnload, 0x22);
						AppendLog(L"[TX] Unload Done sent");

						DWORD lastSend = GetTickCount();

						while (g_tcpRunning.load()) {
							if (g_ackUnloadDone.load(std::memory_order_relaxed)) {
								AppendLog(L"[INFO] Unload Done ACK received from PLC");
								break;
							}

							DWORD now = GetTickCount();
							if (now - lastSend >= 2000) {
								AppendLog(L"[WARN] Unload Done ACK not received, resending...");
								SendSimpleDone(sockUnload, 0x22);
								AppendLog(L"[TX] Unload Done re-sent");
								lastSend = now;
							}

							::Sleep(50);
						}
					}

					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- STOP (즉시 처리) ----------------
			case 0x29: // 축정지
			{
				AppendLog(L"[INFO] PLC -> PC : Stop Motion Request");
				SendSimpleAck(g_clientSock, 0x29);
				AppendLog(L"[TX] Stop Request ACK sent");

				g_driveReady.store(false, std::memory_order_relaxed);

				// ★ Stop 라치 ON (Reset 전까지 명령 차단)
				g_stop29Latched.store(true, std::memory_order_release);
				AppendLog(L"[STATE] Stop29 latch ON: block motion/grip commands until Reset(0xFD)");

				// ★ NEW: DO11 블링크 시작 (Reset 전까지)
				StartDO11Blink_UntilReset();
				AppendLog(L"[ACT] DO11 blinking until Reset(0xFD)");

				// ★ 추가: Stop 들어오면 GripCode를 0x00으로 강제
				//g_forceGripCodeZero.store(true, std::memory_order_relaxed);
				g_gripHoldCodeZero.store(true, std::memory_order_relaxed);
				g_gripAwaitDone.store(false, std::memory_order_relaxed);
				g_gripSawBusy.store(false, std::memory_order_relaxed);
				g_gripBusyStartTick.store(0, std::memory_order_relaxed); // 선택: stuck 타이머 리셋

				AppendLog(L"[ACT] GripCode HOLD=0x00 (Stop)");

				//DoStopAll(nullptr);
				g_cm.ExecEStop(EStopLevel::Final);
				
				g_servo12Ready.store(false, std::memory_order_release);
				AppendLog(L"[STATE] ServoGate CLOSED (Stop): commands will be ignored until Reset+ServoOn confirmed");


				AppendLog(L"[ACT] All axes QuickStop executed");

				if (WaitAllAxesStopped(1.0, 10000)) {
					AppendLog(L"[INFO] All axes stopped (vel <= 1.0, Motioning OFF)");

					// ★ 추가: axis1, axis2 Servo OFF
					for (int a : { 1, 2 }) {
						long e = g_cm.axisControl->SetServoOn(a, 0);
						if (e != ErrorCode::None) {
							wchar_t msg[128];
							swprintf_s(msg, L"[WARN] Axis%d Servo OFF FAILED (err=%ld)", a, e);
							AppendLog(msg);
						}
						else {
							wchar_t msg[128];
							swprintf_s(msg, L"[ACT] Axis%d Servo OFF OK", a);
							AppendLog(msg);
						}
					}
					//ToggleDO_HW(11, false, nullptr);
					SendSimpleDone(g_clientSock, 0x29);
					AppendLog(L"[TX] Stop Done sent");
					return true;
				}
				else {
					AppendLog(L"[WARN] Stop timeout : some axes still moving or Motioning still ON");
					return false;
				}

			}
				

				// ---------------- RESET ----------------
			case 0xFD: // 리셋
			{
				ToggleDO_HW(11, false, nullptr);
				AppendLog(L"[INFO] PLC -> PC : Reset Request");

				g_cm.ReleaseEStop();

				// ★ Reset 시작: 일단 명령 게이트 닫기
				g_servo12Ready.store(false, std::memory_order_release);
				AppendLog(L"[STATE] ServoGate CLOSED (Reset): wait Axis1&2 Servo ON confirm");


				// ★ Reset 들어오면 Stop 라치 OFF (다시 명령 허용)
				g_stop29Latched.store(false, std::memory_order_release);
				AppendLog(L"[STATE] Stop29 latch OFF: commands enabled");

				for (int a = 0; a < 4; ++a) {
					g_cm.axisControl->ClearAmpAlarm(a);
				}
				SendSimpleAck(g_clientSock, 0xFD);
				AppendLog(L"[TX] Reset ACK sent");

				// 운전 준비 플래그도 리셋
				g_driveReady.store(false, std::memory_order_relaxed);
				g_demoAlarm.store(0x00, std::memory_order_relaxed); // ★ NEW

				// ★ Reset 들어오면: Stop으로 강제하던 Grip 0x00 잠금 해제 → 자동판정 복귀
				//g_forceGripCodeZero.store(false, std::memory_order_relaxed);
				g_gripHoldCodeZero.store(true, std::memory_order_relaxed);
				g_gripAwaitDone.store(false, std::memory_order_relaxed);
				g_gripSawBusy.store(false, std::memory_order_relaxed);
				g_gripBusyStartTick.store(0, std::memory_order_relaxed);

				AppendLog(L"[ACT] GripCode auto-report re-enabled (force/hold cleared) due to Reset");

				// 현재 Servo 상태 확인
				bool s1 = IsAxisServoOn(1);
				bool s2 = IsAxisServoOn(2);

				// ★ 둘 다 OFF일 때만: Axis1 ON -> 5초 대기 -> Axis2 ON
				if (!s1 && !s2) {
					AppendLog(L"[RESET] Axis1&2 ServoOff -> ServoOn Axis1, wait 5s, then ServoOn Axis2");

					EnsureServoOn(1);
					EnsurePosModeNoStop(1);

					std::this_thread::sleep_for(std::chrono::seconds(5));

					EnsureServoOn(2);
					EnsurePosModeNoStop(2);
				}
				else {
					// 그 외에는 OFF된 축만 켜고, 5초 대기는 하지 않음
					if (!s1) {
						AppendLog(L"[RESET] Axis1 ServoOff -> ServoOn Axis1 (no 5s wait)");
						EnsureServoOn(1);
						EnsurePosModeNoStop(1);
					}
					else {
						AppendLog(L"[RESET] Axis1 already ServoOn -> skip");
					}

					if (!s2) {
						AppendLog(L"[RESET] Axis2 ServoOff -> ServoOn Axis2 (no 5s wait)");
						EnsureServoOn(2);
						EnsurePosModeNoStop(2);
					}
					else {
						AppendLog(L"[RESET] Axis2 already ServoOn -> skip");
					}
				}

				// ★ 마지막: Servo ON 확인 모니터 시작(둘 다 ON 될 때까지 계속 확인)
				StartServo12ReadyMonitor();

				return true;
			}
				// ---------------- COMM OPEN ----------------
			case 0xF0: // Comm Open
				if (payLen >= 1 && f[5] == 0x01) {
					AppendLog(L"[INFO] PLC -> PC : Comm Open (통신 오픈)");
					PostTcpStateToMain(L"TCP: COMM OPEN");
				}
				else {
					AppendLog(L"[WARN] Comm Open frame payload unexpected");
				}
				return true;

				// ---------------- TRAVEL (Axis0) ----------------
			case 0x2A: // 주행 포지션 기동
			{
				if (BlockIfServoNotReady(0x2A)) return true;

				if (g_stop29Latched.load(std::memory_order_acquire)) {
					AppendLog(L"[BLOCK] 0x2A ignored: Stop29 latch is ON (wait Reset 0xFD)");
					SendSimpleAck(g_clientSock, 0x2A);
					return true;
				}

				// 인터락 체크
				if (!CheckInterlockBeforeAxisCommand(0)) {
					AppendLog(L"[INTERLOCK] Axis0 blocked: Axis2 limit must be ON in Auto");
					LatchDemoAlarm(DEMO_ALM_TRAVEL_INTERLOCK_FAIL);  // ★ NEW
					return false;
				}

				// ★ NEW: Axis1 ServoOn 확인 (아니면 알람만 띄우고 끝)
				if (!IsAxisServoOn(1)) {
					AppendLog(L"[ALM] Travel blocked: Axis1 ServoOff -> Please ServoOn Axis1");
					LatchDemoAlarm(DEMO_ALM_TRAVEL_SERVO1_OFF);

					// PLC가 "수신"은 알게 ACK는 보내는 걸 추천
					SendSimpleAck(g_clientSock, 0x2A);
					AppendLog(L"[TX] Travel Position ACK sent (but rejected by ServoOff)");
					return true; // 프레임 처리 완료(동작은 안 함)
				}

				unsigned char posNo = (payLen >= 1) ? f[5] : 0;
				wchar_t info[128];
				swprintf_s(info, L"[INFO] PLC -> PC : Travel Position Command, Pos=%u", (unsigned)posNo);
				AppendLog(info);

				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Travel command ignored: motion already in progress");
					LatchDemoAlarm(DEMO_ALM_TRAVEL_BUSY); // ★ NEW
					return false;
				}

				// ACK 전송
				SendSimpleAck(g_clientSock, 0x2A);
				AppendLog(L"[TX] Travel Position ACK sent");

				SOCKET sockTravel = g_clientSock;
				std::thread([sockTravel, posNo]() {
					bool okTravel = false;

					switch (posNo) {
					case 1:
						AppendLog(L"[ACT] Travel Pos1 -> Conveyor");
						ToggleDO_HW(11, true, nullptr);
						GO_Conveyor();
						okTravel = WaitTaskFinished(TaskId::GoConveyor, 30000);
						AppendLog(okTravel
							? L"[ACT] Travel Pos1 -> Conveyor DONE"
							: L"[ACT] Travel Pos1 -> Conveyor FAILED or TIMEOUT");
						if (!okTravel) LatchDemoAlarm(DEMO_ALM_TRAVEL_TO_CONVEYOR_TO); // ★ NEW
						ToggleDO_HW(11, false, nullptr);
						break;
					case 2:
						AppendLog(L"[ACT] Travel Pos2 -> Workstation");
						ToggleDO_HW(11, true, nullptr);
						Go_Workstation();
						okTravel = WaitTaskFinished(TaskId::GoWorkstation, 30000);
						AppendLog(okTravel
							? L"[ACT] Travel Pos2 -> Workstation DONE"
							: L"[ACT] Travel Pos2 -> Workstation FAILED or TIMEOUT");
						if (!okTravel) LatchDemoAlarm(DEMO_ALM_TRAVEL_TO_WORK_TO); // ★ NEW
						ToggleDO_HW(11, false, nullptr);
						break;
					case 3:
						ToggleDO_HW(11, false, nullptr);
						AppendLog(L"[WARN] Travel Pos3 not implemented");
						okTravel = false;
						break;
					default:
						ToggleDO_HW(11, false, nullptr);
						AppendLog(L"[WARN] Travel Position invalid PosNo");
						LatchDemoAlarm(DEMO_ALM_TRAVEL_INVALID_POS); // ★ NEW
						okTravel = false;
						break;
					}

					/*if (okTravel) {
						SendSimpleDone(sockTravel, 0x2A);
						AppendLog(L"[TX] Travel Pos Done sent");
					}*/

					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- HOIST (Axis2) ----------------
			case 0x2B: // 상하 포지션 기동
			{
				if (BlockIfServoNotReady(0x2B)) return true;

				if (g_stop29Latched.load(std::memory_order_acquire)) {
					AppendLog(L"[BLOCK] 0x2B ignored: Stop29 latch is ON (wait Reset 0xFD)");
					SendSimpleAck(g_clientSock, 0x2B);
					return true;
				}

				if (!CheckInterlockBeforeAxisCommand(2)) {
					AppendLog(L"[INTERLOCK] Axis2 blocked: Travel must be at Load/Unload in Auto");
					LatchDemoAlarm(DEMO_ALM_HOIST_INTERLOCK_FAIL); // ★ NEW
					return false;
				}

				// ★ NEW: Axis2 ServoOn 확인 (아니면 알람만 띄우고 끝)
				if (!IsAxisServoOn(2)) {
					AppendLog(L"[ALM] Hoist blocked: Axis2 ServoOff -> Please ServoOn Axis2");
					LatchDemoAlarm(DEMO_ALM_HOIST_SERVO2_OFF);

					SendSimpleAck(g_clientSock, 0x2B);
					AppendLog(L"[TX] Hoist Position ACK sent (but rejected by ServoOff)");
					return true;
				}

				unsigned char posNo = (payLen >= 1) ? f[5] : 0;
				wchar_t info[128];
				swprintf_s(info, L"[INFO] PLC -> PC : Hoist Position Command, Pos=%u", (unsigned)posNo);
				AppendLog(info);

				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Hoist command ignored: motion already in progress");
					LatchDemoAlarm(DEMO_ALM_HOIST_BUSY); // ★ NEW
					return false;
				}

				SendSimpleAck(g_clientSock, 0x2B);
				AppendLog(L"[TX] Hoist Position ACK sent");

				SOCKET sockHoist = g_clientSock;
				std::thread([sockHoist, posNo]() {
					bool okHoist = false;

					switch (posNo) {
					case 1:
						if (!CanAxis2Move_Auto_RequireTravel(0x01)) {
							AppendLog(L"[INTERLOCK] Axis2 blocked: Travel must be at Load/Unload in Auto");
							LatchDemoAlarm(DEMO_ALM_HOIST_INTERLOCK_FAIL); // ★ NEW
							return false;
						}
						AppendLog(L"[ACT] Hoist Pos1 -> Conveyor Down");
						ToggleDO_HW(11, true, nullptr);
						ConveyorDown();
						okHoist = WaitTaskFinished(TaskId::ConveyorDown, 20000);
						AppendLog(okHoist
							? L"[ACT] Hoist Pos1 -> ConveyorDown DONE"
							: L"[ACT] Hoist Pos1 -> ConveyorDown FAILED or TIMEOUT");
						if (!okHoist) LatchDemoAlarm(DEMO_ALM_HOIST_CONVEYORDOWN_TO); // ★ NEW
						ToggleDO_HW(11, false, nullptr);
						break;

					case 2:
						if (!CanAxis2Move_Auto_RequireTravel(0x02)) {
							AppendLog(L"[INTERLOCK] Axis2 blocked: Travel must be at Load/Unload in Auto");
							LatchDemoAlarm(DEMO_ALM_HOIST_INTERLOCK_FAIL); // ★ NEW
							return false;
						}
						AppendLog(L"[ACT] Hoist Pos2 -> Work Down");
						ToggleDO_HW(11, true, nullptr);
						WorkDown();
						okHoist = WaitTaskFinished(TaskId::WorkDown, 20000);
						AppendLog(okHoist
							? L"[ACT] Hoist Pos2 -> WorkDown DONE"
							: L"[ACT] Hoist Pos2 -> WorkDown FAILED or TIMEOUT");
						if (!okHoist) LatchDemoAlarm(DEMO_ALM_HOIST_WORKDOWN_TO); // ★ NEW
						ToggleDO_HW(11, false, nullptr);
						break;

					case 3:
						AppendLog(L"[ACT] Hoist Pos3 -> Up Position");
						ToggleDO_HW(11, true, nullptr);
						DoUp();
						okHoist = WaitTaskFinished(TaskId::LiftUp, 20000);
						AppendLog(okHoist
							? L"[ACT] Hoist Pos3 -> Up DONE"
							: L"[ACT] Hoist Pos3 -> Up FAILED or TIMEOUT");
						if (!okHoist) LatchDemoAlarm(DEMO_ALM_HOIST_UP_TO); // ★ NEW
						ToggleDO_HW(11, false, nullptr);
						break;

					default:
						ToggleDO_HW(11, false, nullptr);
						AppendLog(L"[WARN] Hoist Position invalid PosNo");
						LatchDemoAlarm(DEMO_ALM_HOIST_INVALID_POS); // ★ NEW
						okHoist = false;
						break;
					}

					/*if (okHoist) {
						SendSimpleDone(sockHoist, 0x2B);
						AppendLog(L"[TX] Hoist Pos Done sent");
					}*/

					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- GRIP ----------------
			case 0x2C: // 그립 포지션 기동
			{
				if (BlockIfServoNotReady(0x2C)) return true;

				if (g_stop29Latched.load(std::memory_order_acquire)) {
					AppendLog(L"[BLOCK] 0x2C ignored: Stop29 latch is ON (wait Reset 0xFD)");
					SendSimpleAck(g_clientSock, 0x2C);   // PLC 대기 방지용
					return true;                         // “처리 완료(동작 없음)”로 간주
				}



				unsigned char posNo = (payLen >= 1) ? f[5] : 0;
				wchar_t info[128];
				swprintf_s(info, L"[INFO] PLC -> PC : Grip Position Command, Pos=%u", (unsigned)posNo);
				AppendLog(info);

				// 모션 Busy 체크 (그립도 모션으로 간주)
				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Grip command ignored: motion already in progress");
					return false;
				}

				// ACK 전송
				SendSimpleAck(g_clientSock, 0x2C);
				AppendLog(L"[TX] Grip Position ACK sent");

				SOCKET sockGrip = g_clientSock;
				std::thread([sockGrip, posNo]() {
					// 재진입 방지(그립 전용)
					if (g_gripBusy) {
						AppendLog(L"[BUSY] Grip command ignored: g_gripBusy == true");
						g_motionBusy.store(false);
						return;
					}

					bool okGrip = false;
					bool motioning = g_diStable[0]; // Motioning 상태 (필요시 수정)

					switch (posNo) {
					case 1: // Open
						if (HasBox()) {
							AppendLog(L"[INTERLOCK] Grip Open blocked: HasBox()==true");
							break;
						}
						if (IsGripperAlreadyOpen()) {
							AppendLog(L"[SKIP] Grip already OPEN. No action performed.");
							okGrip = true;
							break;
						}
						BeginGripOpReportGate();
						AppendLog(L"[ACT] Grip Pos1 -> GripOpen");
						g_gripBusy = true;

						DoGripServoOff_Compat(nullptr); // 서보 오프
						Sleep(100);
						ToggleDO_HW(11, motioning, nullptr);
						DoOpen_Compat(nullptr);
						okGrip = WaitUntil(IsGripperOpenAndIdle, 10000);
						ToggleDO_HW(11, motioning, nullptr);

						g_gripBusy = false;
						AppendLog(okGrip
							? L"[ACT] Grip Pos1 -> Open DONE"
							: L"[ACT] Grip Pos1 -> Open FAILED or TIMEOUT");
						break;

					case 2: // Close
						if (IsGripperAlreadyClosed()) {
							AppendLog(L"[SKIP] Grip already CLOSED. No action performed.");
							okGrip = false;
							break;
						}
						BeginGripOpReportGate();
						AppendLog(L"[ACT] Grip Pos2 -> GripClose");
						g_gripBusy = true;

						DoGripServoOff_Compat(nullptr); // 서보 오프
						Sleep(100);
						ToggleDO_HW(11, motioning, nullptr);
						DoClose_Compat(nullptr);
						okGrip = WaitUntil(IsGripperClosedAndIdle, 10000);
						ToggleDO_HW(11, motioning, nullptr);
						g_gripBusy = false;

						AppendLog(okGrip
							? L"[ACT] Grip Pos2 -> Close DONE"
							: L"[ACT] Grip Pos2 -> Close FAILED or TIMEOUT");
						break;

					case 3:
						AppendLog(L"[WARN] Grip Pos3 not implemented");
						okGrip = false;
						break;

					default:
						AppendLog(L"[WARN] Grip Position invalid PosNo");
						okGrip = false;
						break;
					}

					// 현재 프로토콜에서는 Grip에 대해 Done 프레임은 보내지 않고 ACK만 있는 상태 유지
					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- 운전 준비 요구 (Drive Ready) ----------------
			case 0x2F:
			{
				AppendLog(L"[INFO] PLC -> PC : Drive Ready Request (0x2F)");

				if (BlockIfServoNotReady(0x2F)) return true;

				if (g_stop29Latched.load(std::memory_order_acquire)) {
					AppendLog(L"[BLOCK] 0x2F ignored: Stop29 latch is ON (wait Reset 0xFD)");
					SendSimpleAck(g_clientSock, 0x2F);
					return true;
				}

				// 다른 모션 중이면 거부
				bool expectedBusy = false;
				if (!g_motionBusy.compare_exchange_strong(expectedBusy, true)) {
					AppendLog(L"[BUSY] Drive Ready command ignored: motion already in progress");
					return false;
				}

				// ACK 먼저 전송 (02 00 00 FF 01 2F 03)
				SendSimpleAck(g_clientSock, 0x2F);
				AppendLog(L"[TX] Drive Ready ACK sent");

				SOCKET sockReady = g_clientSock;
				std::thread([sockReady]() {
					// 2) 그리퍼 상태 확인
					unsigned char gcode = CalcPosGripCode(); // 0x00이면 중간(애매한) 상태라고 가정
					bool okGrip = false;
					wchar_t info[128];
					swprintf_s(info, L"[ACT] DriveReady: Current GripCode=0x%02X", (unsigned)gcode);
					AppendLog(info);

					if (HasBox()) {
						BeginGripOpReportGate();
						// ★ NEW: 이미 Close면 스킵
						if (gcode == 0x02) {
							AppendLog(L"[SKIP] HasBox()==true and Grip already CLOSED (0x02) -> skip grip action");
							okGrip = true;
						}
						else {
							AppendLog(L"[AUTO] HasBox()==true -> Grip Close");
							DoClose_Compat(g_hDemoWnd);
							okGrip = WaitUntil(IsGripperClosedAndIdle, 5000);
							if (!okGrip) {
								AppendLog(L"[AUTO][WARN] Grip Close or Idle wait FAILED");
							}
							else {
								AppendLog(L"[AUTO] Grip Close DONE (Idle)");
							}
						}
					}
					
					else if (NoBox()) {		
						BeginGripOpReportGate();
						// ★ NEW: 이미 Open이면 스킵
						if (gcode == 0x01) {
							AppendLog(L"[SKIP] NoBox()==true and Grip already OPEN (0x01) -> skip grip action");
							okGrip = true;
						}
						else {
							AppendLog(L"[AUTO] NoBox()==true -> Grip Open");
							DoOpen_Compat(g_hDemoWnd);
							okGrip = WaitUntil(IsGripperOpenAndIdle, 5000);
							if (!okGrip) {
								AppendLog(L"[AUTO][WARN] Grip Open or Idle wait FAILED");
							}
							else {
								AppendLog(L"[AUTO] Grip Open DONE (Idle)");
							}
						}						
					}

					// 1) Axis2가 Limit(Up) 상태가 아니면 먼저 Up으로 정리
					unsigned char hcode = CalcPosHoistCode();
					swprintf_s(info, L"[ACT] DriveReady: Current HoistCode=0x%02X", (unsigned)hcode);
					AppendLog(info);

					if (hcode != 0x03) {
						AppendLog(L"[ACT] DriveReady: Hoist not UP(0x03) -> DoUp()");
						bool started = StartAbsMoveWithProfile(
							2,          // axis
							-1000,      // target position
							3000.0,     // velocity
							1000.0,     // tAcc (ms)
							1000.0      // tDec (ms)
						);

						bool okAxis2 = false;
						if (started) {
							okAxis2 = WaitUntil(IsAxis2LimitOn, 10000);
						}
						else {
							AppendLog(L"[AUTO][WARN] StartAbsMoveWithProfile(axis2) FAILED");
						}
					}
					else {
						AppendLog(L"[ACT] DriveReady: Hoist already UP(0x03)");
					}

					std::this_thread::sleep_for(std::chrono::seconds(1));

					// ===== 3) 최종 상태 재계산 후 DriveReady 결정 =====
					unsigned char gFinal = CalcPosGripCode();
					unsigned char hFinal = CalcPosHoistCode();

					// 운전 준비 완료 플래그 ON
					if (hFinal == 0x03 && gFinal != 0x00) {
						g_driveReady.store(true, std::memory_order_relaxed);
						AppendLog(L"[INFO] DriveReady sequence complete -> driveReady = 1");
						// ★ 요청사항: "모든 동작 완료 후" driveReady가 true가 되면 그때 DO11 OFF
						ToggleDO_HW(11, false, nullptr);
						AppendLog(L"[ACT] DO11 OFF (DriveReady==true)");
					}

					else {
						g_driveReady.store(false, std::memory_order_relaxed);
						AppendLog(L"[INFO] DriveReady NOT complete -> driveReady = 0");
					}

					g_motionBusy.store(false);
					}).detach();

				return true;
			}

			// ---------------- ACK ----------------
			case 0xFF: // ACK
				AppendLog(L"[INFO] PLC -> PC : ACK received");
				if (payLen >= 1) {
					unsigned char ackFor = f[5];
					wchar_t info[128];
					swprintf_s(info, L"[INFO] ACK for OP=0x%02X (%s)",
						ackFor, DecodeOpName(ackFor));
					AppendLog(info);
				}
				// ★ Load / Unload 두 쪽 다 깨워줌
				// (동시에 둘 다 돌지 않게 g_motionBusy로 막아놨으므로 안전)
				g_ackLoadDone.store(true, std::memory_order_relaxed);
				g_ackUnloadDone.store(true, std::memory_order_relaxed);
				return true;

				// ---------------- 기타 ----------------
			default:
			{
				wchar_t info[128];
				swprintf_s(info, L"[WARN] Unsupported OP=0x%02X (%s)", op, DecodeOpName(op));
				AppendLog(info);
			}
			return false;
			} // switch
			}; // HandleEventFrame

		// === RECV LOOP ===
		while (g_tcpRunning.load()) {
			int r = recv(g_clientSock, rbuf, sizeof(rbuf), 0);
			if (r <= 0) {
				AppendLog(L"[TCP] Client disconnected");
				PostTcpStateToMain(L"TCP: LISTENING");
				break;
			}

			// RAW 로그
			{
				unsigned char* p = (unsigned char*)rbuf;
				LogFrameHuman(p, r, L"[RAW]");
			}

			// 7바이트 구버전
			if (r == 7 && (unsigned char)rbuf[0] == 0x02 && (unsigned char)rbuf[6] == 0x03) {
				HandleEventFrame((unsigned char*)rbuf, r);
				continue;
			}

			// 새 이벤트 프레임
			if (r >= 6 && (unsigned char)rbuf[0] == 0x02 && (unsigned char)rbuf[r - 1] == 0x03) {
				HandleEventFrame((unsigned char*)rbuf, r);
				continue;
			}

			// 텍스트 라인 파싱
			line.append(rbuf, r);
			size_t pos;
			while ((pos = line.find_first_of("\r\n")) != std::string::npos) {
				std::string one = line.substr(0, pos);
				size_t next = pos + 1;
				while (next < line.size() && (line[next] == '\r' || line[next] == '\n')) ++next;
				line.erase(0, next);

				auto trimA = [](std::string s) {
					size_t i = 0, j = s.size();
					while (i < j && (s[i] == ' ' || s[i] == '\t')) ++i;
					while (j > i && (s[j - 1] == ' ' || s[j - 1] == '\t')) --j;
					return s.substr(i, j - i);
					};
				one = trimA(one);
				if (one.empty()) continue;

				wchar_t wline[256];
				MultiByteToWideChar(CP_ACP, 0, one.c_str(), -1, wline, 256);
				wchar_t wmsg[300];
				swprintf_s(wmsg, L"[RX ASCII] %s", wline);
				AppendLog(wmsg);

				if (one == "0") {
					HandleCreateDevice();
				}
				else if (one == "1") {
					if (!g_deviceOpened) AppendLog(L"[RX] : Device not created");
					else if (g_commStarted) AppendLog(L"[RX] : Communication already started");
					else {
						bool okComm = StartComm();
						AppendLog(okComm ? L"[RX] : Communication Success" : L"[RX] : Communication Failed");
					}
				}
			}
		}

		// 클라이언트 종료 처리
		g_periodicRun = false;
		CloseClient();
	}

	CloseListen();
	WSACleanup();
	AppendLog(L"[TCP] Server thread exit");
	PostTcpStateToMain(L"TCP: STOPPED");
	g_tcpRunning = false;
}


static void StartTcpServer()
{
	if (g_tcpRunning.load()) return;
	g_tcpRunning = true;
	g_tcpThread = std::thread(TcpServerThreadProc);
	g_tcpThread.detach();
}
static void StopTcpServer()
{
	if (!g_tcpRunning.load()) return;
	g_tcpRunning = false;
	g_periodicRun = false;
	shutdown(g_listenSock, SD_BOTH);
	shutdown(g_clientSock, SD_BOTH);
	CloseClient();
	CloseListen();
}

// ------------------ 수동 이동/체크박스 등 ------------------
static bool IsAxisChecked(HWND hWnd, int axis) {
	int id = (axis == 0) ? ID_CHECK_AXIS_0 : (axis == 1) ? ID_CHECK_AXIS_1 : (axis == 2) ? ID_CHECK_AXIS_2 : ID_CHECK_AXIS_3;
	HWND hb = GetDlgItem(hWnd, id);
	if (!hb) return false;
	return SendMessage(hb, BM_GETCHECK, 0, 0) == BST_CHECKED;
}
static void SetAxisChecked(HWND hWnd, int axis, bool checked) {
	int id = (axis == 0) ? ID_CHECK_AXIS_0 : (axis == 1) ? ID_CHECK_AXIS_1 : (axis == 2) ? ID_CHECK_AXIS_2 : ID_CHECK_AXIS_3;
	HWND hb = GetDlgItem(hWnd, id);
	if (hb) SendMessage(hb, BM_SETCHECK, checked ? BST_CHECKED : BST_UNCHECKED, 0);
}
static void UpdateSelectedAxesTextOnDemand(HWND hWnd) {
	bool sel[4] = {};
	for (int a = 0; a < 4; ++a) sel[a] = IsAxisChecked(hWnd, a);
	TCHAR text[256] = TEXT("Selected: "); bool any = false;
	for (int a = 0; a < 4; ++a) if (sel[a]) { TCHAR t[16]; _stprintf_s(t, TEXT("%s%d"), any ? TEXT(", ") : TEXT(""), a); _tcscat_s(text, t); any = true; }
	if (!any) _tcscat_s(text, TEXT("(none)"));
	SetWindowText(GetDlgItem(hWnd, ID_TXT_SELECTED_AXES), text);
}
static void DoAbsMoveAxis(HWND hWnd, int axis) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return; }

	if (axis == 2) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[2].actualPos;
		double tgtD = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0);
		long long tgt = (long long)std::llround(tgtD);
		if (Axis2IsMinusCommandBlocked(cur, tgt, 0)) {
			Axis2ShowMinusBlockedWarning(hWnd);
			return;
		}
	}

	if (axis == 0) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[0].actualPos;
		double tgtD = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0);
		long long tgt = (long long)std::llround(tgtD);
		long long delta = tgt - cur;
		long long sign = (delta > 0) ? +1 : (delta < 0 ? -1 : 0);
		if (sign != 0 && Axis0IsCommandBlocked(cur, tgt, sign)) {
			Axis0ShowBlockedWarning(hWnd, (int)sign);
			return;
		}
	}

	//DisableAllEnabledSyncGroups();
	if (!EnsureServoOn(axis) || !EnsurePosModeNoStop(axis)) return;
	double tgt = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0);
	double v = GetDlgDouble(hWnd, ID_EDIT_VEL_A(axis), 10000.0);
	double ta = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(axis), 100.0);
	double td = GetDlgDouble(hWnd, ID_EDIT_DECT_A(axis), 100.0);
	StartAbsMoveWithProfile(axis, (long long)std::llround(tgt), v, ta, td);
}
static void DoRelMoveAxis(HWND hWnd, int axis, int dir) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return; }

	// Axis2 보호: 상대 이동 전 차단 검사
	if (axis == 2) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[2].actualPos;
		double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0) * dir;
		long long tgt = cur + (long long)std::llround(step);
		if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)dir)) {
			Axis2ShowMinusBlockedWarning(hWnd);
			return;
		}
	}

	// Axis0 보호: 상대 이동 전 L/R 리밋 방향 차단
	if (axis == 0) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[0].actualPos;
		double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0) * dir;
		long long tgt = cur + (long long)std::llround(step);
		if (Axis0IsCommandBlocked(cur, tgt, (long long)dir)) {
			Axis0ShowBlockedWarning(hWnd, dir);
			return;
		}
	}

	//DisableAllEnabledSyncGroups();
	if (!EnsureServoOn(axis) || !EnsurePosModeNoStop(axis)) return;
	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[axis].actualPos;
	double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(axis), 0.0) * dir;
	double v = GetDlgDouble(hWnd, ID_EDIT_VEL_A(axis), 10000.0);
	double ta = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(axis), 100.0);
	double td = GetDlgDouble(hWnd, ID_EDIT_DECT_A(axis), 100.0);
	StartAbsMoveWithProfile(axis, cur + (long long)std::llround(step), v, ta, td);
}
static void DoMultiAbs(HWND hWnd) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return; }
	//DisableAllEnabledSyncGroups();
	for (int a = 0; a < 4; ++a) if (IsAxisChecked(hWnd, a) && EnsureServoOn(a) && EnsurePosModeNoStop(a)) {

		// Axis2 보호
		if (a == 2) {
			g_cm.GetStatus(&g_status);
			long long cur = (long long)g_status.axesStatus[2].actualPos;
			long long tgt = (long long)std::llround(GetDlgDouble(hWnd, ID_EDIT_POS_A(2), 0.0));
			if (Axis2IsMinusCommandBlocked(cur, tgt, 0)) {
				Axis2ShowMinusBlockedWarning(hWnd);
				continue;
			}
		}

		// Axis0 보호: L/R 리밋에 따른 방향 금지
		if (a == 0) {
			g_cm.GetStatus(&g_status);
			long long cur = (long long)g_status.axesStatus[0].actualPos;
			long long tgt = (long long)std::llround(GetDlgDouble(hWnd, ID_EDIT_POS_A(0), 0.0));
			long long delta = tgt - cur;
			long long sign = (delta > 0) ? +1 : (delta < 0 ? -1 : 0);
			if (sign != 0 && Axis0IsCommandBlocked(cur, tgt, sign)) {
				Axis0ShowBlockedWarning(hWnd, (int)sign);
				continue;
			}
		}

		double tgt = GetDlgDouble(hWnd, ID_EDIT_POS_A(a), 0.0);
		double v = GetDlgDouble(hWnd, ID_EDIT_VEL_A(a), 10000.0);
		double ta = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(a), 100.0);
		double td = GetDlgDouble(hWnd, ID_EDIT_DECT_A(a), 100.0);
		StartAbsMoveWithProfile(a, (long long)std::llround(tgt), v, ta, td);
	}
}
static void DoMultiRel(HWND hWnd) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Info"), MB_ICONWARNING); return; }
	//DisableAllEnabledSyncGroups();
	for (int a = 0; a < 4; ++a) if (IsAxisChecked(hWnd, a) && EnsureServoOn(a) && EnsurePosModeNoStop(a)) {

		// Axis2 보호
		if (a == 2) {
			g_cm.GetStatus(&g_status);
			long long cur = (long long)g_status.axesStatus[2].actualPos;
			double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(2), 0.0);
			long long tgt = cur + (long long)std::llround(step);
			int dir = (step >= 0) ? +1 : -1;
			if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)dir)) {
				Axis2ShowMinusBlockedWarning(hWnd);
				continue;
			}
		}

		// Axis0 보호: 상대 이동 방향 차단
		if (a == 0) {
			g_cm.GetStatus(&g_status);
			long long cur = (long long)g_status.axesStatus[0].actualPos;
			double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(0), 0.0);
			long long tgt = cur + (long long)std::llround(step);
			int dir = (step >= 0) ? +1 : -1;
			if (Axis0IsCommandBlocked(cur, tgt, (long long)dir)) {
				Axis0ShowBlockedWarning(hWnd, dir);
				continue;
			}
		}

		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[a].actualPos;
		double step = GetDlgDouble(hWnd, ID_EDIT_POS_A(a), 0.0);
		double v = GetDlgDouble(hWnd, ID_EDIT_VEL_A(a), 10000.0);
		double ta = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(a), 100.0);
		double td = GetDlgDouble(hWnd, ID_EDIT_DECT_A(a), 100.0);
		StartAbsMoveWithProfile(a, cur + (long long)std::llround(step), v, ta, td);
	}
}
static void DoMultiAlarmReset(HWND hWnd) {
	if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Alarm Reset(Selected)"), MB_ICONWARNING); return; }
	for (int a = 0; a < 4; ++a) {
		if (!IsAxisChecked(hWnd, a)) continue;
		g_cm.motion->Stop(a); g_cm.velocity->Stop(a); if (g_cm.torque) g_cm.torque->StopTrq(a); Sleep(10);
		g_cm.GetStatus(&g_status);
		bool wasOn = g_status.axesStatus[a].servoOn ? true : false;
		if (wasOn) { g_cm.axisControl->SetServoOn(a, 0); Sleep(20); }
		long e = g_cm.axisControl->ClearAmpAlarm(a);
		if (e != ErrorCode::None) { Sleep(30); e = g_cm.axisControl->ClearAmpAlarm(a); }
		if (wasOn) { Sleep(20); g_cm.axisControl->SetServoOn(a, 1); }
		if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Alarm Reset 실패"), e, g_wmx);
	}
}

static long long GetRailPulseClamped()
{
	g_cm.GetStatus(&g_status);
	long long p0 = (long long)g_status.axesStatus[0].actualPos;
	long long p1 = (long long)g_status.axesStatus[1].actualPos * TRAVEL_AXIS1_SIGN;
	long long railPulse = (p0 + p1) / 2;

	if (railPulse < 0) railPulse = 0;
	if (railPulse > TARGET_ZONE5) railPulse = TARGET_ZONE5;
	return railPulse;
}

static void SetZoneAnnounce(HWND hWnd, const TCHAR* msg) { if (HWND h = GetDlgItem(hWnd, ID_TXT_ZONE_ANNOUNCE)) SetWindowText(h, msg); }
static void SetMotionAnnounce(HWND hWnd, const TCHAR* msg) { if (HWND h = GetDlgItem(hWnd, ID_TXT_MOTION_ANNOUNCE)) SetWindowText(h, msg); }

void UpdateTcpUiState(HWND hWnd)
{
	if (!hWnd) return;
	if (HWND h = GetDlgItem(hWnd, ID_TXT_TCP_STATE)) {
		if (g_tcpRunning.load()) {
			wchar_t msg[128];
			swprintf_s(msg, L"TCP: LISTEN %s:%d", g_tcpBindIp, g_tcpBindPort);
			SetWindowTextW(h, msg);
		}
		else {
			SetWindowTextW(h, L"TCP: STOPPED");
		}
	}
}

static void UpdateZoneAnnounce_Pulse(HWND hWnd)
{
	long long rp = GetRailPulseClamped();

	if (!g_zoneInit.load()) {
		int z = (int)(rp / ZONE_PULSES);
		if (rp >= TARGET_ZONE5) z = TARGET_ZONE_IDX;
		g_zoneDisplay = (z < 0) ? 0 : (z > TARGET_ZONE_IDX ? TARGET_ZONE_IDX : z);
		g_zoneInit = true;
	}
	else {
		int z = g_zoneDisplay.load();
		while (z < TARGET_ZONE_IDX && rp >= (long long)(z + 1) * ZONE_PULSES) ++z;
		while (z > 0 && rp <= (long long)(z - 1) * ZONE_PULSES) --z;
		g_zoneDisplay = z;
	}

	int zone = g_zoneDisplay.load();
	TCHAR zmsg[128];
	DemoStage st = g_demoStage.load();

	if (zone == 0) {
		if (st == DemoStage::StartAtZero) _stprintf_s(zmsg, TEXT("Zone: 시작점(0번, 출발)"));
		else if (st == DemoStage::At0Return || st == DemoStage::Complete) _stprintf_s(zmsg, TEXT("Zone: 시작점(0번, 복귀)"));
		else _stprintf_s(zmsg, TEXT("Zone: 시작점(0번)"));
	}
	else if (zone == TARGET_ZONE_IDX) {
		_stprintf_s(zmsg, TEXT("Zone: 목표지점(%d번)"), TARGET_ZONE_IDX);
	}
	else {
		_stprintf_s(zmsg, TEXT("Zone: %d번 구역"), zone);
	}
	SetZoneAnnounce(hWnd, zmsg);
}

static bool WaitAxesToTargets(HWND hWnd,
	const std::vector<std::pair<int, long long>>& axes_targets,
	DWORD timeout_ms,
	bool updateZoneDuringWait)
{
	DWORD t0 = GetTickCount();
	while (g_demoRunning.load()) {
		g_cm.GetStatus(&g_status);
		bool allOK = true;
		for (auto& at : axes_targets) {
			int a = at.first;
			long long tgt = at.second;
			long long act = (long long)g_status.axesStatus[a].actualPos;
			int actVel = (int)std::lround(g_status.axesStatus[a].actualVelocity);
			if (std::llabs(act - tgt) > inpos_tol_counts || std::abs(actVel) > vel_idle_threshold) {
				allOK = false; break;
			}
		}
		if (updateZoneDuringWait) UpdateZoneAnnounce_Pulse(hWnd);
		if (allOK) return true;
		if (GetTickCount() - t0 > timeout_ms) return false;
		Sleep(idle_wait_poll_ms);
	}
	return false;
}

static void UpdateZoneAnnounce_Sensor(HWND hWnd)
{
	int zone = g_sensorZoneCount.load();
	if (zone < 0) zone = 0;
	if (zone > TARGET_ZONE_IDX) zone = TARGET_ZONE_IDX;

	TCHAR zmsg[128];
	if (zone == 0) _stprintf_s(zmsg, TEXT("Zone(Sensor): 시작점(0번)"));
	else if (zone == TARGET_ZONE_IDX) _stprintf_s(zmsg, TEXT("Zone(Sensor): 목표지점(%d번)"), TARGET_ZONE_IDX);
	else _stprintf_s(zmsg, TEXT("Zone(Sensor): %d번 구역"), zone);
	SetZoneAnnounce(hWnd, zmsg);
}

static bool WaitAxesIdle(const std::vector<int>& axes, DWORD timeout_ms) {
	DWORD t0 = GetTickCount();
	while (g_demoRunning.load()) {
		g_cm.GetStatus(&g_status);
		bool allIdle = true;
		for (int a : axes) {
			int actVel = (int)std::lround(g_status.axesStatus[a].actualVelocity);
			if (std::abs(actVel) > vel_idle_threshold) { allIdle = false; break; }
		}
		if (allIdle) return true;
		if (GetTickCount() - t0 > timeout_ms) return false;
		Sleep(idle_wait_poll_ms);
	}
	g_cm.GetStatus(&g_status);
	bool allIdle = true;
	for (int a : axes) {
		int actVel = (int)std::lround(g_status.axesStatus[a].actualVelocity);
		if (std::abs(actVel) > vel_idle_threshold) { allIdle = false; break; }
	}
	return allIdle;
}

// ------------------ Demo helpers ------------------
static bool EnsureStartAtZero(HWND hWnd) {
	DisableAllEnabledSyncGroups();
	SetMotionAnnounce(hWnd, TEXT("Motion: 시작 정렬(모든 축 0 이동)"));
	double v0 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(0), 20000.0);
	double v1 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(1), 20000.0);
	double v2 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(2), 15000.0);
	double v3 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(3), 8000.0);
	double t = 100.0;
	if (!StartAbsMoveWithProfile(0, 0, v0, t, t)) return false;
	if (!StartAbsMoveWithProfile(1, 0, v1, t, t)) return false;
	if (!StartAbsMoveWithProfile(2, 0, v2, t, t)) return false;
	if (!StartAbsMoveWithProfile(3, 0, v3, t, t)) return false;

	if (!WaitAxesToTargets(hWnd, { {0,0},{1,0},{2,0},{3,0} }, 30000, true)) {
		SetMotionAnnounce(hWnd, TEXT("Motion: 시작 정렬 대기 실패"));
		return false;
	}
	return true;
}

static bool Travel_0_to_5_Pulse(HWND hWnd) {
	g_demoStage = DemoStage::TravelingTo5;
	SetMotionAnnounce(hWnd, TEXT("Motion: 0→5 구역 주행(펄스)"));
	double v0 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(0), 20000.0);
	double v1 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(1), 20000.0);
	double t = 100.0;

	if (!StartAbsMoveWithProfile(0, TARGET_ZONE5, v0, t, t)) return false;
	if (!StartAbsMoveWithProfile(1, TARGET_ZONE5 * TRAVEL_AXIS1_SIGN, v1, t, t)) return false;

	bool ok = WaitAxesToTargets(hWnd, { {0,TARGET_ZONE5},{1, TARGET_ZONE5 * TRAVEL_AXIS1_SIGN} }, 60000, true);
	if (!ok) SetMotionAnnounce(hWnd, TEXT("Motion: 0→5 이동 타임아웃"));
	g_demoStage = DemoStage::At5;
	return ok;
}

static bool Travel_5_to_0_Pulse(HWND hWnd) {
	g_demoStage = DemoStage::TravelingTo0;
	SetMotionAnnounce(hWnd, TEXT("Motion: 5→0 구역 주행(펄스)"));
	double v0 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(0), 20000.0);
	double v1 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(1), 20000.0);
	double t = 100.0;

	if (!StartAbsMoveWithProfile(0, 0, v0, t, t)) return false;
	if (!StartAbsMoveWithProfile(1, 0, v1, t, t)) return false;

	bool ok = WaitAxesToTargets(hWnd, { {0,0},{1,0} }, 60000, true);
	if (!ok) SetMotionAnnounce(hWnd, TEXT("Motion: 5→0 이동 타임아웃"));
	g_demoStage = DemoStage::At0Return;
	return ok;
}

static bool Travel_0_to_5_Sensor(HWND hWnd, int targetCounts, DWORD timeout_ms) {
	g_demoStage = DemoStage::TravelingTo5;
	SetMotionAnnounce(hWnd, TEXT("Motion: 0→5 구역 주행(센서 카운트)"));
	Io io(&g_wmx);

	double v0 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(0), 20000.0);
	double v1 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(1), 20000.0);
	double t = 100.0;

	g_cm.GetStatus(&g_status);
	long long cur0 = (long long)g_status.axesStatus[0].actualPos;
	long long cur1 = (long long)g_status.axesStatus[1].actualPos;
	long long tgt0 = cur0 + 100000000LL;
	long long tgt1 = cur1 + (100000000LL * TRAVEL_AXIS1_SIGN);
	StartAbsMoveWithProfile(0, tgt0, v0, t, t);
	StartAbsMoveWithProfile(1, tgt1, v1, t, t);

	bool stableOn = ReadSensorOn(io);
	DWORD lastTick = GetTickCount();
	int count = 0;
	DWORD t0 = GetTickCount();

	while (g_demoRunning.load() && count < targetCounts) {
		if (DebounceReadSensor(io, stableOn, lastTick)) {
			if (stableOn) {
				++count;
				g_sensorZoneCount.store(count);
				UpdateZoneAnnounce_Sensor(hWnd);
			}
		}
		if (GetTickCount() - t0 > timeout_ms) {
			SetMotionAnnounce(hWnd, TEXT("Motion: 센서 주행(전진) 타임아웃"));
			break;
		}
		Sleep(SENSOR_POLL_MS);
	}

	StopAxis(0); StopAxis(1);
	WaitAxesIdle({ 0,1 }, 10000);

	return (count >= targetCounts);
}

static bool Travel_5_to_0_Sensor(HWND hWnd, int /*targetCounts*/, DWORD timeout_ms) {
	g_demoStage = DemoStage::TravelingTo0;
	SetMotionAnnounce(hWnd, TEXT("Motion: 5→0 구역 주행(센서 카운트)"));
	Io io(&g_wmx);

	double v0 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(0), 20000.0);
	double v1 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(1), 20000.0);
	double t = 100.0;

	g_cm.GetStatus(&g_status);
	long long cur0 = (long long)g_status.axesStatus[0].actualPos;
	long long cur1 = (long long)g_status.axesStatus[1].actualPos;
	long long tgt0 = cur0 - 100000000LL;
	long long tgt1 = cur1 - (100000000LL * TRAVEL_AXIS1_SIGN);
	StartAbsMoveWithProfile(0, tgt0, v0, t, t);
	StartAbsMoveWithProfile(1, tgt1, v1, t, t);

	bool stableOn = ReadSensorOn(io);
	DWORD lastTick = GetTickCount();
	DWORD t0 = GetTickCount();

	while (g_demoRunning.load() && g_sensorZoneCount.load() > 0) {
		if (DebounceReadSensor(io, stableOn, lastTick)) {
			if (stableOn) {
				int z = g_sensorZoneCount.load();
				if (z > 0) g_sensorZoneCount.store(z - 1);
				UpdateZoneAnnounce_Sensor(hWnd);
			}
		}
		if (GetTickCount() - t0 > timeout_ms) {
			SetMotionAnnounce(hWnd, TEXT("Motion: 센서 주행(후진) 타임아웃"));
			break;
		}
		Sleep(SENSOR_POLL_MS);
	}

	StopAxis(0); StopAxis(1);
	WaitAxesIdle({ 0,1 }, 10000);

	return g_sensorZoneCount.load() == 0;
}

static bool Hoist_Down_100k(HWND hWnd) {
	g_demoStage = DemoStage::HoistDown;
	SetMotionAnnounce(hWnd, TEXT("Motion: 호이스트 하강 100k"));
	if (!EnsureServoOn(2) || !EnsurePosModeNoStop(2)) return false;
	double v2 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(2), 15000.0);
	double a2ms = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(2), 100.0);
	double d2ms = GetDlgDouble(hWnd, ID_EDIT_DECT_A(2), 100.0);
	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[2].actualPos;
	if (!StartAbsMoveWithProfile(2, cur + HOIST_DOWN_PULSES, v2, a2ms, d2ms)) return false;
	bool ok = WaitAxesToTargets(hWnd, { {2, cur + HOIST_DOWN_PULSES} }, 30000, false);
	return ok;
}

static bool Hoist_Up_100k_to_Zero(HWND hWnd) {
	g_demoStage = DemoStage::HoistUp;
	SetMotionAnnounce(hWnd, TEXT("Motion: 호이스트 상승 → 0"));
	if (!EnsureServoOn(2) || !EnsurePosModeNoStop(2)) return false;
	double v2 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(2), 15000.0);
	double a2ms = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(2), 100.0);
	double d2ms = GetDlgDouble(hWnd, ID_EDIT_DECT_A(2), 100.0);
	if (!StartAbsMoveWithProfile(2, 0, v2, a2ms, d2ms)) return false;
	bool ok = WaitAxesToTargets(hWnd, { {2, 0} }, 30000, false);
	return ok;
}

static bool Gripper_Close_10k(HWND hWnd) {
	g_demoStage = DemoStage::GripClose;
	SetMotionAnnounce(hWnd, TEXT("Motion: 그리퍼 닫기 10k"));
	if (!EnsureServoOn(3) || !EnsurePosModeNoStop(3)) return false;
	double v3 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(3), 8000.0);
	double a3ms = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(3), 100.0);
	double d3ms = GetDlgDouble(hWnd, ID_EDIT_DECT_A(3), 100.0);
	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[3].actualPos;
	if (!StartAbsMoveWithProfile(3, cur + GRIPPER_CLOSE_PULSE, v3, a3ms, d3ms)) return false;
	bool ok = WaitAxesToTargets(hWnd, { {3, cur + GRIPPER_CLOSE_PULSE} }, 20000, false);
	return ok;
}

static bool Gripper_Open_to_Zero(HWND hWnd) {
	g_demoStage = DemoStage::GripOpen;
	SetMotionAnnounce(hWnd, TEXT("Motion: 그리퍼 열기 → 0"));
	if (!EnsureServoOn(3) || !EnsurePosModeNoStop(3)) return false;
	double v3 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(3), 8000.0);
	double a3ms = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(3), 100.0);
	double d3ms = GetDlgDouble(hWnd, ID_EDIT_DECT_A(3), 100.0);
	if (!StartAbsMoveWithProfile(3, 0, v3, a3ms, d3ms)) return false;
	bool ok = WaitAxesToTargets(hWnd, { {3, 0} }, 20000, false);
	return ok;
}

// ======================================================
// =================== Sync Group Window =================
// ======================================================
struct SyncUiState {
	int groupId = 0;
	int masterAxis = -1;
	std::vector<int> slaveAxes;
	bool controlMaster = true;
	int controlAxis = 0;
	std::vector<int> detectedAxes;
};
static SyncUiState g_syncUi;

static void Sync_DetectAxes() {
	g_syncUi.detectedAxes.clear();
	if (!g_commStarted) return;
	g_cm.GetStatus(&g_status);
	for (int ax = 0; ax < 32; ++ax) {
		if (ax < 4) g_syncUi.detectedAxes.push_back(ax);
	}
}

static void Sync_ClearDynamicAxisWidgets(HWND h) {
	for (int ax = 0; ax < 64; ++ax) {
		if (HWND w = GetDlgItem(h, ID_SYNC_MASTER_AXIS_BTN_BASE + ax)) DestroyWindow(w);
	}
	for (int ax = 0; ax < 64; ++ax) {
		if (HWND w = GetDlgItem(h, ID_SYNC_SLAVE_AXIS_CHK_BASE + ax)) DestroyWindow(w);
	}
}

static void Sync_RebuildAxisPickers(HWND h) {
	Sync_ClearDynamicAxisWidgets(h);

	int xMaster = 20, yMaster = 100;
	CreateWindow(TEXT("STATIC"), TEXT("마스터 선택"), WS_CHILD | WS_VISIBLE, xMaster, yMaster - 24, 100, 20, h, 0, 0, 0);

	int btnW = 48, btnH = 26, gap = 6;
	int col = 0, row = 0, maxCols = 8;
	bool first = true;
	for (int a : g_syncUi.detectedAxes) {
		int bx = xMaster + (btnW + gap) * col;
		int by = yMaster + (btnH + gap) * row;
		TCHAR cap[16]; _stprintf_s(cap, TEXT("%d"), a);
		DWORD style = WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | (first ? (WS_GROUP | WS_TABSTOP) : 0);
		HWND b = CreateWindow(TEXT("BUTTON"), cap, style, bx, by, btnW, btnH, h, (HMENU)(INT_PTR)(ID_SYNC_MASTER_AXIS_BTN_BASE + a), 0, 0);
		first = false;
		if (g_syncUi.masterAxis == a) SendMessage(b, BM_SETCHECK, BST_CHECKED, 0);
		++col; if (col >= maxCols) { col = 0; ++row; }
	}

	int xSlave = 20, ySlave = yMaster + 10 + ((row + (col ? 1 : 0)) * (btnH + gap)) + 20;
	CreateWindow(TEXT("STATIC"), TEXT("슬레이브 선택"), WS_CHILD | WS_VISIBLE, xSlave, ySlave - 24, 100, 20, h, 0, 0, 0);

	col = 0; row = 0;
	for (int a : g_syncUi.detectedAxes) {
		int bx = xSlave + (btnW + gap) * col;
		int by = ySlave + (btnH + gap) * row;
		TCHAR cap[16]; _stprintf_s(cap, TEXT("%d"), a);
		HWND b = CreateWindow(TEXT("BUTTON"), cap, WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
			bx, by, btnW, btnH, h, (HMENU)(INT_PTR)(ID_SYNC_SLAVE_AXIS_CHK_BASE + a), 0, 0);
		bool sel = std::find(g_syncUi.slaveAxes.begin(), g_syncUi.slaveAxes.end(), a) != g_syncUi.slaveAxes.end();
		SendMessage(b, BM_SETCHECK, sel ? BST_CHECKED : BST_UNCHECKED, 0);
		++col; if (col >= maxCols) { col = 0; ++row; }
	}
}

static void Sync_CreateAxisListColumns(HWND hList) {
	LVCOLUMN col{};
	col.mask = LVCF_TEXT | LVCF_WIDTH | LVCF_SUBITEM;

	col.pszText = const_cast<LPTSTR>(TEXT("Axis")); col.cx = 60; col.iSubItem = 0; ListView_InsertColumn(hList, 0, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("Role")); col.cx = 90; col.iSubItem = 1; ListView_InsertColumn(hList, 1, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("OpState")); col.cx = 80; col.iSubItem = 2; ListView_InsertColumn(hList, 2, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("PosCmd")); col.cx = 100; col.iSubItem = 3; ListView_InsertColumn(hList, 3, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("ActualPos")); col.cx = 100; col.iSubItem = 4; ListView_InsertColumn(hList, 4, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("CmdVel")); col.cx = 80; col.iSubItem = 5; ListView_InsertColumn(hList, 5, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("ActVel")); col.cx = 80; col.iSubItem = 6; ListView_InsertColumn(hList, 6, &col);
	col.pszText = const_cast<LPTSTR>(TEXT("Err(603F)")); col.cx = 100; col.iSubItem = 7; ListView_InsertColumn(hList, 7, &col);
}

static int StartupEnumToComboIndex(Sync::SyncGroupStartupType::T t) {
	switch (t) {
	case Sync::SyncGroupStartupType::Normal: return 0;
	case Sync::SyncGroupStartupType::CatchUp: return 1;
	default: return 0;
	}
}
static Sync::SyncGroupStartupType::T ComboIndexToStartupEnum(int idx) {
	return (idx == 1) ? Sync::SyncGroupStartupType::CatchUp : Sync::SyncGroupStartupType::Normal;
}

// ---- Helper: Read/Write Config::SyncParam master/slave desync dec ----
static bool ReadSyncParam(Config::SyncParam& sp) {
	if (!g_commStarted || g_syncUi.masterAxis < 0) return false;
	int axis = g_syncUi.masterAxis;
	long e = g_cm.config->GetSyncParam(axis, &sp);
	return (e == ErrorCode::None);
}

static bool WriteSyncParam(const Config::SyncParam& sp) {
	if (!g_commStarted || g_syncUi.masterAxis < 0) return false;
	int axis = g_syncUi.masterAxis;
	long e = g_cm.config->SetSyncParam(axis, (Config::SyncParam*)&sp, nullptr);
	return (e == ErrorCode::None);
}

static void Sync_LoadGroupParamsToUI(HWND hWnd) {
	Sync_DetectAxes();

	Sync::SyncGroup grp{};
	long e = g_cm.sync->GetSyncGroup(g_syncUi.groupId, &grp);

	if (e != ErrorCode::None) {
		g_syncUi.masterAxis = (g_syncUi.detectedAxes.empty() ? -1 : g_syncUi.detectedAxes.front());
		g_syncUi.slaveAxes.clear();

		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_SERVO_ONOFF), CB_SETCURSEL, 1, 0);
		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_STARTUP), CB_SETCURSEL, 0, 0);

		SetDlgInt(hWnd, ID_SYNC_PARAM_CYCLE_RATIO, 1);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_TOLERANCE, 1000.0);

		SetDlgDouble(hWnd, ID_SYNC_PARAM_MAX_CATCH_UP, 0.0);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_VEL, 0.0);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_ACC, 0.0);

		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_USE_MASTER_FB), CB_SETCURSEL, 0, 0);
		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_AMP_ERR_SVON), CB_SETCURSEL, 0, 0);
	}
	else {
		g_syncUi.masterAxis = (int)grp.masterAxis;

		g_syncUi.slaveAxes.clear();
		for (int k = 0; k < (int)grp.slaveAxisCount; ++k)
			g_syncUi.slaveAxes.push_back((int)grp.slaveAxis[k]);

		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_SERVO_ONOFF), CB_SETCURSEL, grp.servoOnOffSynchronization ? 1 : 0, 0);
		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_STARTUP), CB_SETCURSEL, StartupEnumToComboIndex(grp.startupType), 0);

		SetDlgInt(hWnd, ID_SYNC_PARAM_CYCLE_RATIO, (int)grp.gantryLoopCycleRatio);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_MAX_CATCH_UP, grp.maxCatchUpDistance);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_VEL, grp.catchUpVelocity);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_ACC, grp.catchUpAcc);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_TOLERANCE, grp.syncErrorTolerance);

		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_USE_MASTER_FB), CB_SETCURSEL, grp.useMasterFeedback ? 1 : 0, 0);
		SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_AMP_ERR_SVON), CB_SETCURSEL, 0, 0);
	}

	// Read Config::SyncParam master/slave Desync Dec into UI
	Config::SyncParam sp{};
	if (ReadSyncParam(sp)) {
		SetDlgDouble(hWnd, ID_SYNC_PARAM_MASTER_DESYNC_DEC, sp.masterDesyncDec);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_SLAVE_DESYNC_DEC, sp.slaveDesyncDec);
	}
	else {
		SetDlgDouble(hWnd, ID_SYNC_PARAM_MASTER_DESYNC_DEC, 10000.0);
		SetDlgDouble(hWnd, ID_SYNC_PARAM_SLAVE_DESYNC_DEC, 10000.0);
	}

	Sync_RebuildAxisPickers(hWnd);
}

static bool Sync_ApplyGroupParamsFromUI(HWND hWnd) {
	int selMaster = -1;
	for (int a : g_syncUi.detectedAxes) {
		if (SendMessage(GetDlgItem(hWnd, ID_SYNC_MASTER_AXIS_BTN_BASE + a), BM_GETCHECK, 0, 0) == BST_CHECKED) {
			selMaster = a; break;
		}
	}
	if (selMaster < 0) {
		MessageBox(hWnd, TEXT("마스터 축을 선택하세요."), TEXT("Sync Group"), MB_ICONWARNING);
		return false;
	}

	std::vector<int> slaves;
	for (int a : g_syncUi.detectedAxes) {
		if (SendMessage(GetDlgItem(hWnd, ID_SYNC_SLAVE_AXIS_CHK_BASE + a), BM_GETCHECK, 0, 0) == BST_CHECKED) {
			if (a != selMaster) slaves.push_back(a);
		}
	}

	Sync::SyncGroup grp{};
	grp.masterAxis = (unsigned char)selMaster;

	int maxSlaves = (int)(sizeof(grp.slaveAxis) / sizeof(grp.slaveAxis[0]));
	if ((int)slaves.size() > maxSlaves) {
		MessageBox(hWnd, TEXT("슬레이브 축 개수가 최대치를 초과했습니다."), TEXT("Sync Group"), MB_ICONWARNING);
		return false;
	}
	grp.slaveAxisCount = (unsigned char)slaves.size();
	for (int i = 0; i < (int)grp.slaveAxisCount; ++i)
		grp.slaveAxis[i] = (unsigned char)slaves[i];

	grp.servoOnOffSynchronization = (unsigned char)(SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_SERVO_ONOFF), CB_GETCURSEL, 0, 0) == 1 ? 1 : 0);
	{
		int idx = (int)SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_STARTUP), CB_GETCURSEL, 0, 0);
		grp.startupType = ComboIndexToStartupEnum(idx);
	}
	grp.gantryLoopCycleRatio = (unsigned int)GetDlgInt(hWnd, ID_SYNC_PARAM_CYCLE_RATIO, 1);
	grp.maxCatchUpDistance = GetDlgDouble(hWnd, ID_SYNC_PARAM_MAX_CATCH_UP, 0.0);
	grp.catchUpVelocity = GetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_VEL, 0.0);
	grp.catchUpAcc = GetDlgDouble(hWnd, ID_SYNC_PARAM_CATCHUP_ACC, 0.0);
	grp.syncErrorTolerance = GetDlgDouble(hWnd, ID_SYNC_PARAM_TOLERANCE, 1000.0);
	grp.useMasterFeedback = (unsigned char)(SendMessage(GetDlgItem(hWnd, ID_SYNC_PARAM_USE_MASTER_FB), CB_GETCURSEL, 0, 0) == 1 ? 1 : 0);

	auto isDetected = [&](int a) { return std::find(g_syncUi.detectedAxes.begin(), g_syncUi.detectedAxes.end(), a) != g_syncUi.detectedAxes.end(); };
	if (!isDetected(selMaster)) { MessageBox(hWnd, TEXT("선택된 마스터 축이 인식되지 않았습니다."), TEXT("Sync Group"), MB_ICONWARNING); return false; }
	for (int s : slaves) {
		if (!isDetected(s)) { MessageBox(hWnd, TEXT("선택된 슬레이브 축 중 인식되지 않은 축이 있습니다."), TEXT("Sync Group"), MB_ICONWARNING); return false; }
		if (s == selMaster) { MessageBox(hWnd, TEXT("마스터 축은 슬레이브로 중복 지정할 수 없습니다."), TEXT("Sync Group"), MB_ICONWARNING); return false; }
	}

	grp.gantryLoopCycleRatio = std::max(1u, grp.gantryLoopCycleRatio);
	if (grp.syncErrorTolerance <= 0) grp.syncErrorTolerance = 1000.0;

	if (grp.startupType == Sync::SyncGroupStartupType::CatchUp) {
		if (grp.catchUpVelocity <= 0) grp.catchUpVelocity = 50000.0;
		if (grp.catchUpAcc <= 0) grp.catchUpAcc = 100000.0;
		if (grp.maxCatchUpDistance <= 0) grp.maxCatchUpDistance = 10000.0;
	}
	else {
		if (grp.catchUpVelocity < 0) grp.catchUpVelocity = 0.0;
		if (grp.catchUpAcc < 0) grp.catchUpAcc = 0.0;
		if (grp.maxCatchUpDistance < 0) grp.maxCatchUpDistance = 0.0;
	}

	// If enabled, disable first
	Sync::SyncGroupStatus gst{};
	if (g_cm.sync->GetSyncGroupStatus(g_syncUi.groupId, &gst) == ErrorCode::None && gst.enabled) {
		g_cm.sync->EnableSyncGroup(g_syncUi.groupId, 0);
		Sleep(5);
	}

	long se = g_cm.sync->SetSyncGroup(g_syncUi.groupId, grp);
	if (se != ErrorCode::None) { ShowErrMsgBox(TEXT("SetSyncGroup 실패"), se, g_wmx); return false; }

	// Apply Config::SyncParam desync dec from UI
	Config::SyncParam sp{};
	if (ReadSyncParam(sp)) {
		sp.masterDesyncDec = GetDlgDouble(hWnd, ID_SYNC_PARAM_MASTER_DESYNC_DEC, 10000.0);
		sp.slaveDesyncDec = GetDlgDouble(hWnd, ID_SYNC_PARAM_SLAVE_DESYNC_DEC, 10000.0);
		if (!WriteSyncParam(sp)) {
			MessageBox(hWnd, TEXT("Config::SyncParam 적용 실패(master/slave Desync Dec)"), TEXT("Sync Group"), MB_ICONWARNING);
		}
	}

	g_syncUi.masterAxis = selMaster;
	g_syncUi.slaveAxes = slaves;

	return true;
}

static void Sync_EnableGroup(HWND hWnd, bool en) {
	long e = g_cm.sync->EnableSyncGroup(g_syncUi.groupId, en ? 1 : 0);
	if (e != ErrorCode::None) ShowErrMsgBox(en ? TEXT("EnableSyncGroup 실패") : TEXT("DisableSyncGroup 실패"), e, g_wmx);

	// Ensure Config::SyncParam desync dec stays in sync with UI even on Enable button
	if (en) {
		Config::SyncParam sp{};
		if (ReadSyncParam(sp)) {
			sp.masterDesyncDec = GetDlgDouble(hWnd, ID_SYNC_PARAM_MASTER_DESYNC_DEC, sp.masterDesyncDec);
			sp.slaveDesyncDec = GetDlgDouble(hWnd, ID_SYNC_PARAM_SLAVE_DESYNC_DEC, sp.slaveDesyncDec);
			WriteSyncParam(sp);
		}
	}
}

static void Sync_ClearGroupError() {
	long e = g_cm.sync->ClearSyncGroupError(g_syncUi.groupId);
	if (e != ErrorCode::None) ShowErrMsgBox(TEXT("ClearSyncGroupError 실패"), e, g_wmx);
}

static void Sync_AllServoOnOff(bool on) {
	Sync::SyncGroup grp{}; if (g_cm.sync->GetSyncGroup(g_syncUi.groupId, &grp) != ErrorCode::None) return;
	auto turn = [&](int ax) {
		if (on) { EnsureServoOn(ax); EnsurePosModeNoStop(ax); }
		else g_cm.axisControl->SetServoOn(ax, 0);
		};
	turn(grp.masterAxis);
	for (int i = 0; i < (int)grp.slaveAxisCount; ++i) turn(grp.slaveAxis[i]);
}

static int Sync_GetControlAxis(HWND hWnd) {
	bool master = (SendMessage(GetDlgItem(hWnd, ID_SYNC_RAD_MASTER), BM_GETCHECK, 0, 0) == BST_CHECKED);
	if (master) return g_syncUi.masterAxis >= 0 ? g_syncUi.masterAxis : 0;
	return GetDlgInt(hWnd, ID_SYNC_AXIS_SELECT, 0);
}

static void Sync_Control_Jog(HWND hWnd, int sign) {
	if (!IsManualAllowed(hWnd)) return;
	int ax = Sync_GetControlAxis(hWnd);

	// Axis2 보호: 리밋 ON시 -방향 조그 차단
	if (ax == 2) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[2].actualPos;
		long long tgt = cur + (long long)sign * 1000000000LL;
		if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)sign)) {
			Axis2ShowMinusBlockedWarning(hWnd);
			return;
		}
	}

	// Axis0 보호: L/R 리밋에 따른 방향 조그 차단
	if (ax == 0) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[0].actualPos;
		long long tgt = cur + (long long)sign * 1000000000LL;
		if (Axis0IsCommandBlocked(cur, tgt, (long long)sign)) {
			Axis0ShowBlockedWarning(hWnd, sign);
			return;
		}
	}

	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;
	double vpps = GetDlgDouble(hWnd, ID_SYNC_JOG_SPEED, 10000.0);
	double tAcc = GetDlgDouble(hWnd, ID_SYNC_ACC, 100.0);
	double tDec = GetDlgDouble(hWnd, ID_SYNC_DEC, 100.0);

	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[ax].actualPos;

	Motion::PosCommand pc{};
	pc.axis = ax;
	pc.target = cur + (long long)sign * 1000000000LL;
	pc.profile.type = ProfileType::SCurve;
	pc.profile.velocity = (int)std::lround(vpps);
	pc.profile.acc = TimeMsToAcc(vpps, tAcc);
	pc.profile.dec = TimeMsToAcc(vpps, tDec);
	long e = g_cm.motion->StartPos(&pc);
	if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Sync Jog 실패"), e, g_wmx);
}
static void Sync_Control_Stop(HWND, int ax) {
	g_cm.motion->Stop(ax); g_cm.velocity->Stop(ax); if (g_cm.torque) g_cm.torque->StopTrq(ax);
}
static void Sync_Control_Abs(HWND hWnd) {
	if (!IsManualAllowed(hWnd)) return;
	int ax = Sync_GetControlAxis(hWnd);
	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;
	double tgt = GetDlgDouble(hWnd, ID_SYNC_ABS_POS, 0.0);
	double v = GetDlgDouble(hWnd, ID_SYNC_JOG_SPEED, 10000.0);
	double ta = GetDlgDouble(hWnd, ID_SYNC_ACC, 100.0);
	double td = GetDlgDouble(hWnd, ID_SYNC_DEC, 100.0);

	// Axis2 보호: 절대 이동 차단 체크
	if (ax == 2) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[2].actualPos;
		long long tgtLL = (long long)std::llround(tgt);
		if (Axis2IsMinusCommandBlocked(cur, tgtLL, 0)) {
			Axis2ShowMinusBlockedWarning(hWnd);
			return;
		}
	}

	// Axis0 보호: 절대 이동 방향 차단 체크
	if (ax == 0) {
		g_cm.GetStatus(&g_status);
		long long cur = (long long)g_status.axesStatus[0].actualPos;
		long long tgtLL = (long long)std::llround(tgt);
		long long delta = tgtLL - cur;
		long long sign = (delta > 0) ? +1 : (delta < 0 ? -1 : 0);
		if (sign != 0 && Axis0IsCommandBlocked(cur, tgtLL, sign)) {
			Axis0ShowBlockedWarning(hWnd, (int)sign);
			return;
		}
	}

	StartAbsMoveWithProfile(ax, (long long)std::llround(tgt), v, ta, td);
}
static void Sync_Control_Rel(HWND hWnd, int sign) {
	if (!IsManualAllowed(hWnd)) return;
	int ax = Sync_GetControlAxis(hWnd);
	if (!EnsureServoOn(ax) || !EnsurePosModeNoStop(ax)) return;
	double step = GetDlgDouble(hWnd, ID_SYNC_REL_STEP, 0.0) * sign;
	double v = GetDlgDouble(hWnd, ID_SYNC_JOG_SPEED, 10000.0);
	double ta = GetDlgDouble(hWnd, ID_SYNC_ACC, 100.0);
	double td = GetDlgDouble(hWnd, ID_SYNC_DEC, 100.0);
	g_cm.GetStatus(&g_status);
	long long cur = (long long)g_status.axesStatus[ax].actualPos;
	long long tgt = cur + (long long)std::llround(step);

	// Axis2 보호: 상대 이동 차단 체크
	if (ax == 2) {
		if (Axis2IsMinusCommandBlocked(cur, tgt, (long long)sign)) {
			Axis2ShowMinusBlockedWarning(hWnd);
			return;
		}
	}

	// Axis0 보호: 상대 이동 방향 차단 체크
	if (ax == 0) {
		if (Axis0IsCommandBlocked(cur, tgt, (long long)sign)) {
			Axis0ShowBlockedWarning(hWnd, sign);
			return;
		}
	}

	StartAbsMoveWithProfile(ax, tgt, v, ta, td);
}

static void Sync_UpdateMonitor(HWND hWnd) {
	if (!g_commStarted) return;

	Sync::SyncGroupStatus st{};
	long e = g_cm.sync->GetSyncGroupStatus(g_syncUi.groupId, &st);
	if (e == ErrorCode::None) {
		SetWindowText(GetDlgItem(hWnd, ID_SYNC_STATE_ENABLED), st.enabled ? (LPTSTR)TEXT("Enabled") : (LPTSTR)TEXT("Disabled"));
		SetWindowText(GetDlgItem(hWnd, ID_SYNC_STATE_HOMEDONE), st.homeDone ? (LPTSTR)TEXT("Home Done") : (LPTSTR)TEXT("Home Not Done"));
	}

	HWND hList = GetDlgItem(hWnd, ID_SYNC_AXIS_LIST);
	if (!hList) return;

	ListView_DeleteAllItems(hList);

	Sync::SyncGroup grp{}; if (g_cm.sync->GetSyncGroup(g_syncUi.groupId, &grp) != ErrorCode::None) return;

	std::vector<std::pair<int, const TCHAR*>> axes;
	axes.push_back({ (int)grp.masterAxis, TEXT("Master") });
	for (int i = 0; i < (int)grp.slaveAxisCount; ++i) axes.push_back({ (int)grp.slaveAxis[i], TEXT("Slave") });

	g_cm.GetStatus(&g_status);

	for (int i = 0; i < (int)axes.size(); ++i) {
		int ax = axes[i].first;
		const TCHAR* role = axes[i].second;

		TCHAR buf[64];
		LVITEM it{}; it.mask = LVIF_TEXT; it.iItem = i; it.iSubItem = 0;
		_stprintf_s(buf, TEXT("%d"), ax); it.pszText = buf;
		ListView_InsertItem(hList, &it);

		ListView_SetItemText(hList, i, 1, const_cast<LPTSTR>(role));

		const auto& a = g_status.axesStatus[ax];
		const TCHAR* ops = (std::abs((int)std::lround(a.actualVelocity)) > vel_idle_threshold) ? TEXT("MOTION") : TEXT("IDLE");
		ListView_SetItemText(hList, i, 2, const_cast<LPTSTR>(ops));

		_stprintf_s(buf, TEXT("%lld"), (long long)a.posCmd);
		ListView_SetItemText(hList, i, 3, buf);
		_stprintf_s(buf, TEXT("%lld"), (long long)a.actualPos);
		ListView_SetItemText(hList, i, 4, buf);
		_stprintf_s(buf, TEXT("%d"), (int)std::lround(a.velocityCmd));
		ListView_SetItemText(hList, i, 5, buf);
		_stprintf_s(buf, TEXT("%d"), (int)std::lround(a.actualVelocity));
		ListView_SetItemText(hList, i, 6, buf);

		int err603f = 0;
		bool ok603f = ReadAxis_TxPDO_603F(kAxisSlaveId[ax], err603f);
		if (ok603f) _stprintf_s(buf, TEXT("0x%04X"), (unsigned)(err603f & 0xFFFF));
		else _stprintf_s(buf, TEXT("-"));
		ListView_SetItemText(hList, i, 7, buf);
	}
}

static void Sync_CreateUI(HWND h) {
	CreateWindow(TEXT("BUTTON"), TEXT("동기 그룹 설정"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, 10, 480, 440, h, 0, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("동기 그룹"), WS_CHILD | WS_VISIBLE, 20, 40, 70, 22, h, 0, 0, 0);
	HWND hCmb = CreateWindow(TEXT("COMBOBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 100, 36, 120, 200, h, (HMENU)ID_SYNC_GROUP_COMBO, 0, 0);
	for (int gid = 0; gid < 4; ++gid) {
		TCHAR t[16]; _stprintf_s(t, TEXT("Group %d"), gid);
		SendMessage(hCmb, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)t);
	}
	SendMessage(hCmb, CB_SETCURSEL, 0, 0);
	g_syncUi.groupId = 0;

	CreateWindow(TEXT("BUTTON"), TEXT("파라미터"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 500, 10, 520, 250, h, 0, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Servo On/Off Sync"), WS_CHILD | WS_VISIBLE, 510, 40, 120, 22, h, 0, 0, 0);
	HWND cb1 = CreateWindow(TEXT("COMBOBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 640, 36, 100, 200, h, (HMENU)ID_SYNC_PARAM_SERVO_ONOFF, 0, 0);
	SendMessage(cb1, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("Disabled"));
	SendMessage(cb1, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("Enabled"));
	SendMessage(cb1, CB_SETCURSEL, 1, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Startup Type"), WS_CHILD | WS_VISIBLE, 510, 70, 120, 22, h, 0, 0, 0);
	HWND cb2 = CreateWindow(TEXT("COMBOBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 640, 66, 150, 200, h, (HMENU)ID_SYNC_PARAM_STARTUP, 0, 0);
	SendMessage(cb2, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("Normal"));
	SendMessage(cb2, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("CatchUp"));
	SendMessage(cb2, CB_SETCURSEL, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Cycle Ratio [0..10]"), WS_CHILD | WS_VISIBLE, 510, 100, 140, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("1"), WS_CHILD | WS_VISIBLE | WS_BORDER, 660, 96, 80, 24, h, (HMENU)ID_SYNC_PARAM_CYCLE_RATIO, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Max Catch Up Dist [p]"), WS_CHILD | WS_VISIBLE, 510, 130, 140, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 660, 126, 80, 24, h, (HMENU)ID_SYNC_PARAM_MAX_CATCH_UP, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("CatchUp Vel [p/s]"), WS_CHILD | WS_VISIBLE, 760, 100, 130, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 900, 96, 100, 24, h, (HMENU)ID_SYNC_PARAM_CATCHUP_VEL, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("CatchUp Acc [p/s^2]"), WS_CHILD | WS_VISIBLE, 760, 130, 130, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 900, 126, 100, 24, h, (HMENU)ID_SYNC_PARAM_CATCHUP_ACC, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Sync Err Tol [p]"), WS_CHILD | WS_VISIBLE, 510, 160, 140, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("1000"), WS_CHILD | WS_VISIBLE | WS_BORDER, 660, 156, 80, 24, h, (HMENU)ID_SYNC_PARAM_TOLERANCE, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Master Desync Dec [p/s^2]"), WS_CHILD | WS_VISIBLE, 760, 160, 160, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("10000"), WS_CHILD | WS_VISIBLE | WS_BORDER, 930, 156, 80, 24, h, (HMENU)ID_SYNC_PARAM_MASTER_DESYNC_DEC, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Slave Desync Dec [p/s^2]"), WS_CHILD | WS_VISIBLE, 760, 190, 160, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("10000"), WS_CHILD | WS_VISIBLE | WS_BORDER, 930, 186, 80, 24, h, (HMENU)ID_SYNC_PARAM_SLAVE_DESYNC_DEC, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Use Master FB"), WS_CHILD | WS_VISIBLE, 510, 190, 120, 22, h, 0, 0, 0);
	HWND cb3 = CreateWindow(TEXT("COMBOBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST, 640, 186, 100, 200, h, (HMENU)ID_SYNC_PARAM_USE_MASTER_FB, 0, 0);
	SendMessage(cb3, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("Disabled"));
	SendMessage(cb3, CB_ADDSTRING, 0, (LPARAM)(LPCTSTR)TEXT("Enabled"));
	SendMessage(cb3, CB_SETCURSEL, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("Enable"), WS_CHILD | WS_VISIBLE, 510, 220, 80, 26, h, (HMENU)ID_SYNC_PARAM_ENABLE, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Disable"), WS_CHILD | WS_VISIBLE, 600, 220, 80, 26, h, (HMENU)ID_SYNC_PARAM_DISABLE, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Refresh"), WS_CHILD | WS_VISIBLE, 690, 220, 80, 26, h, (HMENU)ID_SYNC_PARAM_REFRESH, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("Control"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, 460, 1010, 260, h, 0, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Master Axis"), WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_GROUP | WS_TABSTOP, 20, 490, 110, 22, h, (HMENU)ID_SYNC_RAD_MASTER, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Sync Axis"), WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON, 140, 490, 100, 22, h, (HMENU)ID_SYNC_RAD_SLAVE, 0, 0);
	SendMessage(GetDlgItem(h, ID_SYNC_RAD_MASTER), BM_SETCHECK, BST_CHECKED, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Axis#"), WS_CHILD | WS_VISIBLE, 250, 490, 40, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 300, 486, 50, 24, h, (HMENU)ID_SYNC_AXIS_SELECT, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Cmd Pos :"), WS_CHILD | WS_VISIBLE, 20, 520, 90, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 110, 516, 100, 24, h, (HMENU)ID_SYNC_POS_CMD, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Actual Pos :"), WS_CHILD | WS_VISIBLE, 220, 520, 90, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 310, 516, 100, 24, h, (HMENU)ID_SYNC_POS_ACT, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Op Status :"), WS_CHILD | WS_VISIBLE, 420, 520, 80, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("IDLE"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 500, 516, 80, 24, h, (HMENU)ID_SYNC_OPSTATE, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("Servo On"), WS_CHILD | WS_VISIBLE, 600, 514, 80, 26, h, (HMENU)ID_SYNC_BTN_SVON, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Home Start"), WS_CHILD | WS_VISIBLE, 690, 514, 80, 26, h, (HMENU)ID_SYNC_BTN_HOME, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Stop"), WS_CHILD | WS_VISIBLE, 780, 514, 60, 26, h, (HMENU)ID_SYNC_BTN_STOP, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Alarm Reset"), WS_CHILD | WS_VISIBLE, 845, 514, 100, 26, h, (HMENU)ID_SYNC_BTN_ALARM_RST, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Jog Speed"), WS_CHILD | WS_VISIBLE, 20, 555, 70, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("100000"), WS_CHILD | WS_VISIBLE | WS_BORDER, 95, 551, 80, 24, h, (HMENU)ID_SYNC_JOG_SPEED, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Accel/Decel"), WS_CHILD | WS_VISIBLE, 190, 555, 80, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("100"), WS_CHILD | WS_VISIBLE | WS_BORDER, 270, 551, 80, 24, h, (HMENU)ID_SYNC_ACC, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("100"), WS_CHILD | WS_VISIBLE | WS_BORDER, 355, 551, 80, 24, h, (HMENU)ID_SYNC_DEC, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Jerk"), WS_CHILD | WS_VISIBLE, 450, 555, 40, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0.75"), WS_CHILD | WS_VISIBLE | WS_BORDER, 490, 551, 60, 24, h, (HMENU)ID_SYNC_JERK, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Cmd Vel"), WS_CHILD | WS_VISIBLE, 560, 555, 80, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 645, 551, 80, 24, h, (HMENU)ID_SYNC_CMD_VEL, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Actual Vel"), WS_CHILD | WS_VISIBLE, 740, 555, 80, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 825, 551, 80, 24, h, (HMENU)ID_SYNC_ACT_VEL, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("JOG CCW"), WS_CHILD | WS_VISIBLE, 915, 545, 80, 30, h, (HMENU)ID_SYNC_BTN_JOG_CCW, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("JOG CW"), WS_CHILD | WS_VISIBLE, 915, 580, 80, 30, h, (HMENU)ID_SYNC_BTN_JOG_CW, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Abs Pos"), WS_CHILD | WS_VISIBLE, 20, 590, 60, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 80, 586, 100, 24, h, (HMENU)ID_SYNC_ABS_POS, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("AbsMove"), WS_CHILD | WS_VISIBLE, 185, 584, 80, 26, h, (HMENU)ID_SYNC_BTN_ABS, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Alt Target"), WS_CHILD | WS_VISIBLE, 280, 590, 70, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER, 350, 586, 100, 24, h, (HMENU)ID_SYNC_ALT_TARGET, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Apply Alt→Abs"), WS_CHILD | WS_VISIBLE, 455, 584, 110, 26, h, (HMENU)ID_SYNC_BTN_APPLY_ALT, 0, 0);

	CreateWindow(TEXT("STATIC"), TEXT("Axis0 0x6063:"), WS_CHILD | WS_VISIBLE, 575, 590, 190, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("-"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 675, 586, 100, 24, h, (HMENU)ID_SYNC_ECAT_6063, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("비상정지"), WS_CHILD | WS_VISIBLE, 20, 625, 90, 26, h, (HMENU)ID_SYNC_BTN_ESTOP_TOGGLE, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("NORMAL"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 115, 627, 105, 24, h, (HMENU)ID_SYNC_TXT_ESTOP_STATE, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("동기 그룹 모니터"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, 730, 1010, 220, h, 0, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("그룹상태:"), WS_CHILD | WS_VISIBLE, 20, 760, 70, 22, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("Disabled"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 90, 756, 100, 24, h, (HMENU)ID_SYNC_STATE_ENABLED, 0, 0);
	CreateWindow(TEXT("EDIT"), TEXT("Home Not Done"), WS_CHILD | WS_VISIBLE | WS_BORDER | ES_READONLY, 200, 756, 120, 24, h, (HMENU)ID_SYNC_STATE_HOMEDONE, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("All Servo On"), WS_CHILD | WS_VISIBLE, 350, 754, 100, 26, h, (HMENU)ID_SYNC_ALL_SERVO_ON, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("All Servo Off"), WS_CHILD | WS_VISIBLE, 455, 754, 100, 26, h, (HMENU)ID_SYNC_ALL_SERVO_OFF, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Home Start"), WS_CHILD | WS_VISIBLE, 560, 754, 90, 26, h, (HMENU)ID_SYNC_GROUP_HOME, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Clear Error"), WS_CHILD | WS_VISIBLE, 655, 754, 90, 26, h, (HMENU)ID_SYNC_GROUP_CLEAR, 0, 0);

	CreateWindow(TEXT("BUTTON"), TEXT("Fastech Manual"), WS_CHILD | WS_VISIBLE, 870, 36, 120, 24, h, (HMENU)ID_SYNC_BTN_FASTECH_MANUAL, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Welcon Manual"), WS_CHILD | WS_VISIBLE, 870, 66, 120, 24, h, (HMENU)ID_SYNC_BTN_WELCON_MANUAL, 0, 0);

	HWND hAxisList = CreateWindow(WC_LISTVIEW, TEXT(""), WS_CHILD | WS_VISIBLE | WS_BORDER | LVS_REPORT | LVS_SINGLESEL,
		20, 790, 980, 150, h, (HMENU)ID_SYNC_AXIS_LIST, 0, 0);
	ListView_SetExtendedListViewStyle(hAxisList, LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);
	Sync_CreateAxisListColumns(hAxisList);

	Sync_LoadGroupParamsToUI(h);
	UpdateEStopUi(h, true);
}

static LRESULT CALLBACK SyncWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	switch (msg) {
	case WM_CREATE:
		Sync_CreateUI(hWnd);
		SetTimer(hWnd, ID_SYNC_TIMER, 100, nullptr);
		return 0;

	case WM_COMMAND:
	{
		int id = LOWORD(wParam);

		if (id == ID_SYNC_GROUP_COMBO && HIWORD(wParam) == CBN_SELCHANGE) {
			int sel = (int)SendMessage((HWND)lParam, CB_GETCURSEL, 0, 0);
			g_syncUi.groupId = sel;
			Sync_LoadGroupParamsToUI(hWnd);
			return 0;
		}
		if (id == ID_SYNC_PARAM_REFRESH) { Sync_LoadGroupParamsToUI(hWnd); return 0; }

		if (id >= ID_SYNC_MASTER_AXIS_BTN_BASE && id < ID_SYNC_MASTER_AXIS_BTN_BASE + 64) {
			for (int a : g_syncUi.detectedAxes) {
				SendMessage(GetDlgItem(hWnd, ID_SYNC_MASTER_AXIS_BTN_BASE + a), BM_SETCHECK, BST_UNCHECKED, 0);
			}
			int ax = id - ID_SYNC_MASTER_AXIS_BTN_BASE;
			SendMessage(GetDlgItem(hWnd, id), BM_SETCHECK, BST_CHECKED, 0);
			g_syncUi.masterAxis = ax;
			return 0;
		}

		if (id == ID_SYNC_PARAM_ENABLE) {
			if (Sync_ApplyGroupParamsFromUI(hWnd)) Sync_EnableGroup(hWnd, true);
			return 0;
		}
		if (id == ID_SYNC_PARAM_DISABLE) { Sync_EnableGroup(hWnd, false); return 0; }

		if (id == ID_SYNC_ALL_SERVO_ON) { Sync_AllServoOnOff(true); return 0; }
		if (id == ID_SYNC_ALL_SERVO_OFF) { Sync_AllServoOnOff(false); return 0; }
		if (id == ID_SYNC_GROUP_CLEAR) { Sync_ClearGroupError(); return 0; }
		if (id == ID_SYNC_GROUP_HOME) {
			Sync::SyncGroup grp{}; if (g_cm.sync->GetSyncGroup(g_syncUi.groupId, &grp) == ErrorCode::None) {
				int ax = grp.masterAxis;
				EnsureServoOn(ax);
				if (EnterPosMode(ax)) g_home.StartHome(ax);
			}
			return 0;
		}

		if (id == ID_SYNC_BTN_SVON) {
			int ax = Sync_GetControlAxis(hWnd);
			EnsureServoOn(ax); EnsurePosModeNoStop(ax);
			return 0;
		}
		if (id == ID_SYNC_BTN_HOME) {
			int ax = Sync_GetControlAxis(hWnd);
			EnsureServoOn(ax); if (EnterPosMode(ax)) g_home.StartHome(ax);
			return 0;
		}
		if (id == ID_SYNC_BTN_STOP) {
			int ax = Sync_GetControlAxis(hWnd);
			Sync_Control_Stop(hWnd, ax);
			return 0;
		}
		if (id == ID_SYNC_BTN_ALARM_RST) {
			int ax = Sync_GetControlAxis(hWnd);
			g_cm.axisControl->ClearAmpAlarm(ax);
			return 0;
		}
		if (id == ID_SYNC_BTN_JOG_CCW) { Sync_Control_Jog(hWnd, -1); return 0; }
		if (id == ID_SYNC_BTN_JOG_CW) { Sync_Control_Jog(hWnd, +1); return 0; }
		if (id == ID_SYNC_BTN_ABS) { Sync_Control_Abs(hWnd); return 0; }
		if (id == ID_SYNC_BTN_REL_P) { Sync_Control_Rel(hWnd, +1); return 0; }
		if (id == ID_SYNC_BTN_REL_M) { Sync_Control_Rel(hWnd, -1); return 0; }

		if (id == ID_SYNC_BTN_APPLY_ALT) {
			int ecat6063 = 0;
			bool ok = ReadAxis0_TxPDO_6063(ecat6063);
			double alt = GetDlgDouble(hWnd, ID_SYNC_ALT_TARGET, 0.0);
			if (!ok) {
				MessageBox(hWnd, TEXT("0x6063 값을 읽을 수 없습니다."), TEXT("Sync Apply Alt→Abs"), MB_ICONWARNING);
				return 0;
			}
			double tgt = alt - (double)ecat6063;
			SetDlgDouble(hWnd, ID_SYNC_ABS_POS, tgt);
			return 0;
		}

		if (id == ID_SYNC_RAD_MASTER || id == ID_SYNC_RAD_SLAVE) {
			if (id == ID_SYNC_RAD_MASTER) {
				SendMessage(GetDlgItem(hWnd, ID_SYNC_RAD_MASTER), BM_SETCHECK, BST_CHECKED, 0);
				SendMessage(GetDlgItem(hWnd, ID_SYNC_RAD_SLAVE), BM_SETCHECK, BST_UNCHECKED, 0);
			}
			else {
				SendMessage(GetDlgItem(hWnd, ID_SYNC_RAD_MASTER), BM_SETCHECK, BST_UNCHECKED, 0);
				SendMessage(GetDlgItem(hWnd, ID_SYNC_RAD_SLAVE), BM_SETCHECK, BST_CHECKED, 0);
			}
			return 0;
		}

		if (id == ID_SYNC_BTN_FASTECH_MANUAL) { ShowErrorManual(hWnd, true); return 0; }
		if (id == ID_SYNC_BTN_WELCON_MANUAL) { ShowErrorManual(hWnd, false); return 0; }

		if (id == ID_SYNC_BTN_ESTOP_TOGGLE) { DoToggleEStop(hWnd, true); return 0; }

	}
	return 0;

	case WM_TIMER:
		if (wParam == ID_SYNC_TIMER) {
			if (g_commStarted) {
				g_cm.GetStatus(&g_status);

				int ax = Sync_GetControlAxis(hWnd);
				const auto& a = g_status.axesStatus[ax];
				TCHAR b[64];
				_stprintf_s(b, TEXT("%lld"), (long long)a.posCmd); SetWindowText(GetDlgItem(hWnd, ID_SYNC_POS_CMD), b);
				_stprintf_s(b, TEXT("%lld"), (long long)a.actualPos); SetWindowText(GetDlgItem(hWnd, ID_SYNC_POS_ACT), b);
				const TCHAR* ops = (std::abs((int)std::lround(a.actualVelocity)) > vel_idle_threshold) ? TEXT("MOTION") : TEXT("IDLE");
				SetWindowText(GetDlgItem(hWnd, ID_SYNC_OPSTATE), (LPTSTR)ops);
				_stprintf_s(b, TEXT("%d"), (int)std::lround(a.velocityCmd)); SetWindowText(GetDlgItem(hWnd, ID_SYNC_CMD_VEL), b);
				_stprintf_s(b, TEXT("%d"), (int)std::lround(a.actualVelocity)); SetWindowText(GetDlgItem(hWnd, ID_SYNC_ACT_VEL), b);

				if (HWND h6063 = GetDlgItem(hWnd, ID_SYNC_ECAT_6063)) {
					int val = 0;
					if (ReadAxis0_TxPDO_6063(val)) {
						TCHAR t[64]; _stprintf_s(t, TEXT("%d"), val);
						SetWindowText(h6063, t);
					}
					else {
						SetWindowText(h6063, TEXT("-"));
					}
				}

				Sync_UpdateMonitor(hWnd);
			}
			UpdateEStopUi(hWnd, true);
		}
		return 0;

	case WM_CLOSE:
		DestroyWindow(hWnd);
		return 0;

	case WM_DESTROY:
		KillTimer(hWnd, ID_SYNC_TIMER);
		//DisableAllEnabledSyncGroups();
		g_hSyncWnd = nullptr;
		return 0;
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}

// ------------------ Demo runners ------------------
struct DemoFlags {
	bool forwardTravelDone = false;
	bool hoistDownDone = false;
	bool gripCloseDone = false;
	bool hoistUpDone = false;
	bool backwardTravelDone = false;
	bool hoistDown2Done = false;
	bool gripOpenDone = false;
	bool hoistUp2Done = false;
};

static bool ExclusiveStopExcept(const std::vector<int>& allowed, DWORD timeout_ms = 5000) {
	for (int a = 0; a < 4; ++a) if (!std::count(allowed.begin(), allowed.end(), a)) StopAxis(a);
	std::vector<int> waitAxes; waitAxes.reserve(4);
	for (int a = 0; a < 4; ++a) if (!std::count(allowed.begin(), allowed.end(), a)) waitAxes.push_back(a);
	if (!waitAxes.empty()) return WaitAxesIdle(waitAxes, timeout_ms);
	return true;
}

static void DisableAllEnabledSyncGroups() {
	if (!g_commStarted) return;

	for (int gid = 0; gid < 4; ++gid) {
		Sync::SyncGroupStatus s{};
		if (g_cm.sync->GetSyncGroupStatus(gid, &s) == ErrorCode::None && s.enabled) {
			g_cm.sync->EnableSyncGroup(gid, 0);
			Sleep(5);
		}

		Sync::SyncGroup grp{};
		if (g_cm.sync->GetSyncGroup(gid, &grp) == ErrorCode::None) {
			auto setPos = [&](int ax) {
				g_cm.axisControl->SetAxisCommandMode(ax, AxisCommandMode::Position);
				};
			setPos(grp.masterAxis);
			for (int i = 0; i < (int)grp.slaveAxisCount; ++i)
				setPos(grp.slaveAxis[i]);
		}

		g_cm.sync->ClearSyncGroupError(gid);
	}
}

static void DemoPulseProc(HWND hWnd) {
	DisableAllEnabledSyncGroups();
	SetMotionAnnounce(hWnd, TEXT("Motion: 데모 시작"));
	g_demoStage = DemoStage::StartAtZero;
	g_demoKind = DemoKind::PulseZones;

	for (int a = 0; a < 4; ++a) { if (!EnsureServoOn(a) || !EnsurePosModeNoStop(a)) { SetMotionAnnounce(hWnd, TEXT("Motion: 준비 실패")); g_demoRunning = false; g_demoKind = DemoKind::None; return; } }
	if (!g_demoRunning.load()) { g_demoKind = DemoKind::None; return; }

	if (!EnsureStartAtZero(hWnd) || !g_demoRunning.load()) { SetMotionAnnounce(hWnd, TEXT("Motion: 초기 보정 실패/중단")); g_demoRunning = false; g_demoKind = DemoKind::None; return; }

	g_zoneInit = false;
	UpdateZoneAnnounce_Pulse(hWnd);

	DemoFlags f;
	if (!g_demoRunning.load()) { g_demoKind = DemoKind::None; return; }

	if (!(f.forwardTravelDone = Travel_0_to_5_Pulse(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; return; }
	if (!(f.hoistDownDone = Hoist_Down_100k(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; return; }
	if (!(f.gripCloseDone = Gripper_Close_10k(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; return; }
	if (!(f.hoistUpDone = Hoist_Up_100k_to_Zero(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; return; }
	if (!(f.backwardTravelDone = Travel_5_to_0_Pulse(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; return; }
	if (!(f.hoistDown2Done = Hoist_Down_100k(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; return; }
	if (!(f.gripOpenDone = Gripper_Open_to_Zero(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; return; }
	if (!(f.hoistUp2Done = Hoist_Up_100k_to_Zero(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; return; }

	if (g_demoRunning.load()) {
		g_cm.GetStatus(&g_status);
		bool allZero = true;
		for (int a = 0; a < 4; ++a) if (std::llabs((long long)g_status.axesStatus[a].actualPos) > inpos_tol_counts) { allZero = false; break; }
		if (!allZero) {
			double v0 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(0), 20000.0);
			double v1 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(1), 20000.0);
			double v2 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(2), 15000.0);
			double v3 = GetDlgDouble(hWnd, ID_EDIT_VEL_A(3), 8000.0);
			double t = 100.0;
			StartAbsMoveWithProfile(0, 0, v0, t, t);
			StartAbsMoveWithProfile(1, 0, v1, t, t);
			StartAbsMoveWithProfile(2, 0, v2, t, t);
			StartAbsMoveWithProfile(3, 0, v3, t, t);
			WaitAxesToTargets(hWnd, { {0,0},{1,0},{2,0},{3,0} }, 30000, false);
		}
	}

	for (int a = 0; a < 4; ++a) StopAxis(a);
	WaitAxesIdle({ 0,1,2,3 }, 5000);

	g_demoStage = DemoStage::Complete;
	SetMotionAnnounce(hWnd, TEXT("Motion: 데모 완료 (모든 축 0, 정지)"));
	g_demoKind = DemoKind::None;
	g_demoRunning = false;
}

static void DemoSensorProc(HWND hWnd) {
	DisableAllEnabledSyncGroups();
	SetMotionAnnounce(hWnd, TEXT("Motion: 데모2(센서) 시작"));
	g_demoStage = DemoStage::StartAtZero;
	g_demoKind = DemoKind::SensorZones;

	for (int a = 0; a < 4; ++a) { if (!EnsureServoOn(a) || !EnsurePosModeNoStop(a)) { SetMotionAnnounce(hWnd, TEXT("Motion: 준비 실패")); g_demoRunning = false; g_demoKind = DemoKind::None; return; } }
	if (!g_demoRunning.load()) { g_demoKind = DemoKind::None; return; }

	if (!EnsureStartAtZero(hWnd) || !g_demoRunning.load()) { SetMotionAnnounce(hWnd, TEXT("Motion: 초기 보정 실패/중단")); g_demoRunning = false; g_demoKind = DemoKind::None; return; }

	g_sensorZoneCount.store(0);
	UpdateZoneAnnounce_Sensor(hWnd);

	DemoFlags f;

	if (!(f.forwardTravelDone = Travel_0_to_5_Sensor(hWnd, FORWARD_TARGET_COUNTS, 120000)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; goto DEMO2_EXIT; }
	if (!(f.hoistDownDone = Hoist_Down_100k(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; goto DEMO2_EXIT; }
	if (!(f.gripCloseDone = Gripper_Close_10k(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; goto DEMO2_EXIT; }
	if (!(f.hoistUpDone = Hoist_Up_100k_to_Zero(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; goto DEMO2_EXIT; }
	if (!(f.backwardTravelDone = Travel_5_to_0_Sensor(hWnd, BACKWARD_TARGET_COUNTS, 120000)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; goto DEMO2_EXIT; }
	if (!(f.hoistDown2Done = Hoist_Down_100k(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; goto DEMO2_EXIT; }
	if (!(f.gripOpenDone = Gripper_Open_to_Zero(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; goto DEMO2_EXIT; }
	if (!(f.hoistUp2Done = Hoist_Up_100k_to_Zero(hWnd)) || !g_demoRunning.load()) { g_demoRunning = false; g_demoKind = DemoKind::None; goto DEMO2_EXIT; }

DEMO2_EXIT:
	for (int a = 0; a < 4; ++a) StopAxis(a);
	WaitAxesIdle({ 0,1,2,3 }, 5000);

	if (g_demoRunning.load()) {
		g_demoStage = DemoStage::Complete;
		SetMotionAnnounce(hWnd, TEXT("Motion: 데모2 완료 (모든 축 정지)"));
	}
	else {
		g_demoStage = DemoStage::Idle;
		SetMotionAnnounce(hWnd, TEXT("Motion: 데모2 중단 (모든 축 정지)"));
	}
	g_demoKind = DemoKind::None;
	g_demoRunning = false;
}

static void Demo0Proc(HWND hWnd) {
	DisableAllEnabledSyncGroups();
	SetMotionAnnounce(hWnd, TEXT("Motion: 데모0 시작 (연속 주행, Position 모드 반복 타깃)"));
	g_demoKind = DemoKind::Demo0;

	for (int a = 0; a < 2; ++a) {
		if (!EnsureServoOn(a) || !EnsurePosModeNoStop(a)) {
			SetMotionAnnounce(hWnd, TEXT("Motion: 준비 실패"));
			g_demoRunning = false; g_demoKind = DemoKind::None; return;
		}
	}
	if (!ExclusiveStopExcept({ 0,1 })) { g_demoRunning = false; g_demoKind = DemoKind::None; return; }

	double vpps_travel = GetDlgDouble(hWnd, ID_EDIT_VEL_A(0), 20000.0);
	double tAcc = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(0), 100.0);
	double tDec = GetDlgDouble(hWnd, ID_EDIT_DECT_A(0), 100.0);

	const long long BIG_STEP = 900000000LL;
	const long long RETARGET_BUFFER = 200000LL;

	long long tgt0 = 0, tgt1 = 0;

	auto issueNextTargetsForward = [&]() {
		g_cm.GetStatus(&g_status);
		long long cur0 = (long long)g_status.axesStatus[0].actualPos;
		long long cur1 = (long long)g_status.axesStatus[1].actualPos;

		tgt0 = cur0 + BIG_STEP;
		tgt1 = cur1 + (BIG_STEP * TRAVEL_AXIS1_SIGN);
		StartAbsMoveWithProfile(0, tgt0, vpps_travel, tAcc, tDec);
		StartAbsMoveWithProfile(1, tgt1, vpps_travel, tAcc, tDec);
		};

	issueNextTargetsForward();
	SetMotionAnnounce(hWnd, TEXT("Motion: Demo0 연속 이동 시작"));

	while (g_demoRunning.load()) {
		g_cm.GetStatus(&g_status);
		long long act0 = (long long)g_status.axesStatus[0].actualPos;
		long long act1 = (long long)g_status.axesStatus[1].actualPos;

		long long rem0 = llabs(tgt0 - act0);
		long long rem1 = llabs(tgt1 - act1);

		if (rem0 <= RETARGET_BUFFER || rem1 <= RETARGET_BUFFER) {
			issueNextTargetsForward();
		}
		Sleep(5);
	}

	for (int a = 0; a < 2; ++a) StopAxis(a);
	WaitAxesIdle({ 0,1 }, 5000);

	SetMotionAnnounce(hWnd, TEXT("Motion: 데모0 종료 (주행축 정지)"));
	g_demoKind = DemoKind::None;
	g_demoRunning = false;
}

// ------------------ 상태 갱신 ------------------
static void UpdateStatus(HWND hWnd) {
	for (int a = 0; a < 4; ++a) {
		const auto& ax = g_status.axesStatus[a];
		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 0))) SetWindowText(h, ax.servoOn ? (TCHAR*)TEXT("ON") : (TCHAR*)TEXT("OFF"));

		long long perr = (long long)ax.posCmd - (long long)ax.actualPos;
		bool inpos = std::llabs(perr) <= inpos_tol_counts;
		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 1))) SetWindowText(h, inpos ? (TCHAR*)TEXT("IN POSITION") : (TCHAR*)TEXT("OUT OF POSITION"));

		int actVel = (int)std::lround(ax.actualVelocity);
		bool motioning = (std::abs(actVel) > vel_idle_threshold);
		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 2))) SetWindowText(h, motioning ? (TCHAR*)TEXT("MOTION") : (TCHAR*)TEXT("IDLE"));

		TCHAR buf[64];
		_stprintf_s(buf, TEXT("%lld"), (long long)ax.posCmd); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 3))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%lld"), (long long)ax.actualPos); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 4))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%d"), (int)std::lround(ax.velocityCmd)); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 5))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%d"), actVel); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 6))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%d"), (int)std::lround(ax.torqueCmd)); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 7))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%d"), (int)std::lround(ax.actualTorque)); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 8))) SetWindowText(h, buf);
		_stprintf_s(buf, TEXT("%lld"), perr); if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 9))) SetWindowText(h, buf);

		bool ampAlm = false;
		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 10))) SetWindowText(h, ampAlm ? (TCHAR*)TEXT("ALARM") : (TCHAR*)TEXT("OK"));

		if (HWND h = GetDlgItem(hWnd, ID_TXT_STATUS(a, 11))) {
			int err603f = 0;
			bool ok603f = ReadAxis_TxPDO_603F(kAxisSlaveId[a], err603f);
			if (ok603f) {
				TCHAR t[64]; _stprintf_s(t, TEXT("0x%04X"), (unsigned)(err603f & 0xFFFF));
				SetWindowText(h, t);
			}
			else {
				SetWindowText(h, TEXT("-"));
			}
		}
	}

	if (HWND hTxt = GetDlgItem(hWnd, ID_TXT_ECAT_6063)) {
		int val = 0;
		if (ReadAxis0_TxPDO_6063(val)) {
			TCHAR t[64]; _stprintf_s(t, TEXT("%d"), val);
			SetWindowText(hTxt, t);
		}
		else {
			SetWindowText(hTxt, TEXT("-"));
		}
	}
	if (HWND hTxt = GetDlgItem(hWnd, ID_TXT_ECAT_603F)) {
		int val = 0;
		if (ReadAxis_TxPDO_603F(kAxisSlaveId[0], val)) {
			TCHAR t[64]; _stprintf_s(t, TEXT("0x%04X"), (unsigned)(val & 0xFFFF));
			SetWindowText(hTxt, t);
		}
		else {
			SetWindowText(hTxt, TEXT("-"));
		}
	}
	if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_LIMIT)) {
		SetWindowText(h, g_ax2ServoReady ? (g_ax2LimitOn ? TEXT("ON") : TEXT("OFF")) : TEXT("WAIT"));
	}
	if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_HOME)) {
		SetWindowText(h, g_ax2ServoReady ? (g_ax2HomeOn ? TEXT("ON") : TEXT("OFF")) : TEXT("WAIT"));
	}

	if (HWND h = GetDlgItem(hWnd, ID_TXT_MODE_STATE)) {
		SetWindowText(h, g_autoMode ? TEXT("MODE: AUTO") : TEXT("MODE: MANUAL"));
	}

	UpdateEStopUi(hWnd, false);
	UpdateTcpUiState(hWnd);
}

void EnsureDirW(const wchar_t* path) {
	if (!path || !path[0]) return;
	// CreateDirectoryW는 하위 한 단계만 만들 수 있음. 전체 트리를 보장하기 위해 분해 생성.
	wchar_t tmp[MAX_PATH];
	wcsncpy_s(tmp, path, _TRUNCATE);
	size_t len = wcslen(tmp);
	for (size_t i = 0; i < len; ++i) {
		if (tmp[i] == L'/' || tmp[i] == L'\\') {
			wchar_t c = tmp[i];
			tmp[i] = 0;
			if (wcslen(tmp) > 0) CreateDirectoryW(tmp, nullptr);
			tmp[i] = c;
		}
	}
	CreateDirectoryW(tmp, nullptr);
}

static void Serial_CreateUI(HWND h)
{
	CreateWindow(TEXT("BUTTON"), TEXT("PLC TCP Monitor"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, 10, 765, 340, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("Bind IP"), WS_CHILD | WS_VISIBLE, 20, 40, 60, 20, h, 0, 0, 0);
	CreateWindow(TEXT("EDIT"), g_tcpBindIp, WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL, 80, 36, 120, 24, h, (HMENU)ID_EDIT_TCP_IP, 0, 0);
	CreateWindow(TEXT("STATIC"), TEXT("Port"), WS_CHILD | WS_VISIBLE, 210, 40, 40, 20, h, 0, 0, 0);
	TCHAR portText[16]; _stprintf_s(portText, TEXT("%d"), g_tcpBindPort);
	CreateWindow(TEXT("EDIT"), portText, WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL, 250, 36, 70, 24, h, (HMENU)ID_EDIT_TCP_PORT, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Start TCP"), WS_CHILD | WS_VISIBLE, 330, 35, 90, 26, h, (HMENU)ID_BTN_TCP_START, 0, 0);
	CreateWindow(TEXT("BUTTON"), TEXT("Stop TCP"), WS_CHILD | WS_VISIBLE, 430, 35, 90, 26, h, (HMENU)ID_BTN_TCP_STOP, 0, 0);
	g_hTcpLogList = CreateWindow(TEXT("LISTBOX"), TEXT(""), WS_CHILD | WS_VISIBLE | WS_BORDER | LBS_NOINTEGRALHEIGHT | WS_VSCROLL,
		20, 80, 700, 110, h, (HMENU)ID_LIST_TCP_LOG, 0, 0);
}

static LRESULT CALLBACK SerialWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg) {
	case WM_CREATE:
		Serial_CreateUI(hWnd);
		return 0;

	case WM_COMMAND:
		if (LOWORD(wParam) == ID_BTN_TCP_START) {
			wchar_t ip[64] = L"0.0.0.0";
			wchar_t portStr[32] = L"9100";
			GetWindowText(GetDlgItem(hWnd, ID_EDIT_TCP_IP), ip, 64);
			GetWindowText(GetDlgItem(hWnd, ID_EDIT_TCP_PORT), portStr, 32);
			wcsncpy_s(g_tcpBindIp, ip, _TRUNCATE);
			g_tcpBindPort = _wtoi(portStr);
			if (g_tcpBindPort <= 0) g_tcpBindPort = 9100;
			StartTcpServer();
			return 0;
		}
		if (LOWORD(wParam) == ID_BTN_TCP_STOP) {
			StopTcpServer();
			return 0;
		}
		return 0;

	case WM_SIZE:
	{
		RECT rc{}; GetClientRect(hWnd, &rc);
		int cx = rc.right - rc.left;
		int cy = rc.bottom - rc.top;
		if (HWND lb = GetDlgItem(hWnd, ID_LIST_TCP_LOG)) {
			SetWindowPos(lb, nullptr, 20, 80, cx - 40, cy - 100, SWP_NOZORDER);
		}
		return 0;
	}

	case WM_CLOSE:
		// 닫을 때 그냥 숨기기
		ShowWindow(hWnd, SW_HIDE);
		DestroyWindow(hWnd);
		return 0;

	case WM_DESTROY:
		if (g_hTcpLogList && !IsWindow(g_hTcpLogList)) g_hTcpLogList = nullptr;
		g_hSerialWnd = nullptr;
		return 0;
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}

//// ------------------ (BEGIN) LOG/SCOPE BLOCK: integrated ------------------
//// WMX3 파일 로그 채널 설정
//const unsigned int kLogCh = 0; // 채널 0 사용
//bool g_wmxLogReady = false;
//
//// 로그 폴더 보장
//void EnsureLogsFolder() {
//	EnsureDirW(L"logs");
//}
//
//// CHANGED: Unified column order and header text (+ Target6063, Now6063)
//static const char* kCsvHeaders[] = {
//	"Axis","CmdPos","ActPos","CmdVel","CmdAcc","CmdDec",
//	"ActualCurrent","ActualTorque","LoadPercent",
//	"Target6063","Now6063",
//	"Done#","CmdTime","DoneTime"
//};
//
//// Log 클래스로 파일 로그 시작/옵션 설정
//void StartDataLogFile()
//{
//	if (!g_commStarted) return;
//
//	const wchar_t* dir = L"C:\\wmx3_logfile";
//	EnsureDirW(dir);
//
//	LogFilePathW pathW{};
//	wcsncpy_s(pathW.dirPath, dir, _TRUNCATE);
//	wcsncpy_s(pathW.fileName, L"motion.csv", _TRUNCATE);
//
//	LogChannelOptions opt{};
//	opt.maxLogFileSize = 4 * 1024 * 1024; // 4MB
//	opt.maxLogFileCount = 10;
//	opt.isRotateFile = true;
//	opt.stopLoggingOnBufferOverflow = false;
//	opt.samplingTimeMilliseconds = 0;
//	opt.samplingPeriodInCycles = 0;
//	opt.precision = 6;
//	opt.isDelimInLastCol = false;
//	strcpy_s(opt.delimiter, ",");
//	opt.triggerOnCondition = 0;
//	opt.triggerOnEvent = 0;
//	opt.triggerEventID = 0;
//
//	g_log.ResetLog(kLogCh);
//	g_log.SetLogOption(kLogCh, &opt);
//	g_log.SetLogFilePath(kLogCh, &pathW);
//	g_log.SetLogHeader(kLogCh, (char**)kCsvHeaders, (unsigned)std::size(kCsvHeaders));
//	g_log.StartLog(kLogCh);
//
//	g_wmxLogReady = true;
//}
//
//// 파일 로그 중지
//void StopDataLogFile()
//{
//	if (!g_wmxLogReady) return;
//	g_log.StopLog(kLogCh);
//	g_wmxLogReady = false;
//}
//
//// 로컬 메모리 로그(리스트뷰 표시용)
//struct LogRow {
//	int axis;
//	long long cmdPos;
//	long long actPos;
//	int vel;
//	int acc;
//	int dec;
//	int actualCurrent;
//	int torqueActual;
//	int loadPercent;
//
//	// CHANGED: add barcode fields
//	long long target6063; // target barcode (intended position in 0x6063 domain)
//	int now6063;          // current barcode read from 0x6063
//
//	unsigned long doneCount;
//	ULONGLONG cmdTick;
//	ULONGLONG doneTick;
//};
//CRITICAL_SECTION g_logCs;
//std::vector<LogRow> g_logBuffer;
//
//// CSV 저장 helpers
//std::wstring FormatTickToTimestamp(ULONGLONG /*tick*/) {
//	// tick 값은 상대시간이므로 사용자 가독성을 위해 저장 시점의 로컬 시간 사용
//	SYSTEMTIME st{};
//	GetLocalTime(&st);
//	wchar_t buf[64];
//	swprintf_s(buf, L"%04d.%02d.%02d_%02d:%02d:%02d.%03d",
//		st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
//	return buf;
//}
//
//// CHANGED: 스냅샷 기반 CSV 저장, 동시성 안전 (+ barcode columns)
//bool SaveLogCSV(const wchar_t* path) {
//	// 폴더 보장
//	wchar_t dir[MAX_PATH]{};
//	wcsncpy_s(dir, path, _TRUNCATE);
//	PathRemoveFileSpecW(dir);
//	if (dir[0]) {
//		EnsureDirW(dir);
//	}
//	else {
//		EnsureLogsFolder();
//	}
//
//	// 스냅샷 생성
//	std::vector<LogRow> snap;
//	EnterCriticalSection(&g_logCs);
//	snap = g_logBuffer;
//	LeaveCriticalSection(&g_logCs);
//
//	std::ofstream f(path, std::ios::binary | std::ios::trunc);
//	if (!f.is_open()) {
//		return false;
//	}
//
//	// UTF-8 BOM
//	const unsigned char bom[3] = { 0xEF,0xBB,0xBF };
//	f.write((const char*)bom, 3);
//
//	auto ws2utf8 = [](const std::wstring& ws) {
//		if (ws.empty()) return std::string();
//		int sz = WideCharToMultiByte(CP_UTF8, 0, ws.c_str(), -1, nullptr, 0, nullptr, nullptr);
//		if (sz <= 1) return std::string();
//		std::string out; out.resize(sz - 1);
//		WideCharToMultiByte(CP_UTF8, 0, ws.c_str(), -1, out.data(), sz, nullptr, nullptr);
//		return out;
//		};
//
//	auto writeQuoted = [&](const std::string& s) {
//		f.put('"');
//		for (char c : s) {
//			if (c == '"') f.put('"');
//			f.put(c);
//		}
//		f.put('"');
//		};
//
//	// header
//	for (size_t i = 0; i < std::size(kCsvHeaders); ++i) {
//		if (i) f << ",";
//		f << kCsvHeaders[i];
//	}
//	f << "\r\n";
//
//	// rows
//	for (const auto& r : snap) {
//		std::string sCmd = ws2utf8(FormatTickToTimestamp(r.cmdTick));
//		std::string sDone = ws2utf8(r.doneTick ? FormatTickToTimestamp(r.doneTick) : L"-");
//
//		f << r.axis << ","
//			<< r.cmdPos << ","
//			<< r.actPos << ","
//			<< r.vel << ","
//			<< r.acc << ","
//			<< r.dec << ","
//			<< r.actualCurrent << ","
//			<< r.torqueActual << ","
//			<< r.loadPercent << ","
//			<< r.target6063 << ","
//			<< r.now6063 << ","
//			<< r.doneCount << ",";
//		writeQuoted(sCmd);
//		f << ",";
//		writeQuoted(sDone);
//		f << "\r\n";
//	}
//
//	f.close();
//	return true;
//}
//
//// 명령 시작시 기록 도우미
//bool StartAbsMoveWithProfile_LogTrack(int axis, long long target, double vpps, double tAcc, double tDec) {
//	bool ok = StartAbsMoveWithProfile(axis, target, vpps, tAcc, tDec);
//	if (ok) {
//		g_axisCmdInfo[axis].axis = axis;
//		g_axisCmdInfo[axis].target = target;
//		g_axisCmdInfo[axis].vel = (int)std::lround(vpps);
//		g_axisCmdInfo[axis].acc = TimeMsToAcc(vpps, tAcc);
//		g_axisCmdInfo[axis].dec = TimeMsToAcc(vpps, tDec);
//		g_axisCmdInfo[axis].startTick = GetTickCount64();
//		g_axisCmdInfo[axis].active = true;
//		g_axisCmdInfo[axis].endTick = 0;
//
//		// CHANGED: resume logging
//		g_axisLogEnabled[axis] = true;
//		g_axisLogRowIdx[axis] = 0;
//	}
//	return ok;
//}
//#define StartAbsMoveWithProfile(axis, target, v,a,d) StartAbsMoveWithProfile_LogTrack(axis, target, v,a,d)
//
//// Torque를 간이 부하율[%]로 환산(예시)
//int TorqueToLoadPercent(int torqueActual)
//{
//	double pct = (double)torqueActual / 100.0;
//	if (pct > 100.0) pct = 100.0;
//	if (pct < -100.0) pct = -100.0;
//	return (int)std::lround(pct);
//}
//
//// Log 클래스로 파일에 1줄 푸시 (same order + barcode)
//void PushOneRecordToFile(const LogRow& r)
//{
//	if (!g_wmxLogReady) return;
//
//	auto ws2utf8 = [](const std::wstring& ws) {
//		if (ws.empty()) return std::string();
//		int sz = WideCharToMultiByte(CP_UTF8, 0, ws.c_str(), -1, nullptr, 0, nullptr, nullptr);
//		if (sz <= 1) return std::string();
//		std::string out; out.resize(sz - 1);
//		WideCharToMultiByte(CP_UTF8, 0, ws.c_str(), -1, out.data(), sz, nullptr, nullptr);
//		return out;
//		};
//	std::string sCmd = ws2utf8(FormatTickToTimestamp(r.cmdTick));
//	std::string sDone = ws2utf8(r.doneTick ? FormatTickToTimestamp(r.doneTick) : L"-");
//
//	char line[1024];
//	_snprintf_s(line, _TRUNCATE,
//		"%d,%lld,%lld,%d,%d,%d,%d,%d,%d,%lld,%d,%lu,\"%s\",\"%s\"\r\n",
//		r.axis, r.cmdPos, r.actPos, r.vel, r.acc, r.dec,
//		r.actualCurrent, r.torqueActual, r.loadPercent,
//		r.target6063, r.now6063,
//		r.doneCount, sCmd.c_str(), sDone.c_str());
//
//	g_log.SetCustomLog(kLogCh, /*moduleId*/1, (void*)line, (unsigned int)strlen(line), LogType::Log);
//}

//// Detect, sample and log with per-axis enable/disable
//void DetectAndLog()
//{
//	if (!g_commStarted) return;
//
//	g_cm.GetStatus(&g_status);
//
//	// 미리 현재 barcode(now6063) 확보(축0 기준)
//	int now6063_axis0 = 0;
//	ReadAxis0_TxPDO_6063(now6063_axis0);
//
//	for (int a = 0; a < 4; ++a) {
//		// current instant data
//		int cur = 0, trq = 0;
//		//ReadAxis_TxPDO_2181_ActualCurrent(kAxisSlaveId[a], cur);
//		//ReadAxis_TxPDO_6077_TorqueActual(kAxisSlaveId[a], trq);
//		long long actPos = (long long)g_status.axesStatus[a].actualPos;
//		long long cmdPos = (long long)g_status.axesStatus[a].posCmd;
//
//		AxisCommandInfo& ci = g_axisCmdInfo[a];
//
//		// CHANGED: Only push samples while enabled for this axis
//		if (g_axisLogEnabled[a].load()) {
//			LogRow row{};
//			row.axis = a;
//			row.cmdPos = cmdPos;
//			row.actPos = actPos;
//			row.vel = ci.vel;
//			row.acc = ci.acc;
//			row.dec = ci.dec;
//			row.actualCurrent = cur;
//			row.torqueActual = trq;
//			row.loadPercent = TorqueToLoadPercent(trq);
//			row.cmdTick = ci.startTick;
//			row.doneTick = 0;
//			row.doneCount = g_cmdDoneCount[a].load();
//
//			// CHANGED: barcode fields
//			// target6063: 축0에 한해 현재 실행 중인 명령 target을 barcode 기준 타겟으로 간주
//			// (축0이 아닐 경우 - 또는 의미 없을 경우 - 0 세팅)
//			if (a == 0) {
//				row.target6063 = ci.target.load(); // target 명령을 barcode 목표로 사용
//				row.now6063 = now6063_axis0;       // 현재 barcode
//			}
//			else {
//				row.target6063 = 0;
//				row.now6063 = now6063_axis0; // 참고용으로 동일 표기
//			}
//
//			EnterCriticalSection(&g_logCs);
//			g_logBuffer.push_back(row);
//			LeaveCriticalSection(&g_logCs);
//
//			PushOneRecordToFile(row);
//
//			g_axisLogRowIdx[a]++; // not used yet, but kept for extension
//		}
//
//		// done detection
//		if (ci.active.load()) {
//			long long tgt = ci.target.load();
//			int actVel = (int)std::lround((double)g_status.axesStatus[a].actualVelocity);
//			if (std::llabs(actPos - tgt) <= inpos_tol_counts && std::abs(actVel) <= vel_idle_threshold) {
//				ci.active = false;
//				ci.endTick = GetTickCount64();
//				unsigned long cnt = ++g_cmdDoneCount[a];
//
//				// final done record
//				LogRow doneR{};
//				doneR.axis = a;
//				doneR.cmdPos = (long long)g_status.axesStatus[a].posCmd;
//				doneR.actPos = actPos;
//				doneR.vel = ci.vel;
//				doneR.acc = ci.acc;
//				doneR.dec = ci.dec;
//				//ReadAxis_TxPDO_2181_ActualCurrent(kAxisSlaveId[a], doneR.actualCurrent);
//				//ReadAxis_TxPDO_6077_TorqueActual(kAxisSlaveId[a], doneR.torqueActual);
//				doneR.loadPercent = TorqueToLoadPercent(doneR.torqueActual);
//				doneR.cmdTick = ci.startTick;
//				doneR.doneTick = ci.endTick;
//				doneR.doneCount = cnt;
//
//				// barcode fields at done
//				int now6063_done = 0;
//				ReadAxis0_TxPDO_6063(now6063_done);
//				if (a == 0) {
//					doneR.target6063 = ci.target.load();
//					doneR.now6063 = now6063_done;
//				}
//				else {
//					doneR.target6063 = 0;
//					doneR.now6063 = now6063_done;
//				}
//
//				EnterCriticalSection(&g_logCs);
//				g_logBuffer.push_back(doneR);
//				LeaveCriticalSection(&g_logCs);
//
//				PushOneRecordToFile(doneR);
//
//				// CHANGED: Disable further sampling for this axis until new command starts
//				g_axisLogEnabled[a] = false;
//			}
//		}
//	}
//}
//
//HWND g_hLogWnd = nullptr;
//enum : int {
//	ID_LOG_LIST = 20001,
//	ID_LOG_BTN_SAVE = 20002,
//	ID_LOG_TIMER = 20003
//};
//
//std::atomic<bool> g_logThreadRun{ false };
//std::thread g_logThread;
//
//void LogThreadProc() {
//	EnsureLogsFolder();
//	while (g_logThreadRun.load()) {
//		if (g_commStarted) {
//			if (!g_wmxLogReady) StartDataLogFile();
//			DetectAndLog();
//		}
//		else {
//			if (g_wmxLogReady) StopDataLogFile();
//		}
//		Sleep(LOG_POLL_MS);
//	}
//	StopDataLogFile();
//}
//
//void LogWnd_EnsureColumns(HWND hList) {
//	if (ListView_GetColumnWidth(hList, 0) > 0) return;
//	LVCOLUMN col{};
//	col.mask = LVCF_TEXT | LVCF_WIDTH | LVCF_SUBITEM;
//	int c = 0;
//	auto addCol = [&](const wchar_t* name, int w) {
//		col.pszText = const_cast<LPWSTR>(name);
//		col.cx = w; col.iSubItem = c;
//		ListView_InsertColumn(hList, c, &col);
//		++c;
//		};
//
//	// CHANGED: order matches CSV (+ Target6063, Now6063)
//	addCol(L"Axis", 50);
//	addCol(L"CmdPos", 110);
//	addCol(L"ActPos", 110);
//	addCol(L"CmdVel", 80);
//	addCol(L"CmdAcc", 90);
//	addCol(L"CmdDec", 90);
//	addCol(L"Actual Current", 120);
//	addCol(L"Actual Torque", 120);
//	addCol(L"Load[%]", 80);
//	addCol(L"Target6063", 120);
//	addCol(L"Now6063", 100);
//	addCol(L"Done#", 70);
//	addCol(L"Cmd Time", 170);
//	addCol(L"Done Time", 170);
//}
//
//// 자동 스크롤 제어: 맨 아래를 보고 있을 때만 자동 스크롤 유지
//static bool IsListViewScrolledToBottom(HWND hList) {
//	int top = ListView_GetTopIndex(hList);
//	int perPage = ListView_GetCountPerPage(hList);
//	int count = ListView_GetItemCount(hList);
//	if (perPage <= 0 || count <= 0) return true;
//	// top + perPage 가 count 이상이면 거의 바닥
//	return (top + perPage) >= count - 1;
//}
//
//void LogWnd_AppendRows(HWND hList) {
//	static size_t shown = 0;
//
//	// 현재 자동 스크롤 가능한지 체크
//	bool autoScroll = IsListViewScrolledToBottom(hList);
//
//	// 스냅샷으로 안전하게 복사
//	std::vector<LogRow> snap;
//	EnterCriticalSection(&g_logCs);
//	if (shown < g_logBuffer.size()) {
//		snap.insert(snap.end(), g_logBuffer.begin() + shown, g_logBuffer.end());
//		shown = g_logBuffer.size();
//	}
//	LeaveCriticalSection(&g_logCs);
//
//	if (snap.empty()) return;
//
//	int baseIndex = ListView_GetItemCount(hList);
//	int idxInsert = baseIndex;
//
//	for (const LogRow& r : snap) {
//		wchar_t buf[128];
//		LVITEM it{};
//		it.mask = LVIF_TEXT; it.iItem = idxInsert; it.iSubItem = 0;
//		_snwprintf_s(buf, _TRUNCATE, L"%d", r.axis);
//		it.pszText = buf;
//		int idx = ListView_InsertItem(hList, &it);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%lld", r.cmdPos);
//		ListView_SetItemText(hList, idx, 1, buf);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%lld", r.actPos);
//		ListView_SetItemText(hList, idx, 2, buf);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%d", r.vel);
//		ListView_SetItemText(hList, idx, 3, buf);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%d", r.acc);
//		ListView_SetItemText(hList, idx, 4, buf);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%d", r.dec);
//		ListView_SetItemText(hList, idx, 5, buf);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%d", r.actualCurrent);
//		ListView_SetItemText(hList, idx, 6, buf);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%d", r.torqueActual);
//		ListView_SetItemText(hList, idx, 7, buf);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%d", r.loadPercent);
//		ListView_SetItemText(hList, idx, 8, buf);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%lld", r.target6063);
//		ListView_SetItemText(hList, idx, 9, buf);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%d", r.now6063);
//		ListView_SetItemText(hList, idx, 10, buf);
//
//		_snwprintf_s(buf, _TRUNCATE, L"%lu", r.doneCount);
//		ListView_SetItemText(hList, idx, 11, buf);
//
//		std::wstring cmdTS = FormatTickToTimestamp(r.cmdTick);
//		std::wstring doneTS = (r.doneTick != 0) ? FormatTickToTimestamp(r.doneTick) : L"-";
//		ListView_SetItemText(hList, idx, 12, const_cast<LPWSTR>(cmdTS.c_str()));
//		ListView_SetItemText(hList, idx, 13, const_cast<LPWSTR>(doneTS.c_str()));
//
//		idxInsert++;
//	}
//
//	// 자동 스크롤 상태일 때만 맨 마지막 행 보이기
//	if (autoScroll) {
//		int cnt = (int)SendMessage(hList, LVM_GETITEMCOUNT, 0, 0);
//		if (cnt > 0) SendMessage(hList, LVM_ENSUREVISIBLE, cnt - 1, FALSE);
//	}
//}
//
//LRESULT CALLBACK LogWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
//	static HWND hList = nullptr;
//	switch (msg) {
//	case WM_CREATE:
//	{
//		CreateWindow(TEXT("BUTTON"), TEXT("Motion Log"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, 10, 1500, 610, hWnd, 0, 0, 0);
//		hList = CreateWindow(WC_LISTVIEW, TEXT(""), WS_CHILD | WS_VISIBLE | WS_BORDER | LVS_REPORT | LVS_SINGLESEL,
//			20, 40, 960, 500, hWnd, (HMENU)ID_LOG_LIST, 0, 0);
//		ListView_SetExtendedListViewStyle(hList, LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);
//		LogWnd_EnsureColumns(hList);
//		CreateWindow(TEXT("BUTTON"), TEXT("Save CSV..."), WS_CHILD | WS_VISIBLE, 20, 550, 120, 28, hWnd, (HMENU)ID_LOG_BTN_SAVE, 0, 0);
//		SetTimer(hWnd, ID_LOG_TIMER, LOG_POLL_MS, nullptr);
//		return 0;
//	}
//	case WM_SIZE:
//	{
//		RECT rc{}; GetClientRect(hWnd, &rc);
//		if (hList) SetWindowPos(hList, nullptr, 20, 40, rc.right - 40, rc.bottom - 80, SWP_NOZORDER);
//		return 0;
//	}
//	case WM_TIMER:
//		if (wParam == ID_LOG_TIMER) {
//			if (hList) LogWnd_AppendRows(hList);
//			return 0;
//		}
//		break;
//	case WM_COMMAND:
//		if (LOWORD(wParam) == ID_LOG_BTN_SAVE) {
//			EnsureLogsFolder();
//			wchar_t file[MAX_PATH] = L"logs\\motion_log.csv";
//			if (SaveLogCSV(file))
//				MessageBox(hWnd, L"Saved: logs\\motion_log.csv", L"Log", MB_ICONINFORMATION);
//			else
//				MessageBox(hWnd, L"Save failed. Check folder permission or path.", L"Log", MB_ICONERROR);
//			return 0;
//		}
//		break;
//	case WM_CLOSE:
//		DestroyWindow(hWnd);
//		return 0;
//	case WM_DESTROY:
//		g_hLogWnd = nullptr;
//		return 0;
//	}
//	return DefWindowProc(hWnd, msg, wParam, lParam);
//}
//
//// Scope 창(간이 뷰어, 추후 그래프 구현 자리)
//HWND g_hScopeWnd = nullptr;
//LRESULT CALLBACK ScopeWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
//	switch (msg) {
//	case WM_CREATE:
//		CreateWindow(TEXT("STATIC"), TEXT("Scope (향후 CSV로 그래프 구현)"), WS_CHILD | WS_VISIBLE, 20, 20, 260, 24, hWnd, 0, 0, 0);
//		return 0;
//	case WM_CLOSE:
//		DestroyWindow(hWnd);
//		return 0;
//	case WM_DESTROY:
//		g_hScopeWnd = nullptr;
//		return 0;
//	}
//	return DefWindowProc(hWnd, msg, wParam, lParam);
//}
//
//// 창 표시 헬퍼
//void ShowLogWindow(HWND parent) {
//	if (!g_hLogWnd || !IsWindow(g_hLogWnd)) {
//		WNDCLASS wc{}; wc.lpszClassName = TEXT("WMX3LogWnd");
//		wc.lpfnWndProc = LogWndProc; wc.hInstance = (HINSTANCE)GetWindowLongPtr(parent, GWLP_HINSTANCE);
//		wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
//		wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
//		RegisterClass(&wc);
//		g_hLogWnd = CreateWindow(TEXT("WMX3LogWnd"), TEXT("Motion Log"),
//			WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_SIZEBOX | WS_MINIMIZEBOX,
//			CW_USEDEFAULT, CW_USEDEFAULT, 1537, 680, parent, nullptr, wc.hInstance, nullptr);
//		ShowWindow(g_hLogWnd, SW_SHOWNORMAL);
//	}
//	else {
//		ShowWindow(g_hLogWnd, SW_SHOWNORMAL);
//		SetForegroundWindow(g_hLogWnd);
//	}
//}
//
//void ShowScopeWindow(HWND parent) {
//	if (!g_hScopeWnd || !IsWindow(g_hScopeWnd)) {
//		WNDCLASS wc{}; wc.lpszClassName = TEXT("WMX3ScopeWnd");
//		wc.lpfnWndProc = ScopeWndProc; wc.hInstance = (HINSTANCE)GetWindowLongPtr(parent, GWLP_HINSTANCE);
//		wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
//		wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
//		RegisterClass(&wc);
//		g_hScopeWnd = CreateWindow(TEXT("WMX3ScopeWnd"), TEXT("Scope (Graph Viewer)"),
//			WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_SIZEBOX | WS_MINIMIZEBOX,
//			CW_USEDEFAULT, CW_USEDEFAULT, 800, 480, parent, nullptr, wc.hInstance, nullptr);
//		ShowWindow(g_hScopeWnd, SW_SHOWNORMAL);
//	}
//	else {
//		ShowWindow(g_hScopeWnd, SW_SHOWNORMAL);
//		SetForegroundWindow(g_hScopeWnd);
//	}
//}
// ------------------ (END) LOG/SCOPE BLOCK ------------------


// ==========================
// Hybrid Barcode Demo (S-curve correction + one-shot final snap under |bcErr|<=100)
// Behavior:
//  - startBcErr = round(mainVel * 0.1)
//  - When |bcErr| > startBcErr: single main move to precomputed motor target
//  - When |bcErr| <= startBcErr: repeatedly issue S-curve abs moves to currentPos + remainingPulses (using correction profile)
//  - When |bcErr| <= snapErr(100): send StartAbsMoveWithProfile ONCE to the target (currentPos + remainingPulses) using correction profile and stop issuing further commands
//  - Stop when |bcErr| <= deadband
// 30 ms sampling
// ==========================

HWND g_hBarcodeWnd = nullptr;

enum : int {
	ID_BC_TIMER = 30001,
	ID_BC_ECAT_6063_NOW = 30002,
	ID_BC_TXT_STATUS = 30030,

	ID_BC_EDIT_TARGET = 30010,
	ID_BC_EDIT_VEL = 30011,
	ID_BC_EDIT_ACC = 30012,
	ID_BC_EDIT_DEC = 30013,

	// Correction profile controls
	ID_BC_EDIT_CORR_VEL = 30101,
	ID_BC_EDIT_CORR_ACC = 30102,
	ID_BC_EDIT_CORR_DEC = 30103,

	ID_BC_EDIT_GEAR = 30040,
	ID_BC_EDIT_WHEEL_D = 30041,
	ID_BC_EDIT_MOT_CPR = 30042,
	ID_BC_EDIT_BC_MM_PER_CNT = 30043,
	ID_BC_TXT_COMPUTED_PULSES = 30050,
	ID_BC_EDIT_CORR_DEADBAND = 30106,
	ID_BC_EDIT_CORR_START_BCERR = 30107,

	ID_BC_BTN_SERVO_ON = 30020,
	ID_BC_BTN_SERVO_OFF = 30021,
	ID_BC_BTN_START = 30022,
	ID_BC_BTN_STOP = 30023,
	ID_BC_COMBO_AXIS = 30150,
};

// ======================================================
void Barcode_SetTxt(HWND h, int id, const wchar_t* s)
{
	if (HWND hh = GetDlgItem(h, id))
		SetWindowTextW(hh, s);
}
void Barcode_SetInt(HWND h, int id, int v)
{
	wchar_t b[64]; _snwprintf_s(b, _TRUNCATE, L"%d", v);
	Barcode_SetTxt(h, id, b);
}
void Barcode_SetLL(HWND h, int id, long long v)
{
	wchar_t b[64]; _snwprintf_s(b, _TRUNCATE, L"%lld", v);
	Barcode_SetTxt(h, id, b);
}
void Barcode_SetD(HWND h, int id, double v)
{
	wchar_t b[64]; _snwprintf_s(b, _TRUNCATE, L"%.3f", v);
	Barcode_SetTxt(h, id, b);
}

// =====================================================
struct HybridBarcodeState
{
	std::atomic<bool> running{ false };
	std::atomic<bool> inCorr{ false };
	std::atomic<bool> finalSnapSent{ false }; // one-shot final command flag

	int bcaxis = 0;
	int moveaxis = 1;
	long long targetBarcodeAbs = 0;   // absolute barcode(6063) target
	long long targetBarcodeRel = 0;   // relative barcode to current 6063
	long long targetMotorPulse = 0;   // pulses for targetBarcodeRel (precomputed at Start)

	// Main profile (for initial long move)
	double mainVel = 10000;           // pps
	double mainAcc = 1000;            // ms
	double mainDec = 1000;            // ms

	// Correction profile (for S-curve corrections and snap)
	double corrVel = 1000;           // pps (default same as mainVel)
	double corrAcc = 1000;             // ms
	double corrDec = 1000;             // ms

	int deadband = 2;                 // barcode cnt deadband
	int startBcErr = 500;             // overridden from mainVel * 0.1 at start

	// Conversion
	double gear = 4.3;
	double wheelDia = 70;             // mm
	double motorCpr = 10000;          // pulses per rev
	double bcMmPerCnt = 0.1;          // mm per barcode count

	bool   stopDelayActive = false;
	int    stopDelayTimer = 0;
};
HybridBarcodeState g_hbc;

// External functions/objects assumed to exist in project:
// - bool g_commStarted;
// - struct { AxisStatus axesStatus[4]; } g_status; with .actualPos (pulses), .actualVelocity (rpm)
// - Controller g_cm; with GetStatus(...), motion->Stop(int), axisControl->SetServoOn(...)
// - bool ReadAxis_TxPDO_6063(int slaveId, int& outVal);
// - int kAxisSlaveId[4];
// - void StartAbsMoveWithProfile(int axis, long long targetAbsPulse, double vel_pps, double acc_ms, double dec_ms);
// - void EnsureServoOn(int axis);
// - void EnsurePosModeNoStop(int axis);
// - void StopAxis(int axis);
// - double GetDlgDouble(HWND, int id, double def);

// =====================================================
static bool Bc_ReadSelectedAxis_6063(int& outVal)
{
	int axis = g_hbc.bcaxis;
	if (axis < 0 || axis >= 4) return false;
	return ReadAxis_TxPDO_6063(kAxisSlaveId[axis], outVal);
}

struct HbcSnapshot {
	double gear, wheelDia, motorCpr, bcMmPerCnt;
};
static inline double HBC_pulsesPerMm(const HbcSnapshot& s)
{
	return s.motorCpr / (3.14159265358979 * s.wheelDia);
}
static inline double HBC_bcToMm(const HbcSnapshot& s, long long bc)
{
	return bc * s.bcMmPerCnt;
}
static inline long long HBC_mmToPulses(const HbcSnapshot& s, double mm)
{
	double pulses = mm * HBC_pulsesPerMm(s);
	return (long long)std::llround(pulses);
}
static inline long long HBC_bcToPulses(const HbcSnapshot& s, long long bc)
{
	return HBC_mmToPulses(s, HBC_bcToMm(s, bc));
}

// Auto compute start error from mainVel: startErr = round(mainVel * 0.1)
static inline int HBC_ComputeAutoStartErr(double mainVel_pps)
{
	return (int)std::llround(mainVel_pps * 0.01);
}

// =====================================================
void HBC_Start(HWND hWnd)
{
	if (!g_commStarted) {
		MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Barcode"), MB_ICONWARNING);
		return;
	}
	g_hbc.running = false;
	g_hbc.inCorr = false;
	g_hbc.finalSnapSent = false;

	// 항상 재시작할 때 초기화
	g_hbc.stopDelayActive = false;
	g_hbc.stopDelayTimer = 0;

	int ax = g_hbc.moveaxis;

	// Read UI parameters - Main profile
	g_hbc.mainVel = GetDlgDouble(hWnd, ID_BC_EDIT_VEL, 10000);
	g_hbc.mainAcc = GetDlgDouble(hWnd, ID_BC_EDIT_ACC, 1000);
	g_hbc.mainDec = GetDlgDouble(hWnd, ID_BC_EDIT_DEC, 1000);

	// Correction profile (defaults: vel same as main, acc/dec 300 ms)
	g_hbc.corrVel = GetDlgDouble(hWnd, ID_BC_EDIT_CORR_VEL, 1000);
	g_hbc.corrAcc = GetDlgDouble(hWnd, ID_BC_EDIT_CORR_ACC, 1000);
	g_hbc.corrDec = GetDlgDouble(hWnd, ID_BC_EDIT_CORR_DEC, 1000);

	g_hbc.deadband = (int)GetDlgDouble(hWnd, ID_BC_EDIT_CORR_DEADBAND, 2);

	// Auto compute startBcErr based on mainVel (0.1x)
	g_hbc.startBcErr = HBC_ComputeAutoStartErr(g_hbc.mainVel);
	Barcode_SetInt(hWnd, ID_BC_EDIT_CORR_START_BCERR, g_hbc.startBcErr);

	// Conversion
	g_hbc.gear = GetDlgDouble(hWnd, ID_BC_EDIT_GEAR, 4.3);
	g_hbc.wheelDia = GetDlgDouble(hWnd, ID_BC_EDIT_WHEEL_D, 70);
	g_hbc.motorCpr = GetDlgDouble(hWnd, ID_BC_EDIT_MOT_CPR, 10000);
	g_hbc.bcMmPerCnt = GetDlgDouble(hWnd, ID_BC_EDIT_BC_MM_PER_CNT, 0.1);

	int now6063 = 0;
	if (!Bc_ReadSelectedAxis_6063(now6063)) {
		MessageBox(hWnd, TEXT("Barcode Read Fail (6063)"), TEXT("Barcode"), MB_ICONWARNING);
		return;
	}

	g_hbc.targetBarcodeAbs = (long long)GetDlgDouble(hWnd, ID_BC_EDIT_TARGET, 0);
	g_hbc.targetBarcodeRel = g_hbc.targetBarcodeAbs - now6063;

	HbcSnapshot s{ g_hbc.gear, g_hbc.wheelDia, g_hbc.motorCpr, g_hbc.bcMmPerCnt };
	g_hbc.targetMotorPulse = HBC_bcToPulses(s, g_hbc.targetBarcodeRel);

	g_cm.GetStatus(&g_status);
	long long curPos = g_status.axesStatus[ax].actualPos;
	long long tgt = curPos + g_hbc.targetMotorPulse;

	// First main move toward target motor position with main profile
	StartAbsMoveWithProfile(ax, tgt, g_hbc.mainVel, g_hbc.mainAcc, g_hbc.mainDec);

	g_hbc.running = true;
	g_hbc.inCorr = false;
	g_hbc.finalSnapSent = false;
}

// =====================================================
void HBC_UpdateUi(HWND hWnd)
{
	int now6063 = 0;
	if (Bc_ReadSelectedAxis_6063(now6063))
		Barcode_SetInt(hWnd, ID_BC_ECAT_6063_NOW, now6063);
	else
		Barcode_SetTxt(hWnd, ID_BC_ECAT_6063_NOW, L"-");

	long long userTarget = (long long)GetDlgDouble(hWnd, ID_BC_EDIT_TARGET, 0.0);
	g_hbc.targetBarcodeAbs = userTarget;
	long long diff = userTarget - now6063;
	g_hbc.targetBarcodeRel = diff;

	HbcSnapshot s{ g_hbc.gear, g_hbc.wheelDia, g_hbc.motorCpr, g_hbc.bcMmPerCnt };
	long long pulses = HBC_bcToPulses(s, diff);
	g_hbc.targetMotorPulse = pulses;

	Barcode_SetLL(hWnd, ID_BC_TXT_COMPUTED_PULSES, pulses);

	wchar_t st[256];
	swprintf_s(st, L"Running=%s  CorrMode=%s  FinalSnap=%s  Rel=%lld  Pulses=%lld  S=%d  DB=%d  Corr[V/A/D]=%.0f/%.0f/%.0f",
		g_hbc.running ? L"Yes" : L"No",
		g_hbc.inCorr ? L"Yes" : L"No",
		g_hbc.finalSnapSent ? L"Yes" : L"No",
		diff, pulses, g_hbc.startBcErr, g_hbc.deadband,
		g_hbc.corrVel, g_hbc.corrAcc, g_hbc.corrDec);
	Barcode_SetTxt(hWnd, ID_BC_TXT_STATUS, st);
}

void HBC_Poll(HWND hWnd)
{
	if (!g_hbc.running) return;

	const long long snapErr = 100;      // snapErr threshold
	const long long finalCheckErr = 2;  // final check threshold

	int ax = g_hbc.moveaxis;

	int now6063 = 0;
	if (!Bc_ReadSelectedAxis_6063(now6063)) return;

	long long bcErr = g_hbc.targetBarcodeAbs - now6063;
	long long eAbs = llabs(bcErr);

	// deadband 안에 있는지 여부
	bool inDeadband = (eAbs <= g_hbc.deadband);

	if (inDeadband)
	{
		// deadband 최초 진입 시 타이머 세팅
		if (!g_hbc.stopDelayActive)
		{
			g_hbc.stopDelayActive = true;

			// 새 시퀀스 시작 시 Start 쪽에서 stopDelayTimer를 0으로 초기화해 둔다고 가정
			// 0일 때만 70으로 세팅해서 "누적" 개념 유지
			if (g_hbc.stopDelayTimer == 0)
				g_hbc.stopDelayTimer = 70;   // 30ms × 70 ≒ 2.1s
		}

		// deadband 안에 있는 동안에만 타이머 감소
		if (g_hbc.stopDelayActive)
		{
			if (--g_hbc.stopDelayTimer <= 0)
			{
				g_hbc.running = false;
				g_hbc.inCorr = false;
				g_hbc.stopDelayActive = false;
				g_hbc.stopDelayTimer = 0;
				return;    // 여기서 종료
			}
		}
	}
	else
	{
		// deadband 밖: 타이머는 멈춰 있고 값만 유지 (pause)
		// g_hbc.stopDelayActive / stopDelayTimer 둘 다 건드리지 않음
	}


	// ============ ENTER CORR ============
	if (!g_hbc.inCorr && eAbs <= g_hbc.startBcErr) {
		g_hbc.inCorr = true;
	}

	if (!g_hbc.inCorr) return;

	// =========================================================
	// CASE 1) startErr 조건 만족 → 첫 번째 보정 (correction profile 사용)
	// =========================================================
	if (!g_hbc.finalSnapSent && eAbs <= g_hbc.startBcErr)
	{
		HbcSnapshot s{ g_hbc.gear, g_hbc.wheelDia, g_hbc.motorCpr, g_hbc.bcMmPerCnt };
		long long remainingPulses = HBC_bcToPulses(s, bcErr);

		g_cm.GetStatus(&g_status);
		long long curPos = g_status.axesStatus[ax].actualPos;

		long long absTarget = curPos + remainingPulses;

		// 첫 번째 보정 실행 - correction profile
		StartAbsMoveWithProfile(ax, absTarget, g_hbc.corrVel, g_hbc.corrAcc, g_hbc.corrDec);

		g_hbc.finalSnapSent = true;
		return;
	}

	// =========================================================
	// CASE 2) finalSnapSent == true → actVel == 0일 때 최종 점검 (correction profile 사용)
	// =========================================================
	if (g_hbc.finalSnapSent)
	{
		g_cm.GetStatus(&g_status);
		double actVel = fabs(g_status.axesStatus[ax].actualVelocity);

		if (actVel == 0 && eAbs > finalCheckErr)
		{
			HbcSnapshot s{ g_hbc.gear, g_hbc.wheelDia, g_hbc.motorCpr, g_hbc.bcMmPerCnt };
			long long remainingPulses = HBC_bcToPulses(s, bcErr);
			long long curPos = g_status.axesStatus[ax].actualPos;
			long long absTarget = curPos + remainingPulses;

			// 두 번째 보정 실행 - correction profile
			StartAbsMoveWithProfile(ax, absTarget, g_hbc.corrVel, g_hbc.corrAcc, g_hbc.corrDec);
		}

		// 이후에는 더 이상 명령을 보내지 않고 deadband 진입까지 대기
		return;
	}

	// =========================================================
	// CASE 3) snapErr 근처가 아닌 normal correction loop (correction profile 사용)
	// =========================================================
	HbcSnapshot s{ g_hbc.gear, g_hbc.wheelDia, g_hbc.motorCpr, g_hbc.bcMmPerCnt };
	long long remainingPulses = HBC_bcToPulses(s, bcErr);

	g_cm.GetStatus(&g_status);
	long long curPos = g_status.axesStatus[ax].actualPos;

	long long absTarget = curPos + remainingPulses;

	StartAbsMoveWithProfile(ax, absTarget, g_hbc.corrVel, g_hbc.corrAcc, g_hbc.corrDec);
}


// =====================================================
// UI and Window Proc
// =====================================================
LRESULT CALLBACK BarcodeWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
	case WM_CREATE:
	{
		// GROUP
		CreateWindow(TEXT("BUTTON"), TEXT("Hybrid Barcode Demo"),
			WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
			10, 10, 1100, 360, hWnd, 0, 0, 0);

		// AXIS
		CreateWindow(TEXT("STATIC"), TEXT("Axis"), WS_CHILD | WS_VISIBLE,
			20, 40, 40, 22, hWnd, 0, 0, 0);

		HWND hAxis = CreateWindow(TEXT("COMBOBOX"), TEXT(""),
			WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
			65, 36, 80, 200, hWnd, (HMENU)ID_BC_COMBO_AXIS, 0, 0);

		SendMessage(hAxis, CB_ADDSTRING, 0, (LPARAM)TEXT("0"));
		SendMessage(hAxis, CB_ADDSTRING, 0, (LPARAM)TEXT("1"));
		SendMessage(hAxis, CB_ADDSTRING, 0, (LPARAM)TEXT("2"));
		SendMessage(hAxis, CB_ADDSTRING, 0, (LPARAM)TEXT("3"));
		SendMessage(hAxis, CB_SETCURSEL, 1, 0);

		// NOW(6063)
		CreateWindow(TEXT("STATIC"), TEXT("Now(6063)"),
			WS_CHILD | WS_VISIBLE,
			20, 80, 100, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("0"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			125, 78, 120, 24, hWnd, (HMENU)ID_BC_ECAT_6063_NOW, 0, 0);

		// TARGET BARCODE ABS
		CreateWindow(TEXT("STATIC"), TEXT("Target Barcode(abs)"),
			WS_CHILD | WS_VISIBLE,
			20, 120, 150, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("0"),
			WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
			175, 118, 120, 24, hWnd, (HMENU)ID_BC_EDIT_TARGET, 0, 0);

		// MAIN MOVE PROFILE
		CreateWindow(TEXT("STATIC"), TEXT("Main Vel[pps]"),
			WS_CHILD | WS_VISIBLE,
			310, 120, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("10000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			400, 118, 90, 24, hWnd, (HMENU)ID_BC_EDIT_VEL, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Main Acc[ms]"),
			WS_CHILD | WS_VISIBLE,
			500, 120, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("1000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			590, 118, 90, 24, hWnd, (HMENU)ID_BC_EDIT_ACC, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Main Dec[ms]"),
			WS_CHILD | WS_VISIBLE,
			690, 120, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("1000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			780, 118, 90, 24, hWnd, (HMENU)ID_BC_EDIT_DEC, 0, 0);

		// CORRECTION PROFILE
		CreateWindow(TEXT("STATIC"), TEXT("Corr Vel[pps]"),
			WS_CHILD | WS_VISIBLE,
			310, 155, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("1000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			400, 153, 90, 24, hWnd, (HMENU)ID_BC_EDIT_CORR_VEL, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Corr Acc[ms]"),
			WS_CHILD | WS_VISIBLE,
			500, 155, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("1000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			590, 153, 90, 24, hWnd, (HMENU)ID_BC_EDIT_CORR_ACC, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Corr Dec[ms]"),
			WS_CHILD | WS_VISIBLE,
			690, 155, 90, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("1000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			780, 153, 90, 24, hWnd, (HMENU)ID_BC_EDIT_CORR_DEC, 0, 0);

		// CONVERSION PARAMS
		CreateWindow(TEXT("STATIC"), TEXT("Gear"),
			WS_CHILD | WS_VISIBLE,
			20, 155, 100, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("4.3"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			125, 153, 90, 24, hWnd, (HMENU)ID_BC_EDIT_GEAR, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("WheelDia[mm]"),
			WS_CHILD | WS_VISIBLE,
			20, 190, 100, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("70"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			125, 188, 90, 24, hWnd, (HMENU)ID_BC_EDIT_WHEEL_D, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Motor CPR"),
			WS_CHILD | WS_VISIBLE,
			230, 155, 100, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("10000"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			330, 153, 90, 24, hWnd, (HMENU)ID_BC_EDIT_MOT_CPR, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Barcode mm/cnt"),
			WS_CHILD | WS_VISIBLE,
			230, 190, 110, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("0.1"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			330, 188, 90, 24, hWnd, (HMENU)ID_BC_EDIT_BC_MM_PER_CNT, 0, 0);

		// COMPUTED PULSES
		CreateWindow(TEXT("STATIC"), TEXT("Computed Pulses"),
			WS_CHILD | WS_VISIBLE,
			20, 225, 140, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("0"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			165, 223, 150, 24, hWnd, (HMENU)ID_BC_TXT_COMPUTED_PULSES, 0, 0);

		// CORRECTION SETTINGS
		CreateWindow(TEXT("STATIC"), TEXT("Deadband(cnt)"),
			WS_CHILD | WS_VISIBLE,
			330, 225, 120, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("2"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			455, 223, 80, 24, hWnd, (HMENU)ID_BC_EDIT_CORR_DEADBAND, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("Start Corr (bcErr)"),
			WS_CHILD | WS_VISIBLE,
			550, 225, 130, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("EDIT"), TEXT("100"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			685, 223, 90, 24, hWnd, (HMENU)ID_BC_EDIT_CORR_START_BCERR, 0, 0);

		// BUTTONS
		CreateWindow(TEXT("BUTTON"), TEXT("Servo ON"),
			WS_CHILD | WS_VISIBLE,
			780, 223, 90, 26, hWnd, (HMENU)ID_BC_BTN_SERVO_ON, 0, 0);

		CreateWindow(TEXT("BUTTON"), TEXT("Servo OFF"),
			WS_CHILD | WS_VISIBLE,
			875, 223, 90, 26, hWnd, (HMENU)ID_BC_BTN_SERVO_OFF, 0, 0);

		CreateWindow(TEXT("BUTTON"), TEXT("Start"),
			WS_CHILD | WS_VISIBLE,
			780, 255, 90, 26, hWnd, (HMENU)ID_BC_BTN_START, 0, 0);

		CreateWindow(TEXT("BUTTON"), TEXT("Stop"),
			WS_CHILD | WS_VISIBLE,
			875, 255, 90, 26, hWnd, (HMENU)ID_BC_BTN_STOP, 0, 0);

		// STATUS
		CreateWindow(TEXT("STATIC"), TEXT("Status"),
			WS_CHILD | WS_VISIBLE,
			20, 290, 70, 22, hWnd, 0, 0, 0);

		CreateWindow(TEXT("STATIC"), TEXT("-"),
			WS_CHILD | WS_VISIBLE | WS_BORDER,
			90, 288, 980, 24, hWnd, (HMENU)ID_BC_TXT_STATUS, 0, 0);

		// TIMER
		SetTimer(hWnd, ID_BC_TIMER, 30, nullptr);
		return 0;
	}

	case WM_COMMAND:
	{
		int id = LOWORD(wParam);

		if (id == ID_BC_COMBO_AXIS && HIWORD(wParam) == CBN_SELCHANGE)
		{
			g_hbc.moveaxis = (int)SendMessage((HWND)lParam, CB_GETCURSEL, 0, 0);
			g_hbc.bcaxis = 0;
			return 0;
		}

		if (id == ID_BC_BTN_SERVO_ON)
		{
			EnsureServoOn(g_hbc.moveaxis);
			EnsurePosModeNoStop(g_hbc.moveaxis);
			return 0;
		}

		if (id == ID_BC_BTN_SERVO_OFF)
		{
			g_cm.axisControl->SetServoOn(g_hbc.moveaxis, 0);
			return 0;
		}

		if (id == ID_BC_BTN_START)
		{
			HBC_Start(hWnd);
			return 0;
		}

		if (id == ID_BC_BTN_STOP)
		{
			g_hbc.running = false;
			g_hbc.inCorr = false;
			g_hbc.finalSnapSent = false;

			// 항상 재시작할 때 초기화
			g_hbc.stopDelayActive = false;
			g_hbc.stopDelayTimer = 0;

			StopAxis(g_hbc.moveaxis);
			return 0;
		}
		return 0;
	}

	case WM_TIMER:
		if (wParam == ID_BC_TIMER)
		{
			HBC_UpdateUi(hWnd);

			if (g_hbc.running)
				HBC_Poll(hWnd);

			return 0;
		}
		break;

	case WM_CLOSE:
		DestroyWindow(hWnd);
		return 0;

	case WM_DESTROY:
		if (g_hBarcodeWnd == hWnd)
			g_hBarcodeWnd = nullptr;
		return 0;
	}

	return DefWindowProc(hWnd, msg, wParam, lParam);
}


// =====================================================
// Show Window
// =====================================================
void ShowBarcodeDemoWindow(HWND parent)
{
	if (!g_hBarcodeWnd || !IsWindow(g_hBarcodeWnd))
	{
		WNDCLASS wc{};
		wc.lpszClassName = TEXT("HybridBarcodeDemoWnd");
		wc.lpfnWndProc = BarcodeWndProc;
		wc.hInstance = (HINSTANCE)GetWindowLongPtr(parent, GWLP_HINSTANCE);
		wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);

		RegisterClass(&wc);

		g_hBarcodeWnd = CreateWindow(
			TEXT("HybridBarcodeDemoWnd"),
			TEXT("Hybrid Barcode Demo"),
			WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_SIZEBOX | WS_MINIMIZEBOX,
			CW_USEDEFAULT, CW_USEDEFAULT,
			1160, 440,
			parent,
			nullptr,
			wc.hInstance,
			nullptr
		);

		ShowWindow(g_hBarcodeWnd, SW_SHOWNORMAL);
		UpdateWindow(g_hBarcodeWnd);
	}
	else
	{
		ShowWindow(g_hBarcodeWnd, SW_SHOWNORMAL);
		SetForegroundWindow(g_hBarcodeWnd);
	}
}







// ------------------ 메인 윈도우 ------------------
#define ID_BTN_DEMO_MAIN 9602
#define ID_BTN_GPIO_MAIN 9603

static void CreateStatusTable(HWND parent, int x, int y, int w, int h) {
	CreateWindow(TEXT("BUTTON"), TEXT("Status"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, x, y, w, h, parent, nullptr, nullptr, nullptr);
	int ox = x + 20, oy = y + 30, colW = 120, rowH = 22;

	CreateWindow(TEXT("STATIC"), TEXT("Axis"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox, oy, 80, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("Servo"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("In Position"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("Motioning"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW * 2, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("CmdPos"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW * 3, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("ActPos"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW * 4, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("CmdVel"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW * 5, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("ActVel"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW * 6, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("CmdTrq"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW * 7, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("ActTrq"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW * 8, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("PosErr"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW * 9, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("AmpAlarm"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW * 10, oy, colW, rowH, parent, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("Err(0x603F)"), WS_CHILD | WS_VISIBLE | SS_CENTER, ox + 90 + colW * 11, oy, colW, rowH, parent, nullptr, nullptr, nullptr);

	oy += rowH + 5;
	for (int a = 0; a < 4; ++a) {
		TCHAR lab[16]; _stprintf_s(lab, TEXT("Axis %d"), a);
		CreateWindow(TEXT("STATIC"), lab, WS_CHILD | WS_VISIBLE | SS_CENTER, ox, oy + a * (rowH + 5), 80, rowH, parent, nullptr, nullptr, nullptr);

		CreateWindow(TEXT("STATIC"), TEXT("OFF"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 0), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("OUT OF POSITION"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 1), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("IDLE"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW * 2, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 2), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW * 3, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 3), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW * 4, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 4), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW * 5, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 5), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW * 6, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 6), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW * 7, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 7), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW * 8, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 8), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("0"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW * 9, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 9), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("OK"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW * 10, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 10), nullptr, nullptr);
		CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | WS_BORDER | SS_CENTER, ox + 90 + colW * 11, oy + a * (rowH + 5), colW, rowH, parent, (HMENU)(INT_PTR)ID_TXT_STATUS(a, 11), nullptr, nullptr);
	}
}

static void CreateAxisGroup(HWND parent, int axis, int x, int y, int w, int h) {
	TCHAR cap[32]; _stprintf_s(cap, TEXT("Axis %d Control"), axis);
	CreateWindow(TEXT("BUTTON"), cap, WS_CHILD | WS_VISIBLE | BS_GROUPBOX, x, y, w, h, parent, nullptr, nullptr, nullptr);

	const int padL = 10, padT = 35; int gx = x, gy = y;
	CreateWindow(TEXT("BUTTON"), TEXT("Servo ON"), WS_CHILD | WS_VISIBLE, gx + padL, gy + padT, 100, 28, parent, (HMENU)(INT_PTR)ID_BTN_SVON_A(axis), nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Servo OFF"), WS_CHILD | WS_VISIBLE, gx + padL + 110, gy + padT, 100, 28, parent, (HMENU)(INT_PTR)ID_BTN_SVOFF_A(axis), nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Home"), WS_CHILD | WS_VISIBLE, gx + padL + 220, gy + padT, 100, 28, parent, (HMENU)(INT_PTR)ID_BTN_HOME_A(axis), nullptr, nullptr);

	int yField = gy + padT + 4; int xStart = gx + padL + 330;
	FieldPos posFP = TightLabeledEdit2(parent, xStart, yField, TEXT("Position"), 10, 110, ID_EDIT_POS_A(axis), TEXT("10000"), TEXT("pulse"), 10); int nextX = posFP.endX + 15;
	FieldPos velFP = TightLabeledEdit2(parent, nextX, yField, TEXT("Velocity"), 10, 110, ID_EDIT_VEL_A(axis), TEXT("10000"), TEXT("pps"), 10); nextX = velFP.endX + 15;
	FieldPos accFP = TightLabeledEdit2(parent, nextX, yField, TEXT("Acc"), 0, 90, ID_EDIT_ACCT_A(axis), TEXT("100"), TEXT("ms"), 10); nextX = accFP.endX + 15;
	FieldPos decFP = TightLabeledEdit2(parent, nextX, yField, TEXT("Dec"), 0, 90, ID_EDIT_DECT_A(axis), TEXT("100"), TEXT("ms"), 10); nextX = decFP.endX;

	int altXStart = gx + padL + 330;
	int altY = yField + 40;
	FieldPos altFP = TightLabeledEdit2(parent, altXStart, altY, TEXT("Alt Target"), 10, 110, ID_EDIT_ALTTGT_A(axis), TEXT("0"), TEXT("pulse"), 10);
	CreateWindow(TEXT("BUTTON"), TEXT("Apply Alt→Abs"), WS_CHILD | WS_VISIBLE, altFP.endX + 15, gy + padT + 40, 120, 28, parent, (HMENU)(INT_PTR)ID_BTN_APPLY_ALT_A(axis), nullptr, nullptr);

	CreateWindow(TEXT("BUTTON"), TEXT("Absolute"), WS_CHILD | WS_VISIBLE, nextX, gy + padT, 110, 28, parent, (HMENU)(INT_PTR)ID_BTN_ABS_A(axis), nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Relative"), WS_CHILD | WS_VISIBLE, nextX + 120, gy + padT, 110, 28, parent, (HMENU)(INT_PTR)ID_BTN_REL_A(axis), nullptr, nullptr);

	HWND hJogP = CreateWindow(TEXT("BUTTON"), TEXT("Jog +"), WS_CHILD | WS_VISIBLE, nextX + 240, gy + padT, 80, 28, parent, (HMENU)(INT_PTR)ID_BTN_JOGP_A(axis), nullptr, nullptr);
	HWND hJogM = CreateWindow(TEXT("BUTTON"), TEXT("Jog -"), WS_CHILD | WS_VISIBLE, nextX + 325, gy + padT, 80, 28, parent, (HMENU)(INT_PTR)ID_BTN_JOGM_A(axis), nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Stop"), WS_CHILD | WS_VISIBLE, nextX + 410, gy + padT, 80, 28, parent, (HMENU)(INT_PTR)ID_BTN_STOP_A(axis), nullptr, nullptr);

	// Jog 버튼: 자동 모드에서 눌리면 즉시 무시
	if (hJogP) {
		JogBtnCtx* c = new JogBtnCtx; c->axis = axis; c->sign = +1; c->isMulti = false; SetWindowSubclass(hJogP, [](HWND hBtn, UINT msg, WPARAM wParam, LPARAM lParam, UINT_PTR uIdSubclass, DWORD_PTR dwRefData)->LRESULT {
			JogBtnCtx* ctx = reinterpret_cast<JogBtnCtx*>(dwRefData);
			HWND hParent = GetParent(hBtn);
			switch (msg) {
			case WM_LBUTTONDOWN: {
				if (g_autoMode.load()) { MessageBeep(MB_ICONWARNING); return 0; }
				if (ctx->isMulti) {}
				else { if (g_multiJogActive) StopMultiJog(); if (g_jogActiveAxis >= 0) StopJogIfActive(); StartJog(hParent, ctx->axis, ctx->sign); }
				SetCapture(hBtn); return 0;
			}
			case WM_LBUTTONUP:
			case WM_CAPTURECHANGED: { if (ctx->isMulti) {} else StopJogIfActive(); ReleaseCapture(); return 0; }
			case WM_KEYDOWN: if (wParam == VK_SPACE || wParam == VK_RETURN) { SendMessage(hBtn, WM_LBUTTONDOWN, 0, 0); return 0; } break;
			case WM_KEYUP: if (wParam == VK_SPACE || wParam == VK_RETURN) { SendMessage(hBtn, WM_LBUTTONUP, 0, 0); return 0; } break;
			case WM_NCDESTROY: RemoveWindowSubclass(hBtn, (SUBCLASSPROC)DefSubclassProc, uIdSubclass); delete ctx; break;
			}
			return DefSubclassProc(hBtn, msg, wParam, lParam);
			}, (UINT_PTR)(0x1000 + axis * 2 + 0), (DWORD_PTR)c);
	}
	if (hJogM) {
		JogBtnCtx* c = new JogBtnCtx; c->axis = axis; c->sign = -1; c->isMulti = false; SetWindowSubclass(hJogM, [](HWND hBtn, UINT msg, WPARAM wParam, LPARAM lParam, UINT_PTR uIdSubclass, DWORD_PTR dwRefData)->LRESULT {
			JogBtnCtx* ctx = reinterpret_cast<JogBtnCtx*>(dwRefData);
			HWND hParent = GetParent(hBtn);
			switch (msg) {
			case WM_LBUTTONDOWN: {
				if (g_autoMode.load()) { MessageBeep(MB_ICONWARNING); return 0; }
				if (ctx->isMulti) {}
				else { if (g_multiJogActive) StopMultiJog(); if (g_jogActiveAxis >= 0) StopJogIfActive(); StartJog(hParent, ctx->axis, ctx->sign); }
				SetCapture(hBtn); return 0;
			}
			case WM_LBUTTONUP:
			case WM_CAPTURECHANGED: { if (ctx->isMulti) {} else StopJogIfActive(); ReleaseCapture(); return 0; }
			case WM_KEYDOWN: if (wParam == VK_SPACE || wParam == VK_RETURN) { SendMessage(hBtn, WM_LBUTTONDOWN, 0, 0); return 0; } break;
			case WM_KEYUP: if (wParam == VK_SPACE || wParam == VK_RETURN) { SendMessage(hBtn, WM_LBUTTONUP, 0, 0); return 0; } break;
			case WM_NCDESTROY: RemoveWindowSubclass(hBtn, (SUBCLASSPROC)DefSubclassProc, uIdSubclass); delete ctx; break;
			}
			return DefSubclassProc(hBtn, msg, wParam, lParam);
			}, (UINT_PTR)(0x1000 + axis * 2 + 1), (DWORD_PTR)c);
	}
}

// 새 컨트롤 ID
#define ID_BTN_MAP_WINDOW 9500

// Map 창 핸들
HWND g_hMapWnd = nullptr;

HWND ShowMapWindow(HWND hParent)
{
	if (!g_hMapWnd || !IsWindow(g_hMapWnd)) {
#ifdef MAPVIEW_CREATE_DECLARED
		g_hMapWnd = CreateMapWindow(hParent, &g_wmx, &g_cm);
#else
		g_hMapWnd = nullptr;
#endif
		if (g_hMapWnd) {
			ShowWindow(g_hMapWnd, SW_SHOWNORMAL);
			UpdateWindow(g_hMapWnd);
			SetForegroundWindow(g_hMapWnd);
		}
		else {
			MessageBox(hParent, TEXT("Map window creation failed."), TEXT("Map"), MB_ICONERROR);
		}
	}
	else {
		ShowWindow(g_hMapWnd, SW_SHOWNORMAL);
		SetForegroundWindow(g_hMapWnd);
	}
	return g_hMapWnd;
}

static void CreateUI(HWND h) {
	SetWindowPos(h, HWND_TOP, 0, 0,
		std::max(1850, GetSystemMetrics(SM_CXSCREEN)),
		std::max(1080, GetSystemMetrics(SM_CYSCREEN)), SWP_SHOWWINDOW);

	CreateWindow(TEXT("BUTTON"), TEXT("Create Device"), WS_CHILD | WS_VISIBLE, 20, 10, 150, 30, h, (HMENU)(INT_PTR)ID_BTN_CREATE_DEVICE, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Start Communication"), WS_CHILD | WS_VISIBLE, 180, 10, 170, 30, h, (HMENU)(INT_PTR)ID_BTN_START_COMM, nullptr, nullptr);

	CreateWindow(TEXT("BUTTON"), TEXT("Demo"), WS_CHILD | WS_VISIBLE, 360, 10, 120, 30, h, (HMENU)(INT_PTR)ID_BTN_DEMO_MAIN, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("GPIO"), WS_CHILD | WS_VISIBLE, 490, 10, 120, 30, h, (HMENU)(INT_PTR)ID_BTN_GPIO_MAIN, nullptr, nullptr);
	// CHANGED: Demo2 버튼을 Barcode Demo 창 오픈으로 변경
	CreateWindow(TEXT("BUTTON"), TEXT("Barcode Demo"), WS_CHILD | WS_VISIBLE, 620, 45, 120, 30, h, (HMENU)(INT_PTR)ID_BTN_DEMO2, nullptr, nullptr);

	CreateWindow(TEXT("BUTTON"), TEXT("비상정지"), WS_CHILD | WS_VISIBLE, 620, 10, 100, 30, h, (HMENU)(INT_PTR)ID_BTN_ESTOP_TOGGLE, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("NORMAL"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, 725, 12, 110, 24, h, (HMENU)(INT_PTR)ID_TXT_ESTOP_STATE, nullptr, nullptr);

	// NEW: Mode buttons and state
	CreateWindow(TEXT("BUTTON"), TEXT("Manual Mode"), WS_CHILD | WS_VISIBLE, 350, 900, 110, 30, h, (HMENU)ID_BTN_MODE_MANUAL, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Auto Mode"), WS_CHILD | WS_VISIBLE, 465, 900, 110, 30, h, (HMENU)ID_BTN_MODE_AUTO, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("MODE: MANUAL"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, 580, 902, 140, 24, h, (HMENU)ID_TXT_MODE_STATE, nullptr, nullptr);


	// Error Manual 버튼
	CreateWindow(TEXT("BUTTON"), TEXT("Fastech Error Manual"), WS_CHILD | WS_VISIBLE, 750, 900, 160, 30, h, (HMENU)(INT_PTR)ID_BTN_FASTECH_MANUAL, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Welcon Error Manual"), WS_CHILD | WS_VISIBLE, 915, 900, 160, 30, h, (HMENU)(INT_PTR)ID_BTN_WELCON_MANUAL, nullptr, nullptr);

	// 요구사항: Error Manual 버튼 바로 아래에 Log... / Scope... 버튼 배치
	CreateWindow(TEXT("BUTTON"), TEXT("Log..."), WS_CHILD | WS_VISIBLE, 750, 935, 160, 30, h, (HMENU)(INT_PTR)ID_BTN_LOG_WINDOW, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Scope..."), WS_CHILD | WS_VISIBLE, 915, 935, 160, 30, h, (HMENU)(INT_PTR)ID_BTN_SCOPE_WINDOW, nullptr, nullptr);

	CreateWindow(TEXT("BUTTON"), TEXT("Sync Group..."), WS_CHILD | WS_VISIBLE, 980, 10, 130, 30, h, (HMENU)(INT_PTR)ID_BTN_SYNC_WINDOW, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Serial Monitor..."), WS_CHILD | WS_VISIBLE, 1120, 10, 140, 30, h, (HMENU)(INT_PTR)ID_BTN_SERIAL_WINDOW, nullptr, nullptr);

	CreateWindow(TEXT("BUTTON"), TEXT("Map..."), WS_CHILD | WS_VISIBLE, 1270, 10, 90, 30, h, (HMENU)(INT_PTR)ID_BTN_MAP_WINDOW, nullptr, nullptr);

	CreateWindow(TEXT("STATIC"), TEXT("Zone: -"), WS_CHILD | WS_VISIBLE | SS_LEFT, 1280, 50, 200, 20, h, (HMENU)(INT_PTR)ID_TXT_ZONE_ANNOUNCE, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("Motion: -"), WS_CHILD | WS_VISIBLE | SS_LEFT, 1500, 15, 200, 20, h, (HMENU)(INT_PTR)ID_TXT_MOTION_ANNOUNCE, nullptr, nullptr);

	CreateWindow(TEXT("STATIC"), TEXT("Barcode:"), WS_CHILD | WS_VISIBLE | SS_LEFT, 1500, 50, 60, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, 1570, 47, 140, 24, h, (HMENU)(INT_PTR)ID_TXT_ECAT_6063, nullptr, nullptr);

	// Axis2 Limit/Home 상태 표시
	CreateWindow(TEXT("STATIC"), TEXT("A2 Limit:"), WS_CHILD | WS_VISIBLE | SS_LEFT, 1400, 910, 70, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, 1470, 907, 140, 24, h, (HMENU)(INT_PTR)ID_TXT_AX2_LIMIT, nullptr, nullptr);

	CreateWindow(TEXT("STATIC"), TEXT("A2 Home:"), WS_CHILD | WS_VISIBLE | SS_LEFT, 1400, 940, 70, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"), WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, 1470, 937, 140, 24, h, (HMENU)(INT_PTR)ID_TXT_AX2_HOME, nullptr, nullptr);

	// ================= Axis0 Limit L/R 상태 표시 (Axis2 옆) =================
	CreateWindow(TEXT("STATIC"), TEXT("A0 L-Lim:"), WS_CHILD | WS_VISIBLE | SS_LEFT, 1170, 910, 70, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"),	WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, 1240, 907, 140, 24, h, (HMENU)(INT_PTR)ID_TXT_AX0_LIMIT_L, nullptr, nullptr);

	CreateWindow(TEXT("STATIC"), TEXT("A0 R-Lim:"), WS_CHILD | WS_VISIBLE | SS_LEFT, 1170, 940, 70, 20, h, nullptr, nullptr, nullptr);
	CreateWindow(TEXT("STATIC"), TEXT("-"),	WS_CHILD | WS_VISIBLE | SS_LEFT | WS_BORDER, 1240, 937, 140, 24, h, (HMENU)(INT_PTR)ID_TXT_AX0_LIMIT_R, nullptr, nullptr);

	int x = 10, y = 70, w = 1800, hgt = 120, gap = 6;
	for (int a = 0; a < 4; ++a) CreateAxisGroup(h, a, x, y + a * (hgt + gap), w, hgt);

	CreateWindow(TEXT("BUTTON"), TEXT("Checked Axis Control"), WS_CHILD | WS_VISIBLE | BS_GROUPBOX, 10, y + 4 * (hgt + gap), 1760, 160, h, nullptr, nullptr, nullptr);
	int gy = y + 4 * (hgt + gap);
	CreateWindow(TEXT("BUTTON"), TEXT("Absolute Move"), WS_CHILD | WS_VISIBLE, 20, gy + 20, 180, 28, h, (HMENU)(INT_PTR)ID_BTN_MULTI_ABS, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Relative Move"), WS_CHILD | WS_VISIBLE, 210, gy + 20, 180, 28, h, (HMENU)(INT_PTR)ID_BTN_MULTI_REL, nullptr, nullptr);

	HWND hMJp = CreateWindow(TEXT("BUTTON"), TEXT("Jog +"), WS_CHILD | WS_VISIBLE, 400, gy + 20, 100, 28, h, (HMENU)(INT_PTR)ID_BTN_MULTI_JOGP, nullptr, nullptr);
	HWND hMJm = CreateWindow(TEXT("BUTTON"), TEXT("Jog -"), WS_CHILD | WS_VISIBLE, 510, gy + 20, 100, 28, h, (HMENU)(INT_PTR)ID_BTN_MULTI_JOGM, nullptr, nullptr);

	CreateWindow(TEXT("BUTTON"), TEXT("Home (Selected)"), WS_CHILD | WS_VISIBLE, 620, gy + 20, 150, 28, h, (HMENU)(INT_PTR)ID_BTN_MULTI_HOME, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Servo ON (Selected)"), WS_CHILD | WS_VISIBLE, 780, gy + 20, 180, 28, h, (HMENU)(INT_PTR)ID_BTN_MULTI_SVON, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Servo OFF (Selected)"), WS_CHILD | WS_VISIBLE, 970, gy + 20, 180, 28, h, (HMENU)(INT_PTR)ID_BTN_MULTI_SVOFF, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Stop All"), WS_CHILD | WS_VISIBLE, 1160, gy + 20, 110, 28, h, (HMENU)(INT_PTR)ID_BTN_MULTI_STOP, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Select All"), WS_CHILD | WS_VISIBLE, 1280, gy + 20, 70, 28, h, (HMENU)ID_BTN_SELECT_ALL, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Clear All"), WS_CHILD | WS_VISIBLE, 1360, gy + 20, 70, 28, h, (HMENU)ID_BTN_CLEAR_ALL, nullptr, nullptr);

	CreateWindow(TEXT("BUTTON"), TEXT("Alarm Reset (Selected)"), WS_CHILD | WS_VISIBLE, 1440, gy + 20, 180, 28, h, (HMENU)(INT_PTR)ID_BTN_MULTI_ALARM_RST, nullptr, nullptr);

	CreateWindow(TEXT("STATIC"), TEXT("Selected: (none)"), WS_CHILD | WS_VISIBLE | SS_LEFT, 20, gy + 55, 1400, 20, h, (HMENU)(INT_PTR)ID_TXT_SELECTED_AXES, nullptr, nullptr);

	int cy = gy + 110 - 30;
	CreateWindow(TEXT("BUTTON"), TEXT("Axis 0"), WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 20, cy, 100, 30, h, (HMENU)ID_CHECK_AXIS_0, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Axis 1"), WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 125, cy, 100, 30, h, (HMENU)ID_CHECK_AXIS_1, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Axis 2"), WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 230, cy, 100, 30, h, (HMENU)ID_CHECK_AXIS_2, nullptr, nullptr);
	CreateWindow(TEXT("BUTTON"), TEXT("Axis 3"), WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 335, cy, 100, 30, h, (HMENU)ID_CHECK_AXIS_3, nullptr, nullptr);

	CreateStatusTable(h, 10, gy + 160, 1760, 300);

	// Multi Jog 버튼: 자동 모드에서 눌리면 즉시 무시
	if (hMJp) {
		JogBtnCtx* c = new JogBtnCtx; c->axis = -1; c->sign = +1; c->isMulti = true;
		SetWindowSubclass(hMJp, [](HWND hBtn, UINT msg, WPARAM wParam, LPARAM lParam, UINT_PTR uIdSubclass, DWORD_PTR dwRefData)->LRESULT {
			if (msg == WM_LBUTTONDOWN) { if (g_autoMode.load()) { MessageBeep(MB_ICONWARNING); return 0; } StartMultiJog(GetParent(hBtn), +1); SetCapture(hBtn); return 0; }
			if (msg == WM_LBUTTONUP || msg == WM_CAPTURECHANGED) { StopMultiJog(); ReleaseCapture(); return 0; }
			if (msg == WM_KEYDOWN && (wParam == VK_SPACE || wParam == VK_RETURN)) { SendMessage(hBtn, WM_LBUTTONDOWN, 0, 0); return 0; }
			if (msg == WM_KEYUP && (wParam == VK_SPACE || wParam == VK_RETURN)) { SendMessage(hBtn, WM_LBUTTONUP, 0, 0); return 0; }
			if (msg == WM_NCDESTROY) { RemoveWindowSubclass(hBtn, (SUBCLASSPROC)DefSubclassProc, uIdSubclass); delete reinterpret_cast<JogBtnCtx*>(dwRefData); }
			return DefSubclassProc(hBtn, msg, wParam, lParam);
			}, 0x2000, (DWORD_PTR)c);
	}
	if (hMJm) {
		JogBtnCtx* c = new JogBtnCtx; c->axis = -1; c->sign = -1; c->isMulti = true;
		SetWindowSubclass(hMJm, [](HWND hBtn, UINT msg, WPARAM wParam, LPARAM lParam, UINT_PTR uIdSubclass, DWORD_PTR dwRefData)->LRESULT {
			if (msg == WM_LBUTTONDOWN) { if (g_autoMode.load()) { MessageBeep(MB_ICONWARNING); return 0; } StartMultiJog(GetParent(hBtn), -1); SetCapture(hBtn); return 0; }
			if (msg == WM_LBUTTONUP || msg == WM_CAPTURECHANGED) { StopMultiJog(); ReleaseCapture(); return 0; }
			if (msg == WM_KEYDOWN && (wParam == VK_SPACE || wParam == VK_RETURN)) { SendMessage(hBtn, WM_LBUTTONDOWN, 0, 0); return 0; }
			if (msg == WM_KEYUP && (wParam == VK_SPACE || wParam == VK_RETURN)) { SendMessage(hBtn, WM_LBUTTONUP, 0, 0); return 0; }
			if (msg == WM_NCDESTROY) { RemoveWindowSubclass(hBtn, (SUBCLASSPROC)DefSubclassProc, uIdSubclass); delete reinterpret_cast<JogBtnCtx*>(dwRefData); }
			return DefSubclassProc(hBtn, msg, wParam, lParam);
			}, 0x2001, (DWORD_PTR)c);
	}
}

// 한 통신 주기 이상 기다리기 위한 헬퍼
static void WaitOneCommCycle()
{
	// 통신주기가 1ms 근처라면 10ms 정도면 충분히 여유 있음
	Sleep(10);
}
static void UpdateDriveReadyByState()
{
	unsigned char grip = CalcPosGripCode();
	unsigned char hoist = CalcPosHoistCode();

	if (grip != 0x00 && hoist == 0x03) {
		g_driveReady.store(true, std::memory_order_relaxed);
	}
}

static void AutoStart(HWND hWnd)
{
	// 이미 한 번 수행했다면 스킵
	bool expected = false;
	if (!g_autoStartDone.compare_exchange_strong(expected, true)) {
		return;
	}

	// UI가 안정화될 시간을 충분히 둡니다. (서비스/드라이버 준비 포함)
	Sleep(1500);

	// 이미 열린 경우는 스킵
	if (!g_deviceOpened) {
		if (!InitDevice()) {
			MessageBox(hWnd, TEXT("AutoStart: CreateDevice 실패. (다른 인스턴스가 이미 사용 중이거나 드라이버 초기화 지연일 수 있습니다)\r\n"
				"프로그램을 다시 실행하거나 관리자 권한으로 실행해 보세요."),
				TEXT("AutoStart"), MB_ICONERROR);
			return;
		}
	}

	Sleep(200); // 한 틱 대기

	if (!g_commStarted) {
		if (!StartComm()) {
			MessageBox(hWnd, TEXT("AutoStart: StartCommunication 실패."), TEXT("AutoStart"), MB_ICONERROR);
			return;
		}
		Sleep(50);
	}

	// Servo ON
	/*for (int a = 0; a < 4; ++a) {
		EnsureServoOn(a);
		EnsurePosModeNoStop(a);
	}*/

	// Servo ON
	EnsureServoOn(1);
	EnsurePosModeNoStop(1);

	//// 5초 대기
	//std::this_thread::sleep_for(std::chrono::seconds(5));

	//EnsureServoOn(1);
	//EnsurePosModeNoStop(1);

	// 5초 대기
	std::this_thread::sleep_for(std::chrono::seconds(5));

	// 그 다음 2번 서보 ON
	EnsureServoOn(2);
	EnsurePosModeNoStop(2);

	//std::this_thread::sleep_for(std::chrono::seconds(3));
	//// Sync Group 0: Master=0, Slave=[1]
	//if (g_commStarted) {
	//	Sync::SyncGroup grp{};
	//	grp.masterAxis = 0;
	//	grp.slaveAxisCount = 1;
	//	grp.slaveAxis[0] = 1;
	//	grp.servoOnOffSynchronization = 1;
	//	grp.startupType = Sync::SyncGroupStartupType::Normal;
	//	grp.gantryLoopCycleRatio = 1;
	//	grp.maxCatchUpDistance = 0.0;
	//	grp.catchUpVelocity = 0.0;
	//	grp.catchUpAcc = 0.0;
	//	grp.syncErrorTolerance = 1000.0;
	//	grp.useMasterFeedback = 0;

	//	Sync::SyncGroupStatus gst{};
	//	if (g_cm.sync->GetSyncGroupStatus(0, &gst) == ErrorCode::None && gst.enabled) {
	//		g_cm.sync->EnableSyncGroup(0, 0);
	//		Sleep(10);
	//	}

	//	long se = g_cm.sync->SetSyncGroup(0, grp);
	//	if (se == ErrorCode::None) {
	//		Sleep(10);
	//		Config::SyncParam sp{};
	//		if (g_cm.config->GetSyncParam(grp.masterAxis, &sp) == ErrorCode::None) {
	//			sp.masterDesyncDec = 10000.0;
	//			sp.slaveDesyncDec = 10000.0;
	//			g_cm.config->SetSyncParam(grp.masterAxis, &sp, nullptr);
	//			Sleep(10);
	//		}
	//		g_cm.sync->EnableSyncGroup(0, 1);
	//	}
	//}

	PostMessage(hWnd, WM_APP_SHOW_DEMO_MIN, 0, 0);

	UpdateEStopUi(hWnd, false);
	UpdateTcpUiState(hWnd);
	
}


static void LaunchGPIOWindow() {
	std::thread([] {
		HINSTANCE hInst = GetModuleHandle(nullptr);
		RunGPIOWindowExternal(hInst);
		}).detach();
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	switch (msg) {
	case WM_CREATE:
	{
		g_hMainWnd = hWnd;
		INITCOMMONCONTROLSEX icc; icc.dwSize = sizeof(icc); icc.dwICC = ICC_WIN95_CLASSES | ICC_LISTVIEW_CLASSES;
		InitCommonControlsEx(&icc);
		CreateUI(hWnd);
		SetTimer(hWnd, ID_TIMER, POLL_MS, nullptr);
		UpdateEStopUi(hWnd, false);
		UpdateTcpUiState(hWnd);

		// TCP 서버는 AutoStart 이후 시작하거나, 필요 시 아래 주석 해제
		//StartTcpServer();

		//// LOG/SCOPE 스레드
		//InitializeCriticalSection(&g_logCs);
		//g_logThreadRun = true;
		//g_logThread = std::thread(LogThreadProc);
		//g_logThread.detach();

		// Axis2 flags reset
		g_ax2LimitOn = false;
		g_ax2HomeOn = false;
		g_ax2LimitLatched = false;
		g_ax2LimitBlocking = false;
		g_ax2StopIssuedOnLimit = false;
		g_ax2HomingStarted = false;
		g_ax2HomeDebounceOn = false;
		g_ax2HomeLastTick = GetTickCount();
		g_ax2HomeRampIssued = false;

		// ================= Axis0 flags reset =================
	// 시작 시에는 리밋이 안 물린 상태(= free = true)로 가정
		g_ax0LimitLFree = true;
		g_ax0LimitRFree = true;

		g_ax0LimitLLatched = false;
		g_ax0LimitRLatched = false;

		// 리밋 블록도 모두 해제 상태로 시작
		g_ax0BlockPlus = false;   // + 방향 명령 허용
		g_ax0BlockMinus = false;   // - 방향 명령 허용

		//// [AUTO] 실행 시 자동 초기화 (지연 증가)
		std::thread([](HWND hMain) {
			bool setupOk = true;   // 전체 초기 세팅 성공 여부 플래그

			// 1) UI/서비스 준비 시간
			std::this_thread::sleep_for(std::chrono::seconds(1));
			AutoStart(hMain);

			std::this_thread::sleep_for(std::chrono::seconds(1));
			DoGripServoOff_Compat(hMain);

			// 2) AutoStart 후 TCP 시작
			StartTcpServer();
			std::this_thread::sleep_for(std::chrono::seconds(1));

			//// 3) 그리퍼 상태 정리 (HasBox 기준)
			//bool okGrip = false;
			//bool g_code = CalcPosGripCode();
			//
			//if (HasBox() && g_code == 0x02) {
			//	//AppendLog(L"[AUTO] HasBox()==true -> Grip Close + ServoOff");
			//	//DoClose_Compat(hMain);
			//	DoGripServoOff_Compat(hMain);
			//	//okGrip = WaitTaskFinished(TaskId::GripClose, 5000);
			//	//if (!okGrip) {
			//	//	
			//	//	AppendLog(L"[AUTO][WARN] Grip Close or Idle wait FAILED");
			//	//	setupOk = false;
			//	//}
			//	//
			//	//SetTaskState(TaskId::GripClose, TaskState::Done);
			//	//DoGripServoOff_Compat(hMain);
			//	//
			//	//AppendLog(L"[AUTO] Grip Close -> GripCode=0x02 (Close) forced");
			//	
			//}
			//else if (HasBox() && (g_code == 0x01 || g_code == 0x00)) {
			//	AppendLog(L"[AUTO] HasBox()==true -> Grip Close + ServoOff");
			//	DoClose_Compat(hMain);
			//	//DoGripServoOff_Compat(hMain);
			//	okGrip = WaitTaskFinished(TaskId::GripClose, 5000);
			//	if (!okGrip) {

			//		AppendLog(L"[AUTO][WARN] Grip Close or Idle wait FAILED");
			//		setupOk = false;
			//	}

			//	
			//	//DoGripServoOff_Compat(hMain);
			//	//SetTaskState(TaskId::GripClose, TaskState::Done);
			//	AppendLog(L"[AUTO] Grip Close -> GripCode=0x02 (Close) forced");
			//}
			//else if(NoBox() && (g_code == 0x02 || g_code == 0x00)){
			//	AppendLog(L"[AUTO] HasBox()==false -> Grip Open + ServoOff");
			//	DoOpen_Compat(hMain);
			//	okGrip = WaitTaskFinished(TaskId::GripOpen, 5000);
			//	if (!okGrip) {
			//		
			//		AppendLog(L"[AUTO][WARN] Grip Open or Idle wait FAILED");
			//		setupOk = false;
			//	}
			//	//DoGripServoOff_Compat(hMain);
			//	//SetTaskState(TaskId::GripOpen, TaskState::Done);
			//	
			//	
			//	AppendLog(L"[AUTO] Grip Open -> GripCode=0x01 (Open) forced");
			//	
			//}
			//else if (NoBox() && g_code == 0x00) {

			//	DoClose_Compat(hMain);

			//	// Close 상태(0x02)로 실제 판정될 때까지 기다림
			//	bool closed = WaitUntil([]() { return CalcPosGripCode() == 0x02; }, 5000);
			//	
			//	// ★ 여기서 5초 대기(카운트)
			//	(void)WaitUntil([]() { return false; }, 5000);

			//	unsigned char codeAfterClose = CalcPosGripCode();
			//	bool nobox = !HasBox();
			//	if (nobox && codeAfterClose != 0x01) {
			//		DoOpen_Compat(hMain);
			//		(void)WaitUntil([]() { return CalcPosGripCode() == 0x01; }, 5000);
			//	}
			//}

			//else if(NoBox() && g_code == 0x01){
			//	DoGripServoOff_Compat(hMain);
			//	
			//	//SetTaskState(TaskId::GripOpen, TaskState::Done);
			//	

			//	AppendLog(L"[AUTO] Grip Open -> GripCode=0x01 (Open) forced");
			//}
			//std::this_thread::sleep_for(std::chrono::seconds(1));

			MessageBox(
						hMain,
						TEXT("초기 세팅이 정상적으로 완료되었습니다."),
						TEXT("초기 세팅"),
						MB_OK | MB_ICONINFORMATION
					);
			

		}, hWnd).detach();
		

	}
	return 0;
	return 0;
	case WM_APP_SHOW_DEMO_MIN:        // ★ AutoStart에서 날린 요청 처리
		ShowDemoControlWindow(hWnd, true);   // 최소화 상태로 생성/표시
		return 0;

	case WM_COMMAND:
	{
		int id = LOWORD(wParam);
		int code = HIWORD(wParam);

		if (id == ID_BTN_MODE_MANUAL) {
			SwitchToManual(hWnd);
			if (HWND h = GetDlgItem(hWnd, ID_TXT_MODE_STATE)) SetWindowText(h, TEXT("MODE: MANUAL"));
			return 0;
		}

		if (id == ID_BTN_MODE_AUTO) {
			SwitchToAuto(hWnd);
			if (HWND h = GetDlgItem(hWnd, ID_TXT_MODE_STATE)) SetWindowText(h, TEXT("MODE: AUTO"));
			// Serial Monitor 창이 없으면 띄워준다
			if (!g_hSerialWnd || !IsWindow(g_hSerialWnd)) {
				WNDCLASS wc{}; wc.lpszClassName = TEXT("WMX3SerialWnd");
				wc.lpfnWndProc = SerialWndProc; wc.hInstance = (HINSTANCE)GetWindowLongPtr(hWnd, GWLP_HINSTANCE);
				wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
				wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
				RegisterClass(&wc);
				g_hSerialWnd = CreateWindow(TEXT("WMX3SerialWnd"), TEXT("Serial Monitor (TCP)"),
					WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_SIZEBOX,
					CW_USEDEFAULT, CW_USEDEFAULT, 800, 400, hWnd, nullptr, wc.hInstance, nullptr);
				ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
				UpdateWindow(g_hSerialWnd);
			}
			else {
				ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
				SetForegroundWindow(g_hSerialWnd);
			}
			return 0;
		}

		if (id == ID_BTN_ESTOP_TOGGLE) { DoToggleEStop(hWnd, false); return 0; }

		if (id == ID_BTN_FASTECH_MANUAL) { ShowErrorManual(hWnd, true); return 0; }
		if (id == ID_BTN_WELCON_MANUAL) { ShowErrorManual(hWnd, false); return 0; }

		//// Log/Scope 버튼
		//if (id == ID_BTN_LOG_WINDOW) { ShowLogWindow(hWnd); return 0; }
		//if (id == ID_BTN_SCOPE_WINDOW) { ShowScopeWindow(hWnd); return 0; }

		if (id == ID_CHECK_AXIS_0 || id == ID_CHECK_AXIS_1 || id == ID_CHECK_AXIS_2 || id == ID_CHECK_AXIS_3) {
			if (code == BN_CLICKED) UpdateSelectedAxesTextOnDemand(hWnd);
			return 0;
		}
		if (id == ID_BTN_CREATE_DEVICE) {
			if (!IsManualAllowed(hWnd)) return 0;
			KillTimer(hWnd, ID_TIMER);
			bool ok = false;
			if (g_deviceOpened) { MessageBox(hWnd, TEXT("Device already created."), TEXT("Info"), MB_ICONINFORMATION); ok = true; }
			else { ok = InitDevice(); MessageBox(hWnd, ok ? TEXT("Success") : TEXT("Fail"), TEXT("Create Device"), ok ? MB_ICONINFORMATION : MB_ICONERROR); }
			SetTimer(hWnd, ID_TIMER, POLL_MS, nullptr);
			return 0;
		}
		if (id == ID_BTN_START_COMM) {
			if (!IsManualAllowed(hWnd)) return 0;
			KillTimer(hWnd, ID_TIMER);
			bool ok = false;
			if (!g_deviceOpened) MessageBox(hWnd, TEXT("Create device first."), TEXT("Start Communication"), MB_ICONWARNING);
			else if (g_commStarted) { MessageBox(hWnd, TEXT("Communication already started."), TEXT("Info"), MB_ICONINFORMATION); ok = true; }
			else { ok = StartComm(); MessageBox(hWnd, ok ? TEXT("Success") : TEXT("Fail"), TEXT("Start Communication"), ok ? MB_ICONINFORMATION : MB_ICONERROR); }
			SetTimer(hWnd, ID_TIMER, POLL_MS, nullptr);
			UpdateEStopUi(hWnd, false);
			return 0;
		}

		if (id == ID_BTN_DEMO_MAIN) {
			if (!IsManualAllowed(hWnd)) return 0;
			ShowDemoControlWindow(hWnd, false);
			return 0;
		}
		if (id == ID_BTN_GPIO_MAIN) {
			if (!IsManualAllowed(hWnd)) return 0;
			LaunchGPIOWindow();
			return 0;
		}

		if (id == ID_BTN_DEMO2) {
			if (!IsManualAllowed(hWnd)) return 0;
			// CHANGED: Demo2는 Barcode 창 오픈
			ShowBarcodeDemoWindow(hWnd);
			return 0;
		}

		if (id == ID_BTN_SYNC_WINDOW) {
			if (!IsManualAllowed(hWnd)) return 0;
			if (!g_deviceOpened || !g_commStarted) {
				MessageBox(hWnd, TEXT("Create device and start communication first."), TEXT("Sync Group"), MB_ICONWARNING);
				return 0;
			}
			if (!g_hSyncWnd || !IsWindow(g_hSyncWnd)) {
				WNDCLASS wc{}; wc.lpszClassName = TEXT("WMX3SyncWnd");
				wc.lpfnWndProc = SyncWndProc; wc.hInstance = (HINSTANCE)GetWindowLongPtr(hWnd, GWLP_HINSTANCE);
				wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
				wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
				RegisterClass(&wc);
				g_hSyncWnd = CreateWindow(TEXT("WMX3SyncWnd"), TEXT("Sync Group Control/Monitor"),
					WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_SIZEBOX,
					CW_USEDEFAULT, CW_USEDEFAULT, 1050, 1000, hWnd, nullptr, wc.hInstance, nullptr);
				ShowWindow(g_hSyncWnd, SW_SHOWNORMAL);
				UpdateWindow(g_hSyncWnd);
				UpdateEStopUi(g_hSyncWnd, true);
			}
			else {
				ShowWindow(g_hSyncWnd, SW_SHOWNORMAL);
				SetForegroundWindow(g_hSyncWnd);
				UpdateEStopUi(g_hSyncWnd, true);
			}
			return 0;
		}

		if (id == ID_BTN_SERIAL_WINDOW) {
			if (!g_hSerialWnd || !IsWindow(g_hSerialWnd)) {
				WNDCLASS wc{}; wc.lpszClassName = TEXT("WMX3SerialWnd");
				wc.lpfnWndProc = SerialWndProc; wc.hInstance = (HINSTANCE)GetWindowLongPtr(hWnd, GWLP_HINSTANCE);
				wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
				wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
				RegisterClass(&wc);
				g_hSerialWnd = CreateWindow(TEXT("WMX3SerialWnd"), TEXT("Serial Monitor (TCP)"),
					WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_SIZEBOX,
					CW_USEDEFAULT, CW_USEDEFAULT, 800, 400, hWnd, nullptr, wc.hInstance, nullptr);
				ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
				UpdateWindow(g_hSerialWnd);
			}
			else {
				ShowWindow(g_hSerialWnd, SW_SHOWNORMAL);
				SetForegroundWindow(g_hSerialWnd);
			}
			return 0;
		}

		for (int a = 0; a < 4; ++a) {
			if (id == ID_BTN_APPLY_ALT_A(a)) {
				if (!IsManualAllowed(hWnd)) return 0;
				int ecat6063 = 0;
				bool ok = ReadAxis0_TxPDO_6063(ecat6063);
				double alt = GetDlgDouble(hWnd, ID_EDIT_ALTTGT_A(a), 0.0);
				if (!ok) {
					MessageBox(hWnd, TEXT("0x6063 값을 읽을 수 없습니다."), TEXT("Apply Alt→Abs"), MB_ICONWARNING);
					return 0;
				}
				double tgt = alt - (double)ecat6063;
				SetDlgDouble(hWnd, ID_EDIT_POS_A(a), tgt);
				return 0;
			}
		}

		for (int a = 0; a < 4; ++a) {
			if (id == ID_BTN_SVON_A(a)) { if (!IsManualAllowed(hWnd)) return 0; long e = g_cm.axisControl->SetServoOn(a, 1); if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Servo ON 실패"), e, g_wmx); return 0; }
			if (id == ID_BTN_SVOFF_A(a)) { if (!IsManualAllowed(hWnd)) return 0; long e = g_cm.axisControl->SetServoOn(a, 0); if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Servo OFF 실패"), e, g_wmx); return 0; }
			if (id == ID_BTN_HOME_A(a)) {
				if (!IsManualAllowed(hWnd)) return 0;
				if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Home"), MB_ICONWARNING); return 0; }
				if (!EnsureServoOn(a)) return 0;
				g_cm.motion->Stop(a); g_cm.velocity->Stop(a); if (g_cm.torque) g_cm.torque->StopTrq(a);
				if (EnterPosMode(a)) { long e = g_home.StartHome(a); if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Home 시작 실패"), e, g_wmx); }
				return 0;
			}
			if (id == ID_BTN_ABS_A(a)) { if (!IsManualAllowed(hWnd)) return 0; if (g_demoRunning.load()) { MessageBox(hWnd, TEXT("Demo running. Stop demo first."), TEXT("Manual Move"), MB_ICONWARNING); return 0; } DoAbsMoveAxis(hWnd, a); return 0; }
			if (id == ID_BTN_REL_A(a)) { if (!IsManualAllowed(hWnd)) return 0; if (g_demoRunning.load()) { MessageBox(hWnd, TEXT("Demo running. Stop demo first."), TEXT("Manual Move"), MB_ICONWARNING); return 0; } DoRelMoveAxis(hWnd, a, +1); return 0; }
			if (id == ID_BTN_STOP_A(a)) { if (!IsManualAllowed(hWnd)) return 0; if (g_jogActiveAxis == a) StopJogIfActive(); StopAxis(a); return 0; }
		}
		if (id == ID_BTN_MULTI_ABS) { if (!IsManualAllowed(hWnd)) return 0; if (g_demoRunning.load()) { MessageBox(hWnd, TEXT("Demo running. Stop demo first."), TEXT("Manual Move"), MB_ICONWARNING); return 0; } DoMultiAbs(hWnd); return 0; }
		if (id == ID_BTN_MULTI_REL) { if (!IsManualAllowed(hWnd)) return 0; if (g_demoRunning.load()) { MessageBox(hWnd, TEXT("Demo running. Stop demo first."), TEXT("Manual Move"), MB_ICONWARNING); return 0; } DoMultiRel(hWnd); return 0; }
		if (id == ID_BTN_MULTI_STOP) {
			if (!IsManualAllowed(hWnd)) return 0;
			g_demoRunning = false;
			g_demoKind = DemoKind::None;
			StopMultiJog();
			for (int a = 0; a < 4; ++a) if (IsAxisChecked(hWnd, a)) StopAxis(a);
			for (int a = 0; a < 4; ++a) StopAxis(a);
			WaitAxesIdle({ 0,1,2,3 }, 5000);
			SetMotionAnnounce(hWnd, TEXT("Motion: Stop All - 데모/동작 정지"));
			return 0;
		}
		if (id == ID_BTN_MULTI_HOME) {
			if (!IsManualAllowed(hWnd)) return 0;
			if (!g_commStarted) { MessageBox(hWnd, TEXT("Start Communication first."), TEXT("Home(Selected)"), MB_ICONWARNING); return 0; }
			for (int a = 0; a < 4; ++a) if (IsAxisChecked(hWnd, a)) {
				if (!EnsureServoOn(a)) continue;
				g_cm.motion->Stop(a); g_cm.velocity->Stop(a); if (g_cm.torque) g_cm.torque->StopTrq(a);
				if (EnterPosMode(a)) { long e = g_home.StartHome(a); if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Home 시작 실패"), e, g_wmx); }
				Sleep(5);
			}
			return 0;
		}
		if (id == ID_BTN_MULTI_SVON) { if (!IsManualAllowed(hWnd)) return 0; for (int a = 0; a < 4; ++a) if (IsAxisChecked(hWnd, a)) { long e = g_cm.axisControl->SetServoOn(a, 1); if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Servo ON(Selected) 실패"), e, g_wmx); } return 0; }
		if (id == ID_BTN_MULTI_SVOFF) { if (!IsManualAllowed(hWnd)) return 0; StopMultiJog(); for (int a = 0; a < 4; ++a) if (IsAxisChecked(hWnd, a)) { long e = g_cm.axisControl->SetServoOn(a, 0); if (e != ErrorCode::None) ShowErrMsgBox(TEXT("Servo OFF(Selected) 실패"), e, g_wmx); } return 0; }
		if (id == ID_BTN_SELECT_ALL) { if (!IsManualAllowed(hWnd)) return 0; for (int a = 0; a < 4; ++a) SetAxisChecked(hWnd, a, true); UpdateSelectedAxesTextOnDemand(hWnd); return 0; }
		if (id == ID_BTN_CLEAR_ALL) { if (!IsManualAllowed(hWnd)) return 0; for (int a = 0; a < 4; ++a) SetAxisChecked(hWnd, a, false); UpdateSelectedAxesTextOnDemand(hWnd); return 0; }

		if (id == ID_BTN_MULTI_ALARM_RST) { if (!IsManualAllowed(hWnd)) return 0; DoMultiAlarmReset(hWnd); return 0; }
		if (id == ID_BTN_MAP_WINDOW) { if (!IsManualAllowed(hWnd)) return 0; ShowMapWindow(hWnd); return 0; }
	}
	return 0;

	case WM_APP_MAP_GOTO:
	{
		MapGotoParam* p = reinterpret_cast<MapGotoParam*>(lParam);
		if (p) {
			if (g_commStarted) {
				int axis = 0;
				if (EnsureServoOn(axis) && EnsurePosModeNoStop(axis)) {
					double v = GetDlgDouble(hWnd, ID_EDIT_VEL_A(axis), 20000.0);
					double ta = GetDlgDouble(hWnd, ID_EDIT_ACCT_A(axis), 100.0);
					double td = GetDlgDouble(hWnd, ID_EDIT_DECT_A(axis), 100.0);
					StartAbsMoveWithProfile(axis, p->target, v, ta, td);
				}
			}
			delete p;
		}
		return 0;
	}

	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
	case WM_MBUTTONUP:
	case WM_KILLFOCUS:
	case WM_CANCELMODE:
		StopJogIfActive(); StopMultiJog(); ReleaseCapture(); return 0;

	case WM_APP_TCP_LOG:
	{
		wchar_t* p = (wchar_t*)lParam;
		if (g_hTcpLogList && p && IsWindow(g_hTcpLogList)) {
			SendMessage(g_hTcpLogList, LB_ADDSTRING, 0, (LPARAM)p);
			int cnt = (int)SendMessage(g_hTcpLogList, LB_GETCOUNT, 0, 0);
			SendMessage(g_hTcpLogList, LB_SETTOPINDEX, cnt - 1, 0);
		}
		if (p) free(p);
		return 0;
	}

	case WM_APP_TCP_STATE:
	{
		wchar_t* p = (wchar_t*)lParam;
		if (p) {
			if (HWND h = GetDlgItem(hWnd, ID_TXT_TCP_STATE)) {
				SetWindowTextW(h, p);
			}
			free(p);
		}
		else {
			UpdateTcpUiState(hWnd);
		}
		return 0;
	}

	case WM_TIMER:
		// ====================== Axis2 ServoOn 확인 & 센서 활성화 지연 ======================
		if (wParam == ID_TIMER)
		{
			// ============================================
			// 0) Start Comm 여부 체크
			// ============================================
			if (!g_commStarted)
			{
				if (HWND h = GetDlgItem(hWnd, ID_TXT_ECAT_6063)) SetWindowText(h, TEXT("-"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_ECAT_603F)) SetWindowText(h, TEXT("-"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_LIMIT)) SetWindowText(h, TEXT("-"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_HOME))  SetWindowText(h, TEXT("-"));

				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX0_LIMIT_L)) SetWindowText(h, TEXT("-"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX0_LIMIT_R))  SetWindowText(h, TEXT("-"));

				UpdateEStopUi(hWnd, false);
				UpdateTcpUiState(hWnd);

				return 0;
			}

			// ============================================
			// 1) EtherCAT 상태 갱신 (항상 즉시 갱신됨)
			// ============================================
			g_cm.GetStatus(&g_status);
			UpdateStatus(hWnd);

			// ADDED: STO 펄스 자동 OFF (oht main에서 발생시킨 펄스)
			if (g_ohtStoPulsePendingOff.load()) {
				DWORD now = GetTickCount();
				if (now - g_ohtStoPulseOnTick.load() >= kOhtStoPulseMs) {
					g_ohtStoPulsePendingOff = false;
					ToggleDO_HW(10, false, nullptr);
				}
			}

			// IO 신호 새로고침(필요 시)
			if (g_ioInitDone.load()) {
				RefreshLevels(hWnd);
			}

			// ============================================
			// 2) Axis2 ServoOn 체크 & 1초 지연 타이머
			// ============================================
			if (!g_ax2ServoReady.load())
			{
				if (IsAxis2ServoOn())
				{
					if (g_ax2ServoOnTime.load() == 0)
					{
						g_ax2ServoOnTime = GetTickCount();
					}
					else
					{
						DWORD diff = GetTickCount() - g_ax2ServoOnTime.load();
						if (diff >= AX2_SENSOR_ENABLE_DELAY_MS)
						{
							g_ax2ServoReady = true;  // ★ 1초 지연 후 활성화 완료
						}
					}
				}
			}

			// ============================================
			// 3) Axis2 Limit / Home 센서 읽기
			//    (활성화 전 → WAIT)
			// ============================================
			bool limitRaw = ReadInputBit(AX2_LIMIT_ADDR, AX2_LIMIT_BIT, AX2_LIMIT_ACTIVE_HIGH);
			bool homeRaw = ReadInputBit(AX2_HOME_ADDR, AX2_HOME_BIT, AX2_HOME_ACTIVE_HIGH);

			g_ax2LimitOn = limitRaw;  // GUI 표시용
			g_ax2HomeOn = homeRaw;

			if (!g_ax2ServoReady.load())
			{
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_LIMIT)) SetWindowText(h, TEXT("WAIT"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_HOME))  SetWindowText(h, TEXT("WAIT"));
			}
			else
			{
				// 활성화 후 실제 값 표시
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_LIMIT)) SetWindowText(h, limitRaw ? TEXT("ON") : TEXT("OFF"));
				if (HWND h = GetDlgItem(hWnd, ID_TXT_AX2_HOME))  SetWindowText(h, homeRaw ? TEXT("ON") : TEXT("OFF"));
			}

			// ================= Axis0 L/R Limit 표시 =================
			// g_ax0LimitLFree / g_ax0LimitRFree 는:
			//   true  = free (리밋 안 물림)
			//   false = limit 감지 상태
			// UI는 직관적으로 "FREE"/"LIMIT" 로 표시
			if (HWND h = GetDlgItem(hWnd, ID_TXT_AX0_LIMIT_L)) {
				SetWindowText(h,
					g_ax0LimitLFree.load()
					? TEXT("OFF")    // 정상
					: TEXT("ON")); // L 리밋 감지
			}
			if (HWND h = GetDlgItem(hWnd, ID_TXT_AX0_LIMIT_R)) {
				SetWindowText(h,
					g_ax0LimitRFree.load()
					? TEXT("OFF")    // 정상
					: TEXT("ON")); // R 리밋 감지
			}

			// ============================================
			// 4) Axis2 Limit / Home 동작 처리 (ServoReady 후만!)
			// ============================================
			if (g_ax2ServoReady.load())
			{
				// ---------- Limit 처리 ----------
				if (limitRaw)
				{
					if (!g_ax2LimitLatched.load())
					{
						// 최초 감지
						g_ax2LimitLatched = true;
						g_ax2LimitBlocking = true;
						g_ax2StopIssuedOnLimit = false;
						g_ax2HomingStarted = false;
					}

					// 1회 Stop + Idle → Home 시퀀스
					Axis2HandleLimitOnceAndHome();
				}
				else
				{
					// Limit OFF → reset
					g_ax2LimitBlocking = false;
					g_ax2LimitLatched = false;
					g_ax2StopIssuedOnLimit = false;
					g_ax2HomingStarted = false;
					g_ax2LimitIdleTime = 0;
				}

				// ---------- Home 처리 (디바운스) ----------
				DWORD now = GetTickCount();

				if (homeRaw)
				{
					if (!g_ax2HomeDebounceOn.load())
					{
						if (now - g_ax2HomeLastTick.load() >= AX2_SENSOR_DEBOUNCE_MS)
						{
							g_ax2HomeDebounceOn = true;

							if (!g_ax2HomeRampIssued.load())
							{
								Axis2HomeSoftDecelTo500();
								g_ax2HomeRampIssued = true;
							}
						}
					}
				}
				else
				{
					// reset
					if (g_ax2HomeDebounceOn.load())
						g_ax2HomeDebounceOn = false;

					g_ax2HomeLastTick = now;
					g_ax2HomeRampIssued = false;
				}
			}

			// ============================================
		// 4-1) Axis0 Left/Right Limit 처리
		//      - 센서 미인식: true, 인식(리밋): false 기준
		//      - L 리밋: + 방향 차단, R 리밋: - 방향 차단
		// ============================================
			{
				// 센서 읽기 결과를 "free" 개념으로 사용
				bool leftFree = ReadInputBit(AX0_LIMIT_L_ADDR, AX0_LIMIT_L_BIT, AX0_LIMIT_L_ACTIVE_HIGH);
				bool rightFree = ReadInputBit(AX0_LIMIT_R_ADDR, AX0_LIMIT_R_BIT, AX0_LIMIT_R_ACTIVE_HIGH);

				g_ax0LimitLFree = leftFree;
				g_ax0LimitRFree = rightFree;

				// ----- Left limit: free(false -> true가 아니라 free(true -> false) 변화가 리밋 인입) -----
				if (!leftFree)
				{
					// L 리밋 처음 감지 시 급정지
					if (!g_ax0LimitLLatched.exchange(true))
					{
						StopAxis(0);      // Axis0 급정지
					}
					// L 리밋 ON 동안 + 방향 명령 차단
					g_ax0BlockPlus = true;
				}
				else
				{
					// L 리밋 해제
					g_ax0LimitLLatched = false;
					g_ax0BlockPlus = false;
				}

				// ----- Right limit -----
				if (!rightFree)
				{
					// R 리밋 처음 감지 시 급정지
					if (!g_ax0LimitRLatched.exchange(true))
					{
						StopAxis(0);      // Axis0 급정지
					}
					// R 리밋 ON 동안 - 방향 명령 차단
					g_ax0BlockMinus = true;
				}
				else
				{
					// R 리밋 해제
					g_ax0LimitRLatched = false;
					g_ax0BlockMinus = false;
				}
			}

			// ============================================
			// 5) Zone 관련 처리 (기존 유지)
			// ============================================
			if (g_demoKind.load() == DemoKind::PulseZones)
				UpdateZoneAnnounce_Pulse(hWnd);
			else if (g_demoKind.load() == DemoKind::SensorZones)
				UpdateZoneAnnounce_Sensor(hWnd);
			else
				UpdateZoneAnnounce_Pulse(hWnd);

			return 0;
		}
		return 0;

	case WM_DESTROY:
		KillTimer(hWnd, ID_TIMER);
		StopJogIfActive(); StopMultiJog();
		g_demoRunning = false;
		for (int a = 0; a < 4; ++a) StopAxis(a);
		StopTcpServer();

		// LOG/SCOPE: thread stop & CS cleanup
		//g_logThreadRun = false;
		//DeleteCriticalSection(&g_logCs);
		// StopDataLogFile()는 스레드 종료 경로에서 호출됨

		ShutdownWMX();
		PostQuitMessage(0);
		return 0;
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}

// ------------------ WinMain ------------------
int APIENTRY _tWinMain(HINSTANCE hInst, HINSTANCE, LPTSTR, int nShow) {
	WNDCLASS wc{};
	wc.lpszClassName = TEXT("WMX3PracticeWnd");
	wc.lpfnWndProc = WndProc; wc.hInstance = hInst;
	wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
	RegisterClass(&wc);

	HWND h = CreateWindow(TEXT("WMX3PracticeWnd"), TEXT("WMX3 Practice - Demo + GPIO; Sync Group + ECAT6063/603F + AltTarget + E-Stop + SerialWnd + Log/Scope"),
		WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
		CW_USEDEFAULT, CW_USEDEFAULT, 1850, 1080, nullptr, nullptr, hInst, nullptr);
	ShowWindow(h, nShow);
	UpdateWindow(h);

	MSG m;
	while (GetMessage(&m, nullptr, 0, 0)) { TranslateMessage(&m); DispatchMessage(&m); }
	return (int)m.wParam;
}

int main() { return _tWinMain(GetModuleHandle(nullptr), nullptr, GetCommandLine(), SW_SHOWNORMAL); }