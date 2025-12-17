#define NOMINMAX
#include <windows.h>
#include <tchar.h>
#include <cmath>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <stdint.h>
#include <algorithm>

// -------- WMX3 / CoreMotion --------
#include "IOApi.h"
#include "WMX3Api.h"
#include "CoreMotionApi.h"

// 외부 WMX3 심볼
using namespace wmx3Api;
extern wmx3Api::WMX3Api g_wmx;
extern wmx3Api::CoreMotion g_cm;
extern Home g_home;

extern bool g_deviceOpened;
extern bool g_commStarted;

extern bool EnsureServoOn(int axis);
extern bool EnsurePosModeNoStop(int axis);
extern int  TimeMsToAcc(double vel_cnt_per_s, double t_ms);
extern bool StartAbsMoveWithProfile(int axis, long long target, double vpps, double tAcc, double tDec);
extern void StopAxis(int axis);

// ========== EtherCAT 0x6063 읽기 ==========
extern bool ReadAxis_TxPDO_6063(int slaveId, int& outVal);
extern const int kAxisSlaveId[4] = { 0, 1, 2, 3 };

// 현재 위치(주행) 코드: Load=0x01, Unload=0x02, 그 외=0x00
extern inline unsigned char CalcPosTravelCode();
extern unsigned char CalcPosGripCode();

extern inline bool IsAxis2LimitOn();


// 외부 atEAPI
#include "atEAPI.h"

#pragma comment(lib, "User32.lib")
#pragma comment(lib, "Gdi32.lib")
#pragma comment(lib, "Comctl32.lib")

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

const TCHAR* TaskName(TaskId id) {
    switch (id) {
    case TaskId::GoWorkstation:      return TEXT("Workstation");
    case TaskId::GoConveyor:         return TEXT("Conveyor");
    case TaskId::LiftUp:             return TEXT("Lift Up");
    case TaskId::WorkDown:           return TEXT("Work Down");
    case TaskId::ConveyorDown:       return TEXT("Conveyor Down");
    case TaskId::GripOpen:           return TEXT("Grip Open");
    case TaskId::GripClose:          return TEXT("Grip Close");
    case TaskId::GripServoOff:       return TEXT("Grip Servo OFF");
    case TaskId::DemoHomeWithBox:    return TEXT("Demo Home(Box)");
    case TaskId::DemoHomeWithoutBox: return TEXT("Demo Home(NoBox)");
    case TaskId::DemoLoad:           return TEXT("Demo Load");
    case TaskId::DemoUnload:         return TEXT("Demo Unload");
    default: return TEXT("Unknown");
    }
}
const TCHAR* TaskStateStr(TaskState s) {
    switch (s) {
    case TaskState::Idle:    return TEXT("대기");
    case TaskState::Running: return TEXT("진행중");
    case TaskState::Done:    return TEXT("완료");
    case TaskState::Failed:  return TEXT("실패");
    case TaskState::Stopped: return TEXT("중단");
    default: return TEXT("-");
    }
}
struct TaskStatus {
    std::atomic<TaskState> state{ TaskState::Idle };
    std::atomic<DWORD>     lastChangeTick{ 0 };
};
TaskStatus g_taskStatus[(int)TaskId::COUNT];

TaskState GetTaskState(TaskId id) {
    return g_taskStatus[(int)id].state.load();
}

// =======================================
// 공통 UI 유틸
// =======================================
static HFONT MakeUIFont(int pt, int weight = FW_NORMAL) {
    LOGFONT lf = { 0 };
    HDC sdc = GetDC(NULL);
    int logPix = GetDeviceCaps(sdc, LOGPIXELSY);
    ReleaseDC(NULL, sdc);
    lf.lfHeight = -MulDiv(pt, logPix, 72);
    lf.lfWeight = weight;
#ifdef UNICODE
    wcscpy_s(lf.lfFaceName, L"Segoe UI");
#else
    strcpy_s(lf.lfFaceName, "Segoe UI");
#endif
    return CreateFontIndirect(&lf);
}

// =======================================
// Demo 상태 표시
// =======================================
HWND g_hDemoWnd = nullptr;
HWND g_hStatusStatics[(int)TaskId::COUNT] = { 0 };
void SetTaskState(TaskId id, TaskState st) {
    int idx = (int)id;
    g_taskStatus[idx].state.store(st, std::memory_order_relaxed);
    g_taskStatus[idx].lastChangeTick.store(GetTickCount(), std::memory_order_relaxed);
    if (g_hDemoWnd && g_hStatusStatics[idx]) {
        std::wstring text = std::wstring(TaskName(id)) + L": " + TaskStateStr(st);
        SetWindowTextW(g_hStatusStatics[idx], text.c_str());
    }
}
void ResetAllTaskStates() {
    for (int i = 0; i < (int)TaskId::COUNT; ++i)
        SetTaskState((TaskId)i, TaskState::Idle);
}

// =======================================
// Barcode follower (기존 유지)
// =======================================
struct BarcodeParams {
    int bcAxis = 0;      // ✅ 6063 읽는 축 (0번)
    int moveAxis = 1;    // ✅ 실제 주행 축 (1번)
    long long targetBarcodeAbs;
    double mainVel = 10000.0;
    double mainAcc = 1000.0;
    double mainDec = 1000.0;
    double corrVel = 1000.0;
    double corrAcc = 1000.0;
    double corrDec = 1000.0;
    int deadband = 2;
    double gear = 4.3;
    double wheelDia = 70.0;
    double motorCpr = 10000.0;
    double bcMmPerCnt = 0.1;
};
class BarcodeFollower {
public:
    void Start(const BarcodeParams& p, TaskId taskToReport) {
        Stop();
        params_ = p;
        reportTask_ = taskToReport;
        running_ = true;
        worker_ = std::thread(&BarcodeFollower::ThreadProc, this);
        worker_.detach();
    }
    void Stop() {
        running_ = false;
        if (params_.moveAxis >= 0 && params_.moveAxis < 4) {
            StopAxis(params_.moveAxis);
        }
    }
    bool IsRunning() const { return running_.load(); }
private:
    std::atomic<bool> running_{ false };
    BarcodeParams params_{};
    TaskId reportTask_ = TaskId::GoWorkstation;

    bool inCorr_ = false;
    bool finalSnapSent_ = false;
    bool stopDelayActive_ = false;
    int  stopDelayTimer_ = 0;

    static constexpr DWORD POLL_MS = 30;
    static constexpr long long kFinalCheckErr = 2;

    int ComputeAutoStartErr(double mainVel) { return (int)std::llround(mainVel * 0.01); }
    bool Read6063Now(int axis, int& out) {
        if (axis < 0 || axis >= 4) return false;
        return ReadAxis_TxPDO_6063(kAxisSlaveId[axis], out);
    }

    struct HbcSnapshot { double gear; double wheelDia; double motorCpr; double bcMmPerCnt; };
    static inline double HBC_pulsesPerMm(const HbcSnapshot& s) {
        return s.motorCpr / (3.14159265358979323846 * s.wheelDia);
    }
    static inline double HBC_bcToMm(const HbcSnapshot& s, long long bc) {
        return bc * s.bcMmPerCnt;
    }
    static inline long long HBC_mmToPulses(const HbcSnapshot& s, double mm) {
        double pulses = mm * HBC_pulsesPerMm(s);
        return (long long)std::llround(pulses);
    }
    static inline long long HBC_bcToPulses(const HbcSnapshot& s, long long bc) {
        return HBC_mmToPulses(s, HBC_bcToMm(s, bc));
    }
    void IssueAbsWithProfile(int axis, long long absTarget, double vpps, double a_ms, double d_ms) {
        StartAbsMoveWithProfile(axis, absTarget, vpps, a_ms, d_ms);
    }
    void ThreadProc() {
        const int bcAx = params_.bcAxis;       // ✅ 바코드 읽기
        const int mvAx = params_.moveAxis;     // ✅ 모션 주행

        if (!g_commStarted || bcAx < 0 || bcAx >= 4 || mvAx < 0 || mvAx >= 4) { SetTaskState(reportTask_, TaskState::Failed); running_ = false; return; }
        if (!EnsureServoOn(mvAx) || !EnsurePosModeNoStop(mvAx)) { SetTaskState(reportTask_, TaskState::Failed); running_ = false; return; }
        SetTaskState(reportTask_, TaskState::Running);

        inCorr_ = false; finalSnapSent_ = false; stopDelayActive_ = false; stopDelayTimer_ = 0;

        int now6063 = 0;
        if (!Read6063Now(bcAx, now6063)) { SetTaskState(reportTask_, TaskState::Failed); running_ = false; return; }

        const long long targetBarcodeAbs = params_.targetBarcodeAbs;
        HbcSnapshot snap{ params_.gear, params_.wheelDia, params_.motorCpr, params_.bcMmPerCnt };
        long long bcErr0 = targetBarcodeAbs - (long long)now6063;
        long long targetMotorPulse = HBC_bcToPulses(snap, bcErr0);

        CoreMotionStatus st{};
        g_cm.GetStatus(&st);
        long long curPos = (long long)st.axesStatus[mvAx].actualPos;
        long long firstTarget = curPos + targetMotorPulse;
        IssueAbsWithProfile(mvAx, firstTarget, params_.mainVel, params_.mainAcc, params_.mainDec);

        int startBcErr = ComputeAutoStartErr(params_.mainVel);
        const int deadband = params_.deadband;
        bool completed = false;

        while (running_.load()) {
            if (!Read6063Now(bcAx, now6063)) { completed = false; break; }
            long long bcErr = targetBarcodeAbs - (long long)now6063;
            long long eAbs = llabs(bcErr);
            // === deadband 누적 체류 시간 로직 ===
            const int deadband = params_.deadband;
            bool inDeadband = (eAbs <= deadband);

            if (inDeadband) {
                // deadband 처음 진입한 순간에만 타이머 초기화
                if (!stopDelayActive_) {
                    stopDelayActive_ = true;

                    // 새 시퀀스 시작할 때 ThreadProc 앞에서 stopDelayTimer_ = 0 이므로
                    // 처음 진입 시에만 70으로 세팅됨
                    if (stopDelayTimer_ == 0) {
                        stopDelayTimer_ = 70;   // 약 2.1초 (누적)
                    }
                }

                // deadband 안에 있는 동안에만 타이머 감소
                if (stopDelayActive_) {
                    if (--stopDelayTimer_ <= 0) {
                        completed = true;
                        break;
                    }
                }
            }
            else {
                // deadband 밖: 타이머 “멈춤” (값 유지, 감소 X)
                // stopDelayActive_는 true 상태로 둬도 되고,
                // 필요하다면 flag를 분리해도 됨.
                // 여기서는 아무것도 안 건드려서 단순히 pause 효과.
            }

            if (!inCorr_ && eAbs <= startBcErr) inCorr_ = true;

            if (inCorr_) {
                g_cm.GetStatus(&st);
                long long cur = (long long)st.axesStatus[mvAx].actualPos;
                double actVel = std::fabs(st.axesStatus[mvAx].actualVelocity);
                if (!finalSnapSent_) {
                    long long remainingPulses = HBC_bcToPulses(snap, bcErr);
                    long long absTarget = cur + remainingPulses;
                    IssueAbsWithProfile(mvAx, absTarget, params_.corrVel, params_.corrAcc, params_.corrDec);
                    finalSnapSent_ = true;
                }
                else {
                    if (actVel < 1.0 && eAbs > kFinalCheckErr) {
                        long long remainingPulses = HBC_bcToPulses(snap, bcErr);
                        long long absTarget = cur + remainingPulses;
                        IssueAbsWithProfile(mvAx, absTarget, params_.corrVel, params_.corrAcc, params_.corrDec);
                    }
                }
            }
            ::Sleep(30);
        }
        if (!running_.load()) SetTaskState(reportTask_, TaskState::Stopped);
        else SetTaskState(reportTask_, completed ? TaskState::Done : TaskState::Failed);

        running_ = false; inCorr_ = false; finalSnapSent_ = false; stopDelayActive_ = false; stopDelayTimer_ = 0;
    }
    std::thread worker_;
};
static BarcodeFollower g_bcRunner;

// =======================================
// Move monitors (기존 유지)
// =======================================
struct MoveMonitorArgs {
    int axis = -1;
    long long target = 0;
    TaskId task;
    double posEps = 10.0;
    double velEps = 2.0;
    DWORD timeoutMs = 15000;
    bool treatStoppedAsDone = false;
};
static void StartMoveAndMonitor(const MoveMonitorArgs& m, double vpps, double accMs, double decMs) {
    if (!g_commStarted) { SetTaskState(m.task, TaskState::Failed); return; }
    if (m.axis < 0 || m.axis >= 4) { SetTaskState(m.task, TaskState::Failed); return; }
    if (!EnsureServoOn(m.axis) || !EnsurePosModeNoStop(m.axis)) {
        SetTaskState(m.task, TaskState::Failed);
        return;
    }

    SetTaskState(m.task, TaskState::Running);

    if (!StartAbsMoveWithProfile(m.axis, m.target, vpps, accMs, decMs)) {
        SetTaskState(m.task, TaskState::Failed);
        return;
    }

    std::thread([m]() {
        DWORD start = GetTickCount();
        CoreMotionStatus st{};
        TaskState result = TaskState::Failed;

        bool   started = false;  // ★ 실제로 “가동을 시작했는지”
        double initialPos = 0.0;
        bool   haveInitPos = false;

        // “움직였다고 인정할” 기준값들
        const double kStartVelThreshold = m.velEps * 2.0; // 이 속도 이상 나와야 시작으로 인정
        const double kStartMoveThreshold = m.posEps;       // 위치 변화도 기준 (필요시 더 작게)

        while (true) {
            // 타임아웃
            if (GetTickCount() - start > m.timeoutMs) {
                result = TaskState::Failed;
                break;
            }

            g_cm.GetStatus(&st);
            auto& ax = st.axesStatus[m.axis];
            double v = std::fabs(ax.actualVelocity);
            double pos = (double)ax.actualPos;

            if (!haveInitPos) {
                initialPos = pos;
                haveInitPos = true;
            }

            double moved = std::fabs(pos - initialPos);
            double e = std::fabs(pos - (double)m.target);

            // ★ 아직 “시작했다고 인정” 안 된 상태라면,
            //   충분히 움직이거나 속도가 컸던 적이 있는지 먼저 체크
            if (!started) {
                if (v > kStartVelThreshold || moved > kStartMoveThreshold) {
                    started = true;   // 이제부터는 v<velEps를 “멈춤”으로 인정
                }
                // 시작 전에는 v가 0이어도 그냥 계속 기다림
                ::Sleep(30);
                continue;
            }

            // ★ 여기부터는 “한번은 제대로 움직인 후” 라고 가정
            if (v < m.velEps) {
                if (e <= m.posEps) {
                    // 목표 위치 근처에서 멈춤 → 정상 완료
                    result = TaskState::Done;
                }
                else {
                    // 목표에서 멀리 멈춤 (리밋 등)
                    // 이 모션이 그걸 성공으로 인정하는지 여부에 따라
                    result = m.treatStoppedAsDone
                        ? TaskState::Done
                        : TaskState::Stopped;
                }
                break;
            }

            ::Sleep(30);
        }

        SetTaskState(m.task, result);
        }).detach();
}

struct ApproachProfile {
    double vpps = 1000.0;
    double accMs = 100.0;
    double decMs = 100.0;
};
// Axis2HomeSoftDecelTo500() 스타일을 일반화한 Approach 버전
static void StartMoveWithApproach(int axis, long long target, TaskId task,
    double mainVpps, double mainAccMs, double mainDecMs,
    double posEps, double velEps, DWORD timeoutMs,
    double approachEps, const ApproachProfile& ap)
{
    if (!g_commStarted) {
        SetTaskState(task, TaskState::Failed);
        return;
    }
    if (axis < 0 || axis >= 4) {
        SetTaskState(task, TaskState::Failed);
        return;
    }
    if (!EnsureServoOn(axis) || !EnsurePosModeNoStop(axis)) {
        SetTaskState(task, TaskState::Failed);
        return;
    }

    // 1) 먼저 target까지 "빠른" 프로파일로 이동 시작 (5000 등)
    Motion::PosCommand fast{};
    fast.axis = axis;
    fast.profile.type = ProfileType::SCurve;
    fast.profile.velocity = (int)std::lround(mainVpps);
    fast.profile.acc = TimeMsToAcc(fast.profile.velocity, mainAccMs);
    fast.profile.dec = TimeMsToAcc(fast.profile.velocity, mainDecMs);
    fast.target = target;

    long err = g_cm.motion->StartPos(&fast);
    if (err != ErrorCode::None) {
        SetTaskState(task, TaskState::Failed);
        return;
    }

    SetTaskState(task, TaskState::Running);

    // 2) 모니터링 스레드: 3단계 상태
    //   - phase 0 : 빠른 구간 (그냥 감시만)
    //   - phase 1 : Axis2HomeSoftDecelTo500 패턴으로 "급 감속용 소타겟" 이동
    //   - phase 2 : 저속(1000)으로 최종 target 접근
    std::thread([=]() {
        DWORD startTick = GetTickCount();
        CoreMotionStatus st{};
        TaskState result = TaskState::Failed;

        enum Phase { FastRun = 0, SoftDecel, FinalApproach };
        Phase phase = FastRun;

        long long softTarget = 0;   // 급감속용 중간 타겟
        bool softTargetSet = false;

        while (true) {
            // ★★★ 1) 전역 Stop(DoStopAll 등) 감지: task가 Running이 아니면 즉시 종료
            TaskState cur = g_taskStatus[(int)task].state.load(std::memory_order_relaxed);
            if (cur == TaskState::Stopped || cur == TaskState::Failed) {
                result = cur;   // 보통 Stopped
                break;
            }

            if (GetTickCount() - startTick > timeoutMs) {
                result = TaskState::Failed;
                break;
            }

            g_cm.GetStatus(&st);
            const auto& axst = st.axesStatus[axis];

            double actPos = (double)axst.actualPos;
            double actVel = std::fabs(axst.actualVelocity);
            double remErr = std::fabs((double)target - actPos);

            if (phase == FastRun) {
                // 아직 빠른 구간: 남은 거리 remErr 를 체크
                // remErr <= approachEps (예: 3500) 이 되면
                // Axis2HomeSoftDecelTo500 스타일로 "현재 위치 기준 소타겟"으로 급감속
                if (remErr <= approachEps) {
                    // 현재 위치, 진행 방향 계산
                    long long cur = (long long)axst.actualPos;
                    long long dist = target - cur;
                    if (dist == 0) {
                        // 이미 타겟 가까우면 바로 종료 처리
                        if (actVel < velEps && remErr <= posEps) {
                            result = TaskState::Done;
                        }
                        else {
                            result = TaskState::Stopped;
                        }
                        break;
                    }

                    int dir = (dist > 0) ? +1 : -1;

                    // Axis2HomeSoftDecelTo500 처럼 "현재 위치에서 작은 step" 만큼
                    // 앞쪽으로 소타겟 잡기. (여기서는 remErr의 절반 정도, 최대 4000 제한)
                    long long maxStep = 4000;  // 필요하면 조정
                    long long step = (long long)remErr / 2;
                    if (step > maxStep) step = maxStep;
                    if (step < 1000)   step = 1000;  // 너무 짧으면 감속 효과가 약하니 최소값

                    step *= dir;
                    softTarget = cur + step;

                    // 혹시 소타겟이 target 을 넘어가지 않도록 보정
                    if (dir > 0 && softTarget > target) softTarget = target;
                    if (dir < 0 && softTarget < target) softTarget = target;

                    // === 1단계 급감속: Axis2HomeSoftDecelTo500 과 같은 방식 ===
                    Motion::PosCommand pc{};
                    pc.axis = axis;
                    pc.target = softTarget;
                    pc.profile.type = ProfileType::SCurve;
                    pc.profile.velocity = (int)std::lround(ap.vpps); // 최종에서 쓸 저속과 비슷하게
                    pc.profile.acc = TimeMsToAcc(pc.profile.velocity, ap.accMs);
                    pc.profile.dec = TimeMsToAcc(pc.profile.velocity, ap.decMs); // decMs 작은 값으로 "확" 감속

                    g_cm.motion->StartPos(&pc);

                    softTargetSet = true;
                    phase = SoftDecel;
                }
            }
            else if (phase == SoftDecel) {
                // 급감속 소타겟(softTarget)으로 가는 중
                // 속도가 충분히 줄고, softTarget 근처에 오면 → 최종 타겟으로 저속 접근 시작
                double eSoft = 0.0;
                if (softTargetSet)
                    eSoft = std::fabs((double)softTarget - actPos);

                // "충분히 감속되었다" 판단 기준:
                //  - 속도 actVel 이 ap.vpps 근처 or 매우 낮아졌을 때
                //  - 소타겟 근처(eSoft <= posEps * 2 정도)
                if (softTargetSet &&
                    actVel <= (ap.vpps + 50.0) &&     // 속도 기준 (적당히 여유)
                    eSoft <= (posEps * 2.0))
                {
                    // === 2단계: 이제 저속(1000)으로 최종 target까지 이동 ===
                    Motion::PosCommand pc2{};
                    pc2.axis = axis;
                    pc2.target = target;
                    pc2.profile.type = ProfileType::SCurve;
                    pc2.profile.velocity = (int)std::lround(ap.vpps);
                    pc2.profile.acc = TimeMsToAcc(pc2.profile.velocity, ap.accMs);
                    pc2.profile.dec = TimeMsToAcc(pc2.profile.velocity, ap.decMs);

                    g_cm.motion->StartPos(&pc2);
                    phase = FinalApproach;
                }
            }
            else if (phase == FinalApproach) {
                // 최종 저속 접근 단계: 평소처럼 Done/Stopped 판정
                if (actVel < velEps) {
                    if (remErr <= posEps) {
                        result = TaskState::Done;
                    }
                    else {
                        result = TaskState::Stopped;
                    }
                    break;
                }
            }

            ::Sleep(10);
        }

        SetTaskState(task, result);
        }).detach();
}

// =======================================
// Axis2 Limit/Home 센서 (기존 유지)
// =======================================
static HWND g_hAx2LimitStatic = nullptr;
static HWND g_hAx2HomeStatic = nullptr;
static const int AX2_LIMIT_ADDR = 8;
static const int AX2_LIMIT_BIT = 1;
static const int AX2_HOME_ADDR = 8;
static const int AX2_HOME_BIT = 2;
static const bool AX2_LIMIT_ACTIVE_HIGH = true;
static const bool AX2_HOME_ACTIVE_HIGH = true;
static const DWORD AX2_SENSOR_POLL_MS = 5;
static const DWORD AX2_SENSOR_DEBOUNCE_MS = 5;
static std::atomic<bool> g_ax2LimitOn{ false };
static std::atomic<bool> g_ax2HomeOn{ false };
static std::atomic<bool>  g_ax2LimitLatched{ false };
static std::atomic<bool>  g_ax2LimitBlocking{ false };
static std::atomic<bool>  g_ax2StopIssuedOnLimit{ false };
static std::atomic<bool>  g_ax2HomingStarted{ false };
static std::atomic<bool>  g_ax2HomeDebounceOn{ false };
static std::atomic<DWORD> g_ax2HomeLastTick{ 0 };
static std::atomic<bool>  g_ax2HomeRampIssued{ false };
static std::atomic<DWORD> g_ax2LimitIdleTime{ 0 };
static std::atomic<bool>  g_ax2ServoReady{ false };
static std::atomic<DWORD> g_ax2ServoOnTime{ 0 };
static const DWORD AX2_SENSOR_ENABLE_DELAY_MS = 1500;
static const double vel_idle_threshold = 1.0;
static const long long inpos_tol_counts = 10;

static bool IsAxis2ServoOn()
{
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    return st.axesStatus[2].servoOn;
}
static bool ReadInputBitRaw(int addr, int bit, bool activeHigh) {
    if (!g_commStarted) return false;
    Io io(&g_wmx);
    unsigned char v = 0;
    long e = io.GetInBitEx(addr, bit, &v);
    if (e != ErrorCode::None) return false;
    bool onRaw = (v != 0);
    return activeHigh ? onRaw : !onRaw;
}
static bool Axis2IsIdle() {
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    int av = (int)std::lround(st.axesStatus[2].actualVelocity);
    long long perr = (long long)st.axesStatus[2].posCmd - (long long)st.axesStatus[2].actualPos;
    return (std::abs(av) <= vel_idle_threshold) && (std::llabs(perr) <= inpos_tol_counts);
}
static int Axis2CurrentDir() {
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    double vcmd = st.axesStatus[2].velocityCmd;
    if (vcmd > vel_idle_threshold) return +1;
    if (vcmd < -vel_idle_threshold) return -1;
    return 0;
}
static void Axis2HomeSoftDecelTo500() {
    if (!g_commStarted) return;
    if (Axis2IsIdle()) return;
    if (!EnsureServoOn(2) || !EnsurePosModeNoStop(2)) return;
    CoreMotionStatus st{}; g_cm.GetStatus(&st);
    long long cur = (long long)st.axesStatus[2].actualPos;
    int dir = Axis2CurrentDir(); if (dir == 0) dir = +1;
    long long smallStep = 5000 * dir;
    long long softTarget = cur + smallStep;
    double newVel = 500.0;
    double accMs = 80.0;
    double decMs = 10.0;
    Motion::PosCommand pc{};
    pc.axis = 2; pc.target = softTarget;
    pc.profile.type = ProfileType::SCurve;
    pc.profile.velocity = (int)std::lround(newVel);
    pc.profile.acc = TimeMsToAcc(pc.profile.velocity, accMs);
    pc.profile.dec = TimeMsToAcc(pc.profile.velocity, decMs);
    g_cm.motion->StartPos(&pc);
    g_ax2HomeRampIssued = true;
}
static void Axis2HandleLimitOnceAndHome() {
    if (!g_commStarted) return;
    if (!g_ax2LimitLatched.load()) return;
    if (!g_ax2StopIssuedOnLimit.exchange(true)) {
        StopAxis(2);
    }
    if (!Axis2IsIdle()) { g_ax2LimitIdleTime = 0; return; }
    if (g_ax2LimitIdleTime.load() == 0) { g_ax2LimitIdleTime = GetTickCount(); return; }
    DWORD elapsed = GetTickCount() - g_ax2LimitIdleTime.load();
    if (elapsed < 2000) return;
    if (!g_ax2HomingStarted.load()) {
        if (EnsureServoOn(2) && EnsurePosModeNoStop(2)) {
            g_home.StartHome(2);
            g_ax2HomingStarted = true;
        }
    }
}
static void UpdateAx2SensorLabels(bool ready, bool limitOn, bool homeOn) {
    if (!g_hDemoWnd) return;
    if (g_hAx2LimitStatic) {
        if (!ready) SetWindowText(g_hAx2LimitStatic, TEXT("Axis2 LIMIT: WAIT"));
        else        SetWindowText(g_hAx2LimitStatic, limitOn ? TEXT("Axis2 LIMIT: ON") : TEXT("Axis2 LIMIT: OFF"));
    }
    if (g_hAx2HomeStatic) {
        if (!ready) SetWindowText(g_hAx2HomeStatic, TEXT("Axis2 HOME: WAIT"));
        else        SetWindowText(g_hAx2HomeStatic, homeOn ? TEXT("Axis2 HOME: ON") : TEXT("Axis2 HOME: OFF"));
    }
}
void Axis2SensorInit()
{
    g_ax2LimitOn = false; g_ax2HomeOn = false;
    g_ax2LimitLatched = false; g_ax2LimitBlocking = false; g_ax2StopIssuedOnLimit = false; g_ax2HomingStarted = false;
    g_ax2HomeDebounceOn = false; g_ax2HomeLastTick = GetTickCount(); g_ax2HomeRampIssued = false;
    g_ax2LimitIdleTime = 0;
    g_ax2ServoReady = false; g_ax2ServoOnTime = 0;
}
void Axis2SensorTimerProc(HWND)
{
    if (!g_commStarted) { UpdateAx2SensorLabels(false, false, false); return; }
    if (!g_ax2ServoReady.load()) {
        if (IsAxis2ServoOn()) {
            if (g_ax2ServoOnTime.load() == 0) { g_ax2ServoOnTime = GetTickCount(); }
            else {
                DWORD diff = GetTickCount() - g_ax2ServoOnTime.load();
                if (diff >= AX2_SENSOR_ENABLE_DELAY_MS) g_ax2ServoReady = true;
            }
        }
    }
    bool limitRaw = ReadInputBitRaw(AX2_LIMIT_ADDR, AX2_LIMIT_BIT, AX2_LIMIT_ACTIVE_HIGH);
    bool homeRaw = ReadInputBitRaw(AX2_HOME_ADDR, AX2_HOME_BIT, AX2_HOME_ACTIVE_HIGH);
    g_ax2LimitOn = limitRaw; g_ax2HomeOn = homeRaw;
    UpdateAx2SensorLabels(g_ax2ServoReady.load(), limitRaw, homeRaw);
    if (g_ax2ServoReady.load()) {
        if (limitRaw) {
            if (!g_ax2LimitLatched.load()) {
                g_ax2LimitLatched = true;
                g_ax2LimitBlocking = true;
                g_ax2StopIssuedOnLimit = false;
                g_ax2HomingStarted = false;
            }
            Axis2HandleLimitOnceAndHome();
        }
        else {
            g_ax2LimitBlocking = false;
            g_ax2LimitLatched = false;
            g_ax2StopIssuedOnLimit = false;
            g_ax2HomingStarted = false;
            g_ax2LimitIdleTime = 0;
        }
        DWORD now = GetTickCount();
        if (homeRaw) {
            if (!g_ax2HomeDebounceOn.load()) {
                if (now - g_ax2HomeLastTick.load() >= AX2_SENSOR_DEBOUNCE_MS) {
                    g_ax2HomeDebounceOn = true;
                    if (!g_ax2HomeRampIssued.load()) {
                        Axis2HomeSoftDecelTo500();
                        g_ax2HomeRampIssued = true;
                    }
                }
            }
        }
        else {
            if (g_ax2HomeDebounceOn.load()) g_ax2HomeDebounceOn = false;
            g_ax2HomeLastTick = now;
            g_ax2HomeRampIssued = false;
        }
    }
}

// =======================================
// GPIO 창 로직을 그대로 통합
// - EAPI 래퍼, 은행 선택, 디바운스, 펄스, 토글 컨트롤, 오버레이 포함
// =======================================
#define BANK_MAX 4
typedef struct {
    uint8_t supPinNum;
    uint32_t supInput;
    uint32_t supOutput;
} GPIOInfo, * PGPIOInfo;

static HINSTANCE g_hEAPIDLL = NULL;
static GPIOInfo g_gpioInfo[BANK_MAX];
static const int kFixedBank = -1;
static int g_gpioBank = -1;

// 마스크
static const uint32_t kMaskDI0_7 = 0x000000FFu;
static const uint32_t kMaskDO8_15 = 0x0000FF00u;

// 출력 펄스 예약
bool  g_outputPendingOff[16] = { false };
DWORD g_outputOnTick[16] = { 0 };
const DWORD kOutputPulseMs = 50;

// 출력 후 입력 안정화 지연
static const UINT kPostOutputInputDelayMs = 8;

// 입력 디바운스
static const int  kDI_SampleCount = 5;
const UINT kPollIntervalMs = 20; // 빠른 폴링
static const int  kDI_BufferDepth = kDI_SampleCount;

// DI 오버레이: Motioning(DI0) 표시 강제 OFF를 위해 사용
static bool g_uiOverlayDIValid[8] = { false };
static bool g_uiOverlayDILevel[8] = { false };

static HWND g_lblDI[8] = { 0 };  // 좌측 DI 라벨
static bool g_cachedLevels[16] = { 0 };

// 토글 컨트롤
static const TCHAR* TOGGLE_CLS = _T("GPIO_CTRL_TOGGLE");
struct ToggleState { bool on = false; bool enabled = true; int pin = -1; };
static HFONT g_fontToggle = NULL;
static HWND  g_swDO[16] = { 0 }; // DO8..15 사용

// 외부 함수 포인터 래퍼
static HINSTANCE GetEAPIInstance() {
    if (g_hEAPIDLL == NULL) g_hEAPIDLL = OpenEAPI();
    return g_hEAPIDLL;
}
static bool InitializeEAPI(HINSTANCE hDLL) {
    uint32_t status;
    EAPIFunction(hDLL, EApiLibInitialize);
    if (EApiLibInitialize) {
        status = EApiLibInitialize();
        if (status != EAPI_STATUS_SUCCESS && status != EAPI_STATUS_INITIALIZED) return false;
    }
    return true;
}
static bool DeInitializeEAPI(void) {
    if (g_hEAPIDLL) {
        uint32_t status;
        EAPIFunction(g_hEAPIDLL, EApiLibUnInitialize);
        if (EApiLibUnInitialize) { status = EApiLibUnInitialize(); (void)status; }
        CloseEAPI(g_hEAPIDLL);
        g_hEAPIDLL = NULL;
    }
    return true;
}

static uint32_t BankValidMask(int bank) {
    uint8_t n = g_gpioInfo[bank].supPinNum;
    if (n == 0) return 0;
    if (n >= 32) return 0xFFFFFFFFu;
    return (1u << n) - 1u;
}
bool EnumerateGPIO() {
    uint32_t status, supportPin, id;
    uint8_t found = 0;
    HINSTANCE hDLL = GetEAPIInstance();
    if (!hDLL) return false;
    if (!InitializeEAPI(hDLL)) return false;
    EAPIFunction(hDLL, EApiGPIOGetDirectionCaps);
    if (!EApiGPIOGetDirectionCaps) return false;
    for (uint8_t i = 0; i < BANK_MAX; i++) {
        id = EAPI_ID_GPIO_BANK(i);
        status = EApiGPIOGetDirectionCaps(id, &g_gpioInfo[i].supInput, &g_gpioInfo[i].supOutput);
        if (status != EAPI_STATUS_SUCCESS) { g_gpioInfo[i].supPinNum = 0; continue; }
        supportPin = g_gpioInfo[i].supInput | g_gpioInfo[i].supOutput;
        if (supportPin > 0) {
            uint8_t j;
            for (j = 32; j > 0; j--) {
                if (supportPin & (1u << (j - 1))) { g_gpioInfo[i].supPinNum = j; break; }
            }
            if (j == 0) g_gpioInfo[i].supPinNum = 0;
        }
        else {
            g_gpioInfo[i].supPinNum = 0;
        }
        found++;
    }
    return found > 0;
}
bool PickBank_DI0_7_DO8_15() {
    if (kFixedBank >= 0 && kFixedBank < BANK_MAX) { g_gpioBank = kFixedBank; return true; }
    for (int b = 0; b < BANK_MAX; ++b) {
        uint32_t valid = BankValidMask(b);
        if ((valid & 0x0000FFFFu) != 0x0000FFFFu) continue;
        uint32_t diOk = g_gpioInfo[b].supInput & kMaskDI0_7;
        uint32_t doOk = g_gpioInfo[b].supOutput & kMaskDO8_15;
        if (diOk == kMaskDI0_7 && doOk == kMaskDO8_15) { g_gpioBank = b; return true; }
    }
    int best = -1, scoreBest = -1;
    for (int b = 0; b < BANK_MAX; ++b) {
        uint32_t valid = BankValidMask(b) & 0x0000FFFFu;
        int diCnt = 0, doCnt = 0;
        for (int i = 0; i < 8; ++i) if ((valid & (1u << i)) && (g_gpioInfo[b].supInput & (1u << i))) diCnt++;
        for (int i = 8; i < 16; ++i) if ((valid & (1u << i)) && (g_gpioInfo[b].supOutput & (1u << i))) doCnt++;
        int score = diCnt + doCnt;
        if (score > scoreBest) { scoreBest = score; best = b; }
    }
    if (best >= 0) { g_gpioBank = best; return true; }
    return false;
}

// EAPI helpers
static bool GPIO_Single_GetDirection(uint32_t globalPin, uint32_t* pBit, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOGetDirection); if (!EApiGPIOGetDirection) return false;
    uint32_t id = EAPI_GPIO_GPIO_ID(globalPin);
    uint32_t dir = 0; uint32_t st = EApiGPIOGetDirection(id, 1u, &dir);
    if (pStatus) *pStatus = st;
    if (st == EAPI_STATUS_SUCCESS) { if (pBit) *pBit = (dir & 1u); return true; }
    return false;
}
static bool GPIO_Single_SetDirection(uint32_t globalPin, bool makeOutput, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOSetDirection); if (!EApiGPIOSetDirection) return false;
    uint32_t id = EAPI_GPIO_GPIO_ID(globalPin);
    uint32_t st = EApiGPIOSetDirection(id, 1u, makeOutput ? 1u : 0u);
    if (pStatus) *pStatus = st;
    return st == EAPI_STATUS_SUCCESS;
}
static bool GPIO_Single_GetLevel(uint32_t globalPin, uint32_t* pBit, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOGetLevel); if (!EApiGPIOGetLevel) return false;
    uint32_t id = EAPI_GPIO_GPIO_ID(globalPin);
    uint32_t lvl = 0; uint32_t st = EApiGPIOGetLevel(id, 1u, &lvl);
    if (pStatus) *pStatus = st;
    if (st == EAPI_STATUS_SUCCESS) { if (pBit) *pBit = (lvl & 1u); return true; }
    return false;
}
static bool GPIO_Single_SetLevel(uint32_t globalPin, bool hi, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOSetLevel); if (!EApiGPIOSetLevel) return false;
    uint32_t id = EAPI_GPIO_GPIO_ID(globalPin);
    uint32_t st = EApiGPIOSetLevel(id, 1u, hi ? 1u : 0u);
    if (pStatus) *pStatus = st;
    return st == EAPI_STATUS_SUCCESS;
}
static bool GPIO_GetDirection_Bank(int bank, uint32_t mask, uint32_t* pValue, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOGetDirection); if (!EApiGPIOGetDirection) return false;
    uint32_t id = EAPI_ID_GPIO_BANK(bank);
    uint32_t dir = 0; uint32_t st = EApiGPIOGetDirection(id, mask, &dir);
    if (pStatus) *pStatus = st;
    if (st == EAPI_STATUS_SUCCESS) { if (pValue) *pValue = dir; return true; }
    return false;
}
static bool GPIO_SetDirection_Bank(int bank, uint32_t mask, uint32_t setVal, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOSetDirection); if (!EApiGPIOSetDirection) return false;
    uint32_t id = EAPI_ID_GPIO_BANK(bank);
    uint32_t st = EApiGPIOSetDirection(id, mask, setVal);
    if (pStatus) *pStatus = st;
    return st == EAPI_STATUS_SUCCESS;
}
static bool GPIO_GetLevel_Bank(int bank, uint32_t mask, uint32_t* pValue, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOGetLevel); if (!EApiGPIOGetLevel) return false;
    uint32_t id = EAPI_ID_GPIO_BANK(bank);
    uint32_t lvl = 0; uint32_t st = EApiGPIOGetLevel(id, mask, &lvl);
    if (pStatus) *pStatus = st;
    if (st == EAPI_STATUS_SUCCESS) { if (pValue) *pValue = lvl; return true; }
    return false;
}
static bool GPIO_SetLevel_Bank(int bank, uint32_t mask, uint32_t setVal, uint32_t* pStatus = nullptr) {
    HINSTANCE hDLL = GetEAPIInstance(); if (!hDLL) return false;
    EAPIFunction(hDLL, EApiGPIOSetLevel); if (!EApiGPIOSetLevel) return false;
    uint32_t id = EAPI_ID_GPIO_BANK(bank);
    uint32_t st = EApiGPIOSetLevel(id, mask, setVal);
    if (pStatus) *pStatus = st;
    return st == EAPI_STATUS_SUCCESS;
}
bool EnsureDO8to15AsOutput_BankFirst() {
    if (g_gpioBank < 0) return false;
    uint32_t valid = BankValidMask(g_gpioBank);
    uint32_t targetMask = (kMaskDO8_15 & valid) & g_gpioInfo[g_gpioBank].supOutput;
    if (!targetMask) return false;
    uint32_t st = 0;
    return GPIO_SetDirection_Bank(g_gpioBank, targetMask, targetMask, &st);
}

// 디바운스 버퍼
static uint8_t g_diSamples[8][kDI_BufferDepth] = { 0 };
static int     g_diSamplePos = 0;
bool    g_diStable[8] = { 0 };

static bool MajorityOfSamples(int pin) {
    int ones = 0, total = 0;
    for (int k = 0; k < kDI_BufferDepth; ++k) {
        ones += (g_diSamples[pin][k] ? 1 : 0);
        total++;
    }
    return ones >= (total + 1) / 2;
}
static bool SampleDI_Once(uint32_t& diBitsOut) {
    diBitsOut = 0;
    if (g_gpioBank < 0) return false;
    uint32_t valid = BankValidMask(g_gpioBank);
    uint32_t maskDI = (kMaskDI0_7 & valid) & g_gpioInfo[g_gpioBank].supInput;
    uint32_t lvl = 0; uint32_t st = 0;
    bool ok = GPIO_GetLevel_Bank(g_gpioBank, maskDI, &lvl, &st);
    if (!ok) {
        lvl = 0;
        for (int i = 0; i < 8; ++i) {
            if (!(maskDI & (1u << i))) continue;
            uint32_t bit = 0, stp = 0;
            if (GPIO_Single_GetLevel((uint32_t)(g_gpioBank * 32 + i), &bit, &stp)) {
                if (bit) lvl |= (1u << i);
            }
        }
        ok = true;
    }
    if (ok) diBitsOut = lvl & 0xFFu;
    return ok;
}
static void ApplyDIOverlayRules() {
    // STO(DO10) 눌렀을 때 Motioning(DI0)을 강제로 OFF로 표시
    if (g_uiOverlayDIValid[0]) {
        if (!g_diStable[0]) {
            g_uiOverlayDIValid[0] = false;
        }
    }
}
static void UpdateDIText(int i, bool supported, bool level, HWND hWnd) {
    if (!g_lblDI[i]) return;
    TCHAR buf[64];
    if (!supported) {
        _stprintf_s(buf, _T("Pin %d : N/A"), i);
        SetWindowText(g_lblDI[i], buf);
        return;
    }
    bool showLevel = level;
    if (g_uiOverlayDIValid[i]) showLevel = g_uiOverlayDILevel[i];
    if (i == 0) _stprintf_s(buf, _T("Motioning : %s"), showLevel ? _T("ON") : _T("OFF"));
    else if (i == 1) _stprintf_s(buf, _T("Catched   : %s"), showLevel ? _T("OFF") : _T("ON"));
    else _stprintf_s(buf, _T("Pin %d : %s"), i, showLevel ? _T("OFF") : _T("ON"));
    SetWindowText(g_lblDI[i], buf);
}

static bool s_servoOffHandledOnce = false;

static void RefreshInputsWithDebounce(HWND hWnd) {
    uint32_t diNow = 0;
    if (!SampleDI_Once(diNow)) return;
    for (int i = 0; i < 8; ++i) g_diSamples[i][g_diSamplePos] = (diNow >> i) & 1u;
    g_diSamplePos = (g_diSamplePos + 1) % kDI_BufferDepth;
    for (int i = 0; i < 8; ++i) {
        bool newStable = MajorityOfSamples(i);
        g_diStable[i] = newStable;
        g_cachedLevels[i] = newStable;
    }
    ApplyDIOverlayRules();
    for (int i = 0; i < 8; ++i) {
        bool sup = (g_gpioInfo[g_gpioBank].supInput & (1u << i)) != 0;
        UpdateDIText(i, sup, g_diStable[i], hWnd);
    }
    // ============================================
    // ★ Motioning(DI0) ON → OFF 전이 감지
    //    → GripOpen / GripClose 를 Done 으로 전환
    // ============================================
    {
        static bool s_prevMotioning = false;
        static bool s_init = false;

        // DI0 = Motioning (input 1번 신호)
        bool curMotioning = g_diStable[0];

        if (!s_init) {
            // 첫 호출 시에는 기준값만 세팅하고 끝
            s_prevMotioning = curMotioning;
            s_init = true;
        }
        else {
            // 이전에 ON이었다가 지금 OFF로 떨어진 순간만 감지
            if (s_prevMotioning && !curMotioning) {
                // GripOpen 이 진행중이면 완료로
                TaskState openState = g_taskStatus[(int)TaskId::GripOpen].state.load();
                if (openState == TaskState::Running) {
                    SetTaskState(TaskId::GripOpen, TaskState::Done);
                }

                // GripClose 도 진행중이면 완료로
                TaskState closeState = g_taskStatus[(int)TaskId::GripClose].state.load();
                if (closeState == TaskState::Running) {
                    SetTaskState(TaskId::GripClose, TaskState::Done);
                }
            }
            // ★ 추가: 프로그램 전체에서 딱 1번만 GripServoOff 상태를 보고
            //         GripOpen / GripClose 를 Done 으로 맞춰준다.
            if (!s_servoOffHandledOnce) {
                TaskState servoOffState =
                    g_taskStatus[(int)TaskId::GripServoOff].state.load();
                if (servoOffState == TaskState::Done) {

                    TaskState openState =
                        g_taskStatus[(int)TaskId::GripOpen].state.load();
                    if (openState == TaskState::Running) {
                        SetTaskState(TaskId::GripOpen, TaskState::Done);
                    }

                    TaskState closeState =
                        g_taskStatus[(int)TaskId::GripClose].state.load();
                    if (closeState == TaskState::Running) {
                        SetTaskState(TaskId::GripClose, TaskState::Done);
                    }

                    // ★ 한 번 처리했으니 다시는 안 하도록 플래그 ON
                    s_servoOffHandledOnce = true;
                }
            }

            // 이전 Motioning 상태 업데이트
            s_prevMotioning = curMotioning;
        }
    }
}
static bool ReadDO16Bits(uint32_t& dirOut, uint32_t& lvlOut) {
    dirOut = 0; lvlOut = 0;
    if (g_gpioBank < 0) return false;
    uint32_t valid = BankValidMask(g_gpioBank);
    uint32_t mask16 = (kMaskDI0_7 | kMaskDO8_15) & valid;
    uint32_t st1 = 0, st2 = 0;
    bool okDir = GPIO_GetDirection_Bank(g_gpioBank, mask16, &dirOut, &st1);
    bool okLvl = GPIO_GetLevel_Bank(g_gpioBank, mask16, &lvlOut, &st2);
    if (!okDir || !okLvl) {
        dirOut = 0; lvlOut = 0;
        for (int i = 0; i < 16; ++i) {
            uint32_t gp = g_gpioBank * 32 + i;
            uint32_t bit = 0, st = 0;
            if (GPIO_Single_GetDirection(gp, &bit, &st)) { if (bit) dirOut |= (1u << i); }
            if (GPIO_Single_GetLevel(gp, &bit, &st)) { if (bit) lvlOut |= (1u << i); }
        }
    }
    return true;
}
static void RefreshOutputs(HWND hWnd) {
    if (g_gpioBank < 0) return;
    uint32_t dir = 0, lvl = 0;
    if (!ReadDO16Bits(dir, lvl)) return;
    for (int i = 8; i < 16; ++i) {
        bool sup = (g_gpioInfo[g_gpioBank].supOutput & (1u << i)) != 0;
        bool on = (lvl & (1u << i)) != 0;
        g_cachedLevels[i] = on;
        if (g_swDO[i]) {
            EnableWindow(g_swDO[i], sup ? TRUE : FALSE);
            ToggleState* st = (ToggleState*)GetWindowLongPtr(g_swDO[i], GWLP_USERDATA);
            if (st) { st->on = sup ? on : false; InvalidateRect(g_swDO[i], NULL, TRUE); }
        }
    }
}
void RefreshLevels(HWND hWnd) {
    if (g_gpioBank < 0) return;
    RefreshInputsWithDebounce(hWnd);
    RefreshOutputs(hWnd);
}

// Toggle control
static void DrawRoundedRect(HDC hdc, const RECT& rc, int r, COLORREF fill, COLORREF border) {
    HBRUSH hb = CreateSolidBrush(fill);
    HPEN   hp = CreatePen(PS_SOLID, 1, border);
    HGDIOBJ ob = SelectObject(hdc, hb);
    HGDIOBJ op = SelectObject(hdc, hp);
    RoundRect(hdc, rc.left, rc.top, rc.right, rc.bottom, r, r);
    SelectObject(hdc, op);
    SelectObject(hdc, ob);
    DeleteObject(hp);
    DeleteObject(hb);
}
static LRESULT CALLBACK ToggleProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    ToggleState* st = (ToggleState*)GetWindowLongPtr(hWnd, GWLP_USERDATA);
    switch (msg) {
    case WM_NCCREATE: {
        CREATESTRUCT* cs = (CREATESTRUCT*)lParam;
        ToggleState* ns = new ToggleState();
        ns->on = false; ns->enabled = true; ns->pin = (int)(INT_PTR)cs->lpCreateParams;
        SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)ns);
        return TRUE;
    }
    case WM_NCDESTROY:
        if (st) { delete st; SetWindowLongPtr(hWnd, GWLP_USERDATA, 0); }
        return 0;
    case WM_ENABLE:
        if (st) st->enabled = (wParam != FALSE);
        InvalidateRect(hWnd, NULL, TRUE);
        return 0;
    case WM_SETFONT:
        g_fontToggle = (HFONT)wParam;
        return 0;
    case WM_GETFONT:
        return (LRESULT)g_fontToggle;
    case WM_LBUTTONDOWN:
    case WM_KEYDOWN:
        if (msg == WM_KEYDOWN && (wParam != VK_SPACE && wParam != VK_RETURN)) break;
        if (st && st->enabled) {
            st->on = !st->on;
            InvalidateRect(hWnd, NULL, TRUE);
            HWND parent = GetParent(hWnd);
            if (parent) SendMessage(parent, WM_COMMAND, MAKELONG(GetDlgCtrlID(hWnd), BN_CLICKED), (LPARAM)hWnd);
        }
        return 0;
    case WM_PAINT: {
        PAINTSTRUCT ps; HDC hdc = BeginPaint(hWnd, &ps);
        RECT rc; GetClientRect(hWnd, &rc);
        HDC memdc = CreateCompatibleDC(hdc);
        HBITMAP membmp = CreateCompatibleBitmap(hdc, rc.right - rc.left, rc.bottom - rc.top);
        HGDIOBJ oldbmp = SelectObject(memdc, membmp);
        HBRUSH bg = CreateSolidBrush(GetSysColor(COLOR_BTNFACE));
        FillRect(memdc, &rc, bg);
        DeleteObject(bg);
        RECT rToggle = rc;
        rToggle.top += 4; rToggle.bottom -= 4; rToggle.left += 4; rToggle.right -= 4;
        COLORREF ColOn = RGB(40, 167, 69);
        COLORREF ColOff = RGB(217, 83, 79);
        COLORREF bgcol = (st && st->on) ? ColOn : ColOff;
        DrawRoundedRect(memdc, rToggle, 18, bgcol, RGB(80, 80, 80));
        int w = rToggle.right - rToggle.left;
        int h = rToggle.bottom - rToggle.top;
        int diameter = h - 6;
        int cx_off = (st && st->on) ? (w - diameter - 6) : 6;
        RECT rKnob = { rToggle.left + cx_off, rToggle.top + 3, rToggle.left + cx_off + diameter, rToggle.top + 3 + diameter };
        HBRUSH knobBrush = CreateSolidBrush(RGB(240, 240, 240));
        HPEN   knobPen = CreatePen(PS_SOLID, 1, RGB(100, 100, 100));
        HGDIOBJ okb = SelectObject(memdc, knobBrush);
        HGDIOBJ okp = SelectObject(memdc, knobPen);
        Ellipse(memdc, rKnob.left, rKnob.top, rKnob.right, rKnob.bottom);
        SelectObject(memdc, okp); DeleteObject(knobPen);
        SelectObject(memdc, okb); DeleteObject(knobBrush);
        const TCHAR* txt = (st && st->on) ? _T("ON") : _T("OFF");
        SetBkMode(memdc, TRANSPARENT);
        SetTextColor(memdc, RGB(255, 255, 255));
        HFONT f = (HFONT)SendMessage(hWnd, WM_GETFONT, 0, 0);
        HGDIOBJ of = NULL; if (f) of = SelectObject(memdc, f);
        RECT rText = rToggle; rText.left += 10; rText.right -= 10;
        DrawText(memdc, txt, -1, &rText, DT_SINGLELINE | DT_CENTER | DT_VCENTER);
        if (of) SelectObject(memdc, of);
        BitBlt(hdc, 0, 0, rc.right - rc.left, rc.bottom - rc.top, memdc, 0, 0, SRCCOPY);
        SelectObject(memdc, oldbmp);
        DeleteObject(membmp);
        DeleteDC(memdc);
        EndPaint(hWnd, &ps);
        return 0;
    }
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}
static void RegisterToggleClass(HINSTANCE hInst) {
    WNDCLASS wc = { 0 };
    wc.lpfnWndProc = ToggleProc;
    wc.hInstance = hInst;
    wc.lpszClassName = TOGGLE_CLS;
    wc.hCursor = LoadCursor(NULL, IDC_HAND);
    wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    RegisterClass(&wc);
}

// 핀 이름
static const TCHAR* g_DOFuncNames[16] = {
    nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,
    _T("Open"),     // 8
    _T("Close"),    // 9
    _T("STO"),      // 10
    _T("LED"),     // 11
    _T("None"),     // 12
    _T("None"),     // 13
    nullptr,        // 14
    nullptr         // 15
};

std::atomic<unsigned char> g_doShadow[32];  // 0/1

// 출력 동작(펄스 예약/오버레이/입력샘플 반영)
void ToggleDO_HW(int pin, bool turnOn, HWND hWnd)
{
    if (g_gpioBank < 0) return;
    uint32_t bit = (1u << pin);
    if (!(g_gpioInfo[g_gpioBank].supOutput & bit)) return;

    (void)EnsureDO8to15AsOutput_BankFirst();

    uint32_t gp = (uint32_t)(g_gpioBank * 32 + pin);
    uint32_t st = 0;
    GPIO_Single_SetDirection(gp, true, &st);
    GPIO_Single_SetLevel(gp, turnOn, &st);

    // ★ DO Shadow 업데이트 (CalcPosGripCode에서 이걸 보고 판단)
    if (0 <= pin && pin < 32) {
        g_doShadow[pin].store(turnOn ? 1 : 0, std::memory_order_relaxed);
    }

    // UI 즉시 반영
    if (g_swDO[pin]) {
        ToggleState* ts = (ToggleState*)GetWindowLongPtr(g_swDO[pin], GWLP_USERDATA);
        if (ts) { ts->on = turnOn; InvalidateRect(g_swDO[pin], NULL, TRUE); }
    }

    // 출력 직후 입력 안정화 지연 후 1회 추가 샘플
    if (kPostOutputInputDelayMs > 0) {
        Sleep(kPostOutputInputDelayMs);
        uint32_t diNow = 0;
        if (SampleDI_Once(diNow)) {
            g_diSamples[0][g_diSamplePos] = (diNow >> 0) & 1u;
            g_diSamples[1][g_diSamplePos] = (diNow >> 1) & 1u;
            g_diSamples[2][g_diSamplePos] = (diNow >> 2) & 1u;
            g_diSamples[3][g_diSamplePos] = (diNow >> 3) & 1u;
            g_diSamples[4][g_diSamplePos] = (diNow >> 4) & 1u;
            g_diSamples[5][g_diSamplePos] = (diNow >> 5) & 1u;
            g_diSamples[6][g_diSamplePos] = (diNow >> 6) & 1u;
            g_diSamples[7][g_diSamplePos] = (diNow >> 7) & 1u;
            g_diSamplePos = (g_diSamplePos + 1) % kDI_BufferDepth;
        }
    }

    // STO(DO10) 눌렀을 때 Motioning(DI0) 표시 강제 OFF
    if (pin == 10 && turnOn) {
        g_uiOverlayDIValid[0] = true;
        g_uiOverlayDILevel[0] = false;
    }

    // 펄스 자동 OFF
    if (pin == 10 && turnOn) { g_outputPendingOff[10] = true; g_outputOnTick[10] = GetTickCount(); }
    else if (!turnOn) { g_outputPendingOff[pin] = false; }

    // 출력 후 곧바로 입력 갱신
    RefreshInputsWithDebounce(hWnd);
}

// 축0 바코드가 Conveyor 위치에 있는지 확인 (0x6063 값 기준)
bool IsAxis0AtConveyorBarcode()
{
    // GO_Conveyor()에서 사용한 타겟 바코드 값과 동일하게 맞춰줌
    const long long targetBc = 476774;   // GO_Conveyor 의 targetBarcodeAbs
    const int bcEps = 5;                 // 허용 오차 (필요시 조정)

    int nowBc = 0;
    if (!ReadAxis_TxPDO_6063(kAxisSlaveId[0], nowBc))
        return false;

    long long diff = (long long)nowBc - targetBc;
    return std::llabs(diff) <= bcEps;
}

// 축0 바코드가 Workstation 위치에 있는지 확인 (0x6063 값 기준)
bool IsAxis0AtWorkstationBarcode()
{
    // GO_Conveyor()에서 사용한 타겟 바코드 값과 동일하게 맞춰줌
    const long long targetBc = 491332;   // GO_Conveyor 의 targetBarcodeAbs
    const int bcEps = 5;                 // 허용 오차 (필요시 조정)

    int nowBc = 0;
    if (!ReadAxis_TxPDO_6063(kAxisSlaveId[0], nowBc))
        return false;

    long long diff = (long long)nowBc - targetBc;
    return std::llabs(diff) <= bcEps;
}

bool IsAxis0AtConveyorBarcodeStopped()
{
    if (!IsAxis0AtConveyorBarcode())
        return false;

    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    double v = std::fabs(st.axesStatus[1].actualVelocity);
    return v <= 1.0;    // 적당한 정지 기준
}

bool IsAxis0AtWorkstationBarcodeStopped()
{
    if (!IsAxis0AtWorkstationBarcode())
        return false;

    CoreMotionStatus st{};
    g_cm.GetStatus(&st);
    double v = std::fabs(st.axesStatus[1].actualVelocity);
    return v <= 1.0;
}


// 축2가 Up 위치(0 근처)인지 확인
bool IsAxis2Up()
{
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);

    const long long targetPos = 0;       // DoUp()에서 사용하는 타겟
    const double posEps = 10.0;          // 위치 허용 오차
    const double velEps = 1.0;           // 속도 허용 오차

    const auto& ax = st.axesStatus[2];

    long long perr = (long long)ax.actualPos - targetPos;
    double    v = std::fabs(ax.actualVelocity);

    return (std::llabs(perr) <= (long long)posEps) && (v <= velEps);
}

// 축2가 Up 위치(0 근처)인지 확인
bool IsAxis2Workdown()
{
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);

    const long long targetPos = 45000;       // DoUp()에서 사용하는 타겟
    const double posEps = 10.0;          // 위치 허용 오차
    const double velEps = 1.0;           // 속도 허용 오차

    const auto& ax = st.axesStatus[2];

    long long perr = (long long)ax.actualPos - targetPos;
    double    v = std::fabs(ax.actualVelocity);

    return (std::llabs(perr) <= (long long)posEps) && (v <= velEps);
}

// 축2가 Up 위치(0 근처)인지 확인
bool IsAxis2Conveyordown()
{
    CoreMotionStatus st{};
    g_cm.GetStatus(&st);

    const long long targetPos = 51600;       // DoUp()에서 사용하는 타겟
    const double posEps = 10.0;          // 위치 허용 오차
    const double velEps = 1.0;           // 속도 허용 오차

    const auto& ax = st.axesStatus[2];

    long long perr = (long long)ax.actualPos - targetPos;
    double    v = std::fabs(ax.actualVelocity);

    return (std::llabs(perr) <= (long long)posEps) && (v <= velEps);
}

// 그리퍼가 Open 상태인지 확인 (DO8 레벨 사용)
bool IsGripperOpen()
{
    // DO 레벨은 기존 ReadDO16Bits() 유틸을 재사용
    uint32_t dir = 0, lvl = 0;
    if (!ReadDO16Bits(dir, lvl))
        return false;

    if (g_gpioBank < 0)
        return false;

    const uint32_t bitOpen = (1u << 8); // Pin 8 = Open
    // 해당 핀을 실제로 출력으로 지원하는지 확인
    if ((g_gpioInfo[g_gpioBank].supOutput & bitOpen) == 0)
        return false;

    // DoOpen_Compat()가 DO8을 ON 시키므로, ON이면 Open 상태로 간주
    bool isOn = (lvl & bitOpen) != 0;
    return isOn;
}

bool IsGripperClosed()
{
    uint32_t dir = 0, lvl = 0;
    if (!ReadDO16Bits(dir, lvl))
        return false;

    if (g_gpioBank < 0)
        return false;

    const uint32_t bitClose = (1u << 9); // Pin 9 = Close
    if ((g_gpioInfo[g_gpioBank].supOutput & bitClose) == 0)
        return false;

    bool isOn = (lvl & bitClose) != 0;
    return isOn;
}
// Motioning 출력(DO11)의 현재 상태 읽기
static bool IsMotioningOutputOn()
{
    uint32_t dir = 0, lvl = 0;
    if (!ReadDO16Bits(dir, lvl))
        return false;  // 읽기 실패하면 일단 OFF 취급(원하면 true로 바꿔도 됨)

    const uint32_t bitMotion = (1u << 11); // Pin 11 = Motioning 출력
    bool on = (lvl & bitMotion) != 0;
    return on;
}


bool IsGripperOpenAndIdle()
{
    // 1) 실제 DO8 상태 기준으로 그리퍼가 Open인지 확인
    if (!IsGripperOpen())
        return false;

    // 2) GripOpen Task가 Done 상태인지 확인
    TaskState openState = g_taskStatus[(int)TaskId::GripOpen].state.load();
    return (openState == TaskState::Done);
}

bool IsGripperClosedAndIdle()
{
    // 1) 실제 DO9 상태 기준으로 그리퍼가 Close인지 확인
    if (!IsGripperClosed())
        return false;

    // 2) GripClose Task가 Done 상태인지 확인
    TaskState closeState = g_taskStatus[(int)TaskId::GripClose].state.load();
    return (closeState == TaskState::Done);
}


// 예: DI1을 "박스 감지" 센서로 쓴다고 가정 (input2가 DI1인 경우)
bool HasBox()
{
    // g_diStable[1] == false  → input2 ON → 박스 있음
    // g_diStable[1] == true → input2 OFF → 박스 없음
    return !g_diStable[1];
}

// 박스 "없음" 상태를 기다릴 때 쓰기 위한 헬퍼
bool NoBox()
{
    return !HasBox();
}

// 모든 축이 정지 + Motioning(DI0) OFF 될 때까지 대기
// 모든 축이 정지 + GripServoOff 태스크 Done 될 때까지 대기
bool WaitAllAxesStopped(double velEps, DWORD timeoutMs)
{
    DWORD start = GetTickCount();
    CoreMotionStatus st{};

    while (true) {
        g_cm.GetStatus(&st);

        bool allStopped = true;
        for (int ax = 0; ax < 4; ++ax) {
            double v = std::fabs(st.axesStatus[ax].actualVelocity);
            if (v > velEps) {
                allStopped = false;
                break;
            }
        }

        // ★ GripServoOff 태스크 상태 읽기
        TaskState servoOffState = g_taskStatus[(int)TaskId::GripServoOff].state.load(std::memory_order_relaxed);
        bool servoOffDone = (servoOffState == TaskState::Done);

        // 둘 다 만족해야 "정지 완료"로 인정
        if (allStopped && servoOffDone) {
            return true;    // 모든 축 멈췄고, GripServoOff도 Done
        }

        if (GetTickCount() - start > timeoutMs) {
            return false;   // 타임아웃 (축 또는 GripServoOff 둘 중 뭔가 아직)
        }

        ::Sleep(20);
    }
}


// 주어진 조건 함수(pred)가 true가 될 때까지 기다리는 유틸
//  - pred() 가 true 이면 즉시 true 리턴
//  - timeoutMs 안에 만족 못하면 false 리턴
bool WaitUntil(bool (*pred)(), DWORD timeoutMs, DWORD pollMs = 20)
{
    DWORD start = GetTickCount();
    while (true) {
        if (pred())
            return true;

        if (GetTickCount() - start > timeoutMs)
            return false;

        ::Sleep(pollMs);
    }
}

// TaskId 기준으로 시퀀스가 끝날 때까지 대기
//  - Done  이면 true
//  - Failed / Stopped / timeout 이면 false
bool WaitTaskFinished(TaskId id, DWORD timeoutMs, DWORD pollMs = 20)
{
    DWORD start = GetTickCount();

    while (true) {
        TaskState st = g_taskStatus[(int)id].state.load();

        // 정상 완료
        if (st == TaskState::Done) {
            return true;
        }

        // 실패 / 중단
        if (st == TaskState::Failed || st == TaskState::Stopped) {
            return false;
        }

        // 타임아웃
        if (GetTickCount() - start > timeoutMs) {
            return false;
        }

        ::Sleep(pollMs);
    }
}


// Load 시퀀스 시작 전 조건 모두 만족하는지 확인
static bool CheckDemoLoadPreconditions()
{
    //if (!IsAxis0AtConveyorBarcode()) {
    //    // LOG("DemoLoad NG: Axis0 not at Conveyor barcode");
    //    return false;
    //}

    if (!IsAxis2Up()) {
        // LOG("DemoLoad NG: Axis2 not Up");
        return false;
    }

    if (!IsGripperOpen()) {
        // LOG("DemoLoad NG: Gripper not Open");
        return false;
    }

    if (HasBox()) {
        // LOG("DemoLoad NG: Already has box (Catched ON)");
        return false;
    }

    return true;
}

// Unload 시퀀스 시작 전 조건:
//  - 축0 : Conveyor 위치
//  - 축2 : Up 상태
//  - Gripper : Close
//  - 박스 있음 (Catched ON)
static bool CheckDemoUnloadPreconditions()
{
    //// 1) 축0이 Conveyor 바코드 위치인가?
    //if (!IsAxis0AtConveyorBarcode())
    //    return false;

    // 2) 축2가 Up 위치인가?
    if (!IsAxis2Up())
        return false;

    // 3) Gripper가 Close 상태인가?
    if (!IsGripperClosed())
        return false;

    // 4) 박스를 실제로 들고 있는가? (Catched == ON)
    if (!HasBox())
        return false;

    return true;
}

void DoGripServoOff_Compat(HWND hWnd);


// =======================================
// Demo 동작: Lift/Down/Go, Stop
// =======================================
void DoOpen_Compat(HWND hWnd)
{
    // 그리퍼 Open 명령 시작
    SetTaskState(TaskId::GripOpen, TaskState::Running);


    //   // 1) STO 펄스 (10번: ON → 타이머로 자동 OFF)
       //DoGripServoOff_Compat(hWnd);

       // 2) Close OFF
    ToggleDO_HW(9, false, hWnd);
    SetTaskState(TaskId::GripClose, TaskState::Idle);

    // 3) Open 신호: 8번을 한번 OFF 했다가 ON (에지 만들기)
    ToggleDO_HW(8, false, hWnd);
    ToggleDO_HW(8, true, hWnd);   // 이 상태가 계속 유지 → Open 상태

    //DoGripServoOff_Compat(hWnd);
}

void DoClose_Compat(HWND hWnd)
{
    // 그리퍼 Close 명령 시작
    SetTaskState(TaskId::GripClose, TaskState::Running);

    // 1) STO 펄스
    //DoGripServoOff_Compat(hWnd);

    // 2) Open OFF
    ToggleDO_HW(8, false, hWnd);
    SetTaskState(TaskId::GripOpen, TaskState::Idle);

    // 3) Close 신호: 9번을 한번 OFF 했다가 ON (에지 만들기)
    ToggleDO_HW(9, false, hWnd);
    ToggleDO_HW(9, true, hWnd);   // 이 상태 유지 → Close 상태
    //DoGripServoOff_Compat(hWnd);

}

void DoGripServoOff_Compat(HWND hWnd) {
    ToggleDO_HW(10, true, hWnd); // STO ON (펄스)
    ToggleDO_HW(10, false, hWnd); // STO ON (펄스)
    std::this_thread::sleep_for(std::chrono::seconds(1));
	ToggleDO_HW(11, false, hWnd); // LED OFF
    SetTaskState(TaskId::GripServoOff, TaskState::Done);
}

void WorkDown() {
    if (!g_commStarted) { SetTaskState(TaskId::WorkDown, TaskState::Failed); return; }
    int ax = 2;
    long long tgt = 45000;
    StartMoveWithApproach(ax, tgt, TaskId::WorkDown,
        10000.0, 1000.0, 1500.0,
        10.0, 2.0, 15000,
        3000.0, { 1000.0, 80.0, 10.0 });
}
void ConveyorDown() {
    if (!g_commStarted) { SetTaskState(TaskId::ConveyorDown, TaskState::Failed); return; }
    int ax = 2;
    long long tgt = 51600;
    StartMoveWithApproach(ax, tgt, TaskId::ConveyorDown,
        10000.0, 1000.0, 1500.0,
        10.0, 2.0, 30000,
        3500, { 1000.0, 80.0, 10.0 });
}
void DoUp() {
    if (!g_commStarted) { SetTaskState(TaskId::LiftUp, TaskState::Failed); return; }
    int ax = 2; long long tgt = 0;
    // 필드 순서: axis, target, task, velEps, posEps, timeoutMs, treatStoppedAsDone
    MoveMonitorArgs m{ ax, tgt, TaskId::LiftUp, 10.0, 2.0, 15000, true };
    StartMoveAndMonitor(m, 10000.0, 3000.0, 3000.0);
}
void DoStopAll(HWND hWnd) {
    DoGripServoOff_Compat(hWnd);
    g_bcRunner.Stop();
    for (int a = 0; a < 4; ++a) StopAxis(a);
    for (int i = 0; i < (int)TaskId::COUNT; ++i) {
        if (g_taskStatus[i].state.load() == TaskState::Running) SetTaskState((TaskId)i, TaskState::Stopped);
    }
    // 펄스 예약 해제
    for (int i = 0; i < 16; ++i) g_outputPendingOff[i] = false;
}
void Go_Workstation() {
    if (!g_commStarted) { SetTaskState(TaskId::GoWorkstation, TaskState::Failed); return; }
    BarcodeParams p{};
    p.bcAxis = 0;          // ✅ 바코드 읽기축
    p.moveAxis = 1;        // ✅ 주행축
    p.targetBarcodeAbs = 491332;
    p.mainVel = 15000.0; p.mainAcc = 1000.0; p.mainDec = 3000.0;
    p.corrVel = 1000.0; p.corrAcc = 1000.0; p.corrDec = 2000.0;
    p.deadband = 2;
    p.gear = 4.3; p.wheelDia = 70.0; p.motorCpr = 10000.0; p.bcMmPerCnt = 0.1;
    g_bcRunner.Start(p, TaskId::GoWorkstation);
}
void GO_Conveyor() {
    if (!g_commStarted) { SetTaskState(TaskId::GoConveyor, TaskState::Failed); return; }
    BarcodeParams p{};
    p.bcAxis = 0;          // ✅ 바코드 읽기축
    p.moveAxis = 1;        // ✅ 주행축
    p.targetBarcodeAbs = 476774;
    p.mainVel = 15000.0; p.mainAcc = 1000.0; p.mainDec = 3000.0;
    p.corrVel = 1000.0; p.corrAcc = 1000.0; p.corrDec = 2000.0;
    p.deadband = 2;
    p.gear = 4.3; p.wheelDia = 70.0; p.motorCpr = 10000.0; p.bcMmPerCnt = 0.1;
    g_bcRunner.Start(p, TaskId::GoConveyor);
}

// 원점 with Box 시퀀스
void StartDemoHomeWithBox()
{
    if (!g_commStarted) {
        SetTaskState(TaskId::DemoHomeWithBox, TaskState::Failed);
        return;
    }

    // ★ 박스가 없으면 시퀀스 시작하지 않음
    if (!HasBox()) {
        SetTaskState(TaskId::DemoHomeWithBox, TaskState::Failed);

        // 필요하면 안내 메시지도 가능
        // if (g_hDemoWnd) {
        //     MessageBox(g_hDemoWnd,
        //         TEXT("원점 with Box 시퀀스를 시작할 수 없습니다.\n")
        //         TEXT("조건: HasBox == TRUE (Catched ON)"),
        //         TEXT("Demo Home(Box)"),
        //         MB_ICONWARNING);
        // }

        return;
    }

    if (g_taskStatus[(int)TaskId::DemoHomeWithBox].state.load() == TaskState::Running)
        return;

    SetTaskState(TaskId::DemoHomeWithBox, TaskState::Running);

    std::thread([]() {
        bool ok = true;

        // 1. 그리퍼 Close (박스 잡기)  → Close 상태 될 때까지 대기
        DoClose_Compat(g_hDemoWnd);
        if (!WaitUntil(HasBox, 5000) || !WaitUntil(IsGripperClosedAndIdle, 5000))
            ok = false;

        // 2. 축2 Up  → Axis2 Up 상태까지 대기
        if (ok) {
            DoUp();
            if (!WaitUntil(IsAxis2Up, 20000))
                ok = false;
        }

        // 3. Workstation 위치로 이동  → 바코드 기준 Workstation 도달까지 대기
        if (ok) {
            Go_Workstation();
            if (!WaitUntil(IsAxis0AtWorkstationBarcodeStopped, 30000))
                ok = false;
        }

        // 4. WorkDown (작업 위치로 하강)  → Axis2 WorkDown 도달까지 대기
        if (ok) {
            WorkDown();
            if (!WaitUntil(IsAxis2Workdown, 20000))
                ok = false;
        }

        // 5. 그리퍼 Open (놓기)  → 그리퍼가 Open 출력 상태가 될 때까지 대기
        if (ok) {
            DoOpen_Compat(g_hDemoWnd);
            if (!WaitUntil(NoBox, 5000) || !WaitUntil(IsGripperOpenAndIdle, 5000))
                ok = false;
        }

        // 6. 축2 Up  → 다시 Up 도달까지 대기
        if (ok) {
            DoUp();
            if (!WaitUntil(IsAxis2Up, 20000))
                ok = false;
        }

        // 7. Conveyor 위치로 이동  → 바코드 기준 Conveyor 도달까지 대기
        if (ok) {
            GO_Conveyor();
            if (!WaitUntil(IsAxis0AtConveyorBarcodeStopped, 30000))
                ok = false;
        }

        SetTaskState(TaskId::DemoHomeWithBox, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}


// 원점 without Box 시퀀스
void StartDemoHomeWithoutBox()
{
    if (!g_commStarted) {
        SetTaskState(TaskId::DemoHomeWithoutBox, TaskState::Failed);
        return;
    }

    // 박스 있으면 실행 안 함 (== 박스 없을 때만 실행)
    if (!NoBox()) {
        SetTaskState(TaskId::DemoHomeWithoutBox, TaskState::Failed);
        return;
    }

    if (g_taskStatus[(int)TaskId::DemoHomeWithoutBox].state.load() == TaskState::Running)
        return;

    SetTaskState(TaskId::DemoHomeWithoutBox, TaskState::Running);

    std::thread([]() {
        bool ok = true;

        // 1. 그리퍼 Open (박스 없음) → Open 상태까지 대기
        DoOpen_Compat(g_hDemoWnd);
        if (!WaitUntil(NoBox, 5000) || !WaitUntil(IsGripperOpenAndIdle, 5000))
            ok = false;

        // 2. 축2 Up → Up 도달까지 대기
        if (ok) {
            DoUp();
            if (!WaitUntil(IsAxis2Up, 20000))
                ok = false;
        }

        // 3. Conveyor 위치로 이동 → Conveyor 바코드 도달까지 대기
        if (ok) {
            GO_Conveyor();
            if (!WaitUntil(IsAxis0AtConveyorBarcodeStopped, 30000))
                ok = false;
        }

        SetTaskState(TaskId::DemoHomeWithoutBox, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}

// 원점 with Box 시퀀스
void StartDemoWorkWithBox()
{
    if (!g_commStarted) {
        SetTaskState(TaskId::DemoWorkWithBox, TaskState::Failed);
        return;
    }

    // ★ 박스가 없으면 시퀀스 시작하지 않음
    if (!HasBox()) {
        SetTaskState(TaskId::DemoWorkWithBox, TaskState::Failed);

        // 필요하면 안내 메시지도 가능
        // if (g_hDemoWnd) {
        //     MessageBox(g_hDemoWnd,
        //         TEXT("원점 with Box 시퀀스를 시작할 수 없습니다.\n")
        //         TEXT("조건: HasBox == TRUE (Catched ON)"),
        //         TEXT("Demo Home(Box)"),
        //         MB_ICONWARNING);
        // }

        return;
    }

    if (g_taskStatus[(int)TaskId::DemoWorkWithBox].state.load() == TaskState::Running)
        return;

    SetTaskState(TaskId::DemoWorkWithBox, TaskState::Running);

    std::thread([]() {
        bool ok = true;

        // 1. Workstation 위치로 이동  → 바코드 기준 Workstation 도달까지 대기
        if (ok) {
            Go_Workstation();
            if (!WaitUntil(IsAxis0AtWorkstationBarcodeStopped, 30000))
                ok = false;
        }

        // 2. WorkDown (작업 위치로 하강)  → Axis2 WorkDown 도달까지 대기
        if (ok) {
            WorkDown();
            if (!WaitUntil(IsAxis2Workdown, 20000))
                ok = false;
        }

        // 3. 그리퍼 Open (놓기)  → 그리퍼가 Open 출력 상태가 될 때까지 대기
        if (ok) {
            DoOpen_Compat(g_hDemoWnd);
            if (!WaitUntil(NoBox, 5000) || !WaitUntil(IsGripperOpenAndIdle, 5000))
                ok = false;
        }

        // 4. 축2 Up  → Axis2 Up 상태까지 대기
        if (ok) {
            DoUp();
            if (!WaitUntil(IsAxis2Up, 20000))
                ok = false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // 5. Conveyor 위치로 이동  → 바코드 기준 Conveyor 도달까지 대기
        if (ok) {
            GO_Conveyor();
            if (!WaitUntil(IsAxis0AtConveyorBarcodeStopped, 30000))
                ok = false;
        }

        SetTaskState(TaskId::DemoWorkWithBox, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}


// 원점 without Box 시퀀스
void StartDemoWorkWithoutBox()
{
    if (!g_commStarted) {
        SetTaskState(TaskId::DemoWorkWithoutBox, TaskState::Failed);
        return;
    }

    // 박스 있으면 실행 안 함 (== 박스 없을 때만 실행)
    if (!NoBox()) {
        SetTaskState(TaskId::DemoWorkWithoutBox, TaskState::Failed);
        return;
    }

    if (g_taskStatus[(int)TaskId::DemoWorkWithoutBox].state.load() == TaskState::Running)
        return;

    SetTaskState(TaskId::DemoWorkWithoutBox, TaskState::Running);

    std::thread([]() {
        bool ok = true;

        // 1. 그리퍼 Open (박스 없음) → Open 상태까지 대기
        DoOpen_Compat(g_hDemoWnd);
        if (!WaitUntil(NoBox, 5000) || !WaitUntil(IsGripperOpenAndIdle, 5000))
            ok = false;

        // 2. 축2 Up → Up 도달까지 대기
        if (ok) {
            DoUp();
            if (!WaitUntil(IsAxis2Up, 20000))
                ok = false;
        }

        // 3. Conveyor 위치로 이동 → Conveyor 바코드 도달까지 대기
        if (ok) {
            GO_Conveyor();
            if (!WaitUntil(IsAxis0AtConveyorBarcodeStopped, 30000))
                ok = false;
        }

        SetTaskState(TaskId::DemoWorkWithoutBox, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}


// Load 시퀀스: Conveyor → Workstation
void StartDemoLoad()
{
    if (!g_commStarted) { SetTaskState(TaskId::DemoLoad, TaskState::Failed); return; }
    if (g_taskStatus[(int)TaskId::DemoLoad].state.load() == TaskState::Running) return;

    // ★ 시퀀스 시작 전 상태 체크:
    //  - 축0 바코드가 Conveyor 위치인지
    //  - 축2가 Up 위치인지
    //  - 그리퍼가 Open 상태인지
    if (!CheckDemoLoadPreconditions()) {
        // 일단 Load 시퀀스 자체는 실패로 표시
        SetTaskState(TaskId::DemoLoad, TaskState::Failed);

        // 1) Axis2가 Limit(Up) 상태가 아니면 먼저 Up으로 정리
        if (!IsAxis2LimitOn()) {
            DoUp();
            (void)WaitUntil(IsAxis2Up, 20000);
        }

        // 2) 그리퍼 상태 확인
        unsigned char gcode = CalcPosGripCode(); // 0x00이면 중간(애매한) 상태라고 가정

        if (HasBox() && gcode != 0x02) {

            // 2-1) 먼저 Close 쪽으로 정리
            DoClose_Compat(g_hDemoWnd);
            (void)WaitUntil(IsGripperClosedAndIdle, 5000);

            // 2-2) Close 상태에서 Servo OFF
            DoGripServoOff_Compat(g_hDemoWnd);

        }
        else if(HasBox() && gcode == 0x02) {
            return;
            // gcode != 0x00 이면 (이미 Open 또는 Close 쪽이라면) 추가 그리퍼 동작 없이 종료

        }
        else if (!HasBox() && gcode != 0x01) {
            // 2-1) 먼저 Close 쪽으로 정리
            DoOpen_Compat(g_hDemoWnd);
            (void)WaitUntil(IsGripperOpenAndIdle, 5000);

            // 2-2) Close 상태에서 Servo OFF
            DoGripServoOff_Compat(g_hDemoWnd);
        }

        // 여기서는 Load 시퀀스를 시작하지 않고 복구만 하고 종료
        return;
    }

    SetTaskState(TaskId::DemoLoad, TaskState::Running);
    std::thread([]() {
        bool ok = true;
        unsigned char code = CalcPosTravelCode();
        // 1. Conveyor 위치로 이동 (필요할 때만)
        if (ok) {
            if (code == 0x01) {
            }
            else {
                GO_Conveyor();
                if (!WaitUntil(IsAxis0AtConveyorBarcodeStopped, 30000)) {
                    ok = false;
                }
				Sleep(1000);
            }
        }

        // 1. ConveyorDown (박스 높이로 하강)
        if (ok) {
            ConveyorDown();
            // 축2가 ConveyorDown 위치에 도달할 때까지 대기
            if (!WaitUntil(IsAxis2Conveyordown, 20000))
                ok = false;
        }

        // 2. 그리퍼 Close (박스 잡기)
        if (ok) {
			DoGripServoOff_Compat(g_hDemoWnd); // Servo ON
            Sleep(500);
            DoClose_Compat(g_hDemoWnd);
            // Load의 목적은 "박스를 잡는 것"이므로 HasBox()를 기준으로 대기
            if (!WaitUntil(HasBox, 5000) || !WaitUntil(IsGripperClosedAndIdle, 5000)) {
                ok = false;
                if(!HasBox()) {
                    bool Ok = true;
                    // 박스가 없다면 Open 상태로 정리
                    DoOpen_Compat(g_hDemoWnd);
                    if (!WaitUntil(IsGripperOpenAndIdle, 5000)) {
						Ok = false;
                    }
                    if (Ok) {
                        DoUp();
                        if (!WaitUntil(IsAxis2Up, 20000))
                            Ok = false;
                    }

                }

            }

        }

        // 3. 축2 Up
        if (ok) {
            DoUp();
            if (!WaitUntil(IsAxis2Up, 20000))
                ok = false;
        }

        Sleep(1000);

        SetTaskState(TaskId::DemoLoad, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}

// Unload 시퀀스: Workstation → Conveyor
void StartDemoUnload()
{
    if (!g_commStarted) { SetTaskState(TaskId::DemoUnload, TaskState::Failed); return; }
    if (g_taskStatus[(int)TaskId::DemoUnload].state.load() == TaskState::Running) return;

    if (!CheckDemoUnloadPreconditions()) {
        // 일단 Load 시퀀스 자체는 실패로 표시
        SetTaskState(TaskId::DemoUnload, TaskState::Failed);

        // 1) Axis2가 Limit(Up) 상태가 아니면 먼저 Up으로 정리
        if (!IsAxis2LimitOn()) {
            DoUp();
            (void)WaitUntil(IsAxis2Up, 20000);
        }

        // 2) 그리퍼 상태 확인
        unsigned char gcode = CalcPosGripCode(); // 0x00이면 중간(애매한) 상태라고 가정

        if (HasBox() && gcode != 0x02) {

            // 2-1) 먼저 Close 쪽으로 정리
            DoClose_Compat(g_hDemoWnd);
            (void)WaitUntil(IsGripperClosedAndIdle, 5000);

            // 2-2) Close 상태에서 Servo OFF
            DoGripServoOff_Compat(g_hDemoWnd);

            
        }
        else if(!HasBox() && gcode != 0x01) {
            // 박스가 없다면 Open 상태로 정리
            DoOpen_Compat(g_hDemoWnd);
            (void)WaitUntil(IsGripperOpenAndIdle, 5000);

            DoGripServoOff_Compat(g_hDemoWnd);
        }
        else if (!HasBox() && gcode == 0x01) {
            return;
        }

        // 여기서는 Load 시퀀스를 시작하지 않고 복구만 하고 종료
        return;
    }

    SetTaskState(TaskId::DemoUnload, TaskState::Running);
    std::thread([]() {
        bool ok = true;
        unsigned char code = CalcPosTravelCode();

        // 1. Workstation 위치로 이동 (필요할 때만)
        if (ok) {
            if (code == 0x02) {
                // 이미 Workstation 바코드 위치 → 이동 스킵
            }
            else {
                Go_Workstation();
                if (!WaitUntil(IsAxis0AtWorkstationBarcodeStopped, 30000))
                    ok = false;
            }
            Sleep(1000);
        }


        // 2. WorkDown (작업 위치로 하강)
        if (ok) {
            WorkDown();
            if (!WaitUntil(IsAxis2Workdown, 20000))
                ok = false;
        }

        // 3. 그리퍼 Open (박스 내려놓기)
        if (ok) {
			DoGripServoOff_Compat(g_hDemoWnd); // Servo ON
            Sleep(500);
            DoOpen_Compat(g_hDemoWnd);
            // Unload 목적: "박스 내려놓고 더 이상 들고 있지 않음" → NoBox() 기준
            if (!WaitUntil(NoBox, 5000) || !WaitUntil(IsGripperOpenAndIdle, 5000))
                ok = false;
        }


        // 4. 축2 Up
        if (ok) {
            DoUp();
            if (!WaitUntil(IsAxis2Up, 20000))
                ok = false;
        }

        // 5. Conveyor 위치로 이동
        if (ok) {
            GO_Conveyor();
            if (!WaitUntil(IsAxis0AtConveyorBarcodeStopped, 30000))
                ok = false;
        }
        Sleep(1000);

        SetTaskState(TaskId::DemoUnload, ok ? TaskState::Done : TaskState::Failed);
        }).detach();
}

// =======================================
// UI 배치
// 좌측: GPIO 제어/표시(토글/라벨)
// 우측: Demo 버튼/상태/센서
// =======================================
enum : UINT_PTR {
    IDT_AX2_SENSOR_POLL = 0x2001,
    IDT_GPIO_REFRESH = 0x2002
};
enum : int {
    // Demo 우측 버튼들
    ID_BTN_WORKSTATION = 11001,
    ID_BTN_CONVEYOR,
    ID_BTN_UP,
    ID_BTN_WORK_DOWN,
    ID_BTN_CONVEYOR_DOWN,
    ID_BTN_STOP_ALL,
    ID_BTN_DEMO_HOME_WITH_BOX,
    ID_BTN_DEMO_HOME_WITHOUT_BOX,
    ID_BTN_DEMO_Work_WITH_BOX,
    ID_BTN_DEMO_Work_WITHOUT_BOX,
    ID_BTN_DEMO_LOAD,
    ID_BTN_DEMO_UNLOAD
};

static void CreateStatusArea(HWND h, int x, int y, int w, int hgt) {
    HWND grp = CreateWindow(TEXT("BUTTON"), TEXT("Status"),
        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
        x, y, w, hgt, h, 0, 0, 0);

    int innerX = x + 10;
    int innerY = y + 25;
    int lineH = 22;
    int colW = w - 20;
    for (int i = 0; i < (int)TaskId::COUNT; ++i) {
        HWND s = CreateWindow(TEXT("STATIC"),
            TaskName((TaskId)i),
            WS_CHILD | WS_VISIBLE,
            innerX, innerY + i * (lineH + 6), colW, lineH, h, 0, 0, 0);
        g_hStatusStatics[i] = s;
        std::wstring text = std::wstring(TaskName((TaskId)i)) + L": " + TaskStateStr(g_taskStatus[i].state.load());
        SetWindowTextW(s, text.c_str());
    }
    (void)grp;
}

static void CreateLeftGPIOUI(HWND h, HINSTANCE hInst, int x, int y, int w, int hgt)
{
    HFONT hTitle = MakeUIFont(16, FW_SEMIBOLD);
    HFONT hText = MakeUIFont(12);

    CreateWindow(TEXT("BUTTON"), TEXT("GPIO"),
        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
        x, y, w, hgt, h, 0, 0, 0);

    HWND hTitleLbl = CreateWindowEx(0, _T("STATIC"), _T("Found 16 GPIO(s)"),
        WS_CHILD | WS_VISIBLE, x + 10, y + 8, 240, 24, h, 0, hInst, 0);
    SendMessage(hTitleLbl, WM_SETFONT, (WPARAM)hTitle, TRUE);

    // DI 0..7 라벨 (2열) - 간격 확대
    int xDI = x + 10, yDI = y + 40;
    int diColW = 200;
    int diRowH = 24;
    for (int i = 0; i < 8; ++i) {
        TCHAR buf[64]; _stprintf_s(buf, _T("Pin %d : --"), i);
        int col = (i % 2);
        int row = (i / 2);
        g_lblDI[i] = CreateWindowEx(0, _T("STATIC"), buf,
            WS_CHILD | WS_VISIBLE, xDI + col * diColW, yDI + row * diRowH, diColW - 10, diRowH, h, 0, hInst, 0);
        SendMessage(g_lblDI[i], WM_SETFONT, (WPARAM)hText, TRUE);
    }

    // 토글 스위치 등록
    RegisterToggleClass(hInst);

    // DO 8..15 토글 스위치 + 라벨 (4열 x 2행)로 변경하여 높이 절약
    int xBase = x + 10, yBase = y + 40 + diRowH * 4 + 20; // DI 섹션 아래 충분한 간격
    int cols = 4;
    int colW = (w - 20) / cols; // 그룹박스 내 가용 폭 분배
    int rowH = 80;              // 각 토글의 세로 공간
    for (int idx = 0; idx < 8; ++idx) {
        int i = 8 + idx;
        int col = idx % cols;
        int row = idx / cols;
        int bx = xBase + col * colW + 5;
        int by = yBase + row * rowH;

        TCHAR lbl[128];
        if (g_DOFuncNames[i])
            _stprintf_s(lbl, _T("Pin %d : %s"), i, g_DOFuncNames[i]);
        else
            _stprintf_s(lbl, _T("Pin %d"), i);

        HWND hLbl = CreateWindowEx(WS_EX_CLIENTEDGE, _T("STATIC"), lbl,
            WS_CHILD | WS_VISIBLE, bx, by, colW - 12, 20, h, 0, hInst, 0);
        SendMessage(hLbl, WM_SETFONT, (WPARAM)hText, TRUE);

        HWND hSw = CreateWindowEx(0, TOGGLE_CLS, _T(""),
            WS_CHILD | WS_VISIBLE | WS_TABSTOP, bx, by + 24, colW - 40, 32, h, (HMENU)(100 + i), hInst, (LPVOID)(INT_PTR)i);
        SendMessage(hSw, WM_SETFONT, (WPARAM)hText, TRUE);
        g_swDO[i] = hSw;
    }

    DeleteObject(hTitle);
    DeleteObject(hText);
}

static void CreateRightDemoUI(HWND h, int x, int y, int w, int hgt)
{
    HFONT hBtn = MakeUIFont(12);

    int yCursor = y;

    // Motion 그룹
    int grpH1 = 160;
    CreateWindow(TEXT("BUTTON"), TEXT("Motion"),
        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
        x, yCursor, w, grpH1, h, 0, 0, 0);
    CreateWindow(TEXT("BUTTON"), TEXT("Workstation"),
        WS_CHILD | WS_VISIBLE,
        x + 20, yCursor + 40, 140, 32, h, (HMENU)ID_BTN_WORKSTATION, 0, 0);
    CreateWindow(TEXT("BUTTON"), TEXT("Conveyor"),
        WS_CHILD | WS_VISIBLE,
        x + 180, yCursor + 40, 140, 32, h, (HMENU)ID_BTN_CONVEYOR, 0, 0);
    yCursor += grpH1 + 12;

    // Lift / Down 그룹
    int grpH2 = 180;
    CreateWindow(TEXT("BUTTON"), TEXT("Lift / Down"),
        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
        x, yCursor, w, grpH2, h, 0, 0, 0);

    CreateWindow(TEXT("BUTTON"), TEXT("UP"),
        WS_CHILD | WS_VISIBLE,
        x + 20, yCursor + 40, 140, 32, h, (HMENU)ID_BTN_UP, 0, 0);
    CreateWindow(TEXT("BUTTON"), TEXT("Work Down"),
        WS_CHILD | WS_VISIBLE,
        x + 180, yCursor + 40, 140, 32, h, (HMENU)ID_BTN_WORK_DOWN, 0, 0);
    CreateWindow(TEXT("BUTTON"), TEXT("Conveyor Down"),
        WS_CHILD | WS_VISIBLE,
        x + 340, yCursor + 40, 160, 32, h, (HMENU)ID_BTN_CONVEYOR_DOWN, 0, 0);
    yCursor += grpH2 + 12;

    // Demo Sequence 그룹 (새로 추가)
    int grpHd = 160;
    CreateWindow(TEXT("BUTTON"), TEXT("Demo Sequence"),
        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
        x, yCursor, w, grpHd, h, 0, 0, 0);

    int btnW = 160;
    int btnH = 32;
    int colGap = 20;
    int startX = x + 20;
    int startY = yCursor + 40;

    // 2열 × 2행 배치
    CreateWindow(TEXT("BUTTON"), TEXT("원점 with Box"),
        WS_CHILD | WS_VISIBLE,
        startX, startY, btnW, btnH, h, (HMENU)ID_BTN_DEMO_HOME_WITH_BOX, 0, 0);
    CreateWindow(TEXT("BUTTON"), TEXT("원점 without Box"),
        WS_CHILD | WS_VISIBLE,
        startX + btnW + colGap, startY, btnW, btnH, h, (HMENU)ID_BTN_DEMO_HOME_WITHOUT_BOX, 0, 0);
    CreateWindow(TEXT("BUTTON"), TEXT("Load"),
        WS_CHILD | WS_VISIBLE,
        startX, startY + btnH + 10, btnW, btnH, h, (HMENU)ID_BTN_DEMO_LOAD, 0, 0);
    CreateWindow(TEXT("BUTTON"), TEXT("Unload"),
        WS_CHILD | WS_VISIBLE,
        startX + btnW + colGap, startY + btnH + 10, btnW, btnH, h, (HMENU)ID_BTN_DEMO_UNLOAD, 0, 0);

    CreateWindow(TEXT("BUTTON"), TEXT("Work with Box"),
        WS_CHILD | WS_VISIBLE,
        startX + btnW * 2 + colGap * 2, startY, btnW, btnH, h, (HMENU)ID_BTN_DEMO_Work_WITH_BOX, 0, 0);
    CreateWindow(TEXT("BUTTON"), TEXT("Work without Box"),
        WS_CHILD | WS_VISIBLE,
        startX + btnW * 2 + colGap * 2, startY + btnH + 10, btnW, btnH, h, (HMENU)ID_BTN_DEMO_Work_WITHOUT_BOX, 0, 0);

    yCursor += grpHd + 12;

    // STOP 그룹
    int grpH3 = 100;
    CreateWindow(TEXT("BUTTON"), TEXT("STOP"),
        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
        x, yCursor, w, grpH3, h, 0, 0, 0);
    CreateWindow(TEXT("BUTTON"), TEXT("STOP ALL"),
        WS_CHILD | WS_VISIBLE,
        x + (w - 140) / 2, yCursor + 40, 140, 34, h, (HMENU)ID_BTN_STOP_ALL, 0, 0);
    yCursor += grpH3 + 12;

    // Status 상자: 라인 수에 넉넉하게
    int statusLines = (int)TaskId::COUNT;
    int lineH = 24;
    int statusH = 25 + statusLines * (lineH + 6) + 10;
    CreateStatusArea(h, x, yCursor, w, statusH);
    yCursor += statusH + 12;

    // Axis2 센서 라벨
    g_hAx2LimitStatic = CreateWindow(TEXT("STATIC"), TEXT("Axis2 LIMIT: -"),
        WS_CHILD | WS_VISIBLE, x + 10, yCursor, w / 2 - 20, 24, h, 0, 0, 0);
    g_hAx2HomeStatic = CreateWindow(TEXT("STATIC"), TEXT("Axis2 HOME: -"),
        WS_CHILD | WS_VISIBLE, x + w / 2 + 10, yCursor, w / 2 - 20, 24, h, 0, 0, 0);
    yCursor += 40;

    // 버튼 폰트 적용
    for (int id : {
        ID_BTN_WORKSTATION, ID_BTN_CONVEYOR, ID_BTN_UP,
            ID_BTN_WORK_DOWN, ID_BTN_CONVEYOR_DOWN, ID_BTN_STOP_ALL,
            ID_BTN_DEMO_HOME_WITH_BOX, ID_BTN_DEMO_HOME_WITHOUT_BOX,
            ID_BTN_DEMO_LOAD, ID_BTN_DEMO_UNLOAD }) {
        HWND b = GetDlgItem(h, id);
        if (b) SendMessage(b, WM_SETFONT, (WPARAM)hBtn, TRUE);
    }
    DeleteObject(hBtn);
}

// 레이아웃 헬퍼: 리사이즈 시 재배치
static void LayoutChildren(HWND hWnd) {
    RECT rc; GetClientRect(hWnd, &rc);
    int totalW = rc.right - rc.left;
    int totalH = rc.bottom - rc.top;

    int margin = 10;
    int gap = 10;

    // 좌측/우측 폭 비율
    int leftW = 560;               // 좌측 GPIO 패널 기본 폭 확대
    if (totalW < 900) leftW = totalW / 2 - gap; // 극단적으로 작아질 때 비율 보정
    int rightX = margin + leftW + gap;
    int rightW = totalW - rightX - margin;

    // 좌측 높이는 전체 높이 - 여백
    int leftH = totalH - margin * 2;
    int rightH = leftH;

    // 좌측 그룹을 전체 높이 사용
    // 만들어진 컨트롤을 재생성하지 않고, 그룹박스 기준으로는 별도 핸들을 보관하지 않았으므로
    // 자식들을 상대적 좌표로 만들었고, 여기서는 레이아웃 재구성이 필요하면 재생성하는 구조이나
    // 간단히 윈도우 전체를 다시 만들어지는 패턴이 아니므로, 리사이즈 대비는 비율만 유지.
    // 이미 생성된 컨트롤의 위치를 옮기려면 핸들을 저장해야 하므로, 여기서는 최초 생성 시 충분한 여백이 있어 겹치지 않게 보장.
    // 따라서 리사이즈는 영향을 덜 주도록 기본 사이즈를 크게 설정함.
    // 필요 시 전체 재구성 로직으로 확장 가능.

    // 좌측 GPIO 영역은 CreateLeftGPIOUI 생성 당시 좌표 고정이므로 리사이즈 영향 최소화
    // 우측은 CreateRightDemoUI 생성 당시 좌표 고정이므로 역시 기본 창을 크게 유지하여 겹침 방지
}

static LRESULT CALLBACK DemoWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CREATE: {
        g_hDemoWnd = hWnd;
        s_servoOffHandledOnce = false;
        ResetAllTaskStates();
        Axis2SensorInit();

        // GPIO 초기화
        if (!EnumerateGPIO() || !PickBank_DI0_7_DO8_15()) {
            MessageBox(hWnd, TEXT("GPIO unavailable or no bank for DI0..7 / DO8..15."), TEXT("GPIO Error"), MB_ICONERROR);
        }
        else {
            (void)EnsureDO8to15AsOutput_BankFirst();
        }

        // 레이아웃
        RECT rc; GetClientRect(hWnd, &rc);
        int totalW = rc.right - rc.left;
        int totalH = rc.bottom - rc.top;
        int margin = 10;
        int gap = 10;

        int leftW = 560;      // 좌측 GPIO 패널 폭 (확대)
        int rightX = margin + leftW + gap;
        int rightW = totalW - rightX - margin;

        // 좌측 패널 생성
        CreateLeftGPIOUI(hWnd, ((LPCREATESTRUCT)lParam)->hInstance, margin, margin, leftW, totalH - margin * 2);

        // 우측 패널 생성
        CreateRightDemoUI(hWnd, rightX, margin, rightW, totalH - margin * 2);

        // 시작 시 레벨 리프레시
        RefreshLevels(hWnd);

        // 타이머 시작
        SetTimer(hWnd, IDT_AX2_SENSOR_POLL, AX2_SENSOR_POLL_MS, nullptr);
        SetTimer(hWnd, IDT_GPIO_REFRESH, kPollIntervalMs, nullptr);

        // 창이 열릴 때 STO 펄스 1회 (오류 클리어)
        DoGripServoOff_Compat(hWnd);

        return 0;
    }
    case WM_SIZE:
        // 필요시 향후 동적 재배치 구현 가능
        LayoutChildren(hWnd);
        return 0;

    case WM_COMMAND: {
        int id = LOWORD(wParam);

        // 좌측 GPIO 토글 클릭(BN_CLICKED): 100+8..100+15
        if (id >= 108 && id <= 115 && HIWORD(wParam) == BN_CLICKED) {
            HWND hSw = (HWND)lParam;
            int pin = id - 100;
            // 현재 토글 상태 조회
            ToggleState* ts = (ToggleState*)GetWindowLongPtr(hSw, GWLP_USERDATA);
            bool on = ts ? ts->on : false;

            // -----------------------------
        // Open(8) / Close(9) 상호 배타 처리
        // -----------------------------
            if (pin == 8 && on) {
                // Open ON 이면 Close 강제 OFF
                ToggleDO_HW(9, false, g_hDemoWnd);                 // HW + UI 둘 다 OFF
                SetTaskState(TaskId::GripClose, TaskState::Idle);  // 상태도 정리(선택)
            }
            else if (pin == 9 && on) {
                // Close ON 이면 Open 강제 OFF
                ToggleDO_HW(8, false, g_hDemoWnd);
                SetTaskState(TaskId::GripOpen, TaskState::Idle);
            }

            // Open/Close는 STO 먼저
            if (on && (pin == 8 || pin == 9)) {
                DoGripServoOff_Compat(g_hDemoWnd);
            }
            ToggleDO_HW(pin, on, g_hDemoWnd);

            // Task state도 업데이트(옵션)
            if (pin == 8) SetTaskState(TaskId::GripOpen, on ? TaskState::Running : TaskState::Stopped);
            if (pin == 9) SetTaskState(TaskId::GripClose, on ? TaskState::Running : TaskState::Stopped);
            if (pin == 10) SetTaskState(TaskId::GripServoOff, on ? TaskState::Running : TaskState::Stopped);

            return 0;
        }

        // 우측 Demo 버튼들
        switch (id) {
        case ID_BTN_WORKSTATION:          Go_Workstation();       return 0;
        case ID_BTN_CONVEYOR:             GO_Conveyor();          return 0;
        case ID_BTN_UP:                   DoUp();                 return 0;
        case ID_BTN_WORK_DOWN:            WorkDown();             return 0;
        case ID_BTN_CONVEYOR_DOWN:        ConveyorDown();         return 0;
        case ID_BTN_STOP_ALL:             DoStopAll(hWnd);        return 0;
        case ID_BTN_DEMO_HOME_WITH_BOX:   StartDemoHomeWithBox(); return 0;
        case ID_BTN_DEMO_HOME_WITHOUT_BOX:StartDemoHomeWithoutBox(); return 0;
        case ID_BTN_DEMO_Work_WITH_BOX:   StartDemoWorkWithBox(); return 0;
        case ID_BTN_DEMO_Work_WITHOUT_BOX:StartDemoWorkWithoutBox(); return 0;
        case ID_BTN_DEMO_LOAD:            StartDemoLoad();        return 0;
        case ID_BTN_DEMO_UNLOAD:          StartDemoUnload();      return 0;
        default: break;
        }
        break;
    }
    case WM_TIMER:
        if (wParam == IDT_AX2_SENSOR_POLL) {
            Axis2SensorTimerProc(hWnd);
            return 0;
        }
        else if (wParam == IDT_GPIO_REFRESH) {
            // 펄스 자동 OFF 처리
            DWORD now = GetTickCount();
            for (int i = 10; i < 11; ++i) {
                if (g_outputPendingOff[i]) {
                    if (now - g_outputOnTick[i] >= kOutputPulseMs) {
                        g_outputPendingOff[i] = false;
                        ToggleDO_HW(i, false, g_hDemoWnd); // 자동 OFF
                    }
                }
            }
            // 주기적으로 DI/DO 상태 갱신
            RefreshLevels(hWnd);

            // =====================================
        // Motioning(DI0) ↔ DO11 1:1 동기화
        // =====================================
            {
                // DemoLoad / DemoUnload 상태 확인
                TaskState loadState = GetTaskState(TaskId::DemoLoad);
                TaskState unloadState = GetTaskState(TaskId::DemoUnload);

                bool demoBusy =
                    (loadState == TaskState::Running) ||
                    (unloadState == TaskState::Running);

                // ★ 데모 시퀀스가 "안 돌 때만" DI0→DO11 동기화 수행
                if (!demoBusy) {
                    static bool s_prevDo11 = false;

                    // 디바운스된 Motioning 입력 (DI0)
                    bool motioning = g_diStable[0];

                    // 상태가 바뀔 때만 DO11을 갱신해서 쓸데없는 출력 반복 방지
                    if (motioning != s_prevDo11) {
                        ToggleDO_HW(11, motioning, g_hDemoWnd);
                        s_prevDo11 = motioning;
                    }
                }
            }
            return 0;
        }
        break;

    case WM_CLOSE:
        DestroyWindow(hWnd);
        return 0;

    case WM_DESTROY:
        KillTimer(hWnd, IDT_AX2_SENSOR_POLL);
        KillTimer(hWnd, IDT_GPIO_REFRESH);
        g_bcRunner.Stop();
        g_hDemoWnd = nullptr;
        DeInitializeEAPI();
        return 0;
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}

// 외부에서 호출하는 진입 함수 (기존 ShowDemoControlWindow 대체)
void ShowDemoControlWindow(HWND hParent, bool minimized)
{
    if (g_hDemoWnd && IsWindow(g_hDemoWnd)) {
        ShowWindow(g_hDemoWnd, minimized ? SW_SHOWMINNOACTIVE : SW_SHOWNORMAL);
        if (!minimized) {
            SetForegroundWindow(g_hDemoWnd);
        }
        return;
    }

    WNDCLASS wc{};
    wc.lpszClassName = TEXT("WMX3DemoGPIOUnifiedWnd");
    wc.lpfnWndProc = DemoWndProc;
    wc.hInstance = (HINSTANCE)GetWindowLongPtr(hParent, GWLP_HINSTANCE);
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);

    ATOM atom = RegisterClass(&wc);
    if (!atom) {
        DWORD err = GetLastError();
        if (err != ERROR_CLASS_ALREADY_EXISTS) {
            MessageBox(hParent, TEXT("Failed to register Demo window class!"), TEXT("Error"), MB_ICONERROR);
            return;
        }
    }

    int winW = 1280;
    int winH = 1000;
    g_hDemoWnd = CreateWindow(
        TEXT("WMX3DemoGPIOUnifiedWnd"), TEXT("Demo + GPIO Control"),
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_THICKFRAME,
        CW_USEDEFAULT, CW_USEDEFAULT, winW, winH, hParent, nullptr, wc.hInstance, nullptr);

    ShowWindow(g_hDemoWnd, minimized ? SW_SHOWMINNOACTIVE : SW_SHOWNORMAL);
    UpdateWindow(g_hDemoWnd);
}