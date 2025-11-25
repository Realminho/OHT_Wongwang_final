#define NOMINMAX
#define _USE_MATH_DEFINES

#include <windows.h>
#include <tchar.h>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "MapView.h"

using namespace wmx3Api;

// POINTF 정의
struct POINTF { float x; float y; };

// 사용자 메시지
#ifndef WM_APP_MAP_GOTO
#define WM_APP_MAP_GOTO (WM_APP + 201)
struct MapGotoParam { long long target; };
#endif

// ==================== 상수 ====================
static const long long kTrackLen = 10000000LL;
static const long long kStationInterval = 2000000LL;
static const int kNumStations = 5; // 짝수/홀수 모두 대응
static const UINT kMapTimerMs = 50;

// ==================== 백버퍼 구조체 ====================
struct BackBuffer {
    HBITMAP hbmp = nullptr;
    HDC memdc = nullptr;
    int w = 0, h = 0;
    void Release(HDC) {
        if (memdc) { DeleteDC(memdc); memdc = nullptr; }
        if (hbmp) { DeleteObject(hbmp); hbmp = nullptr; }
        w = h = 0;
    }
};

// ==================== 맵 컨텍스트 ====================
struct MapCtx {
    HWND hWnd = nullptr;
    HWND hParent = nullptr;
    WMX3Api* pWmx = nullptr;
    CoreMotion* pCm = nullptr;

    RECT rcMap{}, rcPanel{};
    double cx = 0, cy = 0;
    double r = 150;
    double straight = 400;

    COLORREF colBg = RGB(30, 30, 30);
    COLORREF colTrack = RGB(80, 160, 220);
    COLORREF colBot = RGB(255, 200, 0);
    COLORREF colStation = RGB(200, 80, 80);
    COLORREF colText = RGB(230, 230, 230);

    BackBuffer bb;

    long long pos = 0;
    bool hasTarget = false;
    long long targetPos = 0;
    long long atTargetEpsilon = 5000;
    bool homedOnceAfterArrival = false;

    int currentStation = 1; // 현재 스테이션 (1~kNumStations)
};

// ==================== 유틸 함수 ====================
static inline double HalfCircleLen(double r) { return M_PI * r; }
static inline double CapsulePerimeter_Round(double r, double straight) {
    return 2.0 * straight + 2.0 * M_PI * r;
}

// ==================== 레이아웃 ====================
static void Map_Layout(MapCtx* c) {
    RECT rc; GetClientRect(c->hWnd, &rc);
    int W = rc.right - rc.left, H = rc.bottom - rc.top;
    int panelH = 90;
    c->rcPanel = { 0, H - panelH, W, H };
    c->rcMap = { 0, 0, W, H - panelH };

    int mw = c->rcMap.right - c->rcMap.left;
    int mh = c->rcMap.bottom - c->rcMap.top;

    c->cx = c->rcMap.left + mw * 0.5;
    c->cy = c->rcMap.top + mh * 0.65;
    double margin = 40;
    double maxR = std::max(50.0, std::min((mw - 2 * margin) * 0.15, (mh - 2 * margin) * 0.20));
    c->r = maxR;
    c->straight = std::max(200.0, (mw - 2 * margin) - 2.0 * c->r);
}

// ==================== 경로 계산 ====================
static POINTF CapsulePointAt(const MapCtx* c, long long pos)
{
    double perim = CapsulePerimeter_Round(c->r, c->straight);
    long long m = pos % kTrackLen;
    if (m < 0) m += kTrackLen;
    double s = (double)m / (double)kTrackLen;
    double L = s * perim;

    double seg1 = c->straight, seg2 = HalfCircleLen(c->r);
    double seg3 = c->straight, seg4 = HalfCircleLen(c->r);
    double x = 0, y = 0;

    if (L <= seg1) {
        double t = L / seg1;
        x = c->cx - (c->straight * 0.5) + t * (c->straight);
        y = c->cy + c->r;
    }
    else if (L <= seg1 + seg2) {
        double t = (L - seg1) / seg2;
        double ang = M_PI * 0.5 + (-M_PI) * t;
        double cxR = c->cx + c->straight * 0.5;
        x = cxR + c->r * std::cos(ang);
        y = c->cy + c->r * std::sin(ang);
    }
    else if (L <= seg1 + seg2 + seg3) {
        double t = (L - seg1 - seg2) / seg3;
        x = c->cx + (c->straight * 0.5) - t * (c->straight);
        y = c->cy - c->r;
    }
    else {
        double t = (L - seg1 - seg2 - seg3) / seg4;
        double ang = -M_PI * 0.5 - (M_PI)*t; // 시계 방향 통일
        double cxL = c->cx - c->straight * 0.5;
        x = cxL + c->r * std::cos(ang);
        y = c->cy + c->r * std::sin(ang);
    }

    return { (float)x, (float)y };
}

// ==================== 그리기 ====================
static void DrawCapsuleTrack(HDC dc, const MapCtx* c) {
    HPEN pen = CreatePen(PS_SOLID, 5, c->colTrack);
    HPEN old = (HPEN)SelectObject(dc, pen);
    HBRUSH oldB = (HBRUSH)SelectObject(dc, GetStockObject(NULL_BRUSH));

    MoveToEx(dc, (int)(c->cx - c->straight * 0.5), (int)(c->cy + c->r), nullptr);
    LineTo(dc, (int)(c->cx + c->straight * 0.5), (int)(c->cy + c->r));

    Arc(dc,
        (int)(c->cx + c->straight * 0.5 - c->r), (int)(c->cy - c->r),
        (int)(c->cx + c->straight * 0.5 + c->r), (int)(c->cy + c->r),
        (int)(c->cx + c->straight * 0.5), (int)(c->cy + c->r),
        (int)(c->cx + c->straight * 0.5), (int)(c->cy - c->r));

    MoveToEx(dc, (int)(c->cx + c->straight * 0.5), (int)(c->cy - c->r), nullptr);
    LineTo(dc, (int)(c->cx - c->straight * 0.5), (int)(c->cy - c->r));

    Arc(dc,
        (int)(c->cx - c->straight * 0.5 - c->r), (int)(c->cy - c->r),
        (int)(c->cx - c->straight * 0.5 + c->r), (int)(c->cy + c->r),
        (int)(c->cx - c->straight * 0.5), (int)(c->cy - c->r),
        (int)(c->cx - c->straight * 0.5), (int)(c->cy + c->r));

    SelectObject(dc, old);
    DeleteObject(pen);
    SelectObject(dc, oldB);
}

static void DrawStations(HDC dc, const MapCtx* c)
{
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, c->colText);
    HFONT oldF = (HFONT)SelectObject(dc, GetStockObject(DEFAULT_GUI_FONT));

    for (int i = 1; i <= kNumStations; ++i) {
        long long spos = (i - 1) * kStationInterval;
        POINTF pt = CapsulePointAt(c, spos);
        RECT r = { (int)pt.x - 25, (int)pt.y - 25, (int)pt.x + 25, (int)pt.y + 25 };
        HPEN pen = CreatePen(PS_SOLID, 2, c->colStation);
        HBRUSH br = CreateSolidBrush(RGB(60, 20, 20));
        HPEN oldP = (HPEN)SelectObject(dc, pen);
        HBRUSH oldB = (HBRUSH)SelectObject(dc, br);
        Ellipse(dc, r.left, r.top, r.right, r.bottom);
        SelectObject(dc, oldP); DeleteObject(pen);
        SelectObject(dc, oldB); DeleteObject(br);

        TCHAR name[32];
        _stprintf_s(name, TEXT("Station %d"), i);
        RECT tr = { r.left, r.bottom + 2, r.right, r.bottom + 20 };
        DrawText(dc, name, -1, &tr, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    }

    SelectObject(dc, oldF);
}

static void DrawRobot(HDC dc, const MapCtx* c)
{
    POINTF pt = CapsulePointAt(c, c->pos);
    int hw = 8, hh = 8;
    HPEN pen = CreatePen(PS_SOLID, 2, RGB(20, 20, 20));
    HBRUSH br = CreateSolidBrush(c->colBot);
    HPEN oldP = (HPEN)SelectObject(dc, pen);
    HBRUSH oldB = (HBRUSH)SelectObject(dc, br);
    Rectangle(dc, (int)pt.x - hw, (int)pt.y - hh, (int)pt.x + hw, (int)pt.y + hh);
    SelectObject(dc, oldP); DeleteObject(pen);
    SelectObject(dc, oldB); DeleteObject(br);
}

// ==================== 홈 함수 ====================
static void TryHomeAxis0(MapCtx* c)
{
    if (!c || !c->pCm) return;

    int axis = 0; // 홈을 수행할 축 번호
    static Home g_home(c->pCm); // CoreMotion 기반 Home 객체 생성

    // 홈 시퀀스 시작
    g_home.StartHome(axis);


}

// ==================== 상태 확인 ====================
static bool IsAtTarget(const MapCtx* c)
{
    if (!c->hasTarget) return false;
    long long diff = llabs(c->pos - c->targetPos);
    return diff <= c->atTargetEpsilon;
}

// ==================== 버튼 ====================
#define ID_BTN_BASE 10001

static void CreateStationButtons(MapCtx* c)
{
    for (int i = 1; i <= kNumStations; ++i) {
        TCHAR label[64];
        _stprintf_s(label, TEXT("Station %d"), i);
        CreateWindow(TEXT("BUTTON"), label,
            WS_CHILD | WS_VISIBLE,
            10 + (i - 1) * 170, c->rcPanel.top + 35, 160, 28,
            c->hWnd, (HMENU)(ID_BTN_BASE + i), nullptr, nullptr);
    }
}

// ==================== 메인 윈도우 ====================
static LRESULT CALLBACK MapWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    MapCtx* c = (MapCtx*)GetWindowLongPtr(hWnd, GWLP_USERDATA);
    switch (msg) {
    case WM_CREATE:
    {
        auto cs = (CREATESTRUCT*)lParam;
        c = new MapCtx();
        c->hWnd = hWnd;
        c->hParent = cs->hwndParent;
        if (cs->lpCreateParams) {
            void** pp = (void**)cs->lpCreateParams;
            c->pWmx = (WMX3Api*)pp[0];
            c->pCm = (CoreMotion*)pp[1];
        }
        SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)c);
        Map_Layout(c);
        CreateStationButtons(c);
        SetTimer(hWnd, 1, kMapTimerMs, nullptr);
        return 0;
    }
    case WM_TIMER:
        if (c && wParam == 1) {
            CoreMotionStatus st{};
            c->pCm->GetStatus(&st);
            c->pos = (long long)st.axesStatus[0].actualPos;
            if (c->hasTarget && !c->homedOnceAfterArrival && IsAtTarget(c)) {
                c->homedOnceAfterArrival = true;
                TryHomeAxis0(c);
            }
            InvalidateRect(hWnd, nullptr, FALSE);
        }
        return 0;

    case WM_COMMAND:
        if (c) {
            int id = LOWORD(wParam);
            if (id >= ID_BTN_BASE + 1 && id <= ID_BTN_BASE + kNumStations) {
                int targetStation = id - ID_BTN_BASE;
                int diff = targetStation - c->currentStation;

                // 항상 시계방향(증가 방향)으로만 이동
                int forwardSteps = (diff >= 0) ? diff : (diff + kNumStations);
                int steps = forwardSteps;
                long long moveLen = steps * kStationInterval;


                c->targetPos = c->pos + moveLen;
                c->hasTarget = true;
                c->homedOnceAfterArrival = false;

                MapGotoParam* p = new MapGotoParam();
                p->target = moveLen;
                PostMessage(c->hParent, WM_APP_MAP_GOTO, 0, (LPARAM)p);
            }
        }
        return 0;

    case WM_PAINT:
    {
        PAINTSTRUCT ps; HDC hdc = BeginPaint(hWnd, &ps);
        HBRUSH bg = CreateSolidBrush(RGB(30, 30, 30));
        FillRect(hdc, &ps.rcPaint, bg);
        DeleteObject(bg);
        DrawCapsuleTrack(hdc, c);
        DrawStations(hdc, c);
        DrawRobot(hdc, c);
        EndPaint(hWnd, &ps);
        return 0;
    }

    case WM_DESTROY:
        if (c) {
            KillTimer(hWnd, 1);
            delete c;
            SetWindowLongPtr(hWnd, GWLP_USERDATA, 0);
        }
        return 0;
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}

// ==================== 생성 함수 ====================
HWND CreateMapWindow(HWND hParent, wmx3Api::WMX3Api* pWmx, wmx3Api::CoreMotion* pCm)
{
    WNDCLASS wc{};
    wc.lpszClassName = TEXT("WMX3MapWnd");
    wc.lpfnWndProc = MapWndProc;
    wc.hInstance = (HINSTANCE)GetWindowLongPtr(hParent, GWLP_HINSTANCE);
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    RegisterClass(&wc);

    void* params[2] = { pWmx, pCm };

    RECT wr{}; SystemParametersInfo(SPI_GETWORKAREA, 0, &wr, 0);
    int W = 900, H = 600, X = wr.left + 50, Y = wr.top + 50;

    HWND w = CreateWindowEx(
        WS_EX_APPWINDOW,
        TEXT("WMX3MapWnd"), TEXT("Circular Map (Shortest Path + Auto Home)"),
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_SIZEBOX,
        X, Y, W, H, hParent, nullptr, wc.hInstance, (LPVOID)params);
    return w;
}
