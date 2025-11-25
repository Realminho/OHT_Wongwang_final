#include "stdafx.h"
#include "atEAPI.h"
#include <windows.h>
#include <stdint.h>
#include <stdio.h>
#include <tchar.h>
#include <string>
#include <vector>
#include <algorithm>

#pragma comment(lib, "User32.lib")
#pragma comment(lib, "Gdi32.lib")
#pragma comment(lib, "Comctl32.lib")

// ===================== EAPI 래퍼 =====================
static HINSTANCE g_hDLL = NULL;

#define BANK_MAX 4

typedef struct {
    uint8_t supPinNum; /* 0 ~ 31 */
    uint32_t supInput;
    uint32_t supOutput;
} GPIOInfo, * PGPIOInfo;

static GPIOInfo info[BANK_MAX];
static const int kFixedBank = -1;
static int g_bank = -1;

static const uint32_t kMaskDI0_7 = 0x000000FFu;
static const uint32_t kMaskDO8_15 = 0x0000FF00u;

// 출력 후 자동 OFF 예약(모든 DO를 펄스 출력으로 사용)
static bool  g_outputPendingOff[16] = { false };
static DWORD g_outputOnTick[16] = { 0 };
static const DWORD kOutputPulseMs = 50;   // ON 후 50ms 뒤 자동 OFF

// 출력 후 입력 안정화 지연(ms)
static const UINT kPostOutputInputDelayMs = 8;

// 입력 디바운스 파라미터
static const int  kDI_SampleCount = 5;      // 샘플 개수(다수결)
static const UINT kPollIntervalMs = 20;     // 타이머 주기
static const int  kDI_BufferDepth = kDI_SampleCount;

// 강제 UI 오버레이(표시 강제용): 실제 캐시 값을 바꾸지 않고 UI만 덮어씀
static bool g_uiOverlayDIValid[8] = { false };
static bool g_uiOverlayDILevel[8] = { false };

static void ToggleDO_HW(int pin, bool turnOn);

// EAPI 인스턴스
HINSTANCE GetEAPIInstance() {
    if (g_hDLL == NULL) g_hDLL = OpenEAPI();
    return g_hDLL;
}
bool InitilizeEAPI(HINSTANCE hDLL) {
    uint32_t status;
    EAPIFunction(hDLL, EApiLibInitialize);
    if (EApiLibInitialize) {
        status = EApiLibInitialize();
        if (status != EAPI_STATUS_SUCCESS && status != EAPI_STATUS_INITIALIZED) return false;
    }
    return true;
}
bool DeInitilizeEAPI(void) {
    if (g_hDLL) {
        uint32_t status;
        EAPIFunction(g_hDLL, EApiLibUnInitialize);
        if (EApiLibUnInitialize) { status = EApiLibUnInitialize(); (void)status; }
        CloseEAPI(g_hDLL);
        g_hDLL = NULL;
    }
    return true;
}
bool EnumerateGPIO(void) {
    uint32_t status, supportPin, id;
    uint8_t found = 0;
    HINSTANCE hDLL = GetEAPIInstance();
    if (!hDLL) return false;
    if (!InitilizeEAPI(hDLL)) return false;

    EAPIFunction(hDLL, EApiGPIOGetDirectionCaps);
    if (!EApiGPIOGetDirectionCaps) return false;

    for (uint8_t i = 0; i < BANK_MAX; i++) {
        id = EAPI_ID_GPIO_BANK(i);
        status = EApiGPIOGetDirectionCaps(id, &info[i].supInput, &info[i].supOutput);
        if (status != EAPI_STATUS_SUCCESS) { info[i].supPinNum = 0; continue; }
        supportPin = info[i].supInput | info[i].supOutput;
        if (supportPin > 0) {
            uint8_t j;
            for (j = 32; j > 0; j--) {
                if (supportPin & (1u << (j - 1))) { info[i].supPinNum = j; break; }
            }
            if (j == 0) info[i].supPinNum = 0;
        }
        else info[i].supPinNum = 0;
        found++;
    }
    return found > 0;
}

static uint32_t BankValidMask(int bank) {
    uint8_t n = info[bank].supPinNum;
    if (n == 0) return 0;
    if (n >= 32) return 0xFFFFFFFFu;
    return (1u << n) - 1u;
}

static bool PickBank_DI0_7_DO8_15() {
    if (kFixedBank >= 0 && kFixedBank < BANK_MAX) { g_bank = kFixedBank; return true; }
    for (int b = 0; b < BANK_MAX; ++b) {
        uint32_t valid = BankValidMask(b);
        if ((valid & 0x0000FFFFu) != 0x0000FFFFu) continue;
        uint32_t diOk = info[b].supInput & kMaskDI0_7;
        uint32_t doOk = info[b].supOutput & kMaskDO8_15;
        if (diOk == kMaskDI0_7 && doOk == kMaskDO8_15) { g_bank = b; return true; }
    }
    int best = -1, scoreBest = -1;
    for (int b = 0; b < BANK_MAX; ++b) {
        uint32_t valid = BankValidMask(b) & 0x0000FFFFu;
        int diCnt = 0, doCnt = 0;
        for (int i = 0; i < 8; ++i) if ((valid & (1u << i)) && (info[b].supInput & (1u << i))) diCnt++;
        for (int i = 8; i < 16; ++i) if ((valid & (1u << i)) && (info[b].supOutput & (1u << i))) doCnt++;
        int score = diCnt + doCnt;
        if (score > scoreBest) { scoreBest = score; best = b; }
    }
    if (best >= 0) { g_bank = best; return true; }
    return false;
}

// Single helpers
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

// Bank helpers
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

static bool EnsureDO8to15AsOutput_BankFirst() {
    if (g_bank < 0) return false;
    uint32_t valid = BankValidMask(g_bank);
    uint32_t targetMask = (kMaskDO8_15 & valid) & info[g_bank].supOutput;
    if (!targetMask) return false;
    uint32_t st = 0;
    return GPIO_SetDirection_Bank(g_bank, targetMask, targetMask, &st);
}

// ===================== 커스텀 토글 스위치 =====================
static const TCHAR* TOGGLE_CLS = _T("GPIO_CTRL_TOGGLE");

struct ToggleState {
    bool on = false;
    bool enabled = true;
    int  pin = -1; // DO 핀 번호 (8..15)
};

static COLORREF ColOn = RGB(40, 167, 69); // 초록
static COLORREF ColOff = RGB(217, 83, 79); // 빨강
static HFONT    g_font = NULL;

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
        ns->on = false;
        ns->enabled = true;
        ns->pin = (int)(INT_PTR)cs->lpCreateParams;
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
        g_font = (HFONT)wParam;
        return 0;

    case WM_GETFONT:
        return (LRESULT)g_font;

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
        PAINTSTRUCT ps;
        HDC hdc; hdc = BeginPaint(hWnd, &ps);
        RECT rc; GetClientRect(hWnd, &rc);

        HDC     memdc = CreateCompatibleDC(hdc);
        HBITMAP membmp = CreateCompatibleBitmap(hdc, rc.right - rc.left, rc.bottom - rc.top);
        HGDIOBJ oldbmp = SelectObject(memdc, membmp);

        HBRUSH bg = CreateSolidBrush(GetSysColor(COLOR_BTNFACE));
        FillRect(memdc, &rc, bg);
        DeleteObject(bg);

        RECT rToggle = rc;
        rToggle.top += 4;
        rToggle.bottom -= 4;
        rToggle.left += 4;
        rToggle.right -= 4;
        COLORREF bgcol = (st && st->on) ? ColOn : ColOff;
        DrawRoundedRect(memdc, rToggle, 18, bgcol, RGB(80, 80, 80));

        int w = rToggle.right - rToggle.left;
        int h = rToggle.bottom - rToggle.top;
        int diameter = h - 6;
        int cx_off = (st && st->on) ? (w - diameter - 6) : 6;
        RECT rKnob = { rToggle.left + cx_off, rToggle.top + 3,
                       rToggle.left + cx_off + diameter, rToggle.top + 3 + diameter };
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
        HFONT   f = (HFONT)SendMessage(hWnd, WM_GETFONT, 0, 0);
        HGDIOBJ of = NULL; if (f) of = SelectObject(memdc, f);
        RECT rText = rToggle;
        rText.left += 10;
        rText.right -= 10;
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

static void Toggle_SetOn(HWND h, bool on) {
    ToggleState* st = (ToggleState*)GetWindowLongPtr(h, GWLP_USERDATA);
    if (st) { st->on = on; InvalidateRect(h, NULL, TRUE); }
}
static bool Toggle_GetOn(HWND h) {
    ToggleState* st = (ToggleState*)GetWindowLongPtr(h, GWLP_USERDATA);
    return st ? st->on : false;
}

// ===================== 메인 UI =====================
static const int ID_BTN_REFRESH = 1001;
static const int ID_BTN_FORCE_OUTPUT = 1002;
static const int ID_TIMER_REFRESH = 2001;

static HWND g_swDO[16] = { 0 }; // 8..15 사용
static HWND g_lblDI[8] = { 0 };
static bool g_cachedLevels[16] = { 0 };

// 입력 디바운스 버퍼
static uint8_t g_diSamples[8][kDI_BufferDepth] = { 0 };
static int     g_diSamplePos = 0;
static bool    g_diStable[8] = { 0 }; // 다수결 확정 값

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

static void UpdateDIText(int i, bool supported, bool level, HWND hWnd) {
    if (!g_lblDI[i]) return;
    TCHAR buf[64];

    if (!supported) {
        _stprintf_s(buf, _T("Pin %d : N/A"), i);
        SetWindowText(g_lblDI[i], buf);
        return;
    }

    // UI 오버레이 우선 적용(표시만 강제)
    bool showLevel = level;
    if (g_uiOverlayDIValid[i]) {
        showLevel = g_uiOverlayDILevel[i];
    }

    // 커스텀 이름
    if (i == 0) {
        _stprintf_s(buf, _T("Motioning : %s"), showLevel ? _T("ON") : _T("OFF"));
    }
    else if (i == 1) {
        _stprintf_s(buf, _T("Catched : %s"), showLevel ? _T("ON") : _T("OFF"));
    }
    else {
        _stprintf_s(buf, _T("Pin %d : %s"), i, showLevel ? _T("OFF") : _T("ON"));
    }

    SetWindowText(g_lblDI[i], buf);
}

// 다수결로 디바운스 결과 계산
static bool MajorityOfSamples(int pin) {
    int ones = 0, total = 0;
    for (int k = 0; k < kDI_BufferDepth; ++k) {
        ones += (g_diSamples[pin][k] ? 1 : 0);
        total++;
    }
    // tie는 이전 안정값 유지하도록(여기서는 >=로 채택)
    return ones >= (total + 1) / 2;
}

// DI 샘플 획득(은행 우선, 실패시 단핀 폴백)
static bool SampleDI_Once(uint32_t& diBitsOut) {
    diBitsOut = 0;
    if (g_bank < 0) return false;

    uint32_t valid = BankValidMask(g_bank);
    uint32_t maskDI = (kMaskDI0_7 & valid) & info[g_bank].supInput;

    uint32_t lvl = 0; uint32_t st = 0;
    bool ok = GPIO_GetLevel_Bank(g_bank, maskDI, &lvl, &st);
    if (!ok) {
        // 폴백: 단핀 읽기
        lvl = 0;
        for (int i = 0; i < 8; ++i) {
            if (!(maskDI & (1u << i))) continue;
            uint32_t bit = 0, stp = 0;
            if (GPIO_Single_GetLevel((uint32_t)(g_bank * 32 + i), &bit, &stp)) {
                if (bit) lvl |= (1u << i);
            }
        }
        // 단핀 폴백 읽기는 성공 간주
        ok = true;
    }

    if (ok) {
        diBitsOut = lvl & 0xFFu;
    }
    return ok;
}

// DO 8..15 레벨 읽기(은행 우선)
static bool ReadDO16Bits(uint32_t& dirOut, uint32_t& lvlOut) {
    dirOut = 0; lvlOut = 0;
    if (g_bank < 0) return false;

    uint32_t valid = BankValidMask(g_bank);
    uint32_t mask16 = (kMaskDI0_7 | kMaskDO8_15) & valid;

    uint32_t st1 = 0, st2 = 0;
    bool okDir = GPIO_GetDirection_Bank(g_bank, mask16, &dirOut, &st1);
    bool okLvl = GPIO_GetLevel_Bank(g_bank, mask16, &lvlOut, &st2);
    if (!okDir || !okLvl) {
        // 폴백(단핀)
        dirOut = 0; lvlOut = 0;
        for (int i = 0; i < 16; ++i) {
            uint32_t gp = g_bank * 32 + i;
            uint32_t bit = 0, st = 0;
            if (GPIO_Single_GetDirection(gp, &bit, &st)) { if (bit) dirOut |= (1u << i); }
            if (GPIO_Single_GetLevel(gp, &bit, &st)) { if (bit) lvlOut |= (1u << i); }
        }
    }
    return true;
}

// DI 오버레이 규칙
static void ApplyDIOverlayRules() {
    // Motioning(DI0)을 output3(STO) 동작 시 강제로 OFF로 표시하기 위해
    // g_uiOverlayDIValid[0] 를 사용한다.
    // 실제 입력이 OFF(=g_diStable[0] == false)로 안정되면
    // 오버레이를 해제해서 이후에는 실제 입력 상태를 그대로 표시.
    if (g_uiOverlayDIValid[0]) {
        if (!g_diStable[0]) {
            // 실제로도 OFF가 되었으므로 오버레이 해제
            g_uiOverlayDIValid[0] = false;
            // g_uiOverlayDILevel[0] 값은 의미 없음
        }
    }
}

// 입력 갱신(디바운스 포함)
static void RefreshInputsWithDebounce(HWND hWnd) {
    uint32_t diNow = 0;
    if (!SampleDI_Once(diNow)) {
        // 실패시 건너뜀
        return;
    }

    // 샘플 버퍼에 기록
    for (int i = 0; i < 8; ++i) {
        g_diSamples[i][g_diSamplePos] = (diNow >> i) & 1u;
    }
    g_diSamplePos = (g_diSamplePos + 1) % kDI_BufferDepth;

    // 다수결로 안정값 계산
    for (int i = 0; i < 8; ++i) {
        bool newStable = MajorityOfSamples(i);
        g_diStable[i] = newStable;
        g_cachedLevels[i] = newStable; // 캐시에 안정값 반영
    }

    // DI 오버레이 규칙 적용
    ApplyDIOverlayRules();

    // DI 라벨 갱신
    for (int i = 0; i < 8; ++i) {
        bool sup = (info[g_bank].supInput & (1u << i)) != 0;
        UpdateDIText(i, sup, g_diStable[i], hWnd);
    }
}

static void RefreshOutputs(HWND hWnd) {
    if (g_bank < 0) return;

    uint32_t dir = 0, lvl = 0;
    if (!ReadDO16Bits(dir, lvl)) return;

    for (int i = 8; i < 16; ++i) {
        bool sup = (info[g_bank].supOutput & (1u << i)) != 0;
        bool on = (lvl & (1u << i)) != 0;
        g_cachedLevels[i] = on;
        if (g_swDO[i]) {
            EnableWindow(g_swDO[i], sup ? TRUE : FALSE);
            Toggle_SetOn(g_swDO[i], sup ? on : false);
        }
    }

    TCHAR title[128];
    _stprintf_s(title, _T("Found 16 GPIO(s) - Bank %d"), g_bank);
    SetWindowText(hWnd, title);
}

static void RefreshLevels(HWND hWnd) {
    if (g_bank < 0) return;
    RefreshInputsWithDebounce(hWnd); // 입력(디바운스)
    RefreshOutputs(hWnd);            // 출력
}

// DO 실제 토글 + 펄스 예약
static void ToggleDO_HW(int pin, bool turnOn)
{
    if (g_bank < 0) return;
    uint32_t bit = (1u << pin);
    if (!(info[g_bank].supOutput & bit)) return;

    (void)EnsureDO8to15AsOutput_BankFirst();

    uint32_t gp = (uint32_t)(g_bank * 32 + pin);
    uint32_t st = 0;
    GPIO_Single_SetDirection(gp, true, &st);
    GPIO_Single_SetLevel(gp, turnOn, &st);

    // UI 즉시 반영(스위치 상태)
    if (g_swDO[pin]) Toggle_SetOn(g_swDO[pin], turnOn);

    // 출력 직후 입력 안정화: 약간의 지연 후 1회 재샘플 → 샘플 버퍼에 반영되도록 처리
    if (kPostOutputInputDelayMs > 0) {
        Sleep(kPostOutputInputDelayMs);
        uint32_t diNow = 0;
        if (SampleDI_Once(diNow)) {
            for (int i = 0; i < 8; ++i) {
                g_diSamples[i][g_diSamplePos] = (diNow >> i) & 1u;
            }
            g_diSamplePos = (g_diSamplePos + 1) % kDI_BufferDepth;
        }
    }

    // output3(STO, pin 10) 동작 시에는 Motioning(DI0)을 강제로 OFF로 표시
    if (pin == 10 && turnOn) {
        g_uiOverlayDIValid[0] = true;   // 오버레이 활성
        g_uiOverlayDILevel[0] = false;  // 표시값: OFF
    }

    // 펄스 동작: ON 되면 50ms 후 자동 OFF 예약, OFF 시에는 예약 해제
    if (turnOn) {
        g_outputPendingOff[pin] = true;
        g_outputOnTick[pin] = GetTickCount();
    }
    else {
        g_outputPendingOff[pin] = false;
    }
}

// UI 생성
static void CreateControls(HWND hWnd, HINSTANCE hInst) {
    HFONT hTitle = MakeUIFont(16, FW_SEMIBOLD);
    HFONT hText = MakeUIFont(12);

    HWND hTitleLbl = CreateWindowEx(0, _T("STATIC"), _T("Found 16 GPIO(s)"),
        WS_CHILD | WS_VISIBLE, 10, 8, 220, 24, hWnd, 0, hInst, 0);
    SendMessage(hTitleLbl, WM_SETFONT, (WPARAM)hTitle, TRUE);

    HWND hBtnRef = CreateWindowEx(0, _T("BUTTON"), _T("Refresh"),
        WS_CHILD | WS_VISIBLE | WS_TABSTOP, 260, 8, 90, 26, hWnd, (HMENU)ID_BTN_REFRESH, hInst, 0);
    SendMessage(hBtnRef, WM_SETFONT, (WPARAM)hText, TRUE);

    HWND hBtnForce = CreateWindowEx(0, _T("BUTTON"), _T("Set DO8..15 as Output"),
        WS_CHILD | WS_VISIBLE | WS_TABSTOP, 360, 8, 180, 26, hWnd, (HMENU)ID_BTN_FORCE_OUTPUT, hInst, 0);
    SendMessage(hBtnForce, WM_SETFONT, (WPARAM)hText, TRUE);

    // DI 0..7 라벨
    int xDI = 10, yDI = 50;
    for (int i = 0; i < 8; ++i) {
        TCHAR buf[64]; _stprintf_s(buf, _T("Pin %d : --"), i);
        g_lblDI[i] = CreateWindowEx(0, _T("STATIC"), buf,
            WS_CHILD | WS_VISIBLE, xDI, yDI + (i / 4) * 60 + (i % 4) * 20, 140, 20, hWnd, 0, hInst, 0);
        SendMessage(g_lblDI[i], WM_SETFONT, (WPARAM)hText, TRUE);
    }

    // 토글 스위치 등록
    RegisterToggleClass(hInst);

    // DO 8..15 기능 이름
    static const TCHAR* g_DOFuncNames[16] = {
        nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,
        _T("Open"),     // 8
        _T("Close"),    // 9
        _T("STO"),      // 10 (output3)
        _T("None"),     // 11
        _T("None"),     // 12
        _T("None"),     // 13
        nullptr,        // 14
        nullptr         // 15
    };

    // DO 8..15 토글 스위치 + 핀 라벨 생성
    int xBase = 180, yBase = 50;
    for (int i = 8; i < 16; ++i) {
        int col = (i - 8) % 3;
        int row = (i - 8) / 3;
        int bx = xBase + col * 140;
        int by = yBase + row * 70;

        TCHAR lbl[128];
        if (g_DOFuncNames[i])
            _stprintf_s(lbl, _T("Pin %d : %s"), i, g_DOFuncNames[i]);
        else
            _stprintf_s(lbl, _T("Pin %d"), i);

        HWND hLbl = CreateWindowEx(WS_EX_CLIENTEDGE, _T("STATIC"), lbl,
            WS_CHILD | WS_VISIBLE, bx, by, 140, 18, hWnd, 0, hInst, 0);
        SendMessage(hLbl, WM_SETFONT, (WPARAM)hText, TRUE);

        HWND hSw = CreateWindowEx(0, TOGGLE_CLS, _T(""),
            WS_CHILD | WS_VISIBLE | WS_TABSTOP, bx, by + 22, 120, 32, hWnd, (HMENU)(100 + i), hInst, (LPVOID)(INT_PTR)i);
        SendMessage(hSw, WM_SETFONT, (WPARAM)hText, TRUE);
        g_swDO[i] = hSw;
    }

    // 초기 디바운스/오버레이 버퍼 비움
    ZeroMemory(g_diSamples, sizeof(g_diSamples));
    g_diSamplePos = 0;
    ZeroMemory(g_diStable, sizeof(g_diStable));
    ZeroMemory(g_uiOverlayDIValid, sizeof(g_uiOverlayDIValid));
    ZeroMemory(g_uiOverlayDILevel, sizeof(g_uiOverlayDILevel));

    RefreshLevels(hWnd);

    DeleteObject(hTitle);
    DeleteObject(hText);
}

// ===================== 윈도우 프로시저 =====================
static const TCHAR* MAIN_CLS = _T("GPIO_CTRL_MAIN");

static LRESULT CALLBACK GPIO_WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CREATE: {
        CreateControls(hWnd, ((LPCREATESTRUCT)lParam)->hInstance);
        // 빠른 폴링(20ms) → 5샘플(약 100ms) 디바운스
        SetTimer(hWnd, ID_TIMER_REFRESH, kPollIntervalMs, NULL);

        // 창이 열릴 때마다 output3 동작(STO, DO10 펄스) 실행 → 에러 클리어용
        ToggleDO_HW(10, true);

        return 0;
    }
    case WM_COMMAND: {
        int id = LOWORD(wParam);
        if (id == ID_BTN_REFRESH) {
            RefreshLevels(hWnd);
        }
        else if (id == ID_BTN_FORCE_OUTPUT) {
            (void)EnsureDO8to15AsOutput_BankFirst();
            RefreshLevels(hWnd);
        }
        else if (id >= 100 + 8 && id <= 100 + 15 && HIWORD(wParam) == BN_CLICKED) {
            HWND hSw = (HWND)lParam;
            int  pin = id - 100;
            bool on = Toggle_GetOn(hSw);

            // output1(Open, pin 8) / output2(Close, pin 9)이 ON으로 눌리면
            // 먼저 output3(STO, pin 10)을 펄스로 동작시켜 오류 제거 후 해당 output 동작
            if (on && (pin == 8 || pin == 9)) {
                ToggleDO_HW(10, true);  // output3 동작
            }

            // 실제 선택된 DO 핀 동작
            ToggleDO_HW(pin, on);

            // 출력 후 곧바로 입력 리프레시(버퍼 샘플 추가 포함)
            RefreshInputsWithDebounce(hWnd);
        }
        return 0;
    }
    case WM_TIMER:
        if (wParam == ID_TIMER_REFRESH) {
            DWORD now = GetTickCount();
            // Output 자동 OFF (50ms 펄스 처리)
            for (int i = 8; i < 16; i++) {
                if (g_outputPendingOff[i]) {
                    if (now - g_outputOnTick[i] >= kOutputPulseMs) {
                        g_outputPendingOff[i] = false;
                        ToggleDO_HW(i, false);          // 자동 OFF
                        RefreshInputsWithDebounce(hWnd);
                    }
                }
            }

            // 주기적인 DI/DO 리프레시
            RefreshLevels(hWnd);
        }
        return 0;
    case WM_DESTROY:
        KillTimer(hWnd, ID_TIMER_REFRESH);
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}

// ===================== 외부 실행용 런처 (WinMain 제거 버전) =====================
extern "C" __declspec(dllexport) int RunGPIOWindowExternal(HINSTANCE hInst) {
    if (!EnumerateGPIO()) {
        MessageBox(NULL, _T("GPIO Unavailable!"), _T("Error"), MB_ICONERROR);
        DeInitilizeEAPI(); return 1;
    }
    if (!PickBank_DI0_7_DO8_15()) {
        MessageBox(NULL, _T("No suitable bank (DI0..7 / DO8..15)."), _T("Error"), MB_ICONERROR);
        DeInitilizeEAPI(); return 1;
    }
    (void)EnsureDO8to15AsOutput_BankFirst();

    WNDCLASS wc = { 0 };
    wc.lpfnWndProc = GPIO_WndProc;
    wc.hInstance = hInst;
    wc.lpszClassName = MAIN_CLS;
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);

    ATOM atom = RegisterClass(&wc);
    if (!atom) {
        DWORD err = GetLastError();
        if (err != ERROR_CLASS_ALREADY_EXISTS) {
            DeInitilizeEAPI();
            return 1;
        }
    }

    HWND hWnd = CreateWindowEx(0, MAIN_CLS, _T("Found 16 GPIO(s)"),
        WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 600, 380,
        NULL, NULL, hInst, NULL);
    if (!hWnd) { DeInitilizeEAPI(); return 1; }

    ShowWindow(hWnd, SW_SHOWNORMAL);
    UpdateWindow(hWnd);

    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0)) { TranslateMessage(&msg); DispatchMessage(&msg); }

    DeInitilizeEAPI();
    return (int)msg.wParam;
}
