#include "modal_dialog.h"
#include "BinaryData.h"

#ifdef _WIN32
#include <windows.h>
#include <dwmapi.h>
#endif

void apply_wayverb_window_style(TopLevelWindow* w) {
    if (!w) return;

    const auto icon = juce::ImageCache::getFromMemory(
            BinaryData::wayverb_png, BinaryData::wayverb_pngSize);
    if (icon.isValid()) {
        if (auto* peer = w->getPeer()) {
            peer->setIcon(icon);
        }
    }

#ifdef _WIN32
    if (auto* peer = w->getPeer()) {
        HWND hwnd = static_cast<HWND>(peer->getNativeHandle());
        COLORREF purple = RGB(92, 0, 163);
        constexpr DWORD DWMWA_CAPTION_COLOR_VAL = 35;
        DwmSetWindowAttribute(
                hwnd, DWMWA_CAPTION_COLOR_VAL, &purple, sizeof(purple));
    }
#endif
}
