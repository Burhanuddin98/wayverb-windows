#include "LoadWindow.h"

#include "BinaryData.h"

#ifdef _WIN32
#include <windows.h>
#include <dwmapi.h>
#endif

LoadWindow::LoadWindow(juce::String name,
                       DocumentWindow::TitleBarButtons buttons,
                       const std::string& file_formats,
                       juce::ApplicationCommandManager& command_manager)
        : DocumentWindow(name, juce::Colours::lightgrey, buttons) {
    content_component.setSize(600, 400);
    content_component.set_valid_file_formats(file_formats);
    setContentNonOwned(&content_component, true);
    setUsingNativeTitleBar(true);
    centreWithSize(getWidth(), getHeight());
    setVisible(true);
    setResizable(false, false);
    setResizeLimits(400, 300, 100000, 100000);
    setVisible(true);
    addKeyListener(command_manager.getKeyMappings());

    // Set wayverb logo as window icon
    {
        const auto icon = juce::ImageCache::getFromMemory(
                BinaryData::wayverb_png, BinaryData::wayverb_pngSize);
        if (icon.isValid()) {
            if (auto* peer = getPeer()) {
                peer->setIcon(icon);
            }
        }
    }

#ifdef _WIN32
    // Purple title bar via Windows 11 DWM API
    {
        if (auto* peer = getPeer()) {
            HWND hwnd = static_cast<HWND>(peer->getNativeHandle());
            COLORREF purple = RGB(92, 0, 163);
            constexpr DWORD DWMWA_CAPTION_COLOR_VAL = 35;
            DwmSetWindowAttribute(
                    hwnd, DWMWA_CAPTION_COLOR_VAL, &purple, sizeof(purple));
        }
    }
#endif
}

void LoadWindow::closeButtonPressed() {
    juce::JUCEApplication::getInstance()->systemRequestedQuit();
}

const FileDropComponent& LoadWindow::get_content() const {
    return content_component;
}

FileDropComponent& LoadWindow::get_content() { return content_component; }

void LoadWindow::addListener(FileDropComponent::Listener* l) {
    content_component.addListener(l);
}

void LoadWindow::removeListener(FileDropComponent::Listener* l) {
    content_component.removeListener(l);
}
