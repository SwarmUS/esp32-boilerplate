#ifndef __MOCK_UI_H_
#define __MOCK_UI_H_

#include <bsp/IUserInterface.h>
#include <cstdarg>
#include <gmock/gmock.h>

class UserInterfaceMock final : public IUserInterface {
  public:
    int& m_printCallCounter;

    UserInterfaceMock(int& printCounter) : m_printCallCounter(printCounter) {}
    ~UserInterfaceMock() override = default;

    // GMock des not support variable arguments, so lets mock it ourselves
    int print(const char* format, ...) const override {
        va_list args;
        va_start(args, format);
        m_printCallCounter++;
        va_end(args);
        return 0;
    }

    int print(const char* format, va_list args) const override {
        (void)format;
        (void)args;
        m_printCallCounter++;
        return 0;
    }

    int logInfo(const char* format, va_list args) const override {
        (void)format;
        (void)args;
        m_printCallCounter++;
        return 0;
    }

    int logDebug(const char* format, va_list args) const override {
        (void)format;
        (void)args;
        m_printCallCounter++;
        return 0;
    }

    int logWarn(const char* format, va_list args) const override {
        (void)format;
        (void)args;
        m_printCallCounter++;
        return 0;
    }

    int logError(const char* format, va_list args) const override {
        (void)format;
        (void)args;
        m_printCallCounter++;
        return 0;
    }
};

#ifdef __cplusplus
extern "C" {
#endif

extern void app_main() {}

#ifdef __cplusplus
}
#endif

#endif // __MOCK_UI_H_
