#include "UserInterface.h"
#include "esp_log.h"
#include <cstdio>

int UserInterface::print(const char* format, va_list args) const {
    int ret = vprintf(format, args);
    printf("\n\r");
    return ret;
}

int UserInterface::print(const char* format, ...) const {
    va_list args;
    va_start(args, format);
    int retValue = print(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::logInfo(const char* format, va_list args) const {
    esp_log_writev(ESP_LOG_INFO, "Logger", format, args);
    printf("\r\n");
    return 0;
}

int UserInterface::logDebug(const char* format, va_list args) const {
    esp_log_writev(ESP_LOG_INFO, "Logger", format, args);
    printf("\r\n");
    return 0;
};

int UserInterface::logWarn(const char* format, va_list args) const {
    esp_log_writev(ESP_LOG_INFO, "Logger", format, args);
    printf("\r\n");
    return 0;
};

int UserInterface::logError(const char* format, va_list args) const {
    esp_log_writev(ESP_LOG_INFO, "Logger", format, args);
    printf("\r\n");
    return 0;
}