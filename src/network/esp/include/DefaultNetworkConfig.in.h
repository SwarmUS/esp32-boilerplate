#ifndef __DEFAULTNETWORKCONFIG_H__
#define __DEFAULTNETWORKCONFIG_H__
// clang-format off

#include <logger/ILogger.h>

/**
 * @brief Template file used by CMake to provide default or user-provided values for the network configuration
 */

#define DEFAULT_SSID "@SSID@"
#define DEFAULT_PASSWORD "@PASSWORD@"
#define DEFAULT_AUTH_MODE @AUTH_MODE@
#define DEFAULT_PORT @COMMUNICATION_PORT@

// clang-format on
#endif //__DEFAULTNETWORKCONFIG_H__
