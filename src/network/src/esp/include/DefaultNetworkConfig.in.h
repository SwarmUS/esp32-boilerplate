#ifndef __DEFAULTNETWORKCONFIG_H__
#define __DEFAULTNETWORKCONFIG_H__
// clang-format off

#include <logger/ILogger.h>
#include <cstdint>

/**
 * @brief Template file used by CMake to provide default or user-provided values for the network configuration
 */

constexpr bool gs_isRouter = @IS_ROUTER@;
#define DEFAULT_SSID "@SSID@"
#define DEFAULT_PASSWORD "@PASSWORD@"
#define DEFAULT_AUTH_MODE @AUTH_MODE@
#define DEFAULT_UNICAST_PORT @UNICAST_PORT@
#define DEFAULT_BROADCAST_INPUT_PORT @BROADCAST_INPUT_PORT@
#define DEFAULT_BROADCAST_OUTPUT_PORT @BROADCAST_OUTPUT_PORT@
constexpr uint16_t gs_MAX_DATAGRAM_SIZE = @MAX_DATAGRAM_SIZE@;

// clang-format on
#endif //__DEFAULTNETWORKCONFIG_H__
