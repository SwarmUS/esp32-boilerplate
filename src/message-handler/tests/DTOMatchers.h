#ifndef HIVE_CONNECT_DTOMATCHERS_H
#define HIVE_CONNECT_DTOMATCHERS_H

#include "hivemind-host/MessageDTO.h"
#include "gmock/gmock.h"

// Simple matcher to check that DTO matches inside function call. If required
MATCHER_P(NetworkApiDTOMatcher, expected, "Equality matcher for NetworkApiDTO)") {
    auto value = std::get<0>(arg).getApiCall();
    auto ref = expected.getApiCall();

    // Check if both contains std::monostate
    if (std::holds_alternative<std::monostate>(value) &&
        std::holds_alternative<std::monostate>(ref)) {
        return true;
    }

    // When both contain an IpDiscoveryDTO, check that IPs match
    if (auto* valIP = std::get_if<IPDiscoveryDTO>(&value)) {
        if (auto* expectedIP = std::get_if<IPDiscoveryDTO>(&ref)) {
            return valIP->getIP() == expectedIP->getIP();
        }
    }

    // Return false otherwise
    return false;
}

#endif // HIVE_CONNECT_DTOMATCHERS_H
