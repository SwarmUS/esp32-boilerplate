#ifndef HIVE_CONNECT_INETWORKMANAGER_H
#define HIVE_CONNECT_INETWORKMANAGER_H

#include <string>

enum class NetworkStatus { NotConnected = 0, Connecting, Connected };

class INetworkManager {
  public:
    /**
     * @brief Starts the network manager and attempts connection
     */
    virtual void start() = 0;

    /**
     * @brief Gets the status of the network
     */
    virtual NetworkStatus getNetworkStatus() = 0;

    /**
     * @brief Returns ID of the module in the network as a char array. In the wifi network, this
     * will be an IP address and on ROS probably a topic.
     * @param [out] buffer The buffer to write the IP into
     * @param [in] maxLength The maximum length of the buffer in bytes
     * @return True if operation successful, false otherwise
     */
    virtual bool getSelfIP(char* buffer, size_t maxLength) = 0;

    /**
     * @brief Returns the IP from a agent ID
     * @param [in] agentID The target agent ID
     * @param [out] buffer The buffer to store the IP
     * @param [in] maxLength The maxLength of the buffer to write into
     * @return True if the IP was known and it could be written in the supplied buffer, false
     * otherwise.
     */
    virtual bool getIPFromAgentID(uint16_t agentID, char* buffer, size_t maxLength) const = 0;

    /**
     * @brief Add a agent to the the list of known agents
     * @param [in] agentID the id to the agent to register
     * @param [in] ip The ip of the agent to register
     * @return true if the agent was added or already present, false if the agent could not be
     * registered
     */
    virtual bool registerAgent(uint16_t agentID, const char* ip) = 0;
};

#endif // HIVE_CONNECT_INETWORKMANAGER_H
