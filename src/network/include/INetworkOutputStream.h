#ifndef HIVE_CONNECT_INETWORKOUTPUTSTREAM_H
#define HIVE_CONNECT_INETWORKOUTPUTSTREAM_H

#include <common/IProtobufOutputStream.h>

class INetworkOutputStream : public IProtobufOutputStream {
  public:
    ~INetworkOutputStream() override = default;

    /**
     * @brief Set the destination to which the stream need to be sent
     *
     * @param [in] destination Target of the message as string. Can IP address or ROS topic
     *
     * @return true if target reachable, false if not
     */
    virtual bool setDestination(const char* destination) = 0;

    /**
     *@brief Closes the stream
     *
     *@return true if the operation was successful, false if not (i.e. the socket was already
     *closed)
     **/
    virtual bool close() = 0;
};

#endif // HIVE_CONNECT_INETWORKOUTPUTSTREAM_H
