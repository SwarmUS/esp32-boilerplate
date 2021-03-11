#ifndef HIVE_CONNECT_INETWORKSERIALIZER_H
#define HIVE_CONNECT_INETWORKSERIALIZER_H

#include <common/IProtobufStream.h>

class INetworkSerializer : public IProtobufStream {
  public:
    virtual ~INetworkSerializer() = default;

    /**
     *@brief Receives data from the remote target
     *
     *@param [out] data buffer for the reception of the data
     *
     *@param [in] length maximum size of the data buffer
     *
     *@return true if the operation was successful, false if not
     **/
    virtual bool receive(uint8_t* data, uint16_t length) = 0;

    /**
     *@brief Sends data to the remote target
     *
     *@param [in] data buffer  to send to the remote server
     *
     *@param [in] length data size of the data buffer
     *
     *@return true if the operation was successful, false if not
     **/
    virtual bool send(const uint8_t* data, uint16_t length) = 0;

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

#endif // HIVE_CONNECT_INETWORKSERIALIZER_H
