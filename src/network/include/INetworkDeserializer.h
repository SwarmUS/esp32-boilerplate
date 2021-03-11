#ifndef HIVE_CONNECT_INETWORKDESERIALIZER_H
#define HIVE_CONNECT_INETWORKDESERIALIZER_H

#include <common/IProtobufStream.h>

class INetworkDeserializer : public IProtobufStream {
  public:
    virtual ~INetworkDeserializer() = default;

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
     *@brief Closes the stream
     *
     *@return true if the deserializer is ready to send/receive bytes
     *closed)
     **/
    virtual bool isReady() = 0;
};

#endif // HIVE_CONNECT_INETWORKDESERIALIZER_H
