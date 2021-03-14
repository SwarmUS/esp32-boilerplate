#ifndef HIVE_CONNECT_INETWORKINPUTSTREAM_H
#define HIVE_CONNECT_INETWORKINPUTSTREAM_H

#include <common/IProtobufInputStream.h>

class INetworkInputStream : public IProtobufInputStream {
  public:
    ~INetworkInputStream() override = default;

    /**
     *@brief Check if the stream is ready
     *
     *@return true if the deserializer is ready to send/receive bytes
     *)
     **/
    virtual bool isReady() = 0;

    /**
     * @brief Start the reception stream
     *
     * @return true if successfully started
     */
    virtual bool start() = 0;

    /**
     * @brief Stop the reception stream
     *
     * @return true if successfully stopped
     */
    virtual bool stop() = 0;
};

#endif // HIVE_CONNECT_INETWORKINPUTSTREAM_H
