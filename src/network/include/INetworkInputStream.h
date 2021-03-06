#ifndef HIVE_CONNECT_INETWORKINPUTSTREAM_H
#define HIVE_CONNECT_INETWORKINPUTSTREAM_H

#include <pheromones/IProtobufInputStream.h>

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

    /**
     * @brief Close the currently set client. To be called only when deserialization has concluded.
     */
    virtual void closeCurrentClient() = 0;
};

#endif // HIVE_CONNECT_INETWORKINPUTSTREAM_H
