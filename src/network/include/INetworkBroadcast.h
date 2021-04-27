#ifndef HIVE_CONNECT_INETWORKBROADCAST_H
#define HIVE_CONNECT_INETWORKBROADCAST_H

#include "pheromones/IProtobufStream.h"

class INetworkBroadcast : public IProtobufStream {
  public:
    ~INetworkBroadcast() override = default;

    /**
     *@brief Used to read the data received in broadcast instance
     *@param [out] data buffer where the received data will be stored
     *@param [in] length maxmimum length of the data to store in the buffer
     *@return true if the operation was successful, false if not
     **/
    bool receive(uint8_t* data, uint16_t length) override = 0;

    /**
     *@brief Used to send the serialized data when fully serialized.
     *@param [in] data buffer the data to send is stored
     *@param [in] length size of the data in the buffer
     *@return true if the operation was successful, false if not
     **/
    bool send(const uint8_t* data, uint16_t length) override = 0;

    /**
     * @brief Start the broadcasting instance (create sockets and such)
     * @return true if successful, false otherwise
     */
    virtual bool start() = 0;

    /**
     * @brief Stop the broadcasting instance (close sockets and such)
     * @return true if successful, false otherwise
     */
    virtual bool stop() = 0;

    /**
     * @brief Returns the if the service was started or not
     * @return true if started, false otherwise
     */
    virtual bool isStarted() const = 0;
};
#endif // HIVE_CONNECT_INETWORKBROADCAST_H
