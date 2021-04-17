#ifndef HIVE_CONNECT_MESSAGEHANDLERCONTAINER_H
#define HIVE_CONNECT_MESSAGEHANDLERCONTAINER_H

#include "MessageDispatcher.h"
#include "NetworkAPIHandler.h"
#include "NotificationQueue.h"

namespace MessageHandlerContainer {

/**
 * @brief create NetworkAPIHandler instance
 * @return A new NetworkApiHandler instance
 */
NetworkAPIHandler createNetworkApiHandler();

/**
 *@brief create a message dispatcher
 *@return A new message dispatcher */
MessageDispatcher createMessageDispatcher(IHiveMindHostDeserializer& deserializer,
                                          INetworkAPIHandler& networkApiHandler);

/**
 * @brief get the Hivemind output queue
 * @return a reference to the Hivemind output queue
 */
NotificationQueue<MessageDTO>& getHivemindOutputQueue();

/**
 * @brief get the network unicast output queue
 * @return a reference to the Hivemind output queue
 */
NotificationQueue<MessageDTO>& getUnicastOutputQueue();

/**
 * @brief get the network broadcast output queue
 * @return a reference to the Hivemind output queue
 */
NotificationQueue<MessageDTO>& getBroadcastOutputQueue();
} // namespace MessageHandlerContainer

#endif // HIVE_CONNECT_MESSAGEHANDLERCONTAINER_H
