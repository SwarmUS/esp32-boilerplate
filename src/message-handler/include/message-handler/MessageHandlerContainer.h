#ifndef HIVE_CONNECT_MESSAGEHANDLERCONTAINER_H
#define HIVE_CONNECT_MESSAGEHANDLERCONTAINER_H

#include "MessageDispatcher.h"
#include "NetworkAPIHandler.h"
#include "ThreadSafeQueue.h"

namespace MessageHandlerContainer {

/**
 * @brief get the networkAPIHandler instance
 * @return the instance of the networkApiHandler
 */
NetworkAPIHandler getNetworkApiHandler();

/**
 *@brief create a message dispatcher
 *@return A new message dispatcher */
MessageDispatcher createMessageDispatcher(IHiveMindHostDeserializer& deserializer,
                                          INetworkAPIHandler& networkApiHandler);

/**
 * @brief get the Hivemind output queue
 * @return a reference to the Hivemind output queue
 */
ThreadSafeQueue<MessageDTO>& getHivemindOutputQueue();

/**
 * @brief get the network unicast output queue
 * @return a reference to the Hivemind output queue
 */
ThreadSafeQueue<MessageDTO>& getUnicastOutputQueue();

/**
 * @brief get the network broadcast output queue
 * @return a reference to the Hivemind output queue
 */
ThreadSafeQueue<MessageDTO>& getBroadcastOutputQueue();
} // namespace MessageHandlerContainer

#endif // HIVE_CONNECT_MESSAGEHANDLERCONTAINER_H
