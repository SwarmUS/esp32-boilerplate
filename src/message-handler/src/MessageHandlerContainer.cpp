#include "MessageHandlerContainer.h"
#include "ConditionVariable.h"
#include "NetworkContainer.h"
#include "ThreadSafeQueue.h"
#include "bsp/Container.h"
#include "cpp-common/CircularQueue.h"
#include "cpp-common/CircularQueueStack.h"
#include "logger/LoggerContainer.h"

// Could be raised in the future if needed
constexpr uint16_t gc_queueMaxSize = 8;

NetworkAPIHandler MessageHandlerContainer::createNetworkApiHandler() {
    return NetworkAPIHandler(BspContainer::getBSP(), LoggerContainer::getLogger(),
                             NetworkContainer::getNetworkManager());
}

MessageDispatcher MessageHandlerContainer::createMessageDispatcher(
    IHiveMindHostDeserializer& deserializer, INetworkAPIHandler& networkApiHandler) {
    return MessageDispatcher(getHivemindOutputQueue(), getUnicastOutputQueue(),
                             getBroadcastOutputQueue(), deserializer, networkApiHandler,
                             BspContainer::getBSP(), LoggerContainer::getLogger(),
                             NetworkContainer::getNetworkManager());
}

NotificationQueue<MessageDTO>& MessageHandlerContainer::getHivemindOutputQueue() {
    static Mutex s_mutex(10);
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_hivemindMsgQueue;
    static ConditionVariable s_hivemindConditionVar;
    static ThreadSafeQueue<MessageDTO> s_hivemindMsgQueueThreadSafe(s_hivemindMsgQueue, s_mutex);
    static NotificationQueue s_hivemindNotificationQueue(s_hivemindMsgQueue,
                                                         s_hivemindConditionVar);

    return s_hivemindNotificationQueue;
}

NotificationQueue<MessageDTO>& MessageHandlerContainer::getUnicastOutputQueue() {
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_unicastMsgQueue;
    static Mutex s_mutex(10);
    static ThreadSafeQueue<MessageDTO> s_unicastMsgQueueThreadSafe(s_unicastMsgQueue, s_mutex);
    static ConditionVariable s_unicastConditionVar;
    static NotificationQueue<MessageDTO> s_unicastNotificationQueue(s_unicastMsgQueueThreadSafe,
                                                                    s_unicastConditionVar);

    return s_unicastNotificationQueue;
}

NotificationQueue<MessageDTO>& MessageHandlerContainer::getBroadcastOutputQueue() {
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_broadcastMsgQueue;
    static Mutex s_mutex(10);
    static ThreadSafeQueue<MessageDTO> s_broadcastMsgQueueThreadSafe(s_broadcastMsgQueue, s_mutex);
    static ConditionVariable s_broadcastConditionVar;
    static NotificationQueue<MessageDTO> s_broadcastNotificationQueue(s_broadcastMsgQueueThreadSafe,
                                                                      s_broadcastConditionVar);

    return s_broadcastNotificationQueue;
}