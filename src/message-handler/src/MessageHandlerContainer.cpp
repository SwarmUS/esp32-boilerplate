#include "MessageHandlerContainer.h"
#include "NetworkContainer.h"
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
                             BspContainer::getBSP(), LoggerContainer::getLogger());
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getHivemindOutputQueue() {
    static Mutex s_mutex(10);
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_hivemindMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_hivemindMsgQueueThreadSafe(s_hivemindMsgQueue, s_mutex);

    return s_hivemindMsgQueueThreadSafe;
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getUnicastOutputQueue() {
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_unicastMsgQueue;
    static Mutex s_mutex(10);
    static ThreadSafeQueue<MessageDTO> s_unicastMsgQueueThreadSafe(s_unicastMsgQueue, s_mutex);

    return s_unicastMsgQueueThreadSafe;
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getBroadcastOutputQueue() {
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_broadcastMsgQueue;
    static Mutex s_mutex(10);
    static ThreadSafeQueue<MessageDTO> s_broadcastMsgQueueThreadSafe(s_broadcastMsgQueue, s_mutex);

    return s_broadcastMsgQueueThreadSafe;
}