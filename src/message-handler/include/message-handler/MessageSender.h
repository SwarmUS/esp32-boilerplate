#ifndef __MESSAGESENDER_H_
#define __MESSAGESENDER_H_

#include "IMessageSender.h"
#include <INotificationQueue.h>
#include <bsp/IBSP.h>
#include <logger/ILogger.h>
#include <pheromones/HiveMindHostSerializer.h>

class MessageSender : IMessageSender {
  public:
    MessageSender(INotificationQueue<MessageDTO>& inputQueue,
                  IHiveMindHostSerializer& serializer,
                  IBSP& bsp,
                  ILogger& logger);

    ~MessageSender() override = default;

    bool greet() override;

    bool processAndSerialize() override;

  private:
    INotificationQueue<MessageDTO>& m_inputQueue;
    IHiveMindHostSerializer& m_serializer;
    IBSP& m_bsp;
    ILogger& m_logger;
};

#endif // __MESSAGESENDER_H_
