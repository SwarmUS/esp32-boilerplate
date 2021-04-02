
#include "DTOMatchers.h"
#include "message-handler/MessageDispatcher.h"
#include "mocks/BSPMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include "mocks/NetworkAPIHandlerMock.h"

class MessageDispatcherFixture : public testing::Test {
  protected:
    MessageDispatcher* m_messageDispatcher;

    CircularQueueInterfaceMock<MessageDTO> m_hivemindQueue;
    CircularQueueInterfaceMock<MessageDTO> m_broadcastQueue;
    CircularQueueInterfaceMock<MessageDTO> m_unicastQueue;
    HiveMindHostDeserializerInterfaceMock m_deserializer;
    NetworkAPIHandlerMock m_handler;
    BSPMock* m_bsp;
    LoggerInterfaceMock m_logger;
    MessageDTO m_message;

    IPDiscoveryDTO* m_ipDiscovery;
    GreetingDTO* m_greeting;

    uint16_t m_uuid = 69;
    const uint16_t m_remoteUUID = 42;
    const uint16_t m_remoteIP = 12345;

    void SetUp() override {
        m_bsp = new BSPMock();

        m_ipDiscovery = new IPDiscoveryDTO(m_remoteIP);
        m_greeting = new GreetingDTO(m_uuid);

        m_messageDispatcher =
            new MessageDispatcher(m_hivemindQueue, m_unicastQueue, m_broadcastQueue, m_deserializer,
                                  m_handler, *m_bsp, m_logger);
    }

    void TearDown() override {
        delete m_bsp;
        delete m_ipDiscovery;
        delete m_greeting;
        delete m_messageDispatcher;
    }
};

TEST_F(MessageDispatcherFixture, MessageDispatcherFixture_deserializeAndDispatch_validGreeting) {
    // Given
    m_message = MessageDTO(m_uuid, m_uuid, m_uuid);
    EXPECT_CALL(m_deserializer, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));

    EXPECT_CALL(*m_bsp, setHiveMindUUID(m_greeting->getId())).Times(1);

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcherFixture_deserializeAndDispatch_failDeserialization) {
    // Given
    m_message = MessageDTO(m_uuid, m_uuid, m_uuid);
    EXPECT_CALL(m_deserializer, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(false)));

    EXPECT_CALL(*m_bsp, setHiveMindUUID(testing::_)).Times(0);

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture, MessageDispatcherFixture_deserializeAndDispatch_validIpDisovery) {
    // Given
    const NetworkApiDTO apiCall(*m_ipDiscovery);
    m_message = MessageDTO(m_uuid, m_uuid, apiCall);
    EXPECT_CALL(m_deserializer, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));

    EXPECT_CALL(m_handler, handleApiCall(testing::_, testing::_))
        .With(testing::AllOf(NetworkApiDTOMatcher(apiCall)))
        .WillOnce(testing::Return(
            std::optional<NetworkApiDTO>({}))); // Behavior could change in the future.

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}