#include "message-handler/MessageSender.h"
#include "mocks/BSPMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/HiveMindHostSerializerInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"

class MessageSenderFixture : public testing::Test {
  protected:
    MessageSender* m_messageSender;

    BSPMock* m_bspMock;

    CircularQueueInterfaceMock<MessageDTO> m_inputQueue;
    HiveMindHostSerializerInterfaceMock m_serializer;
    LoggerInterfaceMock m_logger;
    MessageDTO m_message;

    uint16_t m_uuid = 69;
    void SetUp() override {
        m_bspMock = new BSPMock(m_uuid);
        m_messageSender = new MessageSender(m_inputQueue, m_serializer, *m_bspMock, m_logger);
    }

    void TearDown() override {
        delete m_messageSender;
        delete m_bspMock;
    }
};

TEST_F(MessageSenderFixture, MessageSender_processAndSerialize_validMessage) {
    // Given
    EXPECT_CALL(m_inputQueue, peek).Times(1).WillOnce(testing::Return(m_message));
    EXPECT_CALL(m_inputQueue, pop).Times(1);
    EXPECT_CALL(m_serializer, serializeToStream(testing::_))
        .Times(1)
        .WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageSender->processAndSerialize();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageSenderFixture, MessageSender_processAndSerialize_emptyMessage) {
    // Given
    const std::optional<std::reference_wrapper<const MessageDTO>> emptyMessage = {};
    EXPECT_CALL(m_inputQueue, peek).Times(1).WillOnce(testing::Return(emptyMessage));
    EXPECT_CALL(m_inputQueue, pop).Times(0);
    EXPECT_CALL(m_serializer, serializeToStream(testing::_)).Times(0);

    // Then
    bool ret = m_messageSender->processAndSerialize();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageSenderFixture, MessageSender_processAndSerialize_invalidDeserialization) {
    // Given
    EXPECT_CALL(m_inputQueue, peek).Times(1).WillOnce(testing::Return(m_message));
    EXPECT_CALL(m_inputQueue, pop).Times(1);
    EXPECT_CALL(m_serializer, serializeToStream(testing::_))
        .Times(1)
        .WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageSender->processAndSerialize();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageSenderFixture, MessageSender_greet_validSerialization) {
    // Given
    EXPECT_CALL(m_inputQueue, peek).Times(0);
    EXPECT_CALL(m_inputQueue, pop).Times(0);
    EXPECT_CALL(m_serializer, serializeToStream(testing::_))
        .Times(1)
        .WillOnce(testing::Return(true));
    EXPECT_CALL(*m_bspMock, getUUID).Times(1).WillOnce(testing::Return(m_uuid));

    // Then
    bool ret = m_messageSender->greet();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageSenderFixture, MessageSender_greet_invalidSerialization) {
    // Given
    EXPECT_CALL(m_inputQueue, peek).Times(0);
    EXPECT_CALL(m_inputQueue, pop).Times(0);
    EXPECT_CALL(m_serializer, serializeToStream(testing::_))
        .Times(1)
        .WillOnce(testing::Return(false));
    EXPECT_CALL(*m_bspMock, getUUID).Times(1).WillOnce(testing::Return(m_uuid));
    // Then
    bool ret = m_messageSender->greet();

    // Expect
    EXPECT_FALSE(ret);
}
