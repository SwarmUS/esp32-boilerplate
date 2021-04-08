#include "NetworkInputStream.h"
#include "NetworkOutputStream.h"
#include "message-handler/NetworkDeserializer.h"
#include "message-handler/NetworkSerializer.h"
#include "mocks/LoggerInterfaceMock.h"
#include "mocks/NetworkManagerMock.h"
#include "pheromones/HiveMindHostDeserializer.h"
#include <gtest/gtest.h>

class NetworkSerializerAndDeserializerIntegrationTestsFixture : public testing::Test {
  protected:
    NetworkSerializer* m_serializer;
    NetworkDeserializer* m_deserializer;
    INetworkInputStream* m_inputStream;
    NetworkOutputStream* m_outputStream;
    LoggerInterfaceMock m_logger;
    NetworkManagerMock m_networkManager;

    void SetUp() override {
        m_inputStream = new NetworkInputStream(m_logger, 9000);
        m_outputStream = new NetworkOutputStream(m_logger);

        m_serializer = new NetworkSerializer(*m_outputStream, m_networkManager);
        m_deserializer = new NetworkDeserializer(*m_inputStream);
    }

    void TearDown() override {
        delete m_deserializer;
        delete m_serializer;
        delete m_outputStream;
        delete m_inputStream;
    }
};

TEST_F(NetworkSerializerAndDeserializerIntegrationTestsFixture, IntegrationTests) {
    GreetingDTO greeting(42);
    MessageDTO message(1, 2, greeting);

    m_inputStream->start();

    EXPECT_CALL(m_networkManager, getIPFromAgentID(2))
        .WillOnce(testing::Return(std::optional<uint32_t>(9000)));

    ASSERT_TRUE(m_serializer->serializeToStream(message));

    EXPECT_CALL(m_networkManager, getIPFromAgentID(2))
        .WillOnce(testing::Return(std::optional<uint32_t>(9000)));
    ASSERT_TRUE(m_serializer->serializeToStream(message));

    MessageDTO messageReceived;
    ASSERT_TRUE(m_deserializer->deserializeFromStream(messageReceived));
    ASSERT_TRUE(std::holds_alternative<GreetingDTO>(messageReceived.getMessage()));

    MessageDTO messageReceived2;
    ASSERT_TRUE(m_deserializer->deserializeFromStream(messageReceived2));
    ASSERT_TRUE(std::holds_alternative<GreetingDTO>(messageReceived2.getMessage()));
}