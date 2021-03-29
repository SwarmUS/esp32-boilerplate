#include "NetworkInputStream.h"
#include "NetworkOutputStream.h"
#include "mocks/LoggerInterfaceMock.h"
#include "mocks/NetworkManagerMock.h"
#include <gtest/gtest.h>

class NetworkUnicastStreamFixture : public testing::Test {
  public:
    void SetUp() override {
        m_networkManagerMock = new NetworkManagerMock();
        m_inputStream = new NetworkInputStream(m_logger, 9000);
        m_outputStream = new NetworkOutputStream(m_logger);
    }

    void TearDown() override {
        delete m_outputStream;
        delete m_inputStream;
    }

  protected:
    LoggerInterfaceMock m_logger;
    NetworkManagerMock* m_networkManagerMock;
    NetworkInputStream* m_inputStream;
    NetworkOutputStream* m_outputStream;
};

TEST_F(NetworkUnicastStreamFixture, test_DataExchange_working) {
    // Can listen for inbound connection
    EXPECT_TRUE(m_inputStream->start());

    // Can connect to server
    EXPECT_FALSE(m_outputStream->setDestination("1200")); // testing invalid port
    EXPECT_TRUE(m_outputStream->setDestination("9000")); // Connecting to valid port

    // Can send message
    char message[] = "This is a test message";
    EXPECT_TRUE(m_outputStream->send((uint8_t*)message, sizeof(message)));

    // Can receive message
    char letter = 0;
    for (int i = 0; i < strlen(message); i++) {
        EXPECT_TRUE(m_inputStream->receive((uint8_t*)&letter, sizeof(letter)));
        EXPECT_EQ(letter, message[i]);
    }
}