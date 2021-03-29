#include "NetworkInputStream.h"
#include "NetworkOutputStream.h"
#include "mocks/LoggerInterfaceMock.h"
#include "mocks/NetworkManagerMock.h"
#include <gtest/gtest.h>

class NetworkUnicastStreamFixture : public testing::Test {
  public:
    void SetUp() override {
        m_networkManagerMock = new NetworkManagerMock();
        m_inputStream = new NetworkInputStream(m_logger, 9001);
        m_outputStream = new NetworkOutputStream(m_logger);
    }

    void TearDown() override {
        /*delete m_networkManagerMock;
        delete m_outputStream;
        delete m_inputStream;*/
    }

  protected:
    NetworkManagerMock* m_networkManagerMock;
    NetworkInputStream* m_inputStream;
    NetworkOutputStream* m_outputStream;
    LoggerInterfaceMock m_logger;
};

TEST_F(NetworkUnicastStreamFixture, test_Server_start_and_stop_success) {
    EXPECT_TRUE(m_inputStream->start());
    // Reset
    EXPECT_TRUE(m_inputStream->stop());
}

TEST_F(NetworkUnicastStreamFixture, test_Client_connect_success) {
    // Given
    EXPECT_TRUE(m_inputStream->start());

    // Expect
    EXPECT_TRUE(m_outputStream->setDestination("9001"));

    // Reset
    EXPECT_TRUE(m_inputStream->stop());
}

TEST_F(NetworkUnicastStreamFixture, test_Client_connect_failure) {
    // Given
    EXPECT_TRUE(m_inputStream->stop());

    // Expect
    EXPECT_FALSE(m_outputStream->setDestination("9001"));

    // Reset
    EXPECT_TRUE(m_inputStream->stop());
    EXPECT_TRUE(m_outputStream->close());
}

TEST_F(NetworkUnicastStreamFixture, test_Client_transmit_succes) {
    // Given
    EXPECT_TRUE(m_inputStream->start());
    EXPECT_TRUE(m_outputStream->setDestination("9001"));

    // When
    char message[] = "Test message";

    // Expect
    EXPECT_TRUE(m_outputStream->send((uint8_t*)message, sizeof(message)));
    EXPECT_TRUE(m_outputStream->close());

    // Reset
    EXPECT_TRUE(m_inputStream->stop());
}

TEST_F(NetworkUnicastStreamFixture, test_Client_transmit_and_receive_succes) {
    // Given
    EXPECT_TRUE(m_inputStream->start());
    EXPECT_TRUE(m_outputStream->setDestination("9001"));

    // When
    char message[] = "Test message";
    EXPECT_TRUE(m_outputStream->send((uint8_t*)message, sizeof(message)));

    char letter = 0;
    for (int i = 0; i <= sizeof(message); i++) {
        EXPECT_TRUE(m_inputStream->receive((uint8_t*)&letter, 1));
        EXPECT_EQ(letter, message[i]);
    }

    // Reset
    EXPECT_TRUE(m_inputStream->stop());
}