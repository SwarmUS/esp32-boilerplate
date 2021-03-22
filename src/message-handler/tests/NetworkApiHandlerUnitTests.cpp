#include "message-handler/NetworkAPIHandler.h"
#include "mocks/BSPMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include "gtest/gtest.h"

class NetworkAPIHandlerUnitTests : public testing::Test {
  protected:
    BSPMock* m_bsp;
    LoggerInterfaceMock* m_logger;
    NetworkAPIHandler* m_handler;

    uint16_t m_boardID = 0;
    int m_logCallCounter;
    std::string m_logLastFormat;

    void SetUp() override {
        m_bsp = new BSPMock();
        m_logger = new LoggerInterfaceMock();
        m_handler = new NetworkAPIHandler(*m_bsp, *m_logger);
    }

    void TearDown() override {
        delete m_logger;
        delete m_bsp;
        delete m_handler;
    }
};

TEST_F(NetworkAPIHandlerUnitTests, NetworkAPIHandler_handleCall_valid) {
    // Given
    IPDiscoveryDTO ipDiscoveryDto;
    NetworkApiDTO apiCall(ipDiscoveryDto);

    // Then
    auto ret = m_handler->handleApiCall(apiCall);

    // Expect
    // Todo: once feature is properly implemented, add more substantial check on Id dis
    EXPECT_TRUE(ret.has_value());
}

TEST_F(NetworkAPIHandlerUnitTests, NetworkAPIHandler_handleCall_empty) {
    // Given
    NetworkApiDTO apiCall;

    // Then
    auto ret = m_handler->handleApiCall(apiCall);

    // Expect
    EXPECT_FALSE(ret.has_value());
}