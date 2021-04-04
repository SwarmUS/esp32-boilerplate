#include "message-handler/NetworkAPIHandler.h"
#include "mocks/BSPMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include "mocks/NetworkManagerMock.h"
#include "gtest/gtest.h"

class NetworkAPIHandlerUnitTests : public testing::Test {
  protected:
    BSPMock* m_bsp;
    LoggerInterfaceMock* m_logger;
    NetworkAPIHandler* m_handler;
    NetworkManagerMock* m_networkManager;

    uint16_t m_boardID = 0;
    int m_logCallCounter;
    std::string m_logLastFormat;

    void SetUp() override {
        m_bsp = new BSPMock();
        m_logger = new LoggerInterfaceMock();
        m_networkManager = new NetworkManagerMock();
        m_handler = new NetworkAPIHandler(*m_bsp, *m_logger, *m_networkManager);
    }

    void TearDown() override {
        delete m_logger;
        delete m_bsp;
        delete m_handler;
        delete m_networkManager;
    }
};

TEST_F(NetworkAPIHandlerUnitTests, NetworkAPIHandler_handleCall_valid) {
    // Given

    IPDiscoveryDTO ipDiscoveryDto(1);
    NetworkApiDTO apiCall(ipDiscoveryDto);
    MessageDTO message(69, 0, apiCall);

    // Then
    EXPECT_CALL(*m_networkManager, registerAgent(69, 1)).WillOnce(testing::Return(true));
    auto ret = m_handler->handleApiCall(message.getSourceId(), apiCall);

    // Expect
    EXPECT_TRUE(std::holds_alternative<std::optional<NetworkApiDTO>>(ret));
    EXPECT_FALSE(reinterpret_cast<const std::optional<NetworkApiDTO>&>(ret).has_value());
}