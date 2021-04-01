#ifndef HIVE_CONNECT_BSPMOCK_H
#define HIVE_CONNECT_BSPMOCK_H

#include "bsp/IBSP.h"
#include <gmock/gmock.h>

class BSPMock final : public IBSP {
  public:
    explicit BSPMock() : m_boardID(0) { setActions(); }
    explicit BSPMock(const uint16_t& boardID) : m_boardID(boardID) { setActions(); }

    MOCK_METHOD(void, initChip, (), (override));
    MOCK_METHOD(uint16_t, getHiveMindUUID, (), (override));
    MOCK_METHOD(void, setHiveMindUUID, (uint16_t), (override));
    MOCK_METHOD(ChipInfo, getChipInfo, (), (override));

    uint16_t m_boardID;

    void setActions() {
        // Set of default behavior
        ON_CALL(*this, getHiveMindUUID).WillByDefault(testing::Return(m_boardID));
        ON_CALL(*this, setHiveMindUUID).WillByDefault(testing::SaveArg<0>(&this->m_boardID));
    }
};

#endif // HIVE_CONNECT_BSPMOCK_H
