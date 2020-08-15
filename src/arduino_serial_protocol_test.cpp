#include "gtest/gtest.h"

#include "arduino_serial_protocol.h"
#include "arduino_serial_protocol_string.h"

#include <memory.h>
#include <ostream>
#include <vector>


#define CHECK_EQ(_expect, _got) \
if (_expect != _got) \
return testing::AssertionFailure() \
<< #_got \
<< " expected to be " << _expect \
<< ", got " << _got << " line:" << __LINE__


std::ostream& operator<<(std::ostream& os, ArduinoSerialGeneralResult result)
{
    os << StringInfo<ArduinoSerialGeneralResult>::toString(result);
    return os;
}

std::ostream& operator<<(std::ostream& os, ArduinoSerialOperation operation)
{
    os << StringInfo<ArduinoSerialOperation>::toString(operation);
    return os;
}

std::ostream& operator<<(std::ostream& os, ArduinoSerialReadResult result)
{
    os << StringInfo<ArduinoSerialReadResult>::toString(result);
    return os;
}


const uint8_t SYNC_STROBE[] = {0xD3, 0x74, 0xE5, 0x52};


class FArduinoSerialProtocol : public ::testing::Test
{
public:
    FArduinoSerialProtocol()
    {}

    void SetUp() override
    {
        protocol.reset(new ArduinoSerialProtocol{ArduinoSerialProtocol::createSecondary()});
    }

    void TearDown() override
    {
        protocol.reset();
    }

    testing::AssertionResult syncSecondary()
    {
        auto operation = protocol->nextOperation();
        CHECK_EQ(ArduinoSerialOperation::READ_HEADER, operation.read_operation);
        CHECK_EQ(1, operation.bytes_to_read);
        CHECK_EQ(0, operation.id);

        auto read_result = protocol->readBytes(SYNC_STROBE, 1);
        CHECK_EQ(ArduinoSerialReadResult::OK, read_result.read_result);
        CHECK_EQ(1, read_result.bytes_read);

        auto operation2 = protocol->nextOperation();
        CHECK_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
        CHECK_EQ(1, operation2.bytes_to_read);
        CHECK_EQ(0, operation2.id);

        auto read_result2 = protocol->readBytes(&SYNC_STROBE[1], 1);
        CHECK_EQ(ArduinoSerialReadResult::OK, read_result2.read_result);
        CHECK_EQ(1, read_result2.bytes_read);

        auto operation3 = protocol->nextOperation();
        CHECK_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
        CHECK_EQ(1, operation3.bytes_to_read);
        CHECK_EQ(0, operation3.id);

        auto read_result3 = protocol->readBytes(&SYNC_STROBE[2], 1);
        CHECK_EQ(ArduinoSerialReadResult::OK, read_result3.read_result);
        CHECK_EQ(1, read_result3.bytes_read);

        auto operation4 = protocol->nextOperation();
        CHECK_EQ(ArduinoSerialOperation::READ_HEADER, operation4.read_operation);
        CHECK_EQ(1, operation4.bytes_to_read);
        CHECK_EQ(0, operation4.id);

        auto read_result4 = protocol->readBytes(&SYNC_STROBE[3], 1);
        CHECK_EQ(ArduinoSerialReadResult::OK, read_result4.read_result);
        CHECK_EQ(1, read_result4.bytes_read);

        auto operation5 = protocol->nextOperation();
        CHECK_EQ(ArduinoSerialOperation::SEND_SYNC_REPLY, operation5.read_operation);
        CHECK_EQ(0, operation5.bytes_to_read);
        CHECK_EQ(0, operation5.id);

        auto sync_reply = std::vector<uint8_t>(protocol->syncReplyHeaderSize(), 0);
        auto sync_result = protocol->writeSyncReplyHeader(sync_reply.data());
        CHECK_EQ(ArduinoSerialGeneralResult::OK, sync_result);
        CHECK_EQ(0xD3, sync_reply.at(0));
        CHECK_EQ(0x74, sync_reply.at(1));
        CHECK_EQ(0xE5, sync_reply.at(2));
        CHECK_EQ(0x25, sync_reply.at(3));

        auto sync_result2 = protocol->syncReplySent();
        CHECK_EQ(ArduinoSerialGeneralResult::OK, sync_result2);

        auto operation6 = protocol->nextOperation();
        CHECK_EQ(ArduinoSerialOperation::READ_HEADER, operation6.read_operation);
        CHECK_EQ(1, operation6.bytes_to_read);
        CHECK_EQ(0, operation6.id);

        return testing::AssertionSuccess();
    }


protected:
    std::unique_ptr<ArduinoSerialProtocol> protocol;

};


TEST(ArduinoSerialProtocol, Basic)
{
    auto protocol = ArduinoSerialProtocol::createSecondary();

    EXPECT_EQ(8, protocol.headerSize());
    EXPECT_EQ(4, protocol.syncHeaderSize());
    EXPECT_EQ(4, protocol.syncReplyHeaderSize());

    EXPECT_EQ(8, protocol.packetSize(0));
    EXPECT_EQ(9, protocol.packetSize(1));
    EXPECT_EQ(16, protocol.packetSize(8));
}

TEST(ArduinoSerialProtocol, IDs)
{
    auto protocol = ArduinoSerialProtocol::createSecondary();

    EXPECT_EQ(1, protocol.createNextPacketId());
    EXPECT_EQ(2, protocol.createNextPacketId());
    EXPECT_EQ(3, protocol.createNextPacketId());
}

TEST_F(FArduinoSerialProtocol, Sync)
{
    EXPECT_TRUE(syncSecondary());
}

TEST_F(FArduinoSerialProtocol, Sync2)
{
    EXPECT_TRUE(syncSecondary());
    EXPECT_TRUE(syncSecondary());
}

TEST_F(FArduinoSerialProtocol, NotSynced)
{
    auto operation = protocol->nextOperation();
    EXPECT_EQ(ArduinoSerialOperation::READ_HEADER, operation.read_operation);
    EXPECT_EQ(1, operation.bytes_to_read);
    EXPECT_EQ(0, operation.id);

    auto header = std::vector<uint8_t>(protocol->syncReplyHeaderSize(), 0);
    auto payload = std::vector<uint8_t>(2, 0);
    auto id = protocol->createNextPacketId();
    auto header_result = protocol->writeHeader(header.data(), id,
            payload.data(), 2);
    EXPECT_EQ(ArduinoSerialGeneralResult::ERROR_NOT_SYNCED, header_result);

    auto sync_result = protocol->syncReplySent();
    EXPECT_EQ(ArduinoSerialGeneralResult::ERROR_WRONG_STATE, sync_result);

    uint8_t packet_strobe = 0xA5;
    auto read_result = protocol->readBytes(&packet_strobe, 1);
    EXPECT_EQ(ArduinoSerialReadResult::ERROR_UNEXPECTED_DATA, read_result.read_result);
    EXPECT_EQ(1, read_result.bytes_read);
}

TEST_F(FArduinoSerialProtocol, CreatePacket)
{
    ASSERT_TRUE(syncSecondary());

    auto data = std::vector<uint8_t>(protocol->packetSize(2), 0);
    auto header_result = protocol->writeHeader(data.data(), 1,
            data.data() + protocol->headerSize(), 2);
    EXPECT_EQ(ArduinoSerialGeneralResult::OK, header_result);
    EXPECT_EQ(0xA5, data.at(0));
    EXPECT_EQ(0x63, data.at(1));
    EXPECT_EQ(0x00, data.at(2));
    EXPECT_EQ(0x01, data.at(3));
    EXPECT_EQ(0x02, data.at(4));
    EXPECT_EQ(0x1B, data.at(5));
    EXPECT_EQ(0xFA, data.at(6));
    EXPECT_EQ(0xBB, data.at(7));
    EXPECT_EQ(0x00, data.at(8));
    EXPECT_EQ(0x00, data.at(9));
}

TEST_F(FArduinoSerialProtocol, CreatePacket2)
{
    ASSERT_TRUE(syncSecondary());

    auto data = std::vector<uint8_t>(protocol->packetSize(4), 0);
    auto* packet = data.data() + protocol->headerSize();
    packet[0] = 0x0A;
    packet[1] = 0x2B;
    packet[2] = 0x30;
    packet[3] = 0x45;
    auto header_result = protocol->writeHeader(data.data(), 1,
            data.data() + protocol->headerSize(), 4);
    EXPECT_EQ(ArduinoSerialGeneralResult::OK, header_result);
    EXPECT_EQ(0xA5, data.at(0));
    EXPECT_EQ(0x63, data.at(1));
    EXPECT_EQ(0x00, data.at(2));
    EXPECT_EQ(0x01, data.at(3));
    EXPECT_EQ(0x04, data.at(4));
    EXPECT_EQ(0x09, data.at(5));
    EXPECT_EQ(0x24, data.at(6));
    EXPECT_EQ(0xEA, data.at(7));
    EXPECT_EQ(0x0A, data.at(8));
    EXPECT_EQ(0x2B, data.at(9));
    EXPECT_EQ(0x30, data.at(10));
    EXPECT_EQ(0x45, data.at(11));
}

TEST_F(FArduinoSerialProtocol, CreatePacketPayloadTooBig)
{
    ASSERT_TRUE(syncSecondary());

    auto data = std::vector<uint8_t>(protocol->packetSize(256), 0);
    auto header_result = protocol->writeHeader(data.data(), 1,
                                               data.data() + protocol->headerSize(), 256);
    EXPECT_EQ(ArduinoSerialGeneralResult::ERROR_PAYLOAD_SIZE_TOO_BIG, header_result);
}

TEST_F(FArduinoSerialProtocol, ReceivePacket)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x02, 0x1B, 0xFA, 0xBB,
            0x00, 0x00};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
    ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

    auto operation4 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
    ASSERT_EQ(2, operation4.bytes_to_read);
    ASSERT_EQ(1, operation4.id);

    auto result4 = protocol->readBytes(data + protocol->headerSize(), 2);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result4.read_result);
    ASSERT_EQ(2, result4.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
    ASSERT_EQ(1, operation5.bytes_to_read);
    ASSERT_EQ(0, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacket2)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
    ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

    auto operation4 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
    ASSERT_EQ(4, operation4.bytes_to_read);
    ASSERT_EQ(1, operation4.id);

    auto result4 = protocol->readBytes(data + protocol->headerSize(), 4);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result4.read_result);
    ASSERT_EQ(4, result4.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
    ASSERT_EQ(1, operation5.bytes_to_read);
    ASSERT_EQ(0, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceiveTwoPackets)
{
    ASSERT_TRUE(syncSecondary());

    {
        const uint8_t data[] = {
                0xA5, 0x63, 0x00, 0x01,
                0x02, 0x1B, 0xFA, 0xBB,
                0x00, 0x00};

        auto operation1 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
        ASSERT_EQ(1, operation1.bytes_to_read);
        ASSERT_EQ(0, operation1.id);

        auto result1 = protocol->readBytes(data, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
        ASSERT_EQ(1, result1.bytes_read);

        auto operation2 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
        ASSERT_EQ(1, operation2.bytes_to_read);
        ASSERT_EQ(0, operation2.id);

        auto result2 = protocol->readBytes(data + 1, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
        ASSERT_EQ(1, result2.bytes_read);

        auto operation3 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
        ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
        ASSERT_EQ(0, operation3.id);

        auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
        ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

        auto operation4 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
        ASSERT_EQ(2, operation4.bytes_to_read);
        ASSERT_EQ(1, operation4.id);

        auto result4 = protocol->readBytes(data + protocol->headerSize(), 2);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result4.read_result);
        ASSERT_EQ(2, result4.bytes_read);

        auto operation5 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
        ASSERT_EQ(1, operation5.bytes_to_read);
        ASSERT_EQ(0, operation5.id);
    }
    {
        const uint8_t data[] = {
                0xA5, 0x63, 0x00, 0x02,
                0x04, 0x36, 0x95, 0x7F,
                0x0A, 0x2B, 0x30, 0x45};

        auto operation1 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
        ASSERT_EQ(1, operation1.bytes_to_read);
        ASSERT_EQ(0, operation1.id);

        auto result1 = protocol->readBytes(data, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
        ASSERT_EQ(1, result1.bytes_read);

        auto operation2 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
        ASSERT_EQ(1, operation2.bytes_to_read);
        ASSERT_EQ(0, operation2.id);

        auto result2 = protocol->readBytes(data + 1, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
        ASSERT_EQ(1, result2.bytes_read);

        auto operation3 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
        ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
        ASSERT_EQ(0, operation3.id);

        auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
        ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

        auto operation4 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
        ASSERT_EQ(4, operation4.bytes_to_read);
        ASSERT_EQ(2, operation4.id);

        auto result4 = protocol->readBytes(data + protocol->headerSize(), 4);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result4.read_result);
        ASSERT_EQ(4, result4.bytes_read);

        auto operation5 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
        ASSERT_EQ(1, operation5.bytes_to_read);
        ASSERT_EQ(0, operation5.id);
    }
}

TEST_F(FArduinoSerialProtocol, ReceivePacketSyncError1)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA6, 0x63, 0xA5, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_UNEXPECTED_DATA, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_UNEXPECTED_DATA, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(1, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result4 = protocol->readBytes(data + 2, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result4.read_result);
    ASSERT_EQ(1, result4.bytes_read);

    auto operation4 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation4.read_operation);
    ASSERT_EQ(1, operation4.bytes_to_read);
    ASSERT_EQ(0, operation4.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketSyncError2)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x93, 0xA5, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_UNEXPECTED_DATA, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(1, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result4 = protocol->readBytes(data + 2, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result4.read_result);
    ASSERT_EQ(1, result4.bytes_read);

    auto operation4 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation4.read_operation);
    ASSERT_EQ(1, operation4.bytes_to_read);
    ASSERT_EQ(0, operation4.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketErrorHeaderID1)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x10, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_CHECKSUM, result3.read_result);
    ASSERT_EQ(4, result3.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
    ASSERT_EQ(1, operation5.bytes_to_read);
    ASSERT_EQ(0, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketErrorHeaderID2)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x00,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_CHECKSUM, result3.read_result);
    ASSERT_EQ(4, result3.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
    ASSERT_EQ(1, operation5.bytes_to_read);
    ASSERT_EQ(0, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketErrorHeaderLength)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x12, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_CHECKSUM, result3.read_result);
    ASSERT_EQ(4, result3.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
    ASSERT_EQ(1, operation5.bytes_to_read);
    ASSERT_EQ(0, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketErrorHeaderCRC)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x12, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_CHECKSUM, result3.read_result);
    ASSERT_EQ(4, result3.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
    ASSERT_EQ(1, operation5.bytes_to_read);
    ASSERT_EQ(0, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketErrorPayloadCRC1)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x09, 0x20, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
    ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

    auto operation4 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
    ASSERT_EQ(4, operation4.bytes_to_read);
    ASSERT_EQ(1, operation4.id);

    auto result4 = protocol->readBytes(data + protocol->headerSize(), 4);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_CHECKSUM, result4.read_result);
    ASSERT_EQ(4, result4.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
    ASSERT_EQ(1, operation5.bytes_to_read);
    ASSERT_EQ(0, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketErrorPayloadCRC2)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x09, 0x24, 0xBA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
    ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

    auto operation4 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
    ASSERT_EQ(4, operation4.bytes_to_read);
    ASSERT_EQ(1, operation4.id);

    auto result4 = protocol->readBytes(data + protocol->headerSize(), 4);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_CHECKSUM, result4.read_result);
    ASSERT_EQ(4, result4.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
    ASSERT_EQ(1, operation5.bytes_to_read);
    ASSERT_EQ(0, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketErrorPayload)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x3B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
    ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

    auto operation4 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
    ASSERT_EQ(4, operation4.bytes_to_read);
    ASSERT_EQ(1, operation4.id);

    auto result4 = protocol->readBytes(data + protocol->headerSize(), 4);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_CHECKSUM, result4.read_result);
    ASSERT_EQ(4, result4.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
    ASSERT_EQ(1, operation5.bytes_to_read);
    ASSERT_EQ(0, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketInsufficientLength1)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 0);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_INSUFFICIENT_DATA_LENGTH, result1.read_result);
    ASSERT_EQ(0, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketInsufficientLength2)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 0);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_INSUFFICIENT_DATA_LENGTH, result2.read_result);
    ASSERT_EQ(0, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(1, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketInsufficientLength3)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 3);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_INSUFFICIENT_DATA_LENGTH, result3.read_result);
    ASSERT_EQ(0, result3.bytes_read);

    auto operation4 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation4.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation4.bytes_to_read);
    ASSERT_EQ(0, operation4.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketInsufficientLength4)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 1);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
    ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

    auto operation4 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
    ASSERT_EQ(4, operation4.bytes_to_read);
    ASSERT_EQ(1, operation4.id);

    auto result4 = protocol->readBytes(data + protocol->headerSize(), 3);
    ASSERT_EQ(ArduinoSerialReadResult::ERROR_INSUFFICIENT_DATA_LENGTH, result4.read_result);
    ASSERT_EQ(0, result4.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation5.read_operation);
    ASSERT_EQ(4, operation5.bytes_to_read);
    ASSERT_EQ(1, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketExtensiveLength)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45,
            0x00, 0x00, 0x00, 0x00};

    auto operation1 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
    ASSERT_EQ(1, operation1.bytes_to_read);
    ASSERT_EQ(0, operation1.id);

    auto result1 = protocol->readBytes(data, 16);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
    ASSERT_EQ(1, result1.bytes_read);

    auto operation2 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
    ASSERT_EQ(1, operation2.bytes_to_read);
    ASSERT_EQ(0, operation2.id);

    auto result2 = protocol->readBytes(data + 1, 15);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
    ASSERT_EQ(1, result2.bytes_read);

    auto operation3 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
    ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
    ASSERT_EQ(0, operation3.id);

    auto result3 = protocol->readBytes(data + 2, 14);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
    ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

    auto operation4 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
    ASSERT_EQ(4, operation4.bytes_to_read);
    ASSERT_EQ(1, operation4.id);

    auto result4 = protocol->readBytes(data + protocol->headerSize(), 8);
    ASSERT_EQ(ArduinoSerialReadResult::OK, result4.read_result);
    ASSERT_EQ(4, result4.bytes_read);

    auto operation5 = protocol->nextOperation();
    ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
    ASSERT_EQ(1, operation5.bytes_to_read);
    ASSERT_EQ(0, operation5.id);
}

TEST_F(FArduinoSerialProtocol, ReceivePacketsHeaderCRCError)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x08, 0x24, 0xEA,
            0x0A, 0x2B, 0x30, 0x45,
            0xA5, 0x63, 0x00, 0x02,
            0x04, 0x36, 0x95, 0x7F,
            0x0A, 0x2B, 0x30, 0x45};

    {
        auto operation1 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
        ASSERT_EQ(1, operation1.bytes_to_read);
        ASSERT_EQ(0, operation1.id);

        auto result1 = protocol->readBytes(data, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
        ASSERT_EQ(1, result1.bytes_read);

        auto operation2 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
        ASSERT_EQ(1, operation2.bytes_to_read);
        ASSERT_EQ(0, operation2.id);

        auto result2 = protocol->readBytes(data + 1, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
        ASSERT_EQ(1, result2.bytes_read);

        auto operation3 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
        ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
        ASSERT_EQ(0, operation3.id);

        auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
        ASSERT_EQ(ArduinoSerialReadResult::ERROR_CHECKSUM, result3.read_result);
        ASSERT_EQ(4, result3.bytes_read);

        auto operation5 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
        ASSERT_EQ(1, operation5.bytes_to_read);
        ASSERT_EQ(0, operation5.id);

        for (int i = 0; i < 6; ++i)
        {
            auto result_tmp = protocol->readBytes(data + (6 + i), 1);
            ASSERT_EQ(ArduinoSerialReadResult::NOPE, result_tmp.read_result);
            ASSERT_EQ(1, result_tmp.bytes_read);

            auto operation_tmp = protocol->nextOperation();
            ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation_tmp.read_operation);
            ASSERT_EQ(1, operation_tmp.bytes_to_read);
            ASSERT_EQ(0, operation_tmp.id);
        }
    }
    {
        auto operation1 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
        ASSERT_EQ(1, operation1.bytes_to_read);
        ASSERT_EQ(0, operation1.id);

        auto result1 = protocol->readBytes(data + 12, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
        ASSERT_EQ(1, result1.bytes_read);

        auto operation2 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
        ASSERT_EQ(1, operation2.bytes_to_read);
        ASSERT_EQ(0, operation2.id);

        auto result2 = protocol->readBytes(data + 13, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
        ASSERT_EQ(1, result2.bytes_read);

        auto operation3 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
        ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
        ASSERT_EQ(0, operation3.id);

        auto result3 = protocol->readBytes(data + 14, protocol->headerSize() - 2);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
        ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

        auto operation4 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
        ASSERT_EQ(4, operation4.bytes_to_read);
        ASSERT_EQ(2, operation4.id);

        auto result4 = protocol->readBytes(data + (12 + protocol->headerSize()), 4);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result4.read_result);
        ASSERT_EQ(4, result4.bytes_read);

        auto operation5 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
        ASSERT_EQ(1, operation5.bytes_to_read);
        ASSERT_EQ(0, operation5.id);
    }
}

TEST_F(FArduinoSerialProtocol, ReceivePacketsPayloadCRCError)
{
    ASSERT_TRUE(syncSecondary());

    const uint8_t data[] = {
            0xA5, 0x63, 0x00, 0x01,
            0x04, 0x09, 0x24, 0xEA,
            0x0A, 0x2B, 0x31, 0x45,
            0xA5, 0x63, 0x00, 0x02,
            0x04, 0x36, 0x95, 0x7F,
            0x0A, 0x2B, 0x30, 0x45};

    {
        auto operation1 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
        ASSERT_EQ(1, operation1.bytes_to_read);
        ASSERT_EQ(0, operation1.id);

        auto result1 = protocol->readBytes(data, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
        ASSERT_EQ(1, result1.bytes_read);

        auto operation2 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
        ASSERT_EQ(1, operation2.bytes_to_read);
        ASSERT_EQ(0, operation2.id);

        auto result2 = protocol->readBytes(data + 1, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
        ASSERT_EQ(1, result2.bytes_read);

        auto operation3 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
        ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
        ASSERT_EQ(0, operation3.id);

        auto result3 = protocol->readBytes(data + 2, protocol->headerSize() - 2);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
        ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

        auto operation4 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
        ASSERT_EQ(4, operation4.bytes_to_read);
        ASSERT_EQ(1, operation4.id);

        auto result4 = protocol->readBytes(data + protocol->headerSize(), 4);
        ASSERT_EQ(ArduinoSerialReadResult::ERROR_CHECKSUM, result4.read_result);
        ASSERT_EQ(4, result4.bytes_read);

        auto operation5 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
        ASSERT_EQ(1, operation5.bytes_to_read);
        ASSERT_EQ(0, operation5.id);
    }
    {
        auto operation1 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation1.read_operation);
        ASSERT_EQ(1, operation1.bytes_to_read);
        ASSERT_EQ(0, operation1.id);

        auto result1 = protocol->readBytes(data + 12, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result1.read_result);
        ASSERT_EQ(1, result1.bytes_read);

        auto operation2 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation2.read_operation);
        ASSERT_EQ(1, operation2.bytes_to_read);
        ASSERT_EQ(0, operation2.id);

        auto result2 = protocol->readBytes(data + 13, 1);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result2.read_result);
        ASSERT_EQ(1, result2.bytes_read);

        auto operation3 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation3.read_operation);
        ASSERT_EQ(protocol->headerSize() - 2, operation3.bytes_to_read);
        ASSERT_EQ(0, operation3.id);

        auto result3 = protocol->readBytes(data + 14, protocol->headerSize() - 2);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result3.read_result);
        ASSERT_EQ(protocol->headerSize() - 2, result3.bytes_read);

        auto operation4 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_PAYLOAD, operation4.read_operation);
        ASSERT_EQ(4, operation4.bytes_to_read);
        ASSERT_EQ(2, operation4.id);

        auto result4 = protocol->readBytes(data + (12 + protocol->headerSize()), 4);
        ASSERT_EQ(ArduinoSerialReadResult::OK, result4.read_result);
        ASSERT_EQ(4, result4.bytes_read);

        auto operation5 = protocol->nextOperation();
        ASSERT_EQ(ArduinoSerialOperation::READ_HEADER, operation5.read_operation);
        ASSERT_EQ(1, operation5.bytes_to_read);
        ASSERT_EQ(0, operation5.id);
    }
}
