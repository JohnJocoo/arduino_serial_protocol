#include "arduino_serial_protocol.h"


constexpr const uint8_t STROBE_1 = 0xA5;
constexpr const uint8_t STROBE_2 = 0x63;

constexpr const uint8_t SYNC_STROBE_1 = 0xD3;
constexpr const uint8_t SYNC_STROBE_2 = 0x74;
constexpr const uint8_t SYNC_STROBE_3 = 0xE5;
constexpr const uint8_t SYNC_STROBE_4 = 0x52;


enum class State : char
{
    WAITING_SYNC,
    IDLE,
    READ_STROBE_2,
    READ_STROBE_3,
    READ_STROBE_4,
    READ_HEADER,
    WAIT_READ_PAYLOAD
};


ArduinoSerialProtocol::ArduinoSerialProtocol(bool is_primary)
: is_primary{is_primary}
, is_synced{false}
, state{State::WAITING_SYNC}
{}

ArduinoSerialProtocol ArduinoSerialProtocol::createPrimary()
{
    return ArduinoSerialProtocol{true};
}

ArduinoSerialProtocol ArduinoSerialProtocol::createSecondary()
{
    return ArduinoSerialProtocol{false};
}

uint16_t ArduinoSerialProtocol::createPacketWriteHeader(
        void* header, void* payload, size_t payload_size);

NextReadOperation ArduinoSerialProtocol::nextReadOperation() const;

ReceiveResult ArduinoSerialProtocol::readBytes(
        const void* data, size_t data_size);


