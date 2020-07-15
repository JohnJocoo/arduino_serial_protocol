#pragma once

#include <stddef.h>
#include <stdint.h>

enum class ReadOperation
{
    NOPE,
    READ_HEADER_PROCESS,
    READ_PAYLOAD
};

enum class ReadResult
{
    NOPE,
    ERROR_UNEXPECTED_DATA,
    ERROR_CHECKSUM,
    ERROR_INSUFFICIENT_DATA_LENGTH
};

struct NextReadOperation
{
    ReadOperation read_operation;
    size_t bytes_to_read;
    uint16_t id;
};

struct ReceiveResult
{
    ReadResult read_result;
    size_t bytes_read;
};


class ArduinoSerialProtocol
{
public:
    static ArduinoSerialProtocol createPrimary();
    static ArduinoSerialProtocol createSecondary();

    ArduinoSerialProtocol(const ArduinoSerialProtocol&) = delete;
    ArduinoSerialProtocol(ArduinoSerialProtocol&&) = default;

    ~ArduinoSerialProtocol() = default;

    constexpr size_t headerSize() const
    { return 8; }

    size_t packetSize(size_t payload_size) const
    { return headerSize() + payload_size; }

    uint16_t createPacketWriteHeader(void* header, void* payload,
                                     size_t payload_size);

    NextReadOperation nextReadOperation() const;

    ReceiveResult readBytes(const void* data, size_t data_size);

private:
    explicit ArduinoSerialProtocol(bool is_primary);

    bool is_primary;
    bool is_synced;
    char state;

}; // class ArduinoSerialProtocol
