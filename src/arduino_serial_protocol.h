#pragma once

#include <stddef.h>
#include <stdint.h>

enum class ArduinoSerialGeneralResult
{
    OK,
    ERROR_WRONG_STATE,
    ERROR_NOT_SYNCED,
    ERROR_PAYLOAD_SIZE_TOO_BIG,
    ERROR_UNDEFINED
};

enum class ArduinoSerialOperation
{
    NOPE,
    READ_HEADER,
    READ_PAYLOAD,
    SEND_SYNC_REPLY
};

enum class ArduinoSerialReadResult
{
    NOPE,
    OK,
    ERROR_UNEXPECTED_DATA,
    ERROR_CHECKSUM,
    ERROR_INSUFFICIENT_DATA_LENGTH
};

using ArduinoSerialProtocolID = uint16_t;

struct ArduinoSerialNextOperation
{
    ArduinoSerialOperation read_operation;
    size_t bytes_to_read;
    ArduinoSerialProtocolID id;
};

struct ArduinoSerialReceiveResult
{
    ArduinoSerialReadResult read_result;
    size_t bytes_read;
};


class ArduinoSerialProtocol
{
public:
    static ArduinoSerialProtocol createSecondary();

    ArduinoSerialProtocol(const ArduinoSerialProtocol&) = delete;
    ArduinoSerialProtocol(ArduinoSerialProtocol&&) = default;

    ~ArduinoSerialProtocol() = default;

    size_t headerSize() const
    { return 8; }

    size_t syncHeaderSize() const
    { return 4; }

    size_t syncReplyHeaderSize() const
    { return 4; }

    size_t packetSize(size_t payload_size) const
    { return headerSize() + payload_size; }

    ArduinoSerialProtocolID createNextPacketId();

    ArduinoSerialGeneralResult
    writeHeader(void* header, ArduinoSerialProtocolID id,
                const void* payload, size_t payload_size) const;

    ArduinoSerialGeneralResult writeSyncReplyHeader(void* header) const;

    ArduinoSerialGeneralResult syncReplySent();

    ArduinoSerialNextOperation nextOperation() const;

    ArduinoSerialReceiveResult readBytes(const void* data, size_t data_size);

public:
    struct PayloadState
    {
        size_t payload_len;
        uint16_t packet_id;
        uint16_t crc16;
        uint16_t crc16_header;
    };

private:
    explicit ArduinoSerialProtocol();

    char state;
    bool was_synced : 1;
    bool scan_strobe : 1;
    uint16_t seq_id;

    PayloadState payload_state;

}; // class ArduinoSerialProtocol
