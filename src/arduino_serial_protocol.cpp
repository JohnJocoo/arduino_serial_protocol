#include "arduino_serial_protocol.h"

#include <string.h>

#ifndef ARDUINO
    #include <netinet/in.h>

    /* CRC-8-CCITT
     * crc8 calculation ported from AVR_LIBC
     * https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
     * Copyright Jack Crenshaw
     * Copyright (c) 2002, 2003, 2004  Marek Michalkiewicz
     * Copyright (c) 2005, 2007 Joerg Wunsch
     * Copyright (c) 2013 Dave Hylands
     * Copyright (c) 2013 Frederic Nadeau
     */
    uint8_t
    _crc8_ccitt_update(uint8_t inCrc, uint8_t inData)
    {
        uint8_t   i;
        uint8_t   data;

        data = inCrc ^ inData;

        for ( i = 0; i < 8; i++ )
        {
            if (( data & 0x80 ) != 0 )
            {
                data <<= 1;
                data ^= 0x07;
            }
            else
            {
                data <<= 1;
            }
        }
        return data;
    }

    /* CRC-CCITT/FALSE
     * crc16 calculation ported from AVR_LIBC
     * https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
     * Copyright Jack Crenshaw
     * Copyright (c) 2002, 2003, 2004  Marek Michalkiewicz
     * Copyright (c) 2005, 2007 Joerg Wunsch
     * Copyright (c) 2013 Dave Hylands
     * Copyright (c) 2013 Frederic Nadeau
     */
    uint16_t
    _crc_ccitt_update(uint16_t crc, uint8_t data)
    {
        crc = crc ^ (int) data << 8;
        size_t i = 8;
        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        } while(--i);

        return crc;
    }

#else
    #include <util/crc16.h>

    #define htons(A) ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))
    #define ntohs(A) htons(A)
#endif


constexpr const uint8_t STROBE_1 = 0xA5;
constexpr const uint8_t STROBE_2 = 0x63;

constexpr const uint8_t SYNC_STROBE_1 = 0xD3;
constexpr const uint8_t SYNC_STROBE_2 = 0x74;
constexpr const uint8_t SYNC_STROBE_3 = 0xE5;
constexpr const uint8_t SYNC_STROBE_4 = 0x52;
constexpr const uint8_t SYNC_STROBE_REPLY = 0x25;

constexpr const size_t HEADER_ID_SIZE = 2;
constexpr const size_t HEADER_PAYLOAD_LEN_SIZE = 1;


enum class State : char
{
    UNDEFINED,
    WAITING_SYNC,
    IDLE,
    READ_STROBE_2,
    READ_SYNC_STROBE_2,
    READ_SYNC_STROBE_3,
    READ_SYNC_STROBE_4,
    WRITE_SYNC_REPLY,
    READ_HEADER,
    READ_PAYLOAD
};


namespace
{

void clear(ArduinoSerialProtocol::PayloadState& payload_state)
{
    payload_state.payload_len = 0;
    payload_state.packet_id = 0;
    payload_state.crc16 = 0;
    payload_state.crc16_header = 0xFFFF;
}

State get_state(char state)
{
    return static_cast<State>(state);
}

void set_state(char& state, State value)
{
    state = static_cast<char>(value);
}

uint8_t calculate_crc8(const void* header)
{
    constexpr const uint8_t crc_len = HEADER_ID_SIZE + HEADER_PAYLOAD_LEN_SIZE;

    const uint8_t* data = static_cast<const uint8_t*>(header);
    uint8_t crc8 = 0;
    for (size_t i = 0; i < crc_len; ++i, ++data)
    {
        crc8 = _crc8_ccitt_update(crc8, *data);
    }
    return crc8;
}

uint16_t calculate_crc16_header(const void* _header)
{
    constexpr const uint8_t crc_len = HEADER_ID_SIZE + HEADER_PAYLOAD_LEN_SIZE + 1;

    const uint8_t* header = static_cast<const uint8_t*>(_header);
    uint16_t crc16 = 0xFFFF;
    for (size_t i = 0; i < crc_len; ++i, ++header)
    {
        crc16 = _crc_ccitt_update(crc16, *header);
    }
    return crc16;
}

uint16_t calculate_crc16_payload(
        uint16_t crc16, const void* _payload, size_t payload_size)
{
    const uint8_t* payload = static_cast<const uint8_t*>(_payload);
    for (size_t i = 0; i < payload_size; ++i, ++payload)
    {
        crc16 = _crc_ccitt_update(crc16, *payload);
    }
    return crc16;
}

template<typename T>
const T* typed_data(const void* data)
{
    return static_cast<const T*>(data);
}

ArduinoSerialReceiveResult
receive_result(ArduinoSerialReadResult read_result, size_t bytes_read)
{
    ArduinoSerialReceiveResult result;
    result.read_result = read_result;
    result.bytes_read = bytes_read;
    return result;
}

ArduinoSerialNextOperation
next_operation(ArduinoSerialOperation operation, size_t bytes_to_read,
               ArduinoSerialProtocolID id = 0)
{
    ArduinoSerialNextOperation result;
    result.read_operation = operation;
    result.bytes_to_read = bytes_to_read;
    result.id = id;
    return result;
}

ArduinoSerialReceiveResult
read_strobe(char& state, const void* _data, const size_t data_size,
           const uint8_t value, const State new_state, const State initial_state)
{
    if (data_size < 1)
        return receive_result(
                ArduinoSerialReadResult::ERROR_INSUFFICIENT_DATA_LENGTH, 0);

    const uint8_t* data = static_cast<const uint8_t*>(_data);
    if (data[0] != value)
    {
        set_state(state, initial_state);

        return receive_result(
                ArduinoSerialReadResult::ERROR_UNEXPECTED_DATA, 1);
    }

    set_state(state, new_state);

    return receive_result(ArduinoSerialReadResult::OK, 1);
}

ArduinoSerialReceiveResult
read_strobe_or_sync(char& state, const void* _data, const size_t data_size)
{
    if (data_size < 1)
        return receive_result(
                ArduinoSerialReadResult::ERROR_INSUFFICIENT_DATA_LENGTH, 0);

    const uint8_t* data = static_cast<const uint8_t*>(_data);
    if (data[0] != STROBE_1 && data[0] != SYNC_STROBE_1)
    {
        set_state(state, State::IDLE);

        return receive_result(
                ArduinoSerialReadResult::ERROR_UNEXPECTED_DATA, 1);
    }

    if (data[0] == STROBE_1)
        set_state(state, State::READ_STROBE_2);
    else
        set_state(state, State::READ_SYNC_STROBE_2);

    return receive_result(ArduinoSerialReadResult::OK, 1);
}

ArduinoSerialReceiveResult
read_header(char& state, ArduinoSerialProtocol::PayloadState& payload_state,
           const void* _data, const size_t data_size)
{
    if (data_size < 6)
        return receive_result(
                ArduinoSerialReadResult::ERROR_INSUFFICIENT_DATA_LENGTH, 0);

    const uint8_t* data = static_cast<const uint8_t*>(_data);
    const uint8_t crc8 = calculate_crc8(data);
    if (crc8 != data[3])
    {
        set_state(state, State::IDLE);

        return receive_result(ArduinoSerialReadResult::ERROR_CHECKSUM, 4);
    }

    payload_state.payload_len = data[2];
    payload_state.packet_id = ntohs(*typed_data<uint16_t>(data));
    payload_state.crc16 = ntohs(*typed_data<uint16_t>(data + 4));
    payload_state.crc16_header = calculate_crc16_header(data);

    set_state(state, State::READ_PAYLOAD);

    return receive_result(ArduinoSerialReadResult::OK, 6);
}

ArduinoSerialReceiveResult
read_payload(char& state, ArduinoSerialProtocol::PayloadState& payload_state,
            const void* data, const size_t data_size)
{
    if (data_size < payload_state.payload_len)
        return receive_result(
                ArduinoSerialReadResult::ERROR_INSUFFICIENT_DATA_LENGTH, 0);

    const uint16_t crc16 = calculate_crc16_payload(
            payload_state.crc16_header, data, payload_state.payload_len);
    if (crc16 != payload_state.crc16)
    {
        size_t p_len = payload_state.payload_len;
        clear(payload_state);
        set_state(state, State::IDLE);

        return receive_result(
                ArduinoSerialReadResult::ERROR_CHECKSUM, p_len);
    }

    size_t p_len = payload_state.payload_len;
    clear(payload_state);
    set_state(state, State::IDLE);

    return receive_result(ArduinoSerialReadResult::OK, p_len);
}

}

ArduinoSerialProtocol ArduinoSerialProtocol::createSecondary()
{
    return ArduinoSerialProtocol{};
}

ArduinoSerialProtocol::ArduinoSerialProtocol()
: state{static_cast<char>(State::WAITING_SYNC)}
, was_synced{false}
, seq_id{0}
{
    clear(payload_state);
}

ArduinoSerialProtocolID
ArduinoSerialProtocol::createNextPacketId()
{
    ArduinoSerialProtocolID result = ++seq_id;
    if (result == 0)
        return ++seq_id;
    return result;
}

ArduinoSerialGeneralResult
ArduinoSerialProtocol::writeHeader(
        void* header, ArduinoSerialProtocolID id,
        const void* payload, size_t payload_size) const
{
    static_assert(sizeof(ArduinoSerialProtocolID) == HEADER_ID_SIZE,
                  "ID size unexpected");
    static_assert(HEADER_PAYLOAD_LEN_SIZE == 1,
                  "Payload length size unexpected");

    if (get_state(state) == State::UNDEFINED)
        return ArduinoSerialGeneralResult::ERROR_UNDEFINED;

    if (get_state(state) == State::WAITING_SYNC)
        return ArduinoSerialGeneralResult::ERROR_NOT_SYNCED;

    if (payload_size > 255)
        return ArduinoSerialGeneralResult::ERROR_PAYLOAD_SIZE_TOO_BIG;

    ArduinoSerialProtocolID net_id = htons(id);
    uint8_t net_size = payload_size;

    uint8_t* data = static_cast<uint8_t*>(header);
    data[0] = STROBE_1;
    data[1] = STROBE_2;
    memcpy(data + 2, &net_id, HEADER_ID_SIZE);
    data[HEADER_ID_SIZE + 2] = net_size;
    data[HEADER_ID_SIZE + 3] = calculate_crc8(data + 2);
    uint16_t crc16 = calculate_crc16_header(data + 2);
    crc16 = calculate_crc16_payload(crc16, payload, payload_size);
    crc16 = htons(crc16);
    memcpy(data + HEADER_ID_SIZE + 4, &crc16, 2);
    return ArduinoSerialGeneralResult::OK;
}

ArduinoSerialGeneralResult
ArduinoSerialProtocol::writeSyncReplyHeader(void* header) const
{
    uint8_t* data = static_cast<uint8_t*>(header);
    data[0] = SYNC_STROBE_1;
    data[1] = SYNC_STROBE_2;
    data[2] = SYNC_STROBE_3;
    data[3] = SYNC_STROBE_REPLY;
    return ArduinoSerialGeneralResult::OK;
}

ArduinoSerialGeneralResult
ArduinoSerialProtocol::syncReplySent()
{
    switch (get_state(state))
    {
        case State::WRITE_SYNC_REPLY:
            set_state(state, State::IDLE);
            was_synced = true;
            clear(payload_state);
            return ArduinoSerialGeneralResult::OK;
        case State::IDLE:
            return ArduinoSerialGeneralResult::OK;
        default:
            break;
    }
    return ArduinoSerialGeneralResult::ERROR_WRONG_STATE;
}

ArduinoSerialNextOperation
ArduinoSerialProtocol::nextOperation() const
{
    switch (get_state(state))
    {
        case State::WAITING_SYNC:
        case State::IDLE:
        case State::READ_STROBE_2:
        case State::READ_SYNC_STROBE_2:
        case State::READ_SYNC_STROBE_3:
        case State::READ_SYNC_STROBE_4:
            return next_operation(ArduinoSerialOperation::READ_HEADER, 1);
        case State::WRITE_SYNC_REPLY:
            return next_operation(ArduinoSerialOperation::SEND_SYNC_REPLY, 0);
        case State::READ_HEADER:
            return next_operation(ArduinoSerialOperation::READ_HEADER, 6);
        case State::READ_PAYLOAD:
            return next_operation(ArduinoSerialOperation::READ_PAYLOAD,
                                  payload_state.payload_len,
                                  payload_state.packet_id);
        default:
            break;
    }
    return next_operation(ArduinoSerialOperation::NOPE, 0);
}

ArduinoSerialReceiveResult
ArduinoSerialProtocol::readBytes(const void* data, size_t data_size)
{
    switch (get_state(state))
    {
        case State::WAITING_SYNC:
            return read_strobe(state, data, data_size,
                               SYNC_STROBE_1, State::READ_SYNC_STROBE_2,
                               State::WAITING_SYNC);
        case State::IDLE:
            return read_strobe_or_sync(state, data, data_size);
        case State::READ_STROBE_2:
            return read_strobe(state, data, data_size,
                               STROBE_2, State::READ_HEADER,
                               State::IDLE);
        case State::READ_SYNC_STROBE_2:
            return read_strobe(state, data, data_size,
                               SYNC_STROBE_2, State::READ_SYNC_STROBE_3,
                               was_synced ? State::IDLE : State::WAITING_SYNC);
        case State::READ_SYNC_STROBE_3:
            return read_strobe(state, data, data_size,
                               SYNC_STROBE_3, State::READ_SYNC_STROBE_4,
                               was_synced ? State::IDLE : State::WAITING_SYNC);
        case State::READ_SYNC_STROBE_4:
            return read_strobe(state, data, data_size,
                               SYNC_STROBE_4, State::WRITE_SYNC_REPLY,
                               was_synced ? State::IDLE : State::WAITING_SYNC);
        case State::WRITE_SYNC_REPLY:
            return receive_result(ArduinoSerialReadResult::NOPE, 0);
        case State::READ_HEADER:
            return read_header(state, payload_state, data, data_size);
        case State::READ_PAYLOAD:
            return read_payload(state, payload_state, data, data_size);
        default:
            break;
    }

    return receive_result(ArduinoSerialReadResult::NOPE, 0);
}

