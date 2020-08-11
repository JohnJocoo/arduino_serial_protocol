#pragma once

#include <string>
#include <map>

#include "arduino_serial_protocol.h"


template <typename T>
class StringInfo
{
public:
    static std::string toString(const T& value)
    {
        const auto it = string_repr.find(value);
        if (it == string_repr.end())
            return "UNDEFINED";
        return it->second;
    }

private:
    static const std::map<T, std::string> string_repr;

}; // class StringInfo<T>

template <typename T>
void create_string_repr(typename std::map<T, std::string>&);

template <typename T>
std::map<T, std::string> _create_string_repr_map()
{
    typename std::map<T, std::string> result;
    create_string_repr<T>(result);
    return result;
}

template <typename T>
const std::map<T, std::string> StringInfo<T>::string_repr =
        _create_string_repr_map<T>();


template <>
void create_string_repr<ArduinoSerialGeneralResult>(
        std::map<ArduinoSerialGeneralResult, std::string>& map)
{
    using T = std::map<ArduinoSerialGeneralResult, std::string>;
    map = T{
            {ArduinoSerialGeneralResult::OK, "OK"},
            {ArduinoSerialGeneralResult::ERROR_WRONG_STATE, "ERROR_WRONG_STATE"},
            {ArduinoSerialGeneralResult::ERROR_NOT_SYNCED, "ERROR_NOT_SYNCED"},
            {ArduinoSerialGeneralResult::ERROR_PAYLOAD_SIZE_TOO_BIG, "ERROR_PAYLOAD_SIZE_TOO_BIG"},
            {ArduinoSerialGeneralResult::ERROR_UNDEFINED, "ERROR_UNDEFINED"},
    };
}

template <>
void create_string_repr<ArduinoSerialOperation>(
        std::map<ArduinoSerialOperation, std::string>& map)
{
    using T = std::map<ArduinoSerialOperation, std::string>;
    map = T{
            {ArduinoSerialOperation::NOPE, "NOPE"},
            {ArduinoSerialOperation::READ_HEADER, "READ_HEADER"},
            {ArduinoSerialOperation::READ_PAYLOAD, "READ_PAYLOAD"},
            {ArduinoSerialOperation::SEND_SYNC_REPLY, "SEND_SYNC_REPLY"},
    };
}

template <>
void create_string_repr<ArduinoSerialReadResult>(
        std::map<ArduinoSerialReadResult, std::string>& map)
{
    using T = std::map<ArduinoSerialReadResult, std::string>;
    map = T{
            {ArduinoSerialReadResult::OK, "OK"},
            {ArduinoSerialReadResult::NOPE, "NOPE"},
            {ArduinoSerialReadResult::ERROR_UNEXPECTED_DATA, "ERROR_UNEXPECTED_DATA"},
            {ArduinoSerialReadResult::ERROR_CHECKSUM, "ERROR_CHECKSUM"},
            {ArduinoSerialReadResult::ERROR_INSUFFICIENT_DATA_LENGTH, "ERROR_INSUFFICIENT_DATA_LENGTH"},
    };
}
