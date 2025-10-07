/**
 * @file telemetry_protocol_shared.hpp
 * @author Yubo Wang
 * @brief The telemetry protocol shared header file.
 * @version 0.1
 * @date 2025-06-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef TELEMETRY_PROTOCOL_SHARED_HPP
#define TELEMETRY_PROTOCOL_SHARED_HPP
 
#include <cstdint>
#include <cstddef>  // for size_t

namespace telemetry_protocol {
    // Define the telemetry protocol version
    constexpr uint8_t PROTOCOL_VERSION = 1;

    enum class CmdID : uint8_t
    {
        ESTOP = 0x00,
        Ping   = 0x01,
        Standby = 0x02,
        ServoTest = 0x03,
        // CalibrateAndLaunch = 0x04,
        Calibrate = 0x05,
        LaunchDetect = 0x06,
    };

    enum class DataID : uint8_t
    {
        MCUState = 0x00,
        ErrorState = 0x01,
        Altitude_AGL = 0x02,
        BaroPressure = 0x03,
        AccelX = 0x04,
        AccelY = 0x05,
        AccelZ = 0x06,
        GyroX = 0x07,
        GyroY = 0x08,
        GyroZ = 0x09,
        OrientX = 0x0A,
        OrientY = 0x0B,
        OrientZ = 0x0C,
        PyroStates = 0x0D,
        PitchServoCMD = 0x0E,
        YawServoCMD = 0x0F,
    };

    #pragma pack(push,1)
    struct CommandPacket
    {
        uint8_t version;          // Protocol version
        CmdID cmd_id;            // Command ID
    };

    struct DownlinkPacket
    {
        uint8_t version;          // Protocol version
        DataID data_id;            // Command ID
        uint32_t timestamp;       // Timestamp in milliseconds
        size_t data_length;       // Length of the data field
        uint8_t data[8];        // Data field (up to 8 bytes)
    };


    #pragma pack(pop)

} // namespace telemetry_protocol

#endif // TELEMETRY_PROTOCOL_SHARED_HPP