/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: daq_threads.hpp
Auth: Yubo Wang
Desc: Header file for telemtry and logging thread

*/

#include "radio_cdh_task.hpp"

namespace radio_cdh {
    uint8_t recv_buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t recv_len;
    uint8_t from; // address of the sender
    uint8_t send_buf[RH_RF95_MAX_MESSAGE_LEN];

    void radioInit() {
        if (!manager.init()) {
            state_manager::setError(ErrorState::RF);
            Serial.println("Radio init failed");
        }
        if (!rf95.setFrequency(RF95_FREQ)) {
            state_manager::setError(ErrorState::RF);
            Serial.println("Radio set frequency failed");
        }
        rf95.setTxPower(20, false);
        rf95.setThisAddress(RADIO_ROCKET_ADDRESS);
        rf95.setHeaderFrom(RADIO_ROCKET_ADDRESS);
        rf95.setHeaderTo(RADIO_GS_ADDRESS);

        xTaskCreate(
            radioRxTask, "RadioRx", 512, NULL, 1, NULL
        );
        xTaskCreate(
            radioTxTask, "RadioTx", 512, NULL, 1, NULL
        );
    }

    void radioRxTask(void*) {

        while (true) {
            if (manager.available())
            {
                recv_len = sizeof(recv_buf);
                if (manager.recvfromAck(recv_buf, &recv_len, &from))
                {
                    if (recv_len >= sizeof(telemetry_protocol::CommandPacket))
                    {
                        telemetry_protocol::CommandPacket cmd;
                        memcpy(&cmd, recv_buf, sizeof(cmd));
                        if (cmd.version != telemetry_protocol::PROTOCOL_VERSION) {
                            Serial.println("Received command with invalid version: " + String(cmd.version));
                            continue; // ignore invalid commands
                        }
                        Serial.println("Received command: " + String(static_cast<int>(cmd.cmd_id)));
                        switch (cmd.cmd_id) {
                            case telemetry_protocol::CmdID::ESTOP:
                                state_manager::setError(ErrorState::GENERAL);
                                state_manager::requestState(MCUState::STBY);
                                break;
                            case telemetry_protocol::CmdID::Ping:
                                // Respond with a ping
                                send_buf[0] = static_cast<uint8_t>(telemetry_protocol::CmdID::Ping);
                                manager.sendtoWait(send_buf, 1, from);
                                break;
                            case telemetry_protocol::CmdID::Standby:
                                state_manager::requestState(MCUState::STBY);
                                break;
                            case telemetry_protocol::CmdID::ServoTest:
                                state_manager::requestState(MCUState::SERVOTEST);
                                break;
                            case telemetry_protocol::CmdID::Calibrate:
                                state_manager::requestState(MCUState::CALIBRATING);
                                break;
                            case telemetry_protocol::CmdID::LaunchDetect:
                                state_manager::requestState(MCUState::LAUNCH_DETECT);
                                break;
                            default:
                                Serial.println("Unknown command received: " + String(static_cast<int>(cmd.cmd_id)));
                                state_manager::setError(ErrorState::FSM);
                                break;
                        }
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));   // yield
        }
    }

    void radioTxTask(void*) {
        
        TickType_t xLastWakeTime = xTaskGetTickCount();
        BaseType_t xWasDelayed;

        const TickType_t period = pdMS_TO_TICKS(1000 / DOWNLINK_FREQUENCY);

        while (true) {
            /* -------- gather local data -------- */
            telemetry_protocol::DownlinkPacket data{};
            data.timestamp    = xTaskGetTickCount() * portTICK_PERIOD_MS;
            data.version      = telemetry_protocol::PROTOCOL_VERSION;

            // send MCU state
            data.data_id      = telemetry_protocol::DataID::MCUState;
            data.data_length  = sizeof(MCUState);
            memcpy(data.data, &state_manager::currentState, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);

            // send Error state
            data.data_id      = telemetry_protocol::DataID::ErrorState;
            data.data_length  = sizeof(ErrorState);
            memcpy(data.data, &state_manager::errorState_, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            // send Altitude AGL
            data.data_id      = telemetry_protocol::DataID::Altitude_AGL;
            data.data_length  = sizeof(float);
            float altitude_agl = sensors::baro::baroData_.alt_agl;
            memcpy(data.data, &altitude_agl, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);

            // send Baro Pressure
            data.data_id      = telemetry_protocol::DataID::BaroPressure;
            data.data_length  = sizeof(float);
            float baro_pressure = sensors::baro::baroData_.pressure;
            memcpy(data.data, &baro_pressure, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            // send Accel X
            data.data_id      = telemetry_protocol::DataID::AccelX;
            data.data_length  = sizeof(float);
            float accel_x = sensors::IMU_main::accelData_.x;
            memcpy(data.data, &accel_x, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            // send Accel Y
            data.data_id      = telemetry_protocol::DataID::AccelY;
            data.data_length  = sizeof(float);
            float accel_y = sensors::IMU_main::accelData_.y;
            memcpy(data.data, &accel_y, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            // send Accel Z
            data.data_id      = telemetry_protocol::DataID::AccelZ;
            data.data_length  = sizeof(float);
            float accel_z = sensors::IMU_main::accelData_.z;
            memcpy(data.data, &accel_z, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            // send Gyro X
            data.data_id      = telemetry_protocol::DataID::GyroX;
            data.data_length  = sizeof(float);
            float gyro_x = sensors::IMU_main::gyroData_.x;
            memcpy(data.data, &gyro_x, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            // send Gyro Y
            data.data_id      = telemetry_protocol::DataID::GyroY;
            data.data_length  = sizeof(float);
            float gyro_y = sensors::IMU_main::gyroData_.y;
            memcpy(data.data, &gyro_y, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            // send Gyro Z
            data.data_id      = telemetry_protocol::DataID::GyroZ;
            data.data_length  = sizeof(float);
            float gyro_z = sensors::IMU_main::gyroData_.z;
            memcpy(data.data, &gyro_z, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            
            // send Orient X
            data.data_id      = telemetry_protocol::DataID::OrientX;
            data.data_length  = sizeof(float);
            float orient_x = att_est_tasks::att_estimator_.newest_attitude_quat.get_roll(); // in radians
            memcpy(data.data, &orient_x, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            // send Orient Y
            data.data_id      = telemetry_protocol::DataID::OrientY;
            data.data_length  = sizeof(float);
            float orient_y = att_est_tasks::att_estimator_.newest_attitude_quat.get_pitch(); // in radians
            memcpy(data.data, &orient_y, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            // send Orient Z
            data.data_id      = telemetry_protocol::DataID::OrientZ;
            data.data_length  = sizeof(float);
            float orient_z = att_est_tasks::att_estimator_.newest_attitude_quat.get_yaw(); // in radians
            memcpy(data.data, &orient_z, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            
            // send Pyro States

            // send Pitch Servo Command
            data.data_id      = telemetry_protocol::DataID::PitchServoCMD;
            data.data_length  = sizeof(float);
            float pitch_servo_cmd = control::tvcMount_.pitch_cmd_rad_; // in radians
            memcpy(data.data, &pitch_servo_cmd, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);
            // send Yaw Servo Command
            data.data_id      = telemetry_protocol::DataID::YawServoCMD;
            data.data_length  = sizeof(float);
            float yaw_servo_cmd = control::tvcMount_.yaw_cmd_rad_; // in radians
            memcpy(data.data, &yaw_servo_cmd, data.data_length);
            /* -------- send data -------- */
            manager.sendtoWait((uint8_t*)&data, sizeof(data), RADIO_GS_ADDRESS);

            // wait for the next period

            xWasDelayed = xTaskDelayUntil(&xLastWakeTime, period);
            if (!xWasDelayed) {
                // state_manager::setError(ErrorState::CONTROL);
                Serial.println("Control loop delayed");
            }
        }
    }
}