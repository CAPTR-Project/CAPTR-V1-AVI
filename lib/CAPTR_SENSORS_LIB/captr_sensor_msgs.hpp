#ifndef SENSOR_MSGS_HPP
#define SENSOR_MSGS_HPP

#include <ArduinoEigenDense.h>
#include <arduino_freertos.h>

namespace sensor_msgs {
    class GyroMsg {
    
    public:
        GyroMsg();

        Eigen::Vector3d toVector();

        SemaphoreHandle_t ready;

        float x = 0; // [rad/s]
        float y = 0; // [rad/s]
        float z = 0; // [rad/s]
    };

    class AccelMsg {
    
    public:
        AccelMsg();
        Eigen::Vector3d toVector();

        SemaphoreHandle_t ready;

        float x = 0; // [m/s^2]
        float y = 0; // [m/s^2]
        float z = 0; // [m/s^2]
    };

    class MagMsg {
    
    public:
        MagMsg();
        Eigen::Vector3d toVector();

        SemaphoreHandle_t ready;
        
        float x = 0; // [uT]
        float y = 0; // [uT]
        float z = 0; // [uT]
    };

    class BaroMsg {
    
    public:
        BaroMsg();
        SemaphoreHandle_t ready;

        float pres_msl = 0; // [Pa]
        float pres_agl = 0; // [Pa]
        float pressure = 0; // [Pa]
        float alt_msl = 0; // [m]
        float alt_agl = 0; // [m]
        // float temperature = 0; // [deg C]
    }; 

}

#endif