#ifndef SENSOR_MSGS_HPP
#define SENSOR_MSGS_HPP

namespace sensor_msgs {
    class GyroMsg {
    
    public:
        double x; // [rad/s]
        double y; // [rad/s]
        double z; // [rad/s]
    };

    class AccelMsg {
    
    public:
        double x; // [m/s^2]
        double y; // [m/s^2]
        double z; // [m/s^2]
    };

    class MagMsg {
    
    public:
        double x; // [uT]
        double y; // [uT]
        double z; // [uT]
    };

    class BaroMsg {
    
    public:
        double pres_msl; // [Pa]
        double pres_agl; // [Pa]
        double pressure; // [Pa]
        double alt_msl; // [m]
        double alt_agl; // [m]
        // double temperature; // [deg C]
    }; 

}

#endif