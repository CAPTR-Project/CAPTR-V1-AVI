/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: radio_cdh.hpp
Auth: Yubo Wang
Desc: Header file for telemtry and logging thread

*/

#ifndef RADIO_CDH_TASK_HPP
#define RADIO_CDH_TASK_HPP

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include "arduino_freertos.h"

#include "config.hpp"
#include "telemetry_protocol_shared.hpp"
#include "state_mgmt/state_manager_task.hpp"


namespace radio_cdh {

    // ================================= Vars ====================================

    inline RH_RF95 rf95(RFM95_CS, RFM95_INT);
    inline RHReliableDatagram manager(rf95, RADIO_ROCKET_ADDRESS);
    
    // ============================ Function Prototypes ==============================

    void radioInit();

    void radioRxTask   (void* arg);
    void radioTxTask  (void* arg);

} // namespace radio_cdh

#endif // RADIO_CDH_TASK_HPP