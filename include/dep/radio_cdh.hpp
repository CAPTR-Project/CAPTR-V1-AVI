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

#ifndef RADIO_CDH_HPP
#define RADIO_CDH_HPP

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#include "globals.hpp"
#include "config.hpp"

namespace radio_cdh {

    // ================================= Vars ====================================

    RH_RF95 rf95(RFM95_CS, RFM95_INT);
    RHReliableDatagram manager(rf95, RADIO_ADDRESS);
    

    // ============================ Function Prototypes ==============================

    void radio_cdh_start();
    void radio_cdh_thread(void*);

} // namespace radio_cdh

#endif