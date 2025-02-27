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

#include "radio_cdh.hpp"


namespace radio_cdh {
    uint8_t recv_buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t recv_len;
    uint8_t send_buf[RH_RF95_MAX_MESSAGE_LEN];

    void radio_cdh_start() {
        
    }

    void radio_cdh_thread(void*) {
        if (!manager.init()) {
            error_state_.store(ErrorState::RF);
        }
        if (!rf95.setFrequency(RF95_FREQ)) {
            error_state_.store(ErrorState::RF);
        }
        rf95.setTxPower(20, false);
        while (true) {
            if (manager.waitAvailableTimeout(200)) {
                manager.recvfromAckTimeout(recv_buf, &recv_len, 100);
            } else {
                // send data
                manager.sendto(send_buf, (uint8_t) RH_RF95_MAX_MESSAGE_LEN, RADIO_GS_ADDRESS);
            }
           
        }
    }

}