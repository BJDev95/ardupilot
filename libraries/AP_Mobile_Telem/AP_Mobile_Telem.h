// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __AP_MOBILE_TELEM_H__
#define __AP_MOBILE_TELEM_H__

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_SerialManager/AP_SerialManager.h>

#ifndef MOBILE_BUFSIZE_IN
#define MOBILE_BUFSIZE_IN 64
#endif
#ifndef MOBILE_BUFSIZE_OUT
#define MOBILE_BUFSIZE_OUT 128
#endif
#ifndef MOBILE_BAUD
#define MOBILE_BAUD 57600
#endif

AP_HAL::UARTDriver* _port;
char login_username[] = "";
char login_password[] = "";
char login_apn[] = "internet";
char hostname[] = "www.dronex.ch";
char _IP[20];
int _IP_port = 14550; // or e.g. 5760
char _IP_protocol[] = "TCP";

int _timeout = 0;
#define TIMEOUT 400
bool _initialised_uart = false;
bool isReady = false; // GSM modem is Ready to receive AT Commands flag
String checker = "";

class AP_Mobile_Telem
{
public:
    //constructor
    AP_Mobile_Telem();
    void AP_Mobile_transmit();
    void setup();
    void loop();
    char* get_ip() { return _IP; }
    char* get_protocol() { return _IP_protocol; }
    char* get_port() {
        if (_IP_port == 0)
            _IP_port = 14550; // set default if unset
        sprintf (_port_str, "%d", (uint16_t) _IP_port);
        return _port_str;
    }
    uint32_t get_baudrate() { return _baudrate; }

private:
    bool sendATCommand(String input, String output, int wait);
    bool CheckResp(String output, int wait);
    void advanced_configurations();
    
    // init_uart_for_sport - initialise uart for use by sport
    void init_uart_for_sport();

    // sport_tick - main call to send updates to transmitter when protocol is MobileSPORT
    //  called by scheduler at a high rate
    void sport_tick(int16_t* msg);
    
    void mobile_send_byte(uint8_t value);
    size_t write(const uint8_t *buffer, size_t size);
    bool checkTimeout();
    bool powerUpOrDown();
    uint16_t read_byte() { return _port->read(); }
    void SimpleCommand(char* comm) { _port->print(comm); }
    void SimpleCommand(int comm) { _port->print(comm); }
    void SimpleCommandln(char* comm) { _port->println(comm); }
    void SimpleCommandln(int comm) { _port->println(comm); }
    int setPIN(char *pin);
    int changeNSIPmode(char mode);
    bool call(char* number, unsigned int wait);
    int getIMEI(char* imei);
};
#endif
