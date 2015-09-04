// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 
 Inspired by work done here https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>
 
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

/*
 Mobile Telemetry library
 */
#include "AP_Mobile_Telem.h"
extern const AP_HAL::HAL& hal;

/*
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_Common/AP_Common.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Notify/AP_Notify.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <SITL/SITL.h>
#include <Filter/Filter.h>
#include <AP_Param/AP_Param.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/UARTDriver.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
 */

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// TODO: Reconnect, Reset, Power Modem

AP_Mobile_Telem::AP_Mobile_Telem() { }

// init - perform require initialisation including detecting which protocol to use
void AP_Mobile_Telem::init(const AP_SerialManager& serial_manager) {
    _port = hal.uartD;
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Mobile_Telem::sport_tick, void));
    
    if (_port != NULL) {
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
    init_modem();
    
}

#if MOBILE_TELEMETRY_APP == TRUE
void AP_Mobile_Telem::setup() {
#else
    void AP_Mobile_Telem::init_modem() {
#endif
        //_port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        
        //Allocate large enough buffers on uartD to support mavlink
        // If too big: hal.uartD->begin(MOBILE_BAUD, MOBILE_BUFSIZE_IN, MOBILE_BUFSIZE_OUT);
        
        // Remove? As this is already called by the init function
        _port = hal.uartD;
        _port->begin(MOBILE_BAUD, 128, 256);
        
        _initialised_uart = true;
        
        /* Setup GCS_Mavlink library's comm 0 port. */
        //mavlink_comm_port[0] = hal.uartD; // BJ: Evtl ändern!
        
        //pinMode(resetPin, OUTPUT); // Set reset pin to output
        // Wait 3 Seconds for power on (Add Auto on feature through GSM module power button pin
        //startUPWait();
        
        // Initializing GSM Module
        _port->print("AT\r");
        while(!isReady){
            if(_port->available()) {
                // Do we need this?
                char temp[128];
                unsigned int i=0;
                char data;
                while((data = _port->read()) >= 0) {
                    temp[i++] = data;
                }
                temp[i] = 0;
                string b = temp; // Store response string in "b"
                
                if(b.indexOf("+CREG: 1") > 0 || b.indexOf("+CREG: 2") > 0) { // Remove this?
                    isReady = true;
                    //while(!sendATCommand("AT+IPR=57600","OK",100)); // Uncomment to set Modem Baudrate
                    //while(!sendATCommand("AT&W","OK",100)); // Uncomment to write settings to non-volatile memory
                    while(!sendATCommand("AT V1 E1 X1 S0=0","OK",100)); // Set error response and do not pickup on ring
                    while(!sendATCommand("AT+CREG=2","OK",100)); // Set various notice messages and parameters
                    while(!sendATCommand("AT+CMEE=2","OK",100));
                    while(!sendATCommand("AT+CR=1","OK",100));
                    while(!sendATCommand("AT+CRC=1","OK",100));
                    while(!sendATCommand("AT+CSNS=4","OK",100));
                    while(!sendATCommand("AT+CSMINS=1","OK",100));
                    while(!sendATCommand("AT+CSCLK=0","OK",100));
                    while(!sendATCommand("AT+CIURC=1","OK",100));
                    while(!sendATCommand("AT+CGEREP=2","OK",100));
                    while(!sendATCommand("AT+CIPMUX=0","OK",100)); // Single channel communication (ie only one socket can be opened)
                    while(!sendATCommand("AT+CIPMODE=1","OK",100)); // Transparent bridge mode
                    while(!sendATCommand("AT+CIPCCFG=8,10,400,0","OK",100)); // GPRS params
                    _port->print("AT+CMUX=0,0,4,32768,10,3,30,10,2\r"); // GPRS/IP params
                    delay(2000);
                    while(!sendATCommand("AT+CGATT?","OK",1000)); // Make sure GPRS is Attached
                    while(!sendATCommand("AT+CSTT= \"internet\",\"\",\"\"","OK",1000)); // AT+CSTT="APN","username","password" - login to service provider/carrier
                    while(!sendATCommand("AT+CIICR","OK",1000)); // Connect!
                    while(!sendATCommand("AT+CIFSR",".",100)); // Get IP address (for info only);
                    while(!sendATCommand("AT+CLPORT=\"UDP\",8888","OK",100)); // Prep UDP Port 8888
                    while(!sendATCommand("AT+CIPSTART=\"UDP\",\"drone.dyndns.com\",8888","OK",1000));  // AT+CIPSTART="protocol","ip address or domain","port #"
                    
                    /*Serial.begin(57600); // Start Serial port for communication with Ardupilot
                     digitalWrite(LED,HIGH); // Turn on All Good LED
                     digitalWrite(ERR,LOW); // Turn off Warning LED
                     delay(1000);
                     digitalWrite(LED,LOW);
                     } else {
                     digitalWrite(ERR,HIGH); // Turn on Warning LED
                     digitalWrite(LED,LOW); // Turn off All Good LED
                     */
                }
            }
        }
    }
}

bool AP_Mobile_Telem::sendATCommand(string input, string output, int wait) { // AT Command, Expected Result, Delay before issue command
    bool resp = false;
    _port->print(input + "\r"); // Send input command to modem
    delay(wait); // Delay function time "wait" variable
    
    while(!resp) {
        if(_port->available()) { // Check if response is available
            char temp[128];
            unsigned int i=0;
            char data;
            while((data = _port->read()) >= 0) {
                temp[i++] = data;
            }
            temp[i] = 0;
            
            string b = temp; // Store response string in "b"
            _port->read(); // Clear buffer again
            
#if DEBUG == TRUE
            ::printf("\nSent command: ");
            i=0;
            while((data = temp[i++]) != 0) {
                if (data == '\r') {
                    ::printf("\\r\n");
                } else if (data == '\n') {
                    ::printf("\\n\n");
                } else if (data == ' ') {
                    ::printf("<SPC>");
                } else
                    ::printf("%c", data);
            }
            ::printf("\n");
#endif
            if(b.indexOf(output) > 0){ // True if expected result is returned
                resp = true;
                //count = 0;
                //digitalWrite(LED,HIGH); // Green LED ON
                //digitalWrite(ERR,LOW); // Warning LED OFF
                //digitalWrite(LED,LOW); // Green LED OFF
                return resp;
            } else if(b.indexOf("DEACT") > 0 || b.indexOf("NORMAL POWER DOWN") > 0) { // Check if lost connection or modem power down
                if(powerUpOrDown()){ // Reset comm hardware
                    //digitalWrite(resetPin,LOW); // Restart Modem
                }
            } else {
                //digitalWrite(ERR,HIGH); // Warning LED ON
                //count++;
                return resp;
            }
        }
    }
}

bool AP_Mobile_Telem::CheckResp(string output, int wait) { // AT Command, Expected Result, Delay before issue command
    bool resp = false;
    while(!resp) {
        if(_port->available()) { // Check if response is available
            char temp[128];
            unsigned int i=0;
            char data;
            while((data = _port->read()) >= 0) {
                temp[i++] = data;
            }
            temp[i] = 0;
            
            string b = temp; // Store response string in "b"
            _port->read(); // Clear buffer again
            
#if DEBUG == TRUE
            ::printf("\nSent command: ");
            i=0;
            while((data = temp[i++]) != 0) {
                if (data == '\r') {
                    ::printf("\\r\n");
                } else if (data == '\n') {
                    ::printf("\\n\n");
                } else if (data == ' ') {
                    ::printf("<SPC>");
                } else
                    ::printf("%c", data);
            }
            ::printf("\n");
#endif
            if(b.indexOf(output) > 0){ // True if expected result is returned
                resp = true;
                //count = 0;
                //digitalWrite(LED,HIGH); // Green LED ON
                //digitalWrite(ERR,LOW); // Warning LED OFF
                //digitalWrite(LED,LOW); // Green LED OFF
                return resp;
            } else if(b.indexOf("DEACT") > 0 || b.indexOf("NORMAL POWER DOWN") > 0) { // Check if lost connection or modem power down
                if(powerUpOrDown()){ // Reset comm hardware
                    //digitalWrite(resetPin,LOW); // Restart Modem
                }
            } else {
                //digitalWrite(ERR,HIGH); // Warning LED ON
                //count++;
                return resp;
            }
        }
    }
}

void AP_Mobile_Telem::advanced_configurations() {
    // Reset to the factory settings
    sendATCommand("AT&F", 1000, 50, "OK", 5);
    // switch off echo
    sendATCommand("ATE0", 500, 50, "OK", 5);
    // setup fixed baud rate
    //sendATCommand("AT+IPR=9600", 500, 50, "OK", 5);
    // setup mode
    //sendATCommand("AT#SELINT=1", 500, 50, "OK", 5);
    // Switch ON User LED - just as signalization we are here
    //sendATCommand("AT#GPIO=8,1,1", 500, 50, "OK", 5);
    // Sets GPIO9 as an input = user button
    //sendATCommand("AT#GPIO=9,0,0", 500, 50, "OK", 5);
    // allow audio amplifier control
    //sendATCommand("AT#GPIO=5,0,2", 500, 50, "OK", 5);
    // Switch OFF User LED- just as signalization we are finished
    //sendATCommand("AT#GPIO=8,0,1", 500, 50, "OK", 5);
    
    // check comm line
    //if (CLS_FREE != GetCommLineStatus()) return;
    SetCommLineStatus(CLS_ATCMD);
    // Request calling line identification
    sendATCommand(F("AT+CLIP=1"), 500, 50, "OK", 5);
    // Mobile Equipment Error Code
    sendATCommand(F("AT+CMEE=0"), 500, 50, "OK", 5);
    // Echo canceller enabled
    //sendATCommand("AT#SHFEC=1", 500, 50, "OK", 5);
    // Ringer tone select (0 to 32)
    //sendATCommand("AT#SRS=26,0", 500, 50, "OK", 5);
    // Microphone gain (0 to 7) - response here sometimes takes
    // more than 500msec. so 1000msec. is more safety
    //sendATCommand("AT#HFMICG=7", 1000, 50, "OK", 5);
    // set the SMS mode to text
    sendATCommand(F("AT+CMGF=1"), 500, 50, "OK", 5);
    // Auto answer after first ring enabled
    // auto answer is not used
    //sendATCommand("ATS0=1", 500, 50, "OK", 5);
    // select ringer path to handsfree
    //sendATCommand("AT#SRP=1", 500, 50, "OK", 5);
    // select ringer sound level
    //sendATCommand("AT+CRSL=2", 500, 50, "OK", 5);
    // we must release comm line because SetSpeakerVolume()
    // checks comm line if it is free
    //SetCommLineStatus(CLS_FREE);
    // select speaker volume (0 to 14)
    //SetSpeakerVolume(9);
    // init SMS storage
    //InitSMSMemory();
    // select phonebook memory storage
    sendATCommand(F("AT+CPBS=\"SM\""), 1000, 50, "OK", 5);
    sendATCommand(F("AT+CIPSHUT"), 500, 50, "SHUT OK", 5);
}

/*
 init_uart_for_sport - initialise uart for use by sport
 this must be called from sport_tick which is called from the 1khz scheduler
 because the UART begin must be called from the same thread as it is used from
 */
void AP_Mobile_Telem::init_uart_for_sport() {
    // initialise uart
    _port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
    _initialised_uart = true;
}

void AP_Mobile_Telem::sport_tick() {
    // has UART been initialised?
    if (!_initialised_uart) {
        init_uart_for_sport();
    }
    
    int16_t numc;
    numc = _port->available();
    
    // check if available is negative
    if (numc < 0) {
        return;
    }
    
    // this is the constant for hub data frame
    /*if (_port->txspace() < 19) {
     return;
     }*/
    
    for (int16_t i = 0; i < numc; i++) {
        msg[i] = _port->read();
    }
}

void AP_Mobile_Telem::mobile_send_byte(uint8_t value) {
    _port->write(&value, sizeof(value));
}

// TODO: Make more efficient. Memcpy and things.
size_t AP_Mobile_Telem::write(const uint8_t *buffer, size_t size) {
    size_t result = 0;
    //while(result < size &&_port->write(buffer[result]), sizeof(buffer[result])) // do we need this?
    while(result < size &&_port->write(buffer[result]))
        result++;
    return result;
}

bool AP_Mobile_Telem::checkTimeout() {
    if (!_port->available() && _timeout++ >= TIMEOUT) {
        ::printf("TIMEOUT!\n");
        // Send some harmless dummy spaces. This finishes messages that were never sent (for whatever crazy reason).
        int16_t numSpaces = _port->txspace();
        if (numSpaces > 62)
            numSpaces = 62;
        for(int16_t i = 0; i < numSpaces; i++) {
            _port->write(' ');
        }
        _port->write('\r');
        _port->write('\n');
        _timeout = TIMEOUT/2;
        return true;
    }
    return false;
}

void AP_Mobile_Telem::AP_Mobile_transmit() {
    if (!_initialised_uart)
        init_uart_for_sport();
    
    if (checkTimeout())
        return;
    
    sport_tick();
#if DEBUG == TRUE
    ::printf("%s\n", msg);
#endif
    write((const) msg, sizeof(msg)/sizeof(msg[0]));
    clean_buffer();
}

// TODO: Better way to clean buffer!
void AP_Mobile_Telem::clean_buffer() {
    int size = sizeof(msg)/sizeof(msg[0]);
    for (int i = 0, i < size, i++)
        msg[i] = "";
}

//Start Up Wait Period with LEDs
/*
 void startUPWait(){
 digitalWrite(LED,HIGH);
 digitalWrite(ERR,LOW);
 delay(500);
 digitalWrite(LED,LOW);
 digitalWrite(ERR,HIGH);
 delay(500);
 digitalWrite(LED,HIGH);
 digitalWrite(ERR,LOW);
 delay(500);
 digitalWrite(LED,LOW);
 digitalWrite(ERR,HIGH);
 delay(500);
 digitalWrite(LED,HIGH);
 digitalWrite(ERR,HIGH);
 delay(1000);
 digitalWrite(LED,LOW);
 digitalWrite(ERR,LOW);
 }*/

/*
 void loop() {
 //digitalWrite(LED,HIGH); // Keep Green LED ON while connected
 // Relay All GSM Module communication to Autopilot
 if(_port->available()) {
 char b=_port->read();
 _port->write(b);
 // Check For Disconnection
 checker += b;
 if(checker.indexOf("\n") > 0 || checker.indexOf("\r") > 0) { // Check for new line
 if(checker.indexOf("DEACT") > 0 || checker.indexOf("NORMA") > 0) { // Check for lost connection or Modem power down
 //digitalWrite(LED,LOW); // Green LED OFF
 //digitalWrite(ERR,HIGH); // Warning LED ON
 if(powerUpOrDown()) { // Reset Comm Hardware
 // TODO: implement a modem reset function
 //digitalWrite(resetPin,LOW); // Reset Modem
 }
 }
 checker = "";
 }
 //digitalWrite(ERR,HIGH); // Flash Warning LED on data send/receive
 //digitalWrite(ERR,LOW);
 }
 
 // Relay all Autopilot communication to GSM module and USB (USB for monitor/debug only)
 if(_port->available()){
 char c=_port->read();
 _port->write(c);
 //digitalWrite(ERR,HIGH); // Flash Warning LED on data send/receive
 //digitalWrite(ERR,LOW);
 }
 }*/

bool AP_Mobile_Telem::powerUpOrDown() // Cycle power on Modem
{
    /*bool powered = false;
     pinMode(powerKey, OUTPUT);
     digitalWrite(powerKey,LOW);
     delay(1000);
     digitalWrite(powerKey,HIGH);
     delay(2000);
     digitalWrite(powerKey,LOW);
     delay(3000);*/
    powered = true;
    return powered;
}

/*
 static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
 {
 if (uart == NULL) {
 // that UART doesn't exist on treadhis platform
 return;
 }
 uart->printf("Hello on UART %s at %.3f seconds\n",
 name, hal.scheduler->millis()*0.001f);
 }*/

/*
 * Main driver of the state machine.
 */
/*
 void task(void) {
 do {
 _state->task(*this);
 hal.scheduler->delay(1000); // BJ added
 } while (_port->available() > 0);
 }*/

/*
 uint16_t read_byte() {
 return _port->read();
 }*/

#if MOBILE_TELEMETRY_APP == TRUE
void AP_Mobile_Telem::loop() {
    transmit();
    hal.scheduler->delay(1000);
}
#endif

/*
 void SimpleCommand(char* comm) {
 _port->print(comm);
 }
 
 void SimpleCommand(int comm) {
 _port->print(comm);
 }
 
 void SimpleCommandln(char* comm) {
 _port->println(comm);
 }
 
 void SimpleCommandln(int comm) {
 _port->println(comm);
 }*/

int AP_Mobile_Telem::setPIN(char *pin) {
    if(!_port->available())
        return -1;
    
    //AT command to set PIN.
    SimpleCommand("AT+CPIN=");
    SimpleCommandln(pin);
    if(CheckResp("OK", 5000))
        return 0;
    return 1;
}

int AP_Mobile_Telem::changeNSIPmode(char mode) {
    SimpleCommand("AT+QIDNSIP=");
    SimpleCommandln(mode);
    if(CheckResp("OK", 5000))
        return 0;
    return 1;
}

bool AP_Mobile_Telem::call(char* number, unsigned int wait) {
    if(!_port->available())
        return false;
    
    SimpleCommand("ATD");
    SimpleCommand(number);
    SimpleCommandln(";");
    hal.scheduler->delay(wait);
    SimpleCommandln("ATH");
    return true;
    
}

int AP_Mobile_Telem::getIMEI(char *imei) {
    //AT command to get IMEI.
    SimpleCommandln("AT+GSN");
    
    bool resp = false;
    while(!resp) {
        if(_port->available()) { // Check if response is available
            char temp[128];
            unsigned int i=0;
            char data;
            while((data = _port->read()) >= 0) {
                temp[i++] = data;
            }
            temp[i] = 0;
            if(i > 16)
                ::printf("Error\n");
            
            *imei = temp; // Store response string in "b"
            _port->read(); // Clear buffer again
        } else if(a > 10)
            break;
        else {
            a++;
            hal.scheduler->delay(1000);
        }
    }
    
    //Expect "OK".
    if(CheckResp("OK", 5000))
        return 0;
    else
        return -1;
}

#if MOBILE_TELEMETRY_APP == TRUE
AP_HAL_MAIN();
#endif