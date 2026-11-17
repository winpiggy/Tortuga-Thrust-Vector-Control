#ifndef BLUETOOTH_H
#define BLUETOOTH_H
#include <Arduino.h>
#include "pins.h"
#define bleSerial Serial2

class Bluetooth {

    private: 
        int txPin = TXD;
        int rxPin = RXD;
        int statePin = STATE;

        HardwareSerial &serial;
    

    public:

    Bluetooth(HardwareSerial &port) : serial(port) {}

    enum class BluetoothState {
        DISCONNECTED,
        CONNECTED
    };


    BluetoothState getState() {

        while(serial.available()) {
            char c = serial.read();
            if (c == 'C') {
                return BluetoothState::CONNECTED;
            } else if (c == 'D') {
                return BluetoothState::DISCONNECTED;
            }
        }
    }
};








#endif // BLUETOOTH_H
