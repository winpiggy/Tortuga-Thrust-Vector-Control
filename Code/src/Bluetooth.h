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

    explicit Bluetooth(HardwareSerial &port) : serial(port) {}

    enum class BluetoothState {
        DISCONNECTED,
        CONNECTED
    };

    // Most HC-05 / HM-10 style modules use 9600 baud by default
    void begin(unsigned long baud = 9600);

    // Reads the module's STATE pin, which is HIGH while a device is connected
    BluetoothState getState();
    bool isConnected();

    HardwareSerial &port() { return serial; }
};


#endif // BLUETOOTH_H
