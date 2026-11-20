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

        char lineBuffer[32];
        size_t lineLength = 0;


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

    // Non-blocking: reads whatever bytes are waiting and returns a complete
    // command (upper-cased, no line ending) once a '\n' or '\r' arrives.
    // Returns nullptr if no full line is ready yet. Call every loop.
    const char* readCommand();

    void println(const char* message) { serial.println(message); }

    HardwareSerial &port() { return serial; }
};


#endif // BLUETOOTH_H
