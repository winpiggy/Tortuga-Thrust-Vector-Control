#include "Bluetooth.h"

void Bluetooth::begin(unsigned long baud) {
    pinMode(statePin, INPUT);
    serial.begin(baud);
}

Bluetooth::BluetoothState Bluetooth::getState() {
    return isConnected() ? BluetoothState::CONNECTED : BluetoothState::DISCONNECTED;
}

bool Bluetooth::isConnected() {
    return digitalRead(statePin) == HIGH;
}

const char* Bluetooth::readCommand() {
    while (serial.available() > 0) {
        char c = serial.read();
        if (c == '\n' || c == '\r') {
            if (lineLength == 0) {
                continue; // ignore the second half of "\r\n" and blank lines
            }
            lineBuffer[lineLength] = '\0';
            lineLength = 0;
            return lineBuffer;
        }
        if (lineLength < sizeof(lineBuffer) - 1) {
            lineBuffer[lineLength++] = toupper(c);
        }
    }
    return nullptr;
}
