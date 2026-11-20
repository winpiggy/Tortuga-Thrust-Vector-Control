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
