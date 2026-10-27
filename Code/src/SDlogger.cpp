#include "pins.h"
#include "SDlogger.h"


 int SDlogger::getchipSelectPin() const{
    return chipSelect;
}

int SDlogger::updateLogFile(const char* filename) {
    if (!logFile) {
        // If the log file is not open, try to open it
        logFile = SD.open(filename, FILE_WRITE);
        if (!logFile) {
            Serial.println("Failed to open log file for updating");
            return -1; // Error code for failure to open file
        }
    }
    return 0; // Success
}
