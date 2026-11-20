#include "pins.h"
#include "SDlogger.h"


bool SDlogger::begin() {
    if (!SD.begin(chipSelect)) {
        Serial.println("Failed to initialize SD card");
        return false;
    }
    return true;
}

int SDlogger::createLogFile(const char* baseName) {
    char filename[32];
    for (int i = 0; i < 100; i++) {
        snprintf(filename, sizeof(filename), "%s%02d.CSV", baseName, i);
        if (!SD.exists(filename)) {
            logFile = SD.open(filename, FILE_WRITE);
            if (!logFile) {
                Serial.println("Failed to create log file");
                return -1;
            }
            Serial.print("Logging to ");
            Serial.println(filename);
            lastFlush = millis();
            return 0;
        }
    }
    Serial.println("No free log file names left (delete old logs)");
    return -1;
}

void SDlogger::logData(const char* line) {
    if (!logFile) {
        return;
    }
    logFile.println(line);

    // Writing to the card is slow, so only push data out every logFlushMs.
    // If power is lost, at most that much data is lost.
    if (millis() - lastFlush >= logFlushMs) {
        flush();
    }
}

void SDlogger::logData(const String& line) {
    logData(line.c_str());
}

void SDlogger::flush() {
    if (logFile) {
        logFile.flush();
    }
    lastFlush = millis();
}
