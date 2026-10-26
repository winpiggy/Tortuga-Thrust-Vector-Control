#include <Arduino.h>
#include "SDlogger.h"
#include "setup.h"


bool SDlogger::begin() {
    if (!SD.begin(chipSelect)) {
        Serial.println("Failed to initialize SD card");
        return false;
    }
    return true;
}

bool SDlogger::createLogFile() {
    // Never overwrite an old flight: pick the first name that doesn't exist yet
    for (int i = 0; i < 100; i++) {
        snprintf(fileName, sizeof(fileName), "flight%02d.csv", i);
        if (!SD.exists(fileName)) {
            logFile = SD.open(fileName, FILE_WRITE);
            open = static_cast<bool>(logFile);
            if (!open) {
                Serial.println("Failed to create log file");
                return false;
            }
            logData("time_us,state,ax,ay,az,gx,gy,gz,pitch,yaw,roll,temp_c,pressure_pa,alt_m,rel_alt_m,pitch_cmd,yaw_cmd");
            flush();
            return true;
        }
    }
    Serial.println("No free log file names left");
    return false;
}

void SDlogger::logData(const char* line) {
    if (!open) {
        return;
    }
    logFile.println(line);
}

void SDlogger::logFlightData(const FlightData &data) {
    if (!open) {
        return;
    }

    logFile.printf("%lu,%s,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f\n",
        data.timeUs, stateName(data.state),
        data.ax, data.ay, data.az,
        data.gx, data.gy, data.gz,
        data.pitch, data.yaw, data.roll,
        data.temperature, data.pressure, data.altitude, data.relativeAltitude,
        data.pitchCommand, data.yawCommand);

    // Writes are buffered in RAM; flushing every line would be far too slow for the loop,
    // but flushing periodically limits what is lost if power is cut.
    if (millis() - lastFlush >= logFlushMs) {
        flush();
    }
}

void SDlogger::flush() {
    if (open) {
        logFile.flush();
    }
    lastFlush = millis();
}

void SDlogger::close() {
    if (open) {
        logFile.close();
        open = false;
    }
}
