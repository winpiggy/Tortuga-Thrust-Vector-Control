#ifndef SDLOGGER_H
#define SDLOGGER_H

#include "pins.h"
#include "SD.h"
#include "setup.h"

class SDlogger {

    private:

        int chipSelect = BUILTIN_SDCARD;
        File logFile;
        unsigned long lastFlush = 0;

    public:
        explicit SDlogger(int chipSelectPin = BUILTIN_SDCARD) : chipSelect(chipSelectPin) {}

        // Start the SD card; returns false if no card is found
        bool begin();

        // Opens a new numbered file (e.g. FLIGHT00.CSV, FLIGHT01.CSV...) so old
        // flights are never overwritten. Returns 0 on success, -1 on failure.
        int createLogFile(const char* baseName);

        // Writes one line. Data is written to the card every logFlushMs.
        void logData(const char* line);
        void logData(const String& line);

        // Force buffered data onto the card now
        void flush();

        bool isOpen() { return logFile; }
        int getchipSelectPin() const { return chipSelect; }

};


#endif // SDLOGGER_H
