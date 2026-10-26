#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <SD.h>
#include "flight_data.h"

class SDlogger {

    private:

        int chipSelect;
        File logFile;
        bool open = false;
        char fileName[16] = "";
        unsigned long lastFlush = 0;


    public:
        explicit SDlogger(int chipSelectPin = BUILTIN_SDCARD) : chipSelect(chipSelectPin) {}
        bool begin();

        // Opens the next unused flightNN.csv and writes the column header.
        bool createLogFile();

        void logData(const char* line);
        void logFlightData(const FlightData &data);

        void flush();
        void close();

        bool isOpen() const { return open; }
        const char* getFileName() const { return fileName; }
        int getchipSelectPin() const { return chipSelect; }

};


#endif // SDLOGGER_H
