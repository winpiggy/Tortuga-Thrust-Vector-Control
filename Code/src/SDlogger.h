#ifndef SDLOGGER_H
#define SDLOGGER_H

#include "pins.h"
#include "SD.h"
#include "setup.h"

class SDlogger {

    private:

        int chipSelect = BUILTIN_SDCARD;
        File logFile;
        

    public:
        SDlogger(int chipSelectPin);
        bool begin();

        void logData(const String& data);

        int createLogFile(const char* filename);
        int updateLogFile(const char* filename);

        int getchipSelectPin() const { return chipSelect; }

};


#endif // SDLOGGER_H