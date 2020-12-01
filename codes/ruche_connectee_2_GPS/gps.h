#pragma once


#include <TinyGPS++.h>
#include <HardwareSerial.h>

//#define GPS_TX 17
//#define GPS_RX 16

#define GPS_TX 15
#define GPS_RX 4


class gps
{
    public:
        void init();
        bool checkGpsFix();
        void buildPacket(uint8_t txBuffer[22]);
        void encode();

    private:
        uint32_t LatitudeBinary, LongitudeBinary;
        uint16_t altitudeGps;
        uint8_t hdopGps;
        char t[32]; // used to sprintf for Serial output
        TinyGPSPlus tGps;
};


