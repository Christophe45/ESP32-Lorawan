/*******************************************************************************
 * The Things Network - ABP Feather
 * 
 * Example of using an Adafruit Feather M0 and DHT22 with a
 * single-channel TheThingsNetwork gateway.
 * 
 * This uses ABP (Activation by Personalization), where session keys for
 * communication would be assigned/generated by TTN and hard-coded on the device.
 * 
 * Learn Guide: https://learn.adafruit.com/lora-pi
 * 
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 * Copyright (c) 2018 Brent Rubell, Adafruit Industries
 * 
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "HX711.h"
#include "soc/rtc.h"

//Deep sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
//#define TIME_TO_SLEEP 7200      /* Time ESP32 will go to sleep (in seconds) */
#define TIME_TO_SLEEP 30      /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;

//BME sensor
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// supply control
const int transistorPin = 16;

//capteur de poids
#define DOUT  25 //pour ESP32 wemos
#define CLK  26
float weight = 0;
float weightcorrection = 3.390;
float calibration_factor = 21650;  //You must change this factor depends on your scale,sensors and etc.

// Vbat is connected to GPIO 32 
const int batPin = 32;

// variable for storing the Vbat value
int batValue = 0;
float volt=0.00;
float vbat=0.00;


// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.

#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif


// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0xA3, 0x7F, 0xDD, 0x5D, 0x08, 0x31, 0xA3, 0x81, 0xD9, 0x3A, 0x66, 0xD2, 0x5A, 0xE9, 0x02, 0x48 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0xAB, 0x11, 0x9F, 0x5C, 0x20, 0xB8, 0x3D, 0x37, 0x46, 0x89, 0xB0, 0xA9, 0x21, 0x17, 0xE7, 0x34 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed.
#ifndef COMPILE_REGRESSION_TEST
static const u4_t DEVADDR = 0x260110A9;
#else
static const u4_t DEVADDR = 0;
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// payload to send to TTN gateway
static uint8_t payload[9];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping for Adafruit Feather M0 LoRa
const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 13,
           // LBT cal for the Adafruit Feather M0 LoRa, in dB
 //  .dio = {12, 14, 17}
     .dio = {27, 14, 17}
};


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
             digitalWrite (transistorPin, LOW);
             Deep_Sleep_Now();
           // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
       
        // read the temperature from the BME280
        float temp = bme.readTemperature();
        Serial.print("Temperature: "); Serial.print(temp);
        Serial.println(" *C");
        float humidity = bme.readHumidity();
        Serial.print("humidité: ");Serial.print(humidity);
        Serial.println("% ");

        // read weight from the HX711
        HX711 scale(DOUT, CLK);
        scale.set_scale(calibration_factor); //Adjust to this calibration factor
        Serial.print("Reading: ");
        weight = scale.get_units();
        weight = weight + weightcorrection;
        Serial.print(weight, 2);
        Serial.println(" kg"); // You can change this to other type of weighing value and re-adjust the calibration factor.
        Serial.print(" calibration_factor: ");
        Serial.println(calibration_factor);

        Serial.println("Boot number: " + String(bootCount));

         // Reading potentiometer value
         batValue = analogRead(batPin);
         volt = batValue;
         vbat=volt/732;
         Serial.print("Voltage = ");  
         Serial.println(vbat);

       
int16_t celciusInt = temp * 100; // convert to signed 16 bits integer: 0x0929
int16_t humidityInt = humidity * 100;
int16_t weightInt = weight * 100;
int16_t bootCountInt = bootCount * 100;
int16_t vbatInt = vbat * 100;

uint8_t buffer[10]; // reserve 2 bytes in memory

// Handle high byte (MSB) first; 0x09 for 23.45
// 0x0929 >> 8 shifts the 0x29 out of memory, leaving 0x0009
// 0x0009 does not fit in a single byte, so only 0x09 is stored in buffer[0]:
buffer[0] = celciusInt >> 8;
buffer[1] = celciusInt;
buffer[2] = humidityInt >> 8;
buffer[3] = humidityInt;
buffer[4] = weightInt >> 8;
buffer[5] = weightInt;
buffer[6] = bootCountInt >> 8;
buffer[7] = bootCountInt;
buffer[8] = vbatInt >> 8;
buffer[9] = vbatInt;


// Send on port 1, without asking for confirmation:
LMIC_setTxData2(1, buffer, sizeof(buffer), 0); // 0x0929 for 23.45
        Serial.println(F("Packet queued"));
    }
  
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    delay(4000);
    while (!Serial);
    Serial.begin(115200);
    pinMode (transistorPin, OUTPUT);
    digitalWrite (transistorPin, HIGH);
    delay(1000);
    Serial.println(F("Starting"));
    bme.begin(0x76);  
    ++bootCount;
       rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    
   #if defined(CFG_eu868) // EU channel setup
  int channel = 0;
   int dr = DR_SF7;
   for(int i=0; i<9; i++) { 
     if(i != channel) {
       LMIC_disableChannel(i);
     }
   }
   // Set data rate (SF) and transmit power for uplink
   LMIC_setDrTxpow(dr, 14);    // g-band
#elif defined(CFG_us915) // US channel setup
  // Instead of using selectSubBand, which will cycle through a sub-band of 8
  // channels. We'll configure the device to only use one frequency.
  // First disable all sub-bands
  for (int b = 0; b < 8; ++b) {
    LMIC_disableSubBand(b);
  }
  // Then enable the channel(s) you want to use
  //LMIC_enableChannel(8); // 903.9 MHz
  LMIC_enableChannel(17);
#endif
    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
  os_runloop_once();  
}

void Deep_Sleep_Now() // New function - moded code out of void loop so that the sleep function can be called if we fail to connect to Wi-Fi or Blynk
{
  esp_sleep_enable_timer_wakeup((uint64_t)(TIME_TO_SLEEP) * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");
  Serial.println("Going to sleep now");
  Serial.flush(); 
   esp_deep_sleep_start();
  
  delay(2000);
}
