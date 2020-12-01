#pragma once


// uncomment to use OTAA instead of ABP
//#define USE_OTAA 1


// UPDATE WITH YOUR DEVICE TTN SECRETS
#ifdef USE_OTAA
static PROGMEM u1_t DEVEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Device EUI, hex, lsb
static PROGMEM u1_t APPEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Application EUI, hex, lsb
static PROGMEM u1_t APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // App Key, hex, msb
#else
static PROGMEM u1_t NWKSKEY[16] = { 0x6C, 0x23, 0x53, 0x8B, 0x77, 0x3E, 0x79, 0xDF, 0x23, 0x7C, 0xF8, 0x54, 0xDE, 0x25, 0x84, 0x86 }; // LoRaWAN NwkSKey, network session key, hex, msb
static PROGMEM u1_t APPSKEY[16] = { 0xB2, 0xEB, 0xD0, 0xEF, 0x1C, 0x4E, 0x58, 0xE0, 0x18, 0x9A, 0x3D, 0x7D, 0x23, 0x7D, 0x87, 0x37 }; // LoRaWAN AppSKey, application session key, hex, msb
static PROGMEM u4_t DEVADDR = 0x260133F5 ; // LoRaWAN end-device address (DevAddr), hex, msb
#endif
