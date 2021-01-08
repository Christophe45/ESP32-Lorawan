// 1-channel LoRa Gateway for ESP8266
// Copyright (c) 2016, 2017, 2018 Maarten Westenberg version for ESP8266
// Version 5.3.3
// Date: 2018-08-25
// Author: Maarten Westenberg (mw12554@hotmail.com)
//
// Based on work done by Thomas Telkamp for Raspberry PI 1-ch gateway and many others.
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// NO WARRANTY OF ANY KIND IS PROVIDED
//
// The protocols and specifications used for this 1ch gateway: 
// 1. LoRA Specification version V1.0 and V1.1 for Gateway-Node communication
//	
// 2. Semtech Basic communication protocol between Lora gateway and server version 3.0.0
//	https://github.com/Lora-net/packet_forwarder/blob/master/PROTOCOL.TXT
//
// Notes: 
// - Once call gethostbyname() to get IP for services, after that only use IP
//	 addresses (too many gethost name makes the ESP unstable)
// - Only call yield() in main stream (not for background NTP sync). 
//
// ----------------------------------------------------------------------------------------

#include "ESP-sc-gway.h"						// This file contains configuration of GWay

#if defined (ARDUINO_ARCH_ESP32) || defined(ESP32)
#define ESP32_ARCH 1
#endif

#include <Esp.h>								// ESP8266 specific IDE functions
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>
#include <string>								// C++ specific string functions

#include <SPI.h>								// For the RFM95 bus
#include <TimeLib.h>							// http://playground.arduino.cc/code/time
#include <DNSServer.h>							// Local DNSserver
#include <ArduinoJson.h>
#include <FS.h>									// ESP8266 Specific
#include <WiFiUdp.h>
#include <pins_arduino.h>
#include <gBase64.h>							// https://github.com/adamvr/arduino-base64 (changed the name)

// Local include files
#include "loraModem.h"
#include "loraFiles.h"
#include "sensor.h"
#include "oLED.h"

extern "C" {
#include "lwip/err.h"
#include "lwip/dns.h"
}

#if WIFIMANAGER==1
#include <WiFiManager.h>						// Library for ESP WiFi config through an AP
#endif

#if (GATEWAYNODE==1) || (_LOCALSERVER==1)
#include "AES-128_V10.h"
#endif


// ----------- Specific ESP32 stuff --------------
#if ESP32_ARCH==1								// IF ESP32

#include "WiFi.h"
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#if A_SERVER==1
#include <ESP32WebServer.h>						// Dedicated Webserver for ESP32
#include <Streaming.h>          				// http://arduiniana.org/libraries/streaming/
#endif
#if A_OTA==1
#include <ESP32httpUpdate.h>					// Not yet available
#include <ArduinoOTA.h>
#endif//OTA

// ----------- Generic ESP8266 stuff --------------
#else

#include <ESP8266WiFi.h>						// Which is specific for ESP8266
#include <ESP8266mDNS.h>
extern "C" {
#include "user_interface.h"
#include "c_types.h"
}
#if A_SERVER==1
#include <ESP8266WebServer.h>
#include <Streaming.h>          				// http://arduiniana.org/libraries/streaming/
#endif //A_SERVER
#if A_OTA==1
#include <ESP8266httpUpdate.h>
#include <ArduinoOTA.h>
#endif//OTA

#endif//ESP_ARCH

// ----------- Declaration of vars --------------
uint8_t debug=1;								// Debug level! 0 is no msgs, 1 normal, 2 extensive
uint8_t pdebug=0xFF;							// Allow all atterns (departments)

#if GATEWAYNODE==1
#if _GPS==1
#include <TinyGPS++.h>
TinyGPSPlus gps;
HardwareSerial Serial1(1);
#endif
#endif

// You can switch webserver off if not necessary but probably better to leave it in.
#if A_SERVER==1
#if ESP32_ARCH==1
	ESP32WebServer server(A_SERVERPORT);
#else
	ESP8266WebServer server(A_SERVERPORT);
#endif
#endif
using namespace std;

byte currentMode = 0x81;

bool sx1272 = true;								// Actually we use sx1276/RFM95

uint32_t cp_nb_rx_rcv;							// Number of messages received by gateway
uint32_t cp_nb_rx_ok;							// Number of messages received OK
uint32_t cp_nb_rx_bad;							// Number of messages received bad
uint32_t cp_nb_rx_nocrc;						// Number of messages without CRC
uint32_t cp_up_pkt_fwd;

uint8_t MAC_array[6];

// ----------------------------------------------------------------------------
//
// Configure these values only if necessary!
//
// ----------------------------------------------------------------------------

// Set spreading factor (SF7 - SF12)
sf_t sf 			= _SPREADING;
sf_t sfi 			= _SPREADING;				// Initial value of SF

// Set location, description and other configuration parameters
// Defined in ESP-sc_gway.h
//
float lat			= _LAT;						// Configuration specific info...
float lon			= _LON;
int   alt			= _ALT;
char platform[24]	= _PLATFORM; 				// platform definition
char email[40]		= _EMAIL;    				// used for contact email
char description[64]= _DESCRIPTION;				// used for free form description 

// define servers

IPAddress ntpServer;							// IP address of NTP_TIMESERVER
IPAddress ttnServer;							// IP Address of thethingsnetwork server
IPAddress thingServer;

WiFiUDP Udp;

time_t startTime = 0;							// The time in seconds since 1970 that the server started
												// be aware that UTP time has to succeed for meaningful values.
												// We use this variable since millis() is reset every 50 days...
uint32_t eventTime = 0;							// Timing of _event to change value (or not).
uint32_t sendTime = 0;							// Time that the last message transmitted
uint32_t doneTime = 0;							// Time to expire when CDDONE takes too long
uint32_t statTime = 0;							// last time we sent a stat message to server
uint32_t pulltime = 0;							// last time we sent a pull_data request to server
//uint32_t lastTmst = 0;							// Last activity Timer

#if A_SERVER==1
uint32_t wwwtime = 0;
#endif
#if NTP_INTR==0
uint32_t ntptimer = 0;
#endif

#define TX_BUFF_SIZE  1024						// Upstream buffer to send to MQTT
#define RX_BUFF_SIZE  1024						// Downstream received from MQTT
#define STATUS_SIZE	  512						// Should(!) be enough based on the static text .. was 1024

#if GATEWAYNODE==1
uint16_t frameCount=0;							// We write this to SPIFF file
#endif

// volatile bool inSPI This initial value of mutex is to be free,
// which means that its value is 1 (!)
// 
int mutexSPI = 1;

// ----------------------------------------------------------------------------
// FORWARD DECARATIONS
// These forward declarations are done since other .ino fils are linked by the
// compiler/linker AFTER the main ESP-sc-gway.ino file. 
// And espcesially when calling functions with ICACHE_RAM_ATTR the complier 
// does not want this.
// Solution can also be to pecify less STRICT compile options in Makefile
// ----------------------------------------------------------------------------

void ICACHE_RAM_ATTR Interrupt_0();
void ICACHE_RAM_ATTR Interrupt_1();

int sendPacket(uint8_t *buf, uint8_t length);		// _txRx.ino
void setupWWW();									// _wwwServer.ino
void SerialTime();									// _utils.ino
static void printIP(IPAddress ipa, const char sep, String& response);	// _wwwServer.ino

void init_oLED();									// oLED.ino
void acti_oLED();
void addr_oLED();

void setupOta(char *hostname);
void initLoraModem();								// _loraModem.ino
void cadScanner();
void rxLoraModem();									// _loraModem.ino
void writeRegister(uint8_t addr, uint8_t value);	// _loraModem.ino

void stateMachine();								// _stateMachine.ino

void SerialStat(uint8_t intr);						// _utils.ino

#if MUTEX==1
// Forward declarations
void ICACHE_FLASH_ATTR CreateMutux(int *mutex);
bool ICACHE_FLASH_ATTR GetMutex(int *mutex);
void ICACHE_FLASH_ATTR ReleaseMutex(int *mutex);
#endif

// ----------------------------------------------------------------------------
// DIE is not use actively in the source code anymore.
// It is replaced by a Serial.print command so we know that we have a problem
// somewhere.
// There are at least 3 other ways to restart the ESP. Pick one if you want.
// ----------------------------------------------------------------------------
void die(const char *s)
{
	Serial.println(s);
	if (debug>=2) Serial.flush();

	delay(50);
	// system_restart();						// SDK function
	// ESP.reset();				
	abort();									// Within a second
}

// ----------------------------------------------------------------------------
// gway_failed is a function called by ASSERT in ESP-sc-gway.h
//
// ----------------------------------------------------------------------------
void gway_failed(const char *file, uint16_t line) {
	Serial.print(F("Program failed in file: "));
	Serial.print(file);
	Serial.print(F(", line: "));
	Serial.println(line);
	if (debug>=2) Serial.flush();
}

// ----------------------------------------------------------------------------
// Print leading '0' digits for hours(0) and second(0) when
// printing values less than 10
// ----------------------------------------------------------------------------
void printDigits(unsigned long digits)
{
    // utility function for digital clock display: prints leading 0
    if(digits < 10)
        Serial.print(F("0"));
    Serial.print(digits);
}

// ----------------------------------------------------------------------------
// Print utin8_t values in HEX with leading 0 when necessary
// ----------------------------------------------------------------------------
void printHexDigit(uint8_t digit)
{
    // utility function for printing Hex Values with leading 0
    if(digit < 0x10)
        Serial.print('0');
    Serial.print(digit,HEX);
}


// ----------------------------------------------------------------------------
// Print the current time
// ----------------------------------------------------------------------------
static void printTime() {
	switch (weekday())
	{
		case 1: Serial.print(F("Sunday")); break;
		case 2: Serial.print(F("Monday")); break;
		case 3: Serial.print(F("Tuesday")); break;
		case 4: Serial.print(F("Wednesday")); break;
		case 5: Serial.print(F("Thursday")); break;
		case 6: Serial.print(F("Friday")); break;
		case 7: Serial.print(F("Saturday")); break;
		default: Serial.print(F("ERROR")); break;
	}
	Serial.print(F(" "));
	printDigits(hour());
	Serial.print(F(":"));
	printDigits(minute());
	Serial.print(F(":"));
	printDigits(second());
	return;
}


// ----------------------------------------------------------------------------
// Convert a float to string for printing
// Parameters:
//	f is float value to convert
//	p is precision in decimal digits
//	val is character array for results
// ----------------------------------------------------------------------------
void ftoa(float f, char *val, int p) {
	int j=1;
	int ival, fval;
	char b[7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	
	for (int i=0; i< p; i++) { j= j*10; }

	ival = (int) f;								// Make integer part
	fval = (int) ((f- ival)*j);					// Make fraction. Has same sign as integer part
	if (fval<0) fval = -fval;					// So if it is negative make fraction positive again.
												// sprintf does NOT fit in memory
	if ((f<0) && (ival == 0)) strcat(val, "-");
	strcat(val,itoa(ival,b,10));				// Copy integer part first, base 10, null terminated
	strcat(val,".");							// Copy decimal point
	
	itoa(fval,b,10);							// Copy fraction part base 10
	for (int i=0; i<(p-strlen(b)); i++) {
		strcat(val,"0"); 						// first number of 0 of faction?
	}
	
	// Fraction can be anything from 0 to 10^p , so can have less digits
	strcat(val,b);
}

// ============================================================================
// NTP TIME functions



// ----------------------------------------------------------------------------
// Send the request packet to the NTP server.
//
// ----------------------------------------------------------------------------
int sendNtpRequest(IPAddress timeServerIP) {
	const int NTP_PACKET_SIZE = 48;				// Fixed size of NTP record
	byte packetBuffer[NTP_PACKET_SIZE];

	memset(packetBuffer, 0, NTP_PACKET_SIZE);	// Zeroise the buffer.
	
	packetBuffer[0]  = 0b11100011;   			// LI, Version, Mode
	packetBuffer[1]  = 0;						// Stratum, or type of clock
	packetBuffer[2]  = 6;						// Polling Interval
	packetBuffer[3]  = 0xEC;					// Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12] = 49;
	packetBuffer[13] = 0x4E;
	packetBuffer[14] = 49;
	packetBuffer[15] = 52;	

	
	if (!sendUdp( (IPAddress) timeServerIP, (int) 123, packetBuffer, NTP_PACKET_SIZE)) {
		gwayConfig.ntpErr++;
		gwayConfig.ntpErrTime = now();
		return(0);	
	}
	return(1);
}


// ----------------------------------------------------------------------------
// Get the NTP time from one of the time servers
// Note: As this function is called from SyncINterval in the background
//	make sure we have no blocking calls in this function
// ----------------------------------------------------------------------------
time_t getNtpTime()
{
	gwayConfig.ntps++;
	
    if (!sendNtpRequest(ntpServer))					// Send the request for new time
	{
		if (( debug>=0 ) && ( pdebug & P_MAIN ))
			Serial.println(F("M sendNtpRequest failed"));
		return(0);
	}
	
	const int NTP_PACKET_SIZE = 48;					// Fixed size of NTP record
	byte packetBuffer[NTP_PACKET_SIZE];
	memset(packetBuffer, 0, NTP_PACKET_SIZE);		// Set buffer cntents to zero

    uint32_t beginWait = millis();
	delay(10);
    while (millis() - beginWait < 1500) 
	{
		int size = Udp.parsePacket();
		if ( size >= NTP_PACKET_SIZE ) {
		
			if (Udp.read(packetBuffer, NTP_PACKET_SIZE) < NTP_PACKET_SIZE) {
				break;
			}
			else {
				// Extract seconds portion.
				unsigned long secs;
				secs  = packetBuffer[40] << 24;
				secs |= packetBuffer[41] << 16;
				secs |= packetBuffer[42] <<  8;
				secs |= packetBuffer[43];
				// UTC is 1 TimeZone correction when no daylight saving time
				return(secs - 2208988800UL + NTP_TIMEZONES * SECS_IN_HOUR);
			}
			Udp.flush();	
		}
		delay(100);									// Wait 100 millisecs, allow kernel to act when necessary
    }

	Udp.flush();
	
	// If we are here, we could not read the time from internet
	// So increase the counter
	gwayConfig.ntpErr++;
	gwayConfig.ntpErrTime = now();
#if DUSB>=1
	if (( debug>=0 ) && ( pdebug & P_MAIN )) {
		Serial.println(F("M getNtpTime:: read failed"));
	}
#endif
	return(0); 										// return 0 if unable to get the time
}

// ----------------------------------------------------------------------------
// Set up regular synchronization of NTP server and the local time.
// ----------------------------------------------------------------------------
#if NTP_INTR==1
void setupTime() {
  setSyncProvider(getNtpTime);
  setSyncInterval(_NTP_INTERVAL);
}
#endif


// ============================================================================
// UDP  FUNCTIONS


// ----------------------------------------------------------------------------
// Read DOWN a package from UDP socket, can come from any server
// Messages are received when server responds to gateway requests from LoRa nodes 
// (e.g. JOIN requests etc.) or when server has downstream data.
// We respond only to the server that sent us a message!
// Note: So normally we can forget here about codes that do upstream
// Parameters:
//	Packetsize: size of the buffer to read, as read by loop() calling function
//
// Returns:
//	-1 or false if not read
//	Or number of characters read is success
//
// ----------------------------------------------------------------------------
int readUdp(int packetSize)
{
	uint8_t protocol;
	uint16_t token;
	uint8_t ident; 
	uint8_t buff[32]; 						// General buffer to use for UDP, set to 64
	uint8_t buff_down[RX_BUFF_SIZE];		// Buffer for downstream

//	if ((WiFi.status() != WL_CONNECTED) &&& (WlanConnect(10) < 0)) {
	if (WlanConnect(10) < 0) {
#if DUSB>=1
			Serial.print(F("readdUdp: ERROR connecting to WLAN"));
			if (debug>=2) Serial.flush();
#endif
			Udp.flush();
			yield();
			return(-1);
	}

	yield();
	
	if (packetSize > RX_BUFF_SIZE) {
#if DUSB>=1
		Serial.print(F("readUDP:: ERROR package of size: "));
		Serial.println(packetSize);
#endif
		Udp.flush();
		return(-1);
	}
  
	// We assume here that we know the originator of the message
	// In practice however this can be any sender!
	if (Udp.read(buff_down, packetSize) < packetSize) {
#if DUSB>=1
		Serial.println(F("A readUsb:: Reading less chars"));
		return(-1);
#endif
	}

	// Remote Address should be known
	IPAddress remoteIpNo = Udp.remoteIP();

	// Remote port is either of the remote TTN server or from NTP server (=123)
	unsigned int remotePortNo = Udp.remotePort();

	if (remotePortNo == 123) {
		// This is an NTP message arriving
#if DUSB>=1
		if ( debug>=0 ) {
			Serial.println(F("A readUdp:: NTP msg rcvd"));
		}
#endif
		gwayConfig.ntpErr++;
		gwayConfig.ntpErrTime = now();
		return(0);
	}
	
	// If it is not NTP it must be a LoRa message for gateway or node
	else {
		uint8_t *data = (uint8_t *) ((uint8_t *)buff_down + 4);
		protocol = buff_down[0];
		token = buff_down[2]*256 + buff_down[1];
		ident = buff_down[3];

#if DUSB>=1
	if ((debug>1) && (pdebug & P_MAIN)) {
		Serial.print(F("M readUdp:: message waiting="));
		Serial.print(ident);
		Serial.println();
	}
#endif
		// now parse the message type from the server (if any)
		switch (ident) {

		// This message is used by the gateway to send sensor data to the
		// server. As this function is used for downstream only, this option
		// will never be selected but is included as a reference only
		case PKT_PUSH_DATA: // 0x00 UP
#if DUSB>=1
			if (debug >=1) {
				Serial.print(F("PKT_PUSH_DATA:: size ")); Serial.print(packetSize);
				Serial.print(F(" From ")); Serial.print(remoteIpNo);
				Serial.print(F(", port ")); Serial.print(remotePortNo);
				Serial.print(F(", data: "));
				for (int i=0; i<packetSize; i++) {
					Serial.print(buff_down[i],HEX);
					Serial.print(':');
				}
				Serial.println();
				if (debug>=2) Serial.flush();
			}
#endif
		break;
	
		// This message is sent by the server to acknoledge receipt of a
		// (sensor) message sent with the code above.
		case PKT_PUSH_ACK:	// 0x01 DOWN
#if DUSB>=1
			if (( debug>=2) && (pdebug & P_MAIN )) {
				Serial.print(F("M PKT_PUSH_ACK:: size ")); 
				Serial.print(packetSize);
				Serial.print(F(" From ")); 
				Serial.print(remoteIpNo);
				Serial.print(F(", port ")); 
				Serial.print(remotePortNo);
				Serial.print(F(", token: "));
				Serial.println(token, HEX);
				Serial.println();
			}
#endif
		break;
	
		case PKT_PULL_DATA:	// 0x02 UP
#if DUSB>=1
			Serial.print(F(" Pull Data"));
			Serial.println();
#endif
		break;
	
		// This message type is used to confirm OTAA message to the node
		// XXX This message format may also be used for other downstream communucation
		case PKT_PULL_RESP:	// 0x03 DOWN
#if DUSB>=1
			if (( debug>=0 ) && ( pdebug & P_MAIN )) {
				Serial.println(F("M readUdp:: PKT_PULL_RESP received"));
			}
#endif
//			lastTmst = micros();					// Store the tmst this package was received
			
			// Send to the LoRa Node first (timing) and then do reporting to Serial
			_state=S_TX;
			sendTime = micros();					// record when we started sending the message
			
			if (sendPacket(data, packetSize-4) < 0) {
#if DUSB>=1
				if ( debug>=0 ) {
					Serial.println(F("A readUdp:: Error: PKT_PULL_RESP sendPacket failed"));
				}
#endif
				return(-1);
			}

			// Now respond with an PKT_TX_ACK; 0x04 UP
			buff[0]=buff_down[0];
			buff[1]=buff_down[1];
			buff[2]=buff_down[2];
			//buff[3]=PKT_PULL_ACK;					// Pull request/Change of Mogyi
			buff[3]=PKT_TX_ACK;
			buff[4]=MAC_array[0];
			buff[5]=MAC_array[1];
			buff[6]=MAC_array[2];
			buff[7]=0xFF;
			buff[8]=0xFF;
			buff[9]=MAC_array[3];
			buff[10]=MAC_array[4];
			buff[11]=MAC_array[5];
			buff[12]=0;
#if DUSB>=1
			if (( debug >= 2 ) && ( pdebug & P_MAIN )) {
				Serial.println(F("M readUdp:: TX buff filled"));
			}
#endif
			// Only send the PKT_PULL_ACK to the UDP socket that just sent the data!!!
			Udp.beginPacket(remoteIpNo, remotePortNo);
			if (Udp.write((unsigned char *)buff, 12) != 12) {
#if DUSB>=1
				if (debug>=0)
					Serial.println("A readUdp:: Error: PKT_PULL_ACK UDP write");
#endif
			}
			else {
#if DUSB>=1
				if (( debug>=0 ) && ( pdebug & P_TX )) {
					Serial.print(F("M PKT_TX_ACK:: micros="));
					Serial.println(micros());
				}
#endif
			}

			if (!Udp.endPacket()) {
#if DUSB>=1
				if (( debug>=0 ) && ( pdebug & P_MAIN )) {
					Serial.println(F("M PKT_PULL_DATALL Error Udp.endpaket"));
				}
#endif
			}
			
			yield();
#if DUSB>=1
			if (( debug >=1 ) && (pdebug & P_MAIN )) {
				Serial.print(F("M PKT_PULL_RESP:: size ")); 
				Serial.print(packetSize);
				Serial.print(F(" From ")); 
				Serial.print(remoteIpNo);
				Serial.print(F(", port ")); 
				Serial.print(remotePortNo);	
				Serial.print(F(", data: "));
				data = buff_down + 4;
				data[packetSize] = 0;
				Serial.print((char *)data);
				Serial.println(F("..."));
			}
#endif		
		break;
	
		case PKT_PULL_ACK:	// 0x04 DOWN; the server sends a PULL_ACK to confirm PULL_DATA receipt
#if DUSB>=1
			if (( debug >= 2 ) && (pdebug & P_MAIN )) {
				Serial.print(F("M PKT_PULL_ACK:: size ")); Serial.print(packetSize);
				Serial.print(F(" From ")); Serial.print(remoteIpNo);
				Serial.print(F(", port ")); Serial.print(remotePortNo);	
				Serial.print(F(", data: "));
				for (int i=0; i<packetSize; i++) {
					Serial.print(buff_down[i],HEX);
					Serial.print(':');
				}
				Serial.println();
			}
#endif
		break;
	
		default:
#if GATEWAYMGT==1
			// For simplicity, we send the first 4 bytes too
			gateway_mgt(packetSize, buff_down);
#else

#endif
#if DUSB>=1
			Serial.print(F(", ERROR ident not recognized="));
			Serial.println(ident);
#endif
		break;
		}
#if DUSB>=2
		if (debug>=1) {
			Serial.print(F("readUdp:: returning=")); 
			Serial.println(packetSize);
		}
#endif
		// For downstream messages
		return packetSize;
	}
}//readUdp


// ----------------------------------------------------------------------------
// Send UP an UDP/DGRAM message to the MQTT server
// If we send to more than one host (not sure why) then we need to set sockaddr 
// before sending.
// Parameters:
//	IPAddress
//	port
//	msg *
//	length (of msg)
// return values:
//	0: Error
//	1: Success
// ----------------------------------------------------------------------------
int sendUdp(IPAddress server, int port, uint8_t *msg, int length) {

	// Check whether we are conected to Wifi and the internet
	if (WlanConnect(3) < 0) {
#if DUSB>=1
		if (( debug>=0 ) && ( pdebug & P_MAIN )) {
			Serial.print(F("M sendUdp: ERROR connecting to WiFi"));
			Serial.flush();
		}
#endif
		Udp.flush();
		yield();
		return(0);
	}

	yield();

	//send the update
#if DUSB>=1
	if (( debug>=3 ) && ( pdebug & P_MAIN )) {
		Serial.println(F("M WiFi connected"));
	}
#endif	
	if (!Udp.beginPacket(server, (int) port)) {
#if DUSB>=1
		if (( debug>=1 ) && ( pdebug & P_MAIN )) {
			Serial.println(F("M sendUdp:: Error Udp.beginPacket"));
		}
#endif
		return(0);
	}
	
	yield();
	

	if (Udp.write((unsigned char *)msg, length) != length) {
#if DUSB>=1
		if (( debug<=1 ) && ( pdebug & P_MAIN )) {
			Serial.println(F("M sendUdp:: Error write"));
		}
#endif
		Udp.endPacket();						// Close UDP
		return(0);								// Return error
	}
	
	yield();
	
	if (!Udp.endPacket()) {
#if DUSB>=1
		if (debug>=1) {
			Serial.println(F("sendUdp:: Error Udp.endPacket"));
			Serial.flush();
		}
#endif
		return(0);
	}
	return(1);
}//sendUDP

// ----------------------------------------------------------------------------
// UDPconnect(): connect to UDP (which is a local thing, after all UDP 
// connections do not exist.
// Parameters:
//	<None>
// Returns
//	Boollean indicating success or not
// ----------------------------------------------------------------------------
bool UDPconnect() {

	bool ret = false;
	unsigned int localPort = _LOCUDPPORT;			// To listen to return messages from WiFi
#if DUSB>=1
	if (debug>=1) {
		Serial.print(F("Local UDP port="));
		Serial.println(localPort);
	}
#endif	
	if (Udp.begin(localPort) == 1) {
#if DUSB>=1
		if (debug>=1) Serial.println(F("Connection successful"));
#endif
		ret = true;
	}
	else{
#if DUSB>=1
		if (debug>=1) Serial.println("Connection failed");
#endif
	}
	return(ret);
}//udpConnect


// ----------------------------------------------------------------------------
// Send UP periodic Pull_DATA message to server to keepalive the connection
// and to invite the server to send downstream messages when these are available
// *2, par. 5.2
//	- Protocol Version (1 byte)
//	- Random Token (2 bytes)
//	- PULL_DATA identifier (1 byte) = 0x02
//	- Gateway unique identifier (8 bytes) = MAC address
// ----------------------------------------------------------------------------
void pullData() {

    uint8_t pullDataReq[12]; 								// status report as a JSON object
    int pullIndex=0;
	int i;
	
	uint8_t token_h = (uint8_t)rand(); 						// random token
    uint8_t token_l = (uint8_t)rand();						// random token
	
    // pre-fill the data buffer with fixed fields
    pullDataReq[0]  = PROTOCOL_VERSION;						// 0x01
    pullDataReq[1]  = token_h;
    pullDataReq[2]  = token_l;
    pullDataReq[3]  = PKT_PULL_DATA;						// 0x02
	// READ MAC ADDRESS OF ESP8266, and return unique Gateway ID consisting of MAC address and 2bytes 0xFF
    pullDataReq[4]  = MAC_array[0];
    pullDataReq[5]  = MAC_array[1];
    pullDataReq[6]  = MAC_array[2];
    pullDataReq[7]  = 0xFF;
    pullDataReq[8]  = 0xFF;
    pullDataReq[9]  = MAC_array[3];
    pullDataReq[10] = MAC_array[4];
    pullDataReq[11] = MAC_array[5];
    //pullDataReq[12] = 0/00; 								// add string terminator, for safety
	
    pullIndex = 12;											// 12-byte header
	
    //send the update
	
	uint8_t *pullPtr;
	pullPtr = pullDataReq,
#ifdef _TTNSERVER
    sendUdp(ttnServer, _TTNPORT, pullDataReq, pullIndex);
	yield();
#endif

#if DUSB>=1
	if (pullPtr != pullDataReq) {
		Serial.println(F("pullPtr != pullDatReq"));
		Serial.flush();
	}

#endif
#ifdef _THINGSERVER
	sendUdp(thingServer, _THINGPORT, pullDataReq, pullIndex);
#endif

#if DUSB>=1
    if (( debug>=2 ) && ( pdebug & P_MAIN )) {
		yield();
		Serial.print(F("M PKT_PULL_DATA request, len=<"));
		Serial.print(pullIndex);
		Serial.print(F("> "));
		for (i=0; i<pullIndex; i++) {
			Serial.print(pullDataReq[i],HEX);				// debug: display JSON stat
			Serial.print(':');
		}
		Serial.println();
		if (debug>=2) Serial.flush();
	}
#endif
	return;
}//pullData


// ----------------------------------------------------------------------------
// Send UP periodic status message to server even when we do not receive any
// data. 
// Parameters:
//	- <none>
// ----------------------------------------------------------------------------
void sendstat() {

    uint8_t status_report[STATUS_SIZE]; 					// status report as a JSON object
    char stat_timestamp[32];								// XXX was 24
    time_t t;
	char clat[10]={0};
	char clon[10]={0};

    int stat_index=0;
	uint8_t token_h   = (uint8_t)rand(); 					// random token
    uint8_t token_l   = (uint8_t)rand();					// random token
	
    // pre-fill the data buffer with fixed fields
    status_report[0]  = PROTOCOL_VERSION;					// 0x01
	status_report[1]  = token_h;
    status_report[2]  = token_l;
    status_report[3]  = PKT_PUSH_DATA;						// 0x00
	
	// READ MAC ADDRESS OF ESP8266, and return unique Gateway ID consisting of MAC address and 2bytes 0xFF
    status_report[4]  = MAC_array[0];
    status_report[5]  = MAC_array[1];
    status_report[6]  = MAC_array[2];
    status_report[7]  = 0xFF;
    status_report[8]  = 0xFF;
    status_report[9]  = MAC_array[3];
    status_report[10] = MAC_array[4];
    status_report[11] = MAC_array[5];

    stat_index = 12;										// 12-byte header
	
    t = now();												// get timestamp for statistics
	
	// XXX Using CET as the current timezone. Change to your timezone	
	sprintf(stat_timestamp, "%04d-%02d-%02d %02d:%02d:%02d CET", year(),month(),day(),hour(),minute(),second());
	yield();
	
	ftoa(lat,clat,5);										// Convert lat to char array with 5 decimals
	ftoa(lon,clon,5);										// As IDE CANNOT prints floats
	
	// Build the Status message in JSON format, XXX Split this one up...
	delay(1);
	
    int j = snprintf((char *)(status_report + stat_index), STATUS_SIZE-stat_index, 
		"{\"stat\":{\"time\":\"%s\",\"lati\":%s,\"long\":%s,\"alti\":%i,\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%u.0,\"dwnb\":%u,\"txnb\":%u,\"pfrm\":\"%s\",\"mail\":\"%s\",\"desc\":\"%s\"}}", 
		stat_timestamp, clat, clon, (int)alt, cp_nb_rx_rcv, cp_nb_rx_ok, cp_up_pkt_fwd, 0, 0, 0, platform, email, description);
		
	yield();												// Give way to the internal housekeeping of the ESP8266

    stat_index += j;
    status_report[stat_index] = 0; 							// add string terminator, for safety

#if DUSB>=1
    if (( debug>=2 ) && ( pdebug & P_MAIN )) {
		Serial.print(F("M stat update: <"));
		Serial.print(stat_index);
		Serial.print(F("> "));
		Serial.println((char *)(status_report+12));			// DEBUG: display JSON stat
	}
#endif	
	if (stat_index > STATUS_SIZE) {
#if DUSB>=1
		Serial.println(F("A sendstat:: ERROR buffer too big"));
#endif
		return;
	}
	
    //send the update
#ifdef _TTNSERVER
    sendUdp(ttnServer, _TTNPORT, status_report, stat_index);
	yield();
#endif

#ifdef _THINGSERVER
	sendUdp(thingServer, _THINGPORT, status_report, stat_index);
#endif
	return;
}//sendstat



// ============================================================================
// MAIN PROGRAM CODE (SETUP AND LOOP)


// ----------------------------------------------------------------------------
// Setup code (one time)
// _state is S_INIT
// ----------------------------------------------------------------------------
void setup() {

	char MAC_char[19];								// XXX Unbelievable
	MAC_char[18] = 0;

	Serial.begin(_BAUDRATE);						// As fast as possible for bus
	delay(100);	

#if _GPS==1
	// Pins are define in LoRaModem.h together with other pins
	Serial1.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);// PIN 12-TX 15-RX
#endif

#ifdef ESP32
#if DUSB>=1
	Serial.print(F("ESP32 defined, freq="));
#if _LFREQ==433
	Serial.print(freqs[0]);
	Serial.print(F(" EU433"));
#elif _LFREQ==868
	Serial.print(freqs[0]);
	Serial.print(F(" EU868"));
#endif
	Serial.println();
#endif
#endif
#ifdef ARDUINO_ARCH_ESP32
#if DUSB>=1
	Serial.println(F("ARDUINO_ARCH_ESP32 defined"));
#endif
#endif


#if DUSB>=1
	Serial.flush();

	delay(500);

	if (SPIFFS.begin()) {
		Serial.println(F("SPIFFS init success"));
	}
	else {
	}
#endif	
#if _SPIFF_FORMAT>=1
#if DUSB>=1
	if (( debug >= 0 ) && ( pdebug & P_MAIN )) {
		Serial.println(F("M Format Filesystem ... "));
	}
#endif
	SPIFFS.format();								// Normally disabled. Enable only when SPIFFS corrupt
#if DUSB>=1
	if (( debug >= 0 ) && ( pdebug & P_MAIN )) {
		Serial.println(F("Done"));
	}
#endif
#endif

	Serial.print(F("Assert="));
#if defined CFG_noassert
	Serial.println(F("No Asserts"));
#else
	Serial.println(F("Do Asserts"));
#endif

#if OLED>=1
	init_oLED();
#endif

	delay(500);
	yield();
#if DUSB>=1	
	if (debug>=1) {
		Serial.print(F("debug=")); 
		Serial.println(debug);
		yield();
	}
#endif

	WiFi.mode(WIFI_STA);
	WiFi.setAutoConnect(true);
	//WiFi.begin();
	
	WlanReadWpa();								// Read the last Wifi settings from SPIFFS into memory

	WiFi.macAddress(MAC_array);
	
    sprintf(MAC_char,"%02x:%02x:%02x:%02x:%02x:%02x",
		MAC_array[0],MAC_array[1],MAC_array[2],MAC_array[3],MAC_array[4],MAC_array[5]);
	Serial.print("MAC: ");
    Serial.print(MAC_char);
	Serial.print(F(", len="));
	Serial.println(strlen(MAC_char));

	// We start by connecting to a WiFi network, set hostname
	char hostname[12];

	// Setup WiFi UDP connection. Give it some time and retry x times..
	while (WlanConnect(0) <= 0) {
		Serial.print(F("Error Wifi network connect "));
		Serial.println();
		yield();
	}	
	
	// After there is a WiFi router connection, we can also set the hostname.
#if ESP32_ARCH==1
	sprintf(hostname, "%s%02x%02x%02x", "esp32-", MAC_array[3], MAC_array[4], MAC_array[5]);
	WiFi.setHostname( hostname );
#else
	sprintf(hostname, "%s%02x%02x%02x", "esp8266-", MAC_array[3], MAC_array[4], MAC_array[5]);
	wifi_station_set_hostname( hostname );
#endif	

	
	Serial.print(F("Host "));
#if ESP32_ARCH==1
	Serial.print(WiFi.getHostname());
#else
	Serial.print(wifi_station_get_hostname());
#endif
	Serial.print(F(" WiFi Connected to "));
	Serial.print(WiFi.SSID());
	Serial.print(F(" on IP="));
	Serial.print(WiFi.localIP());
	Serial.println();
	delay(200);
	
	// If we are here we are connected to WLAN
	// So now test the UDP function
	if (!UDPconnect()) {
		Serial.println(F("Error UDPconnect"));
	}
	delay(200);
	
	// Pins are defined and set in loraModem.h
    pinMode(pins.ss, OUTPUT);
	pinMode(pins.rst, OUTPUT);
    pinMode(pins.dio0, INPUT);								// This pin is interrupt
	pinMode(pins.dio1, INPUT);								// This pin is interrupt
	//pinMode(pins.dio2, INPUT);

	// Init the SPI pins
#if ESP32_ARCH==1
	SPI.begin(SCK, MISO, MOSI, SS);
#else
	SPI.begin();
#endif

	delay(500);
	
	// We choose the Gateway ID to be the Ethernet Address of our Gateway card
    // display results of getting hardware address
	// 
    Serial.print("Gateway ID: ");
	printHexDigit(MAC_array[0]);
    printHexDigit(MAC_array[1]);
    printHexDigit(MAC_array[2]);
	printHexDigit(0xFF);
	printHexDigit(0xFF);
    printHexDigit(MAC_array[3]);
    printHexDigit(MAC_array[4]);
    printHexDigit(MAC_array[5]);

    Serial.print(", Listening at SF");
	Serial.print(sf);
	Serial.print(" on ");
	Serial.print((double)freq/1000000);
	Serial.println(" Mhz.");

	if (!WiFi.hostByName(NTP_TIMESERVER, ntpServer))		// Get IP address of Timeserver
	{
		die("Setup:: ERROR hostByName NTP");
	};
	delay(100);
#ifdef _TTNSERVER
	if (!WiFi.hostByName(_TTNSERVER, ttnServer))			// Use DNS to get server IP once
	{
		die("Setup:: ERROR hostByName TTN");
	};
	delay(100);
#endif
#ifdef _THINGSERVER
	if (!WiFi.hostByName(_THINGSERVER, thingServer))
	{
		die("Setup:: ERROR hostByName THING");
	}
	delay(100);
#endif

	// The Over the AIr updates are supported when we have a WiFi connection.
	// The NTP time setting does not have to be precise for this function to work.
#if A_OTA==1
	setupOta(hostname);										// Uses wwwServer 
#endif

	// Set the NTP Time
	// As long as the time has not been set we try to set the time.
#if NTP_INTR==1
	setupTime();											// Set NTP time host and interval
#else
	// If not using the standard libraries, do a manual setting
	// of the time. This meyhod works more reliable than the 
	// interrupt driven method.
	
	//setTime((time_t)getNtpTime());
	while (timeStatus() == timeNotSet) {
#if DUSB>=1
		if (( debug>=0 ) && ( pdebug & P_MAIN )) 
			Serial.println(F("M setupTime:: Time not set (yet)"));
#endif
		delay(500);
		time_t newTime;
		newTime = (time_t)getNtpTime();
		if (newTime != 0) setTime(newTime);
	}
	// When we are here we succeeded in getting the time
	startTime = now();										// Time in seconds
#if DUSB>=1
	Serial.print("Time: "); printTime();
	Serial.println();
#endif
	writeGwayCfg(CONFIGFILE );
#if DUSB>=1
	Serial.println(F("Gateway configuration saved"));
#endif
#endif //NTP_INTR

#if A_SERVER==1	
	// Setup the webserver
	setupWWW();
#endif

	delay(100);												// Wait after setup
	
	// Setup ad initialise LoRa state machine of _loramModem.ino
	_state = S_INIT;
	initLoraModem();
	
	if (_cad) {
		_state = S_SCAN;
		sf = SF7;
		cadScanner();										// Always start at SF7
	}
	else { 
		_state = S_RX;
		rxLoraModem();
	}
	LoraUp.payLoad[0]= 0;
	LoraUp.payLength = 0;						// Init the length to 0

	// init interrupt handlers, which are shared for GPIO15 / D8, 
	// we switch on HIGH interrupts
	if (pins.dio0 == pins.dio1) {
		//SPI.usingInterrupt(digitalPinToInterrupt(pins.dio0));
		attachInterrupt(pins.dio0, Interrupt_0, RISING);		// Share interrupts
	}
	// Or in the traditional Comresult case
	else {
		//SPI.usingInterrupt(digitalPinToInterrupt(pins.dio0));
		//SPI.usingInterrupt(digitalPinToInterrupt(pins.dio1));
		attachInterrupt(pins.dio0, Interrupt_0, RISING);	// Separate interrupts
		attachInterrupt(pins.dio1, Interrupt_1, RISING);	// Separate interrupts		
	}
	
	writeConfig( CONFIGFILE, &gwayConfig);					// Write config

	// activate OLED display
#if OLED>=1
	acti_oLED();
	addr_oLED();
#endif

	Serial.println(F("--------------------------------------"));
}//setup



// ----------------------------------------------------------------------------
// LOOP
// This is the main program that is executed time and time again.
// We need to give way to the backend WiFi processing that 
// takes place somewhere in the ESP8266 firmware and therefore
// we include yield() statements at important points.
//
// Note: If we spend too much time in user processing functions
//	and the backend system cannot do its housekeeping, the watchdog
// function will be executed which means effectively that the 
// program crashes.
// We use yield() a lot to avoid ANY watch dog activity of the program.
//
// NOTE2: For ESP make sure not to do large array declarations in loop();
// ----------------------------------------------------------------------------
void loop ()
{
	uint32_t uSeconds;									// micro seconds
	int packetSize;
	uint32_t nowSeconds = now();
	
	// check for event value, which means that an interrupt has arrived.
	// In this case we handle the interrupt ( e.g. message received)
	// in userspace in loop().
	//
	stateMachine();									// do the state machine
	
	// After a quiet period, make sure we reinit the modem and state machine.
	// The interval is in seconds (about 15 seconds) as this re-init
	// is a heavy operation. 
	// SO it will kick in if there are not many messages for the gateway.
	// Note: Be careful that it does not happen too often in normal operation.
	//
	if ( ((nowSeconds - statr[0].tmst) > _MSG_INTERVAL ) &&
		(msgTime <= statr[0].tmst) ) 
	{
#if DUSB>=1
		if (( debug>=1 ) && ( pdebug & P_MAIN )) {
			Serial.print("M REINIT:: ");
			Serial.print( _MSG_INTERVAL );
			Serial.print(F(" "));
			SerialStat(0);
		}
#endif

		// startReceiver() ??
		if ((_cad) || (_hop)) {
			_state = S_SCAN;
			sf = SF7;
			cadScanner();
		}
		else {
			_state = S_RX;
			rxLoraModem();
		}
		writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
		writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);			// Reset all interrupt flags
		msgTime = nowSeconds;
	}

#if A_SERVER==1
	// Handle the Web server part of this sketch. Mainly used for administration 
	// and monitoring of the node. This function is important so it is called at the
	// start of the loop() function.
	yield();
	server.handleClient();
#endif

#if A_OTA==1
	// Perform Over the Air (OTA) update if enabled and requested by user.
	// It is important to put this function early in loop() as it is
	// not called frequently but it should always run when called.
	//
	yield();
	ArduinoOTA.handle();
#endif

	// I event is set, we know that we have a (soft) interrupt.
	// After all necessary web/OTA services are scanned, we will
	// reloop here for timing purposes. 
	// Do as less yield() as possible.
	// XXX 180326
	if (_event == 1) {
		return;
	}
	else yield();

	
	// If we are not connected, try to connect.
	// We will not read Udp in this loop cycle then
	if (WlanConnect(1) < 0) {
#if DUSB>=1
		if (( debug >= 0 ) && ( pdebug & P_MAIN ))
			Serial.println(F("M ERROR reconnect WLAN"));
#endif
		yield();
		return;										// Exit loop if no WLAN connected
	}
	
	// So if we are connected 
	// Receive UDP PUSH_ACK messages from server. (*2, par. 3.3)
	// This is important since the TTN broker will return confirmation
	// messages on UDP for every message sent by the gateway. So we have to consume them.
	// As we do not know when the server will respond, we test in every loop.
	//
	else {
		while( (packetSize = Udp.parsePacket()) > 0) {
#if DUSB>=2
			Serial.println(F("loop:: readUdp calling"));
#endif
			// DOWNSTREAM
			// Packet may be PKT_PUSH_ACK (0x01), PKT_PULL_ACK (0x03) or PKT_PULL_RESP (0x04)
			// This command is found in byte 4 (buffer[3])
			if (readUdp(packetSize) <= 0) {
#if DUSB>=1
				if (( debug>0 ) && ( pdebug & P_MAIN ))
					Serial.println(F("M readUDP error"));
#endif
				break;
			}
			// Now we know we succesfully received message from host
			else {
				//_event=1;								// Could be done double if more messages received
			}
		}
	}
	
	yield();					// XXX 26/12/2017

	// stat PUSH_DATA message (*2, par. 4)
	//	

    if ((nowSeconds - statTime) >= _STAT_INTERVAL) {	// Wake up every xx seconds
#if DUSB>=1
		if (( debug>=1 ) && ( pdebug & P_MAIN )) {
			Serial.print(F("M STAT:: ..."));
			Serial.flush();
		}
#endif
        sendstat();										// Show the status message and send to server
#if DUSB>=1
		if (( debug>=1 ) && ( pdebug & P_MAIN )) {
			Serial.println(F(" done"));
			if (debug>=2) Serial.flush();
		}
#endif	

		// If the gateway behaves like a node, we do from time to time
		// send a node message to the backend server.
		// The Gateway nod emessage has nothing to do with the STAT_INTERVAL
		// message but we schedule it in the same frequency.
		//
#if GATEWAYNODE==1
		if (gwayConfig.isNode) {
			// Give way to internal some Admin if necessary
			yield();
			
			// If the 1ch gateway is a sensor itself, send the sensor values
			// could be battery but also other status info or sensor info
		
			if (sensorPacket() < 0) {
#if DUSB>=1
				Serial.println(F("sensorPacket: Error"));
#endif
			}
		}
#endif
		statTime = nowSeconds;
    }
	
	yield();

	
	// send PULL_DATA message (*2, par. 4)
	//
	nowSeconds = now();
    if ((nowSeconds - pulltime) >= _PULL_INTERVAL) {	// Wake up every xx seconds
#if DUSB>=1
		if (( debug>=2) && ( pdebug & P_MAIN )) {
			Serial.println(F("M PULL"));
			if (debug>=1) Serial.flush();
		}
#endif
        pullData();										// Send PULL_DATA message to server
		startReceiver();
	
		pulltime = nowSeconds;
    }

	
	// If we do our own NTP handling (advisable)
	// We do not use the timer interrupt but use the timing
	// of the loop() itself which is better for SPI
#if NTP_INTR==0
	// Set the time in a manual way. Do not use setSyncProvider
	// as this function may collide with SPI and other interrupts
	yield();											// 26/12/2017
	nowSeconds = now();
	if (nowSeconds - ntptimer >= _NTP_INTERVAL) {
		yield();
		time_t newTime;
		newTime = (time_t)getNtpTime();
		if (newTime != 0) setTime(newTime);
		ntptimer = nowSeconds;
	}
#endif
	

}//loop
