// 1-channel LoRa Gateway for ESP8266
// Copyright (c) 2016, 2017, 2018 Maarten Westenberg version for ESP8266
// Version 5.3.3
// Date: 2018-08-25
//
// 	based on work done by Thomas Telkamp for Raspberry PI 1ch gateway
//	and many others.
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// NO WARRANTY OF ANY KIND IS PROVIDED
//
// Author: Maarten Westenberg (mw12554@hotmail.com)
//
// This file contains the utilities for time and other functions
// ========================================================================================

// ----------------------------------------------------------------------------
// Fill a HEXadecimal String  from a 4-byte char array
//
// ----------------------------------------------------------------------------
static void printHEX(char * hexa, const char sep, String& response) 
{
	char m;
	m = hexa[0]; if (m<016) response+='0'; response += String(m, HEX);  response+=sep;
	m = hexa[1]; if (m<016) response+='0'; response += String(m, HEX);  response+=sep;
	m = hexa[2]; if (m<016) response+='0'; response += String(m, HEX);  response+=sep;
	m = hexa[3]; if (m<016) response+='0'; response += String(m, HEX);  response+=sep;
}

// ----------------------------------------------------------------------------
// stringTime
// Print the time t into the String reponse. t is of type time_t in seconds.
// Only when RTC is present we print real time values
// t contains number of seconds since system started that the event happened.
// So a value of 100 would mean that the event took place 1 minute and 40 seconds ago
// ----------------------------------------------------------------------------
static void stringTime(time_t t, String& response) {

	if (t==0) { response += "--"; return; }
	
	// now() gives seconds since 1970
	// as millis() does rotate every 50 days
	// So we need another timing parameter
	time_t eTime = t;
	
	// Rest is standard
	byte _hour   = hour(eTime);
	byte _minute = minute(eTime);
	byte _second = second(eTime);
	
	switch(weekday(eTime)) {
		case 1: response += "Sunday "; break;
		case 2: response += "Monday "; break;
		case 3: response += "Tuesday "; break;
		case 4: response += "Wednesday "; break;
		case 5: response += "Thursday "; break;
		case 6: response += "Friday "; break;
		case 7: response += "Saturday "; break;
	}
	response += String() + day(eTime) + "-";
	response += String() + month(eTime) + "-";
	response += String() + year(eTime) + " ";
	
	if (_hour < 10) response += "0";
	response += String() + _hour + ":";
	if (_minute < 10) response += "0";
	response += String() + _minute + ":";
	if (_second < 10) response += "0";
	response += String() + _second;
}


// ----------------------------------------------------------------------------
// SerialTime
// Print the current time on the Serial (USB), with leading 0.
// ----------------------------------------------------------------------------
void SerialTime() 
{

	uint32_t thrs = hour();
	uint32_t tmin = minute();
	uint32_t tsec = second();
			
	if (thrs<10) Serial.print('0'); Serial.print(thrs);
	Serial.print(':');
	if (tmin<10) Serial.print('0'); Serial.print(tmin);
	Serial.print(':');
	if (tsec<10) Serial.print('0'); Serial.print(tsec);
			
	if (debug>=2) Serial.flush();
		
	return;
}

// ----------------------------------------------------------------------------
// SerialStat
// Print the statistics on Serial (USB) port
// ----------------------------------------------------------------------------

void SerialStat(uint8_t intr) 
{
#if DUSB>=1
	if (debug>=0) {
		Serial.print(F("I="));

		if (intr & IRQ_LORA_RXTOUT_MASK) Serial.print(F("RXTOUT "));		// 0x80
		if (intr & IRQ_LORA_RXDONE_MASK) Serial.print(F("RXDONE "));		// 0x40
		if (intr & IRQ_LORA_CRCERR_MASK) Serial.print(F("CRCERR "));		// 0x20
		if (intr & IRQ_LORA_HEADER_MASK) Serial.print(F("HEADER "));		// 0x10
		if (intr & IRQ_LORA_TXDONE_MASK) Serial.print(F("TXDONE "));		// 0x08
		if (intr & IRQ_LORA_CDDONE_MASK) Serial.print(F("CDDONE "));		// 0x04
		if (intr & IRQ_LORA_FHSSCH_MASK) Serial.print(F("FHSSCH "));		// 0x02
		if (intr & IRQ_LORA_CDDETD_MASK) Serial.print(F("CDDETD "));		// 0x01

		if (intr == 0x00) Serial.print(F("  --  "));
			
		Serial.print(F(", F="));
		Serial.print(ifreq);
		Serial.print(F(", SF="));
		Serial.print(sf);
		Serial.print(F(", E="));
		Serial.print(_event);
			
		Serial.print(F(", S="));
		//Serial.print(_state);
		switch (_state) {
			case S_INIT:
				Serial.print(F("INIT "));
			break;
			case S_SCAN:
				Serial.print(F("SCAN "));
			break;
			case S_CAD:
				Serial.print(F("CAD  "));
			break;
			case S_RX:
				Serial.print(F("RX   "));
			break;
			case S_TX:
				Serial.print(F("TX   "));
			break;
			case S_TXDONE:
				Serial.print(F("TXDONE"));
			break;
			default:
				Serial.print(F(" -- "));
		}
		Serial.print(F(", eT="));
		Serial.print( micros() - eventTime );
		Serial.print(F(", dT="));
		Serial.print( micros() - doneTime );
		Serial.println();
	}
#endif
}


	
// ----------------------------------------------------------------------------
// SerialName(id, response)
// Check whether for address a (4 bytes in Unsigned Long) there is a name
// This function only works if _TRUSTED_NODES is set
// ----------------------------------------------------------------------------

int SerialName(char * a, String& response)
{
#if _TRUSTED_NODES>=1
	uint32_t id = ((a[0]<<24) | (a[1]<<16) | (a[2]<<8) | a[3]);

	int i;
	for ( i=0; i< (sizeof(nodes)/sizeof(nodex)); i++) {

		if (id == nodes[i].id) {
#if DUSB >=1
			if (( debug>=3 ) && ( pdebug & P_GUI )) {
				Serial.print(F("G Name="));
				Serial.print(nodes[i].nm);
				Serial.print(F(" for node=0x"));
				Serial.print(nodes[i].id,HEX);
				Serial.println();
			}
#endif
			response += nodes[i].nm;
			return(i);
		}
	}

#endif
	return(-1);									// If no success OR is TRUSTED NODES not defined
}

#if _LOCALSERVER==1
// ----------------------------------------------------------------------------
// inDecodes(id)
// Find the id in Decodes array, and return the index of the item
// Parameters:
//		id: The first field in the array (normally DevAddr id). Must be char[4]
// Returns:
//		The index of the ID in the Array. Returns -1 if not found
// ----------------------------------------------------------------------------
int inDecodes(char * id) {

	uint32_t ident = ((id[3]<<24) | (id[2]<<16) | (id[1]<<8) | id[0]);

	int i;
	for ( i=0; i< (sizeof(decodes)/sizeof(codex)); i++) {
		if (ident == decodes[i].id) {
			return(i);
		}
	}
	return(-1);
}
#endif
