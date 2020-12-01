// If you are in USA on a 915Mhz network instead of an european / 868 Mhz one,
//   you MUST modify the arduino lmic library "config.h" to enable CFG_us915 instead of CFG_eu868.
// That "config.h" should be in the same folder as the "lmic.h" file in your arduino folders.
#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "HX711.h"
#include "soc/rtc.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include <driver/adc.h>

// MPU-6050 registers
#define SIGNAL_PATH_RESET  0x68
#define I2C_SLV0_ADDR      0x37
#define ACCEL_CONFIG       0x1C
#define MOT_THR            0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR            0x20  // This seems wrong // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MOT_DETECT_CTRL    0x69
#define INT_ENABLE         0x38
#define PWR_MGMT           0x6B //SLEEPY TIME
#define INT_STATUS 0x3A
#define MPU6050_ADDRESS 0x68 //AD0 is 0

// UPDATE our "device_config.h" file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "device_config_ruche3.h"
#include "gps.h"

#include <WiFi.h>
#include <esp_sleep.h>

//Deep sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 7200      /* Time ESP32 will go to sleep (in seconds) */
//#define TIME_TO_SLEEP 30      /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;

// DS18B20 sensor
#define ONE_WIRE_BUS 33
#define TEMPERATURE_PRECISION 100
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire); 
float tempruche;


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


int wakeup_reason;

// T-Beam specific hardware
#undef BUILTIN_LED
#define BUILTIN_LED 21

char s[32]; // used to sprintf for Serial output
uint8_t txBuffer[22];
gps gps;
static osjob_t sendjob;

// Those variables keep their values after software restart or wakeup from sleep, not after power loss or hard reset !
RTC_NOINIT_ATTR int RTCseqnoUp, RTCseqnoDn;
#ifdef USE_OTAA
RTC_NOINIT_ATTR u4_t otaaDevAddr;
RTC_NOINIT_ATTR u1_t otaaNetwKey[16];
RTC_NOINIT_ATTR u1_t otaaApRtKey[16];
#endif

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping for TBeams, might not suit the latest version > 1.0 ?
  const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 13,
           // LBT cal for the Adafruit Feather M0 LoRa, in dB
 //  .dio = {27, 14, 17}
     .dio = {27, 14, LMIC_UNUSED_PIN}
};

// These callbacks are only used in over-the-air activation.
#ifdef USE_OTAA
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}
#else
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#endif

/*    Example for using write byte
      Configure the accelerometer for self-test
      writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g */
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.begin();
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

// sens argument configures wake up sensitivity
void configureMPU(int sens){
  writeByte( MPU6050_ADDRESS, 0x6B, 0x00);
  writeByte( MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07);//Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  // writeByte( MPU6050_ADDRESS, I2C_SLV0_ADDR, 0x20);//write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
  writeByte( MPU6050_ADDRESS, ACCEL_CONFIG, 0x01);//Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
  writeByte( MPU6050_ADDRESS, MOT_THR, sens);  //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
  writeByte( MPU6050_ADDRESS, MOT_DUR, 40 );  //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
  writeByte( MPU6050_ADDRESS, MOT_DETECT_CTRL, 0x15); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )
  writeByte( MPU6050_ADDRESS, 0x37, 140 ); // now INT pin is active low
  writeByte( MPU6050_ADDRESS, INT_ENABLE, 0x40 ); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.
  writeByte( MPU6050_ADDRESS, PWR_MGMT, 8 ); // 101000 - Cycle & disable TEMP SENSOR
  writeByte( MPU6050_ADDRESS, 0x6C, 7); // Disable Gyros
}



void storeFrameCounters()
{
  RTCseqnoUp = LMIC.seqnoUp;
  RTCseqnoDn = LMIC.seqnoDn;
  sprintf(s, "Counters stored as %d/%d", LMIC.seqnoUp, LMIC.seqnoDn);
  Serial.println(s);
}

void restoreFrameCounters()
{
  LMIC.seqnoUp = RTCseqnoUp;
  LMIC.seqnoDn = RTCseqnoDn;
  sprintf(s, "Restored counters as %d/%d", LMIC.seqnoUp, LMIC.seqnoDn);
  Serial.println(s);
}


void setOrRestorePersistentCounters()
{
  esp_reset_reason_t reason = esp_reset_reason();
  if ((reason != ESP_RST_DEEPSLEEP) && (reason != ESP_RST_SW))
  {
    Serial.println(F("Counters both set to 0"));
    LMIC.seqnoUp = 0;
    LMIC.seqnoDn = 0;
  }
  else
  {
    restoreFrameCounters();
  }
}

/*
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.println(wakeup_reason);

  switch (wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal accelerometer");break;
    case 3  : Serial.println("Wakeup caused by touchpad"); break;
    case 4  : Serial.println("Wakeup caused by timer");break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
  
}
*/
void onEvent (ev_t ev) {
  switch (ev) {
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
#ifdef USE_OTAA    
      otaaDevAddr = LMIC.devaddr;
      memcpy_P(otaaNetwKey, LMIC.nwkKey, 16);
      memcpy_P(otaaApRtKey, LMIC.artKey, 16);
      sprintf(s, "got devaddr = 0x%X", LMIC.devaddr);
      Serial.println(s);
#endif
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      // TTN uses SF9 for its RX2 window.
      LMIC.dn2Dr = DR_SF9;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received Ack"));
      }
      if (LMIC.dataLen) {
        sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
      }
     storeFrameCounters();
      // Schedule next transmission
      Serial.println("Good night...");
       adc_power_off();
       digitalWrite (transistorPin, LOW);
       Deep_Sleep_Now();
      do_send(&sendjob);
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
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) 
  {  
   // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  { 
   
     //if (gps.checkGpsFix())
      // {

  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  
if (wakeup_reason != 2){
 
  
  Serial.println("Wakeup was caused by deep sleep or starting");
  
      // Prepare upstream data transmission at the next possible time.
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

        // reading DS18B20 value
        DS18B20.requestTemperatures();  
        tempruche = DS18B20.getTempCByIndex(0); // Sensor 0 will capture Temp in Celcius 
        Serial.print("tempruche: "); 
        Serial.println(tempruche); 

         // Alarm!!
         int alarm;
         alarm = 0;
         Serial.print("alarm:"); Serial.println(alarm); 
       
int16_t Celcius = temp * 100; // convert to signed 16 bits integer: 0x0929
int16_t Humidity = humidity * 100;
int16_t Weight = weight * 100;
int16_t BootCount = bootCount;
//int16_t bootCountInt = bootCount;
int16_t Vbat = vbat * 100;
int16_t Tempruche = tempruche * 100;
int16_t Alarm = alarm;

uint8_t txBuffer[24];

// Handle high byte (MSB) first; 0x09 for 23.45
// 0x0929 >> 8 shifts the 0x29 out of memory, leaving 0x0009
// 0x0009 does not fit in a single byte, so only 0x09 is stored in buffer[0]:
txBuffer[10] = Celcius >> 8;
txBuffer[11] = Celcius;
txBuffer[12] = Humidity >> 8;
txBuffer[13] = Humidity;
txBuffer[14] = Weight >> 8;
txBuffer[15] = Weight;
txBuffer[16] = BootCount >> 8;
txBuffer[17] = BootCount;
txBuffer[18] = Vbat >> 8;
txBuffer[19] = Vbat;
txBuffer[20] = Tempruche >> 8;
txBuffer[21] = Tempruche;
txBuffer[22] = Alarm >> 8;
txBuffer[23] = Alarm;
      
      gps.buildPacket(txBuffer);
      LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
      Serial.println(F("Packet queued"));

    }


    else 

// If it's due to the acceleromter and alarm will send
    {
 
  Serial.println("ALARM!!!");
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

        // reading DS18B20 value
        DS18B20.requestTemperatures();  
        tempruche = DS18B20.getTempCByIndex(0); // Sensor 0 will capture Temp in Celcius 
        Serial.print("tempruche: "); 
        Serial.println(tempruche); 

        // Alarm!!
         int alarm;
        alarm = 1;
        Serial.print("alarm:"); Serial.println(alarm); 
       
int16_t Celcius = temp * 100; // convert to signed 16 bits integer: 0x0929
int16_t Humidity = humidity * 100;
int16_t Weight = weight * 100;
int16_t BootCount = bootCount;
//int16_t bootCountInt = bootCount;
int16_t Vbat = vbat * 100;
int16_t Tempruche = tempruche * 100;
int16_t Alarm = alarm;


uint8_t txBuffer[24];

// Handle high byte (MSB) first; 0x09 for 23.45
// 0x0929 >> 8 shifts the 0x29 out of memory, leaving 0x0009
// 0x0009 does not fit in a single byte, so only 0x09 is stored in buffer[0]:
txBuffer[10] = Celcius >> 8;
txBuffer[11] = Celcius;
txBuffer[12] = Humidity >> 8;
txBuffer[13] = Humidity;
txBuffer[14] = Weight >> 8;
txBuffer[15] = Weight;
txBuffer[16] = BootCount >> 8;
txBuffer[17] = BootCount;
txBuffer[18] = Vbat >> 8;
txBuffer[19] = Vbat;
txBuffer[20] = Tempruche >> 8;
txBuffer[21] = Tempruche;
txBuffer[22] = Alarm >> 8;
txBuffer[23] = Alarm;
      
      gps.buildPacket(txBuffer);
      LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
      Serial.println(F("Packet queued"));
  
     }
     //  }
       // else
        //{
      //try again in 3 seconds
      //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
      //}
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  pinMode (transistorPin, OUTPUT);
  digitalWrite (transistorPin, HIGH);
  delay(1000);
  //Serial.println(F("TTN Mapper"));
   bme.begin(0x76); 
   DS18B20.begin(); 
   ++bootCount;
   delay(300);
   delay(500);
   rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();
  gps.init();
 configureMPU(1); // Setup MPU for interrupt, power down gyros & temp sensor
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34,0); delay(1500);//1 = High, 0 = Low
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  
#ifdef USE_OTAA
  esp_reset_reason_t reason = esp_reset_reason();
  if ((reason == ESP_RST_DEEPSLEEP) || (reason == ESP_RST_SW))
  {
    LMIC_setSession(0x1, otaaDevAddr, otaaNetwKey, otaaApRtKey);
  }
#else // ABP
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Disable link check validation
  LMIC_setLinkCheckMode(0);
#endif

  // This must be done AFTER calling LMIC_setSession !
  setOrRestorePersistentCounters();

#ifdef CFG_eu868  
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
#endif

#ifdef CFG_us915
  LMIC_selectSubBand(1);

  //Disable FSB2-8, channels 16-72
  for (int i = 16; i < 73; i++) {
    if (i != 10)
      LMIC_disableChannel(i);
  }
#endif

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14); 

  do_send(&sendjob);
  //pinMode(BUILTIN_LED, OUTPUT);
  //digitalWrite(BUILTIN_LED, LOW);
  
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
