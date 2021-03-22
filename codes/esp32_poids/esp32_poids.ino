
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier


#include "HX711.h"
#include "soc/rtc.h"


// supply control
const int transistorPin = 16;

//capteur de poids
//#define DOUT  17 //pour ESP32 TTGO
//#define CLK  16

//#define DOUT  25 //pour ESP32 TTGO LORA
//#define CLK  26

#define DOUT  25 //pour ESP32 wemos
#define CLK  26

float weight = 0;
float weightcorrection = -3.77;
float calibration_factor = 23600;  //You must change this factor depends on your scale,sensors and etc.


void getWeight()
{
        HX711 scale(DOUT, CLK);
        scale.set_scale(calibration_factor); //Adjust to this calibration factor
        Serial.print("Reading: ");
        weight = scale.get_units();
        weight = weight + weightcorrection;
        Serial.print(weight, 3);
        Serial.println(" kg"); // You can change this to other type of weighing value and re-adjust the calibration factor.
        Serial.print(" calibration_factor: ");
        Serial.println(calibration_factor);
 }
 
void setup()
{
  Serial.begin(115200);
  pinMode (transistorPin, OUTPUT);
 
  digitalWrite (transistorPin, HIGH);  // turn on the supply
  //scale.power_up();
  delay(1000);
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
   }

void loop()
{
  //delay(2000);
  
  getWeight();
  
 delay(1000);
  
  //digitalWrite (transistorPin, LOW);  // turn off the supply
//  scale.power_down();  
 
}
