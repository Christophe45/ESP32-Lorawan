#include "HX711.h"
#include "soc/rtc.h"
#define DOUT  17
#define CLK  16

//void getWeight(void);



float weight = 0.00;
float weightcorrection = 33.75;


// supply control
const int transistorPin = 14;
HX711 scale(DOUT, CLK);
float calibration_factor = 21400;	//You must change this factor depends on your scale,sensors and etc.


void setup() {
  Serial.begin(115200);
  pinMode (transistorPin, OUTPUT);
  digitalWrite (transistorPin, HIGH);  // turn on the supply
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  Serial.println("HX711 calibration sketch");
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  scale.set_scale();
  //scale.tare(); //Reset the scale to 0
  //long zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  //Serial.println(zero_factor);
  delay(2000);

}
void loop() {

  getWeight();

}

void getWeight(){
  scale.set_scale(calibration_factor); //Adjust to this calibration factor
  Serial.print("Reading: ");
  weight = scale.get_units();
  weight = weight + weightcorrection;
  Serial.print(weight, 3);
  Serial.print(" kg"); // You can change this to other type of weighing value and re-adjust the calibration factor.
  Serial.print(" calibration_factor: ");
  Serial.print(calibration_factor);
  Serial.println();
  delay(500);
}

