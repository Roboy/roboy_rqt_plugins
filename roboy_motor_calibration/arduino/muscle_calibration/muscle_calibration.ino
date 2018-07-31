#include "HX711.h"
#include <Wire.h>
#include "rgb_lcd.h"

const int numRows = 2;
const int numCols = 16;
rgb_lcd lcd;

#define calibration_factor -7050.0 //This value is obtained using the SparkFun_HX711_Calibration sketch

#define DOUT  5
#define CLK  4

HX711 scale(DOUT, CLK);

void setup() {
  lcd.begin(numCols, numRows);
  lcd.setRGB(0,0,255);
  lcd.print("muscle calib");
  delay(1000);
  
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare();	//Assuming there is no weight on the scale at start up, reset the scale to 0

  lcd.setCursor(15,1);
  lcd.print("N");
}

void loop() {
  float val = scale.get_units();
  lcd.setCursor(0,1);
  lcd.print(val);
}
