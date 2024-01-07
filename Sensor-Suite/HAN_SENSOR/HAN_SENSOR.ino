#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_SGP30.h"
#include "Adafruit_BME680.h"
#define SEALEVELPRESSURE_HPA (1013.25)
int counter = 0;
const int Methpin=A6;
const int COpin=A7; 
int Methout; 
int COout;
#include <WinsenZE03.h>
WinsenZE03 ZEE;
Adafruit_BME680 bme; 
Adafruit_SGP30 sgp;
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}
void setup() {
  Serial.begin(115200); 
  sgp.begin();
  bme.begin();
//  Serial.print(sgp.serialnumber[0], HEX);
//  Serial.print(sgp.serialnumber[1], HEX);
//  Serial.println(sgp.serialnumber[2], HEX);
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms
}

void loop()
{
   //counter++;
//  if (counter == 30) {
//    counter = 0;
//    uint16_t TVOC_base, eCO2_base;
//    if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
//      Serial.println("Failed to get baseline readings");
//      return;
//    }
  sgp.IAQmeasure();
  bme.performReading();
  Methout= analogRead(Methpin); 
  COout= analogRead(Methpin); 
  Serial.print("C");
  Serial.print(COout);
  Serial.print("M");
  Serial.print(Methout); // Print out the methane value - the analog output - beteewn 0 and 1023
  Serial.print("0"); 
  Serial.print(sgp.TVOC); //ppb
  Serial.print("E"); 
  Serial.print(sgp.eCO2); //ppm
  Serial.print("T");
  Serial.print(bme.temperature);
  Serial.print("P");
  Serial.print(bme.pressure / 100.0);
  Serial.print("H");
  Serial.print(bme.humidity);
  Serial.println("E");
  // Print a blank line for better readability
  // Delay for 2 seconds before the next reading
  delay(2000);
}
