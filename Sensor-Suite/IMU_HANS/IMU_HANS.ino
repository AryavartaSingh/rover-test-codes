#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_SGP30.h"
#include "Adafruit_BME680.h"
#define SEALEVELPRESSURE_HPA (1013.25)
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

float thetaM;
float phiM;
float thetaFold = 0;
float thetaFnew;
float phiFold = 0;
float phiFnew;

float thetaG = 0;
float phiG = 0;

float theta;
float phi;

float thetaRad;
float phiRad;

float Xm;
float Ym;
float psi;

float dt;
unsigned long millisOld;

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055();
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
  myIMU.begin();
  delay(1000);
  int8_t temp = myIMU.getTemp();
  myIMU.setExtCrystalUse(true);
  millisOld = millis();
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
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  thetaM = -atan2(acc.x() / 9.8, acc.z() / 9.8) / 2 / 3.141592654 * 360;
  phiM = -atan2(acc.y() / 9.8, acc.z() / 9.8) / 2 / 3.141592654 * 360;
  phiFnew = 0.95 * phiFold + 0.05 * phiM;
  thetaFnew = 0.95 * thetaFold + 0.05 * thetaM;

  dt = (millis() - millisOld) / 1000.0;
  millisOld = millis();
  theta = (theta + gyr.y() * dt) * 0.95 + thetaM * 0.05;
  phi = (phi - gyr.x() * dt) * 0.95 + phiM * 0.05;
  thetaG = thetaG + gyr.y() * dt;
  phiG = phiG - gyr.x() * dt;

  phiRad = phi / 360 * (2 * 3.14);
  thetaRad = theta / 360 * (2 * 3.14);

  Xm = mag.x() * cos(thetaRad) - mag.y() * sin(phiRad) * sin(thetaRad) + mag.z() * cos(phiRad) * sin(thetaRad);
  Ym = mag.y() * cos(phiRad) + mag.z() * sin(phiRad);

  psi = atan2(Ym, Xm) / (2 * 3.14) * 360;

  // Ensure psi is in the range [0, 360)
  if (psi < 0) {
    psi += 360;
  }

  // Map the heading angle to cardinal directions
  String direction;
  if (psi >= 0 && psi < 22.5)
    direction = "N";
  else if (psi >= 22.5 && psi < 67.5)
    direction = "NE";
  else if (psi >= 67.5 && psi < 112.5)
    direction = "E";
  else if (psi >= 112.5 && psi < 157.5)
    direction = "SE";
  else if (psi >= 157.5 && psi < 180)
    direction = "S";
  else if (psi >= 180 && psi < 202.5)
    direction = "S";
  else if (psi >= 202.5 && psi < 247.5)
    direction = "SW";
  else if (psi >= 247.5 && psi < 292.5)
    direction = "W";
  else if (psi >= 292.5 && psi < 337.5)
    direction = "NW";
  else if (psi >= 337.5 && psi < 360)
    direction = "N";
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
  Serial.print("O"); 
  Serial.print(sgp.TVOC); //ppb
  Serial.print("F"); //old e
  Serial.print(sgp.eCO2); //ppm
  Serial.print("T");
  Serial.print(bme.temperature);
  Serial.print("P");
  Serial.print(bme.pressure / 100.0);
  Serial.print("H");
  Serial.print(bme.humidity);
  Serial.print("D");
  Serial.print(direction);
  Serial.println("X"); //old E
  // Print a blank line for better readability
  // Delay for 2 seconds before the next reading
  phiFold = phiFnew;
  thetaFold = thetaFnew;
  delay(100);
}
