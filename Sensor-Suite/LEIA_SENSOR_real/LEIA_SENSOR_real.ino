#include "AS726X.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define PHSens A1
#define ONE_WIRE_BUS 7

AS726X specbb;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempsens(&oneWire);

int mois;
unsigned long int avgValue;
float b;
int mlow = 320;
int mhigh = 1024;
int buf[10], temp;

void setup() {
  Wire.begin();
  tempsens.begin();
  Serial.begin(115200);
  if (!specbb.begin()) {
    Serial.println("Could not connect to AS726X sensor! Please check your wiring.");
  }
}
float getPH() {
  float phValue;
  for (int i = 0; i < 10; i++) { // Get 10 sample values from the sensor for smoothing the value
    buf[i] = analogRead(PHSens);
    delay(10);
  }
  
  for (int i = 0; i < 9; i++) { // Sort the analog values from small to large
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  
  avgValue = 0;
  for (int i = 2; i < 8; i++) { // Take the average value of 6 center samples
    avgValue += buf[i];
  }
  
  phValue = (float)avgValue * 5.0 / 1024 / 6; // Convert the analog into millivolt
  phValue = 3.5 * phValue; // Convert the millivolt into pH value
  return phValue;
}

float moistmeter() {
  int mois = analogRead(A0);
  float bhendi = map(mois, mhigh, mlow, 0, 100);
  return bhendi;
}

float gettemp() {
  tempsens.requestTemperatures();
  float realtemp = tempsens.getTempCByIndex(0);
  return realtemp;
}

bool getSpecbbValues(float specbbValues[]) {
  if (specbb.begin()) {
    specbb.takeMeasurements();
    specbbValues[0] = specbb.getCalibratedR();
    specbbValues[1] = specbb.getCalibratedS();
    specbbValues[2] = specbb.getCalibratedT();
    specbbValues[3] = specbb.getCalibratedU();
    specbbValues[4] = specbb.getCalibratedV();
    specbbValues[5] = specbb.getCalibratedW();
    return true; // Sensor connected and read successfully
  } else {
    // Assign some default values if specbb is not connected
    for (int i = 0; i < 6; ++i) {
      specbbValues[i] = -1.0; // You can choose any default value
    }
    return false; // Sensor not connected or failed to read
  }
}

void loop() {
  float specbbValues[6]; // Array to store AS726X sensor values

  bool specbbConnected = getSpecbbValues(specbbValues); // Retrieve AS726X sensor values

  float bh = getPH();
  float moimoi = moistmeter();
  float tempo = gettemp();

  if (specbbConnected) {
    Serial.print("S");
    Serial.print(specbbValues[0], 2);
    Serial.print(",");
    Serial.print(specbbValues[1], 2);
    Serial.print(",");
    Serial.print(specbbValues[2], 2);
    Serial.print(",");
    Serial.print(specbbValues[3], 2);
    Serial.print(",");
    Serial.print(specbbValues[4], 2);
    Serial.print(",");
    Serial.print(specbbValues[5], 2);
  } else {
    // Print placeholder values if specbb sensor not connected or encountered an error
    Serial.print("S-1.00,-1.00,-1.00,-1.00,-1.00,-1.00");
  }

  Serial.print("M");
  Serial.print(moimoi);
  Serial.print("T");
  Serial.print(tempo);
  Serial.print("P");
  Serial.print(bh);
  Serial.println("E");

  delay(100); // Add a small delay for stability
}
