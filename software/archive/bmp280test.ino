#include <Wire.h>
#include<Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

unsigned long time_ms;
float temp;
float pres;
float alt;

void setup() {
  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  //REMOVE//
  Serial.begin(115200);
}

void loop() {
  time_ms = millis();
  temp = bmp.readTemperature();
  pres = bmp.readPressure();
  alt= bmp.readAltitude(1013.25);

  Serial.println(alt);

}
