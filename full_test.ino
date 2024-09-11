/*
  ======================================================================
  Author: Rishik Tiwari
  Created: 31 August 2024
  GitHub: https://github.com/rishiktiwari

  Complete demo program for Arduino Uno R4 to use the following sensors:
    > BME680                        (lib: Adafruit BME680)
    > VEML6030                      (lib: SparkFun Ambient Light Sensor - VEML6030)
    > DFRobot Wattmeter SEN0291     (lib: modified from original to support I2C over Qwiic)
    > MAX17048 LiPo Battery Monitor (lib: Adafruit MAX1704X)
  
  The output is displayed on:
    > Waveshare 1.54inch V2 e-Paper display (SPI, lib: externally provided by waveshare)
    > Adafruit NeoPixel Ring 16LED          (Custom protocol, lib: Adafruit NeoPixel)
  ======================================================================
*/

#include <Wire.h>
#include <SPI.h>
#include <stdio.h>

#include "src/display/epd1in54_V2.h"
#include "src/display/epdpaint.h"
#include "src/display/custom_image.h"

#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"

#include "Adafruit_MAX1704X.h"

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#include "src/wattmeter/DFRobot_INA219.h"

#include <Adafruit_NeoPixel.h>



#define LIGHT_SENSOR_ADDR 0x48
#define SEALEVELPRESSURE_HPA (1013.25)
#define NULL_CHAR '\0'
#define OK_STR "OK"
#define FAIL_STR "FAIL"
#define NEOPIXEL_LEDS 16



// E-PAPER DISPLAY
Epd epd;
unsigned char image[1024];
Paint paint(image, 0, 0);
unsigned long time_start_ms;
unsigned long time_now_s;
#define COLORED     0
#define UNCOLORED   1
uint8_t i = 0;
char temp[] = "";

// LIGHT SENSOR
SparkFun_Ambient_Light light(LIGHT_SENSOR_ADDR);
float ls_gain = .125;
int ls_integTime = 100;
long luxVal = 0;

// LiPo Battery Monitor
Adafruit_MAX17048 batmon;
float batVoltage = 0.0;

// BME688 Sensor (Temperature, Pressure, Humidity, Gas)
Adafruit_BME680 bme(&Wire1);

// Wattmeter INA219
DFRobot_INA219_IIC ina219(&Wire1, INA219_I2C_ADDRESS4);
const float ina219Reading_mA = 1000;
const float extMeterReading_mA = 1000;
float busV = 0.0;
float currentMilli = 0.0;
float powerMilli = 0.0;

// NEOPIXEL RING
Adafruit_NeoPixel pixels(NEOPIXEL_LEDS, 6, NEO_GRB + NEO_KHZ800);   // connect Arduino pin 6 to LED data_in.
uint16_t pix_i = 0;


// FLAGS
volatile bool lightSensorReady = false;
volatile bool batmonReady = false;
volatile bool bme688Ready = false;
volatile bool ina219Ready = false;



void setup(){
  Serial.begin(9600);
  Wire1.begin();
  
  epd.LDirInit();
  epd.Clear();

  lightSensorReady = light.begin(Wire1);
  batmonReady = batmon.begin(&Wire1);
  bme688Ready = bme.begin();
  ina219Ready = ina219.begin(&Wire1);
  pixels.begin();
  pixels.clear();

  // LIGHT SENSOR SETUP
  light.setGain(ls_gain);
  light.setIntegTime(ls_integTime);

  // BME688 SETUP
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // INA219 WATTMETER SETUP
  ina219.linearCalibrate(ina219Reading_mA, extMeterReading_mA);

  showGreeting();
  delay(3000);
  showDeviceStatus();
  delay(3000);
  
  if(lightSensorReady && batmonReady && bme688Ready && ina219Ready) {
    epd.Clear();
    epd.HDirInit();
    epd.Display(IMAGE_3);
    delay(2000);
  }
}

void showGreeting() {
  paint.SetWidth(200);
  paint.SetHeight(32);

  // Serial.println("e-Paper paint");
  paint.Clear(COLORED);
  paint.DrawStringAt(8, 4, "Namaste", &Font24, UNCOLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, 68, paint.GetWidth(), paint.GetHeight());
  
  paint.Clear(UNCOLORED);
  paint.DrawStringAt(8, 4, "Rishik", &Font24, COLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, 100, paint.GetWidth(), paint.GetHeight());
  epd.DisplayFrame();
}

void paintNewLine(uint8_t line_num, char* text) {
  paint.SetWidth(200);
  paint.SetHeight(20);

  paint.Clear(UNCOLORED);
  paint.DrawStringAt(2, 2, text, &Font16, COLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, (line_num-1)*20, paint.GetWidth(), paint.GetHeight());
}

void paintNewStatusLine(uint8_t line_num, char device_name[], bool status_flag) {
  sprintf(temp, "%s: %s", device_name, status_flag ? OK_STR : FAIL_STR);
  paintNewLine(line_num, temp);
}

void showDeviceStatus() {
  epd.LDirInit();
  epd.Clear();
  temp[0] = NULL_CHAR;

  paintNewStatusLine(1, "VEML6030", lightSensorReady);
  paintNewStatusLine(2, "BatMon", batmonReady);
  paintNewStatusLine(3, "BME688", bme688Ready);
  paintNewStatusLine(4, "INA219", ina219Ready);
  
  epd.DisplayFrame();
}

void showMeasurements() {
  epd.LDirInit();
  epd.Clear();
  temp[0] = '\0';

  if(lightSensorReady) {
    sprintf(temp, "Light: %d lux", luxVal);
    paintNewLine(1, temp);
  }
  
  if (batmonReady) {
    sprintf(temp, "Battery: %.2fV", batVoltage);
    paintNewLine(2, temp);
  }

  if(bme688Ready) {
    // NOTE: measurements are stable when reading directly
    sprintf(temp, "Temp: %.2f C", bme.temperature);
    paintNewLine(3, temp);

    sprintf(temp, "Hum: %.1f %%", bme.humidity);
    paintNewLine(4, temp);
    
    sprintf(temp, "Pres: %.1f hPa", bme.pressure/100.0);
    paintNewLine(5, temp);

    sprintf(temp, "GasR: %.1f KOhms", bme.gas_resistance/1000.0);
    paintNewLine(6, temp);

    sprintf(temp, "Altitude: %.1fm", bme.readAltitude(SEALEVELPRESSURE_HPA));
    paintNewLine(7, temp);
  }

  // empty line 8

  if (ina219Ready) {
    sprintf(temp, "%.1fV, %.1fmA", busV, currentMilli);
    paintNewLine(9, temp);

    sprintf(temp, "P out: %.1fmW", powerMilli);
    paintNewLine(10, temp);
  }

  epd.DisplayFrame();
}


void loop() {
  pixels.clear();

  if (lightSensorReady) {
    luxVal = light.readLight();
  }

  if (batmonReady) {
    batVoltage = batmon.cellVoltage();
    if (isnan(batVoltage)) { // battery fail, check connection
      batVoltage = -99.9;
    }
  }

  if (bme688Ready) {
    bme.performReading(); // Perform a reading in blocking mode.
  }

  if (ina219Ready) {
    busV = ina219.getBusVoltage_V();
    currentMilli = ina219.getCurrent_mA();
    powerMilli = ina219.getPower_mW();
  }

  // Serial.println(luxVal);
  // Serial.println(batVoltage);
  // Serial.println("---");
  // Serial.println(temperature);
  // Serial.println(pressure);
  // Serial.println(humidity);
  // Serial.println(gasR);
  // Serial.println(altitude);
  // delay(1000);

  // animated led ring
  for (pix_i = 0; pix_i < NEOPIXEL_LEDS; pix_i++) {
    pixels.setPixelColor(pix_i, pixels.Color(0, 50, 30));
    pixels.show();
    delay(60);
  }
  delay(100);
  pixels.clear();
  pixels.show();
  delay(100);

  // DISPLAY THE MEASUREMENTS
  showMeasurements();
  epd.Sleep();
  delay(5000);

  // animated led ring
  for (pix_i = 0; pix_i < NEOPIXEL_LEDS; pix_i++) {
    pixels.setPixelColor(pix_i, pixels.Color(255, 255, 255));
    pixels.show();
    delay(60);
  }

  // DISPLAY THE DEVICE STATUS
  showDeviceStatus();
  delay(3000);
}
