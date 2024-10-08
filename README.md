# Arduino R4 Qwiic Devices Demo

This is a demo code to show modifications required to use I2C devices through the Qwiic port on the newer Arduino R4 boards.

📀 [Demo Video](https://youtu.be/SrD6xGvQST8)

> NOTE: If arduino is unable to open the full_test.ino file then rename to directory name (Arduino_R4_Qwiic_demo.ino)

<span>
    <img src="images/overview.jpg" alt="Circuit overview" width="30%" />
    <img src="images/display-readings.jpeg" alt="Readings displayed on the e-paper display" width="30%" />
</span>


## Prerequisites
- Arduino R4
- Waveshare 1.54 inch E-paper display [[link]](https://www.waveshare.com/1.54inch-e-paper-module.htm)
- Adafruit MAX17048 LiPo Battery Monitor [[link]](https://www.adafruit.com/product/5580)
- SparkFun/Piicodev VEML6030 Ambient Light Sensor [[link]](https://www.sparkfun.com/products/15436)
- Adafruit BME688 - Temperature, Humidity, Pressure and Gas Sensor [[link]](https://www.adafruit.com/product/5046)
- DFRobot Gravity I2C Digital Wattmeter - INA219 [[link]](https://www.dfrobot.com/product-1827.html)
- Adafruit NeoPixel Ring Light [[link]](https://www.adafruit.com/product/1463)

## Software libraries

**From Arduino library manager:**

- Adafruit BME680
- SparkFun Ambient Light Sensor - VEML6030
- Adafruit MAX1704X

**Included in this repo:**

- Waveshare 1.54inch V2 E-paper display [[original docs]](https://www.waveshare.com/wiki/1.54inch_e-Paper_Module_Manual#Working_With_Arduino)
- DFRobot_INA219 SKU: SEN0291 (modified to support Qwiic) [[original lib]](https://github.com/DFRobot/DFRobot_INA219)
