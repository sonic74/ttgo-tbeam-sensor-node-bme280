/*

GPS module

Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

uint32_t humidity_bme;
uint32_t pressure_bme;
uint32_t temperature_bme;

char bme_char[32]; // used to sprintf for Serial output

Adafruit_BME680 bme(&Wire1); // I2C

void bme_setup() {
    Wire1.begin(15,4);
    bool status;
    status = bme.begin();
    if (!status) {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
        while (1);
    }
}

void buildBMEPacket(uint8_t txBuffer[8])
{
  char buffer[40];

  //
  // Humidity
  //

  float humidity = bme.readHumidity();

  sprintf(bme_char, "Humidity: %f", humidity);
  Serial.println(bme_char);

  snprintf(buffer, sizeof(buffer), "Humidity: %10.1f\n", humidity);
  screen_print(buffer);

  // adjust for the f2sflt16 range (-1 to 1)
  humidity /= 1e2;

  // float -> int
  uint16_t payloadHum = LMIC_f2sflt16(humidity);

  // int -> bytes
  byte humLow = lowByte(payloadHum);
  byte humHigh = highByte(payloadHum);

  txBuffer[0] = humLow;
  txBuffer[1] = humHigh;


  //
  // Pressure
  //

  float pressure = bme.readPressure() / 100.0;
  // ws2500.c
  float altitude=326;
  pressure=(pressure + altitude * 0.11) + 0.5;

  sprintf(bme_char, "Pressure: %f", pressure);
  Serial.println(bme_char);

  snprintf(buffer, sizeof(buffer), "Pressure: %10.1f\n", pressure);
  screen_print(buffer);

  // adjust for the f2sflt16 range (-1 to 1)
  pressure /= 1e5;

  // float -> int
  uint16_t payloadPress = LMIC_f2sflt16(pressure);

  // int -> bytes
  byte pressLow = lowByte(payloadPress);
  byte pressHigh = highByte(payloadPress);

  txBuffer[2] = pressLow;
  txBuffer[3] = pressHigh;


  //
  // Temperature
  //

  float temperature = bme.readTemperature();

  sprintf(bme_char, "Temperature: %f", temperature);
  Serial.println(bme_char);

  snprintf(buffer, sizeof(buffer), "Temperature: %10.1f\n", temperature);
  screen_print(buffer);

  // adjust for the f2sflt16 range (-1 to 1)
  temperature /= 1e2;

  // float -> int
  uint16_t payloadTemp = LMIC_f2sflt16(temperature);

  // int -> bytes
  byte tempLow = lowByte(payloadTemp);
  byte tempHigh = highByte(payloadTemp);

  txBuffer[4] = tempLow;
  txBuffer[5] = tempHigh;


  //
  // Gas
  //

  float gas = bme.readGas();

  sprintf(bme_char, "Gas: %f", gas);
  Serial.println(bme_char);

  snprintf(buffer, sizeof(buffer), "Gas: %10.0f\n", gas);
  screen_print(buffer);

  // adjust for the f2sflt16 range (-1 to 1)
  gas /= 1e6;

  // float -> int
  uint16_t payloadGas = LMIC_f2sflt16(gas);

  // int -> bytes
  byte gasLow = lowByte(payloadGas);
  byte gasHigh = highByte(payloadGas);

  txBuffer[6] = gasLow;
  txBuffer[7] = gasHigh;
}
