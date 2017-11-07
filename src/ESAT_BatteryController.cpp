/*
 * This file is part of Theia Space's ESAT EPS library.
 *
 * Theia Space's ESAT EPS library is free software: you can
 * redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Theia Space's ESAT EPS library is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's ESAT EPS library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "ESAT_BatteryController.h"
#include <Wire.h>

ESAT_BatteryControllerClass::ESAT_BatteryControllerClass():
  previousBattery1VoltageReading(0),
  previousBattery2VoltageReading(0),
  previousBatteryCurrentReading(0),
  previousBatteryTemperatureReading(0),
  previousStateOfChargeReading(0),
  previousTotalBatteryVoltageReading(0),
  previousBattery1VoltageReadingTime(0),
  previousBattery2VoltageReadingTime(0),
  previousBatteryCurrentReadingTime(0),
  previousBatteryTemperatureReadingTime(0),
  previousStateOfChargeReadingTime(0),
  previousTotalBatteryVoltageReadingTime(0)
{
}

word ESAT_BatteryControllerClass::readBattery1Voltage()
{
  const unsigned long readingTime = millis();
  if ((readingTime - previousBattery1VoltageReadingTime) < PERIOD)
  {
    return previousBattery1VoltageReading;
  }
  previousBattery1VoltageReadingTime = readingTime;
  previousBattery1VoltageReading = readWord(BATTERY_1_VOLTAGE_REGISTER);
  return previousBattery1VoltageReading;
}

word ESAT_BatteryControllerClass::readBattery2Voltage()
{
  const unsigned long readingTime = millis();
  if ((readingTime - previousBattery2VoltageReadingTime) < PERIOD)
  {
    return previousBattery2VoltageReading;
  }
  previousBattery2VoltageReadingTime = readingTime;
  previousBattery2VoltageReading = readWord(BATTERY_2_VOLTAGE_REGISTER);
  return previousBattery2VoltageReading;
}

word ESAT_BatteryControllerClass::readBatteryCurrent()
{
  const unsigned long readingTime = millis();
  if ((readingTime - previousBatteryCurrentReadingTime) < PERIOD)
  {
    return previousBatteryCurrentReading;
  }
  previousBatteryCurrentReadingTime = readingTime;
  previousBatteryCurrentReading = readWord(BATTERY_CURRENT_REGISTER);
  return previousBatteryCurrentReading;
}

word ESAT_BatteryControllerClass::readBatteryTemperature()
{
  const unsigned long readingTime = millis();
  if ((readingTime - previousBatteryTemperatureReadingTime) < PERIOD)
  {
    return previousBatteryTemperatureReading;
  }
  previousBatteryTemperatureReadingTime = readingTime;
  previousBatteryTemperatureReading = readWord(BATTERY_TEMPERATURE_REGISTER);
  return previousBatteryTemperatureReading;
}

byte ESAT_BatteryControllerClass::readStateOfCharge()
{
  const unsigned long readingTime = millis();
  if ((readingTime - previousStateOfChargeReadingTime) < PERIOD)
  {
    return previousStateOfChargeReading;
  }
  previousStateOfChargeReadingTime = readingTime;
  previousStateOfChargeReading = readWord(STATE_OF_CHARGE_REGISTER);
  return previousStateOfChargeReading;
}

word ESAT_BatteryControllerClass::readTotalBatteryVoltage()
{
  const unsigned long readingTime = millis();
  if ((readingTime - previousTotalBatteryVoltageReadingTime) < PERIOD)
  {
    return previousTotalBatteryVoltageReading;
  }
  previousTotalBatteryVoltageReadingTime = readingTime;
  previousTotalBatteryVoltageReading = readWord(TOTAL_BATTERY_VOLTAGE_REGISTER);
  return previousTotalBatteryVoltageReading;
}

byte ESAT_BatteryControllerClass::readByte(const byte registerNumber)
{
  Wire1.beginTransmission(ADDRESS);
  Wire1.write(registerNumber);
  const byte writeStatus = Wire1.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = Wire1.requestFrom(int(ADDRESS), 1);
  if (bytesRead != 1)
  {
    error = true;
    return 0;
  }
  return Wire1.read();
}

word ESAT_BatteryControllerClass::readWord(const byte registerNumber)
{
  Wire1.beginTransmission(ADDRESS);
  Wire1.write(registerNumber);
  const byte writeStatus = Wire1.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = Wire1.requestFrom(int(ADDRESS), 2);
  if (bytesRead != 2)
  {
    error = true;
    return 0;
  }
  const byte lowByte = Wire1.read();
  const byte highByte = Wire1.read();
  return word(highByte, lowByte);
}

ESAT_BatteryControllerClass ESAT_BatteryController;
