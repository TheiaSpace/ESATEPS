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
  battery1Voltage(0),
  battery2Voltage(0),
  batteryCurrent(0),
  batteryTemperature(0),
  batteryStateOfCharge(0),
  totalBatteryVoltage(0),
  previousError(false),
  previousReadingTime(0)
{
}

void ESAT_BatteryControllerClass::readAll()
{
  const unsigned long readingTime = millis();
  if ((readingTime - previousReadingTime) >= PERIOD)
  {
    error = previousError;
  }
  else
  {
    previousReadingTime = readingTime;
    battery1Voltage = readWord(BATTERY_1_VOLTAGE_REGISTER);
    battery2Voltage = readWord(BATTERY_2_VOLTAGE_REGISTER);
    batteryCurrent = readWord(BATTERY_CURRENT_REGISTER);
    batteryTemperature = readWord(BATTERY_CURRENT_REGISTER);
    batteryStateOfCharge = readByte(BATTERY_STATE_OF_CHARGE_REGISTER);
    totalBatteryVoltage = readWord(TOTAL_BATTERY_VOLTAGE_REGISTER);
    previousError = error;
  }
}

word ESAT_BatteryControllerClass::readBattery1Voltage()
{
  readAll();
  return battery1Voltage;
}

word ESAT_BatteryControllerClass::readBattery2Voltage()
{
  readAll();
  return battery2Voltage;
}

word ESAT_BatteryControllerClass::readBatteryCurrent()
{
  readAll();
  return batteryCurrent;
}

byte ESAT_BatteryControllerClass::readBatteryStateOfCharge()
{
  readAll();
  return batteryStateOfCharge;
}

word ESAT_BatteryControllerClass::readBatteryTemperature()
{
  readAll();
  return batteryTemperature;
}

word ESAT_BatteryControllerClass::readTotalBatteryVoltage()
{
  readAll();
  return totalBatteryVoltage;
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
