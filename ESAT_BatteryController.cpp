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

word ESAT_BatteryControllerClass::readBattery1Voltage()
{
  return readWord(BATTERY_1_VOLTAGE_REGISTER);
}

word ESAT_BatteryControllerClass::readBattery2Voltage()
{
  return readWord(BATTERY_2_VOLTAGE_REGISTER);
}

word ESAT_BatteryControllerClass::readBatteryCurrent()
{
  return readWord(BATTERY_CURRENT_REGISTER);
}

word ESAT_BatteryControllerClass::readBatteryTemperature()
{
  return readWord(BATTERY_TEMPERATURE_REGISTER);
}

byte ESAT_BatteryControllerClass::readStateOfCharge()
{
  return readByte(STATE_OF_CHARGE_REGISTER);
}

word ESAT_BatteryControllerClass::readTotalBatteryVoltage()
{
  return readWord(TOTAL_BATTERY_VOLTAGE_REGISTER);
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
