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

#include <Wire.h>
#include "ESATBatteryController.h"

ESATBatteryController::ESATBatteryController(): error(false)
{
}

int ESATBatteryController::readBattery1Voltage()
{
  return readInteger(battery1VoltageRegister);
}

int ESATBatteryController::readBattery2Voltage()
{
  return readInteger(battery2VoltageRegister);
}


int ESATBatteryController::readBatteryCurrent()
{
  return readInteger(batteryCurrentRegister);
}


int ESATBatteryController::readBatteryTemperature()
{
  return readInteger(batteryTemperatureRegister);
}


byte ESATBatteryController::readStateOfCharge()
{
  return readByte(stateOfChargeRegister);
}


int ESATBatteryController::readTotalBatteryVoltage()
{
  return readInteger(totalBatteryVoltageRegister);
}

byte ESATBatteryController::readByte(const byte registerName)
{
  Wire.beginTransmission(address);
  Wire1.write(registerName);
  const byte wireStatus = Wire1.endTransmission();
  if (wireStatus == 0)
  {
    Wire1.requestFrom(address, 1);
    return Wire1.read();
  }
  else
  {
    error = true;
  }
}

int ESATBatteryController::readInteger(const byte registerName)
{
  Wire1.beginTransmission(address);
  Wire1.write(registerName);
  const byte wireStatus = Wire1.endTransmission();
  if (wireStatus == 0)
  {
    Wire1.requestFrom(address, 2);
    const byte lowByte = Wire1.read();
    const byte highByte = Wire1.read();
    return word(highByte, lowByte);
  }
  else
  {
    error = true;
  }
}

ESATBatteryController BatteryController;
