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

word ESATBatteryController::readBattery1Voltage()
{
  return readWord(battery1VoltageRegister);
}

word ESATBatteryController::readBattery2Voltage()
{
  return readWord(battery2VoltageRegister);
}


word ESATBatteryController::readBatteryCurrent()
{
  return readWord(batteryCurrentRegister);
}


word ESATBatteryController::readBatteryTemperature()
{
  return readWord(batteryTemperatureRegister);
}


word ESATBatteryController::readStateOfCharge()
{
  return word(readByte(stateOfChargeRegister));
}


word ESATBatteryController::readTotalBatteryVoltage()
{
  return readWord(totalBatteryVoltageRegister);
}

byte ESATBatteryController::readByte(const byte registerNumber)
{
  Wire.beginTransmission(address);
  Wire1.write(registerNumber);
  const byte wireStatus = Wire1.endTransmission();
  if (wireStatus == 0)
  {
    Wire1.requestFrom((uint8_t) address, (uint8_t) 1);
    return Wire1.read();
  }
  else
  {
    error = true;
    return 0;
  }
}

word ESATBatteryController::readWord(const byte registerNumber)
{
  Wire1.beginTransmission(address);
  Wire1.write(registerNumber);
  const byte wireStatus = Wire1.endTransmission();
  if (wireStatus == 0)
  {
    Wire1.requestFrom((uint8_t) address, (uint8_t) 2);
    const byte lowByte = Wire1.read();
    const byte highByte = Wire1.read();
    return word(highByte, lowByte);
  }
  else
  {
    error = true;
    return 0;
  }
}

ESATBatteryController BatteryController;
