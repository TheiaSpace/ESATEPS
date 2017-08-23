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

#include <ESATI2CDevice.h>
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
  ESATI2CDevice device(Wire1, address);
  const byte measurement = device.readByte(registerNumber);
  if (device.error)
  {
    error = true;
  }
  return measurement;
}

word ESATBatteryController::readWord(const byte registerNumber)
{
  ESATI2CDevice device(Wire1, address);
  const word measurement = device.readLittleEndianWord(registerNumber);
  if (device.error)
  {
    error = true;
  }
  return measurement;
}

ESATBatteryController BatteryController;
