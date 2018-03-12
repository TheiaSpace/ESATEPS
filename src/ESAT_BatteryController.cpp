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

void ESAT_BatteryControllerClass::read(word registerAddress, byte byteArray[], byte byteArraySize, Protocol theProtocol)
{
  byte writeStatus;
  byte bytesRead;
  byte operationStatus;
  switch(theProtocol)
  {
    case BLOCK_PROTOCOL:
      Wire1.beginTransmission(ADDRESS);
      Wire1.write((byte)registerAddress);
      writeStatus = Wire1.endTransmission();
      delay(DELAY_MILLIS);
      if (writeStatus != 0)
      {
        error = true;
        return;
      }
      // In the block protocol, the first sent byte is the parameter size
      bytesRead = Wire1.requestFrom((byte)ADDRESS, (byte)(1 + byteArraySize));
      delay(DELAY_MILLIS);
      if (bytesRead != 1 + byteArraySize)
      {
        error = true;
        return;
      }
      // We read the parameter size.
      (void) Wire1.read();
      for (byte index = 0; index < byteArraySize; index ++)
      {
        byteArray[index] = Wire1.read();
      }
      break;
    case MANUFACTURER_PROTOCOL:
      operationStatus = BMManufacturerAccess.read(registerAddress,
                                                       byteArray,
                                                       byteArraySize);
      if(operationStatus != BMManufacturerAccess.STATUS_SUCCESS)
      {
        error = true;
        return;
      }
      break;
    case WORD_PROTOCOL:
      Wire1.beginTransmission(ADDRESS);
      Wire1.write((byte)registerAddress);
      writeStatus = Wire1.endTransmission();
      delay(DELAY_MILLIS);
      if (writeStatus != 0)
      {
        error = true;
        return;
      }
      bytesRead = Wire1.requestFrom((byte)ADDRESS, byteArraySize);
      delay(DELAY_MILLIS);
      if (bytesRead != byteArraySize)
      {
        error = true;
        return;
      }
      for (byte index = 0; index < byteArraySize; index ++)
      {
        byteArray[index] = Wire1.read();
      }
      break;
  }
}

void ESAT_BatteryControllerClass::readAll()
{
  const unsigned long readingTime = millis();
  if ((readingTime - previousReadingTime) <= PERIOD)
  {
    error = previousError;
  }
  else
  {
    previousReadingTime = readingTime;
    battery1Voltage = readWord(BATTERY_1_VOLTAGE_REGISTER, WORD_PROTOCOL);
    battery2Voltage = readWord(BATTERY_2_VOLTAGE_REGISTER, WORD_PROTOCOL);
    batteryCurrent = readWord(BATTERY_CURRENT_REGISTER, WORD_PROTOCOL);
    batteryTemperature = readWord(BATTERY_TEMPERATURE_REGISTER, WORD_PROTOCOL);
    batteryStateOfCharge = readByte(RELATIVE_STATE_OF_CHARGE_REGISTER, WORD_PROTOCOL);
    totalBatteryVoltage = readWord(TOTAL_BATTERY_VOLTAGE_REGISTER, WORD_PROTOCOL);
    previousError = error;
  }
}

byte ESAT_BatteryControllerClass::readAbsoluteStateOfCharge()
{
  return readByte(ABSOLUTE_STATE_OF_CHARGE_REGISTER, WORD_PROTOCOL);
}

byte ESAT_BatteryControllerClass::readBalancingConfiguration()
{
  return readByte(BALANCING_CONFIGURATION_REGISTER, MANUFACTURER_PROTOCOL);
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

word ESAT_BatteryControllerClass::readBatteryTemperature()
{
  readAll();
  return batteryTemperature;
}

byte ESAT_BatteryControllerClass::readByte(word registerAddress, Protocol theProtocol)
{
  byte byteArray[1];
  read(registerAddress, byteArray, 1, theProtocol);
  return byteArray[0];
}

unsigned long ESAT_BatteryControllerClass::readChargingStatus()
{
  return readUnsignedLong(CHARGING_STATUS_REGISTER, BLOCK_PROTOCOL);
}

word ESAT_BatteryControllerClass::readChemicalID()
{
  return readWord(CHEMICAL_ID_REGISTER, MANUFACTURER_PROTOCOL);
}

byte ESAT_BatteryControllerClass::readCellOvervoltageRecoveryDelay()
{
  return readByte(CELL_OVERVOLTAGE_RECOVERY_DELAY_REGISTER, MANUFACTURER_PROTOCOL);
}

word ESAT_BatteryControllerClass::readCellOvervoltageRecoveryThreshold()
{
  return readWord(CELL_OVERVOLTAGE_RECOVERY_THRESHOLD_REGISTER, MANUFACTURER_PROTOCOL);
}

word ESAT_BatteryControllerClass::readCellOvervoltageThreshold()
{
  return readWord(CELL_OVERVOLTAGE_THRESHOLD_REGISTER, MANUFACTURER_PROTOCOL);
}

byte ESAT_BatteryControllerClass::readCellUndervoltageRecoveryDelay()
{
  return readByte(CELL_UNDERVOLTAGE_RECOVERY_DELAY_REGISTER, MANUFACTURER_PROTOCOL);
}

word ESAT_BatteryControllerClass::readCellUndervoltageRecoveryThreshold()
{
  return readWord(CELL_UNDERVOLTAGE_RECOVERY_THRESHOLD_REGISTER, MANUFACTURER_PROTOCOL);
}

word ESAT_BatteryControllerClass::readCellUndervoltageThreshold()
{
  return readWord(CELL_UNDERVOLTAGE_THRESHOLD_REGISTER, MANUFACTURER_PROTOCOL);
}

word ESAT_BatteryControllerClass::readCycleCount()
{
  return readWord(CYCLE_COUNT_REGISTER, WORD_PROTOCOL);
}

word ESAT_BatteryControllerClass::readDesignCapacity()
{
  return readWord(DESIGN_CAPACITY_REGISTER, WORD_PROTOCOL);
}

word ESAT_BatteryControllerClass::readDesignVoltage()
{
  return readWord(DESIGN_VOLTAGE_REGISTER, WORD_PROTOCOL);
}

word ESAT_BatteryControllerClass::readDesiredChargingCurrent()
{
  return readWord(DESIRED_CHARGING_CURRENT_REGISTER, WORD_PROTOCOL);
}

word ESAT_BatteryControllerClass::readDesiredChargingVoltage()
{
  return readWord(DESIRED_CHARGING_VOLTAGE_REGISTER, WORD_PROTOCOL);
}

byte ESAT_BatteryControllerClass::readDeviceConfiguration()
{
  return readByte(DEVICE_CONFIGURATION_REGISTER, MANUFACTURER_PROTOCOL);
}

unsigned long ESAT_BatteryControllerClass::readEnabledProtections()
{
  return readUnsignedLong(ENABLED_PROTECTIONS_REGISTER, MANUFACTURER_PROTOCOL);
}

void ESAT_BatteryControllerClass::readFirmwareVersion(byte firmwareVersion[])
{
  read(FIRMWARE_VERSION_REGISTER, firmwareVersion, BM_FIRMWARE_VERSION_LENGTH, MANUFACTURER_PROTOCOL);
}

unsigned long ESAT_BatteryControllerClass::readManufacturingStatus()
{
  return readUnsignedLong(MANUFACTURING_STATUS_REGISTER, BLOCK_PROTOCOL);
}

word ESAT_BatteryControllerClass::readMicrocontrollerTemperature()
{
  return readWord(MICROCONTROLLER_TEMPERATURE_REGISTER, WORD_PROTOCOL);
}

unsigned long ESAT_BatteryControllerClass::readOperationStatus()
{
  return readUnsignedLong(OPERATION_STATUS_REGISTER, BLOCK_PROTOCOL);
}

byte ESAT_BatteryControllerClass::readRelativeStateOfCharge()
{
  readAll();
  return batteryStateOfCharge;
}

unsigned long ESAT_BatteryControllerClass::readSafetyStatus()
{
  return readUnsignedLong(SAFETY_STATUS_REGISTER, BLOCK_PROTOCOL);
}

word ESAT_BatteryControllerClass::readSerialNumber()
{
  return readWord(SERIAL_NUMBER_REGISTER, WORD_PROTOCOL);
}

word ESAT_BatteryControllerClass::readTotalBatteryVoltage()
{
  readAll();
  return totalBatteryVoltage;
}

unsigned long ESAT_BatteryControllerClass::readUnsignedLong(word registerAddress, Protocol theProtocol)
{
  byte byteArray[4];
  read(registerAddress, byteArray, 4, theProtocol);
  return ((unsigned long)byteArray[3] << 24) |
         ((unsigned long)byteArray[2] << 16) |
         ((unsigned long)byteArray[1] << 8) |
         (unsigned long)byteArray[0];
}

word ESAT_BatteryControllerClass::readWord(word registerAddress, Protocol theProtocol)
{
  byte byteArray[2];
  read(registerAddress, byteArray, 2, theProtocol);
  return word(byteArray[1], byteArray[0]);
}

ESAT_BatteryControllerClass ESAT_BatteryController;
