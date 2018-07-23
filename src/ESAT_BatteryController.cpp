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
#include <ESAT_Buffer.h>
#include <ESAT_Util.h>
#include <Wire.h>

ESAT_BatteryControllerClass::ESAT_BatteryControllerClass()
{
  balancingConfiguration = 0;
  battery1Voltage = 0;
  battery2Voltage = 0;
  batteryAbsoluteStateOfCharge = 0;
  batteryCurrent = 0;
  batteryRelativeStateOfCharge = 0;
  batteryTemperature = 0;
  cellOvervoltageRecoveryDelay = 0;
  cellOvervoltageRecoveryThreshold = 0;
  cellOvervoltageThreshold = 0;
  cellUndervoltageRecoveryDelay = 0;
  cellUndervoltageRecoveryThreshold = 0;
  cellUndervoltageThreshold = 0;
  chargingStatus = 0;
  chemicalIdentifier = 0;
  cycleCount = 0;
  designCapacity = 0;
  designVoltage = 0;
  desiredChargingCurrent = 0;
  desiredChargingVoltage = 0;
  deviceConfiguration = 0;
  enabledProtections = 0;
  for (byte index = 0; index < BM_FIRMWARE_VERSION_LENGTH; index++)
  {
    firmwareVersion[index] = 0;
  }
  manufacturingStatus = 0;
  microcontrollerTemperature = 0;
  operationStatus = 0;
  safetyStatus = 0;
  serialNumber = 0;
  totalBatteryVoltage = 0;
  previousError = false;
  previousReadingTime = 0;
  CRC.begin(CRC_POLYNOMIAL);
}

boolean ESAT_BatteryControllerClass::read(const word registerAddress,
                                          byte byteArray[],
                                          const byte byteArraySize,
                                          const Protocol theProtocol)
{
  switch (theProtocol)
  {
    case BLOCK_PROTOCOL:
      return readWithBlockProtocol(registerAddress,
                                   byteArray,
                                   byteArraySize);
      break;
    case MANUFACTURER_PROTOCOL:
      return readWithManufacturerProtocol(registerAddress,
                                          byteArray,
                                          byteArraySize);
      break;
    case WORD_PROTOCOL:
      return readWithWordProtocol(registerAddress,
                                  byteArray,
                                  byteArraySize);
      break;
    default:
      return false;
      break;
  }
  return false;
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
    previousReadingTime               = readingTime;
    balancingConfiguration            = readByte(BALANCING_CONFIGURATION_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    battery1Voltage                   = readWord(BATTERY_1_VOLTAGE_REGISTER,
                                                 WORD_PROTOCOL);
    battery2Voltage                   = readWord(BATTERY_2_VOLTAGE_REGISTER,
                                                 WORD_PROTOCOL);
    batteryAbsoluteStateOfCharge      = readByte(ABSOLUTE_STATE_OF_CHARGE_REGISTER,
                                                 WORD_PROTOCOL);
    batteryCurrent                    = readWord(BATTERY_CURRENT_REGISTER,
                                                 WORD_PROTOCOL);
    batteryRelativeStateOfCharge      = readByte(RELATIVE_STATE_OF_CHARGE_REGISTER,
                                                 WORD_PROTOCOL);
    batteryTemperature                = readWord(BATTERY_TEMPERATURE_REGISTER,
                                                 WORD_PROTOCOL);
    cellOvervoltageRecoveryDelay      = readByte(CELL_OVERVOLTAGE_RECOVERY_DELAY_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    cellOvervoltageRecoveryThreshold  = readWord(CELL_OVERVOLTAGE_RECOVERY_THRESHOLD_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    cellOvervoltageThreshold          = readWord(CELL_OVERVOLTAGE_THRESHOLD_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    cellUndervoltageRecoveryDelay     = readByte(CELL_UNDERVOLTAGE_RECOVERY_DELAY_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    cellUndervoltageRecoveryThreshold = readWord(CELL_UNDERVOLTAGE_RECOVERY_THRESHOLD_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    cellUndervoltageThreshold         = readWord(CELL_UNDERVOLTAGE_THRESHOLD_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    chargingStatus                    = readUnsignedLong(CHARGING_STATUS_REGISTER,
                                                         BLOCK_PROTOCOL);
    chemicalIdentifier                = readWord(CHEMICAL_IDENTIFIER_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    cycleCount                        = readWord(CYCLE_COUNT_REGISTER,
                                                 WORD_PROTOCOL);
    designCapacity                    = readWord(DESIGN_CAPACITY_REGISTER,
                                                 WORD_PROTOCOL);
    designVoltage                     = readWord(DESIGN_VOLTAGE_REGISTER,
                                                 WORD_PROTOCOL);
    desiredChargingCurrent            = readWord(DESIRED_CHARGING_CURRENT_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    desiredChargingVoltage            = readWord(DESIRED_CHARGING_VOLTAGE_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    deviceConfiguration               = readByte(DEVICE_CONFIGURATION_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    enabledProtections                = readUnsignedLong(ENABLED_PROTECTIONS_REGISTER,
                                                         MANUFACTURER_PROTOCOL);
    read(FIRMWARE_VERSION_REGISTER,
         firmwareVersion,
         BM_FIRMWARE_VERSION_LENGTH,
         MANUFACTURER_PROTOCOL);
    manufacturingStatus               = readUnsignedLong(MANUFACTURING_STATUS_REGISTER,
                                                         BLOCK_PROTOCOL);
    microcontrollerTemperature        = readWord(MICROCONTROLLER_TEMPERATURE_REGISTER,
                                                 MANUFACTURER_PROTOCOL);
    operationStatus                   = readUnsignedLong(OPERATION_STATUS_REGISTER,
                                                         MANUFACTURER_PROTOCOL);
    safetyStatus                      = readUnsignedLong(SAFETY_STATUS_REGISTER,
                                                         BLOCK_PROTOCOL);
    serialNumber                      = readWord(SERIAL_NUMBER_REGISTER,
                                                 WORD_PROTOCOL);
    totalBatteryVoltage               = readWord(TOTAL_BATTERY_VOLTAGE_REGISTER,
                                                 WORD_PROTOCOL);
    previousError                     = error;
  }
}

byte ESAT_BatteryControllerClass::readBalancingConfiguration()
{
  readAll();
  return balancingConfiguration;
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

byte ESAT_BatteryControllerClass::readBatteryAbsoluteStateOfCharge()
{
  readAll();
  return batteryAbsoluteStateOfCharge;
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

byte ESAT_BatteryControllerClass::readBatteryRelativeStateOfCharge()
{
  readAll();
  return batteryRelativeStateOfCharge;
}

byte ESAT_BatteryControllerClass::readByte(const word registerAddress,
                                           const Protocol theProtocol)
{
  byte byteArray[1];
  read(registerAddress, byteArray, sizeof(byteArray), theProtocol);
  return byteArray[0];
}

byte ESAT_BatteryControllerClass::readCellOvervoltageRecoveryDelay()
{
  readAll();
  return cellOvervoltageRecoveryDelay;
}

word ESAT_BatteryControllerClass::readCellOvervoltageRecoveryThreshold()
{
  readAll();
  return cellOvervoltageRecoveryThreshold;
}

word ESAT_BatteryControllerClass::readCellOvervoltageThreshold()
{
  readAll();
  return cellOvervoltageThreshold;
}

byte ESAT_BatteryControllerClass::readCellUndervoltageRecoveryDelay()
{
  readAll();
  return cellUndervoltageRecoveryDelay;
}

word ESAT_BatteryControllerClass::readCellUndervoltageRecoveryThreshold()
{
  readAll();
  return cellUndervoltageRecoveryThreshold;
}

word ESAT_BatteryControllerClass::readCellUndervoltageThreshold()
{
  readAll();
  return cellUndervoltageThreshold;
}

unsigned long ESAT_BatteryControllerClass::readChargingStatus()
{
  readAll();
  return chargingStatus;
}

word ESAT_BatteryControllerClass::readChemicalID()
{
  return readChemicalIdentifier();
}

word ESAT_BatteryControllerClass::readChemicalIdentifier()
{
  readAll();
  return chemicalIdentifier;
}

word ESAT_BatteryControllerClass::readCycleCount()
{
  readAll();
  return cycleCount;
}

word ESAT_BatteryControllerClass::readDesignCapacity()
{
  readAll();
  return designCapacity;
}

word ESAT_BatteryControllerClass::readDesignVoltage()
{
  readAll();
  return designVoltage;
}

word ESAT_BatteryControllerClass::readDesiredChargingCurrent()
{
  readAll();
  return desiredChargingCurrent;
}

word ESAT_BatteryControllerClass::readDesiredChargingVoltage()
{
  readAll();
  return desiredChargingVoltage;
}

byte ESAT_BatteryControllerClass::readDeviceConfiguration()
{
  readAll();
  return deviceConfiguration;
}

unsigned long ESAT_BatteryControllerClass::readEnabledProtections()
{
  readAll();
  return enabledProtections;
}

void ESAT_BatteryControllerClass::readFirmwareVersion(byte firmwareValue[])
{
  readAll();
  for (byte index = 0; index < BM_FIRMWARE_VERSION_LENGTH; index++)
  {
    firmwareValue[index] = firmwareVersion[index];
  }
  return;
}

unsigned long ESAT_BatteryControllerClass::readManufacturingStatus()
{
  readAll();
  return manufacturingStatus;
}

word ESAT_BatteryControllerClass::readMicrocontrollerTemperature()
{
  readAll();
  return microcontrollerTemperature;
}

unsigned long ESAT_BatteryControllerClass::readOperationStatus()
{
  readAll();
  return operationStatus;
}

unsigned long ESAT_BatteryControllerClass::readSafetyStatus()
{
  readAll();
  return safetyStatus;
}

word ESAT_BatteryControllerClass::readSerialNumber()
{
  readAll();
  return serialNumber;
}

word ESAT_BatteryControllerClass::readTotalBatteryVoltage()
{
  readAll();
  return totalBatteryVoltage;
}

unsigned long ESAT_BatteryControllerClass::readUnsignedLong(const word registerAddress,
                                                            const Protocol theProtocol)
{
  byte byteArray[4];
  read(registerAddress, byteArray, sizeof(byteArray), theProtocol);
  return ESAT_Util.unsignedLong(byteArray[3],
                                byteArray[2],
                                byteArray[1],
                                byteArray[0]);
}

boolean ESAT_BatteryControllerClass::readWithBlockProtocol(const word registerAddress,
                                                           byte byteArray[],
                                                           const byte byteArraySize)
{
  Wire1.beginTransmission(ADDRESS);
  Wire1.write(byte(registerAddress));
  const byte transmissionStatus = Wire1.endTransmission();
  delay(delayMilliseconds);
  if (transmissionStatus != 0)
  {
    error = true;
    return true;
  }
  // In the block protocol, the first sent byte is the parameter size.
  const byte bytesRead =
    Wire1.requestFrom(byte(ADDRESS), byte(1 + byteArraySize));
  delay(delayMilliseconds);
  if (bytesRead != 1 + byteArraySize)
  {
    error = true;
    return true;
  }
  // We read the parameter size.
  (void) Wire1.read();
  for (byte index = 0; index < byteArraySize; index++)
  {
    byteArray[index] = Wire1.read();
  }
  return false;
}

boolean ESAT_BatteryControllerClass::readWithManufacturerProtocol(const word registerAddress,
                                                                  byte byteArray[],
                                                                  const byte byteArraySize)
{
  const byte numberOfFrames =
    ceil(float(byteArraySize) / TELEMETRY_USER_DATA_MAX_LENGTH);
  byte telecommandFrameBuffer[BM_COMMUNICATION_BUFFER_LENGTH - 1];
  ESAT_Buffer telecommandFrame(telecommandFrameBuffer,
                               sizeof(telecommandFrameBuffer));
  for (byte frame = 0; frame < numberOfFrames; frame++)
  {
    // Data memory address.
    const word actualRegisterAddress =
      registerAddress + frame * TELEMETRY_USER_DATA_MAX_LENGTH;
    const byte registerAddressLow = actualRegisterAddress % 0x100;
    const byte registerAddressHigh = (actualRegisterAddress / 0x100) % 0x100;
    // Frame length.
    byte userDataLength;
    if (frame == (numberOfFrames - 1))
    {
      userDataLength =
        byteArraySize - (numberOfFrames - 1) * TELEMETRY_USER_DATA_MAX_LENGTH;
    }
    else
    {
      userDataLength = TELEMETRY_USER_DATA_MAX_LENGTH;
    }
    const byte numberOfBytesToRequest =
      userDataLength +
      TELEMETRY_MEMORY_ADDRESS_FIELD_LENGTH +
      TELEMETRY_HEADER_LENGTH +
      TELEMETRY_FOOTER_LENGTH;
    // Send data memory address.
    const boolean registerWriteError = write(actualRegisterAddress);
    if (registerWriteError)
    {
      return true;
    }
    // Send command to request the data.
    telecommandFrame.write(ALTERNATE_MANUFACTURER_ACCESS_COMMAND_IDENTIFIER);
    const boolean frameWriteError = writeFrame(telecommandFrameBuffer,
                                               telecommandFrame.length(),
                                               DO_NOT_APPEND_CRC_BYTE);
    if (frameWriteError)
    {
      return true;
    }
    // Request the data memory.
    const byte numberOfBytesReceived =
      Wire1.requestFrom(ADDRESS, numberOfBytesToRequest);
    delay(delayMilliseconds);
    if (numberOfBytesReceived != numberOfBytesToRequest)
    {
      error = true;
      return true;
    }
    const byte receivedPacketDataLength = Wire1.read();
    const byte receivedRegisterAddressLow = Wire1.read();
    const byte receivedRegisterAddressHigh = Wire1.read();
    for (byte index = 0; index < userDataLength; index++)
    {
      const word position = (TELEMETRY_USER_DATA_MAX_LENGTH * frame) + index;
      byteArray[position] = Wire1.read();
    }
    // We do noting with the last byte.  We do not know if it is a CRC byte or
    // part of the user data.
    (void) Wire1.read();
    if (receivedPacketDataLength < (userDataLength +
                                    TELEMETRY_MEMORY_ADDRESS_FIELD_LENGTH))
    {
      error = true;
      return true;
    }
    if ((receivedRegisterAddressLow != registerAddressLow) ||
        (receivedRegisterAddressHigh != registerAddressHigh))
    {
      error = true;
      return true;
    }
  }
  return false;
}

boolean ESAT_BatteryControllerClass::readWithWordProtocol(const word registerAddress,
                                                          byte byteArray[],
                                                          const byte byteArraySize)
{
  Wire1.beginTransmission(ADDRESS);
  Wire1.write(byte(registerAddress));
  const byte transmissionStatus = Wire1.endTransmission();
  delay(delayMilliseconds);
  if (transmissionStatus != 0)
  {
    error = true;
    return true;
  }
  const byte bytesRead = Wire1.requestFrom(byte(ADDRESS), byteArraySize);
  delay(delayMilliseconds);
  if (bytesRead != byteArraySize)
  {
    error = true;
    return true;
  }
  for (byte index = 0; index < byteArraySize; index++)
  {
    byteArray[index] = Wire1.read();
  }
  return false;
}

word ESAT_BatteryControllerClass::readWord(const word registerAddress,
                                           const Protocol theProtocol)
{
  byte byteArray[2];
  read(registerAddress, byteArray, sizeof(byteArray), theProtocol);
  return word(byteArray[1], byteArray[0]);
}

boolean ESAT_BatteryControllerClass::seal()
{
  const boolean writeError = writeSealRegister();
  if (writeError)
  {
    return true;
  }
  byte operationStatus8[4];
  const boolean readError = read(OPERATION_STATUS_REGISTER,
                                 operationStatus8,
                                 sizeof(operationStatus8),
                                 MANUFACTURER_PROTOCOL);
  if (readError)
  {
    return true;
  }
  const unsigned long operationStatus32 =
    ESAT_Util.unsignedLong(operationStatus8[3],
                           operationStatus8[2],
                           operationStatus8[1],
                           operationStatus8[0]);
  const unsigned long securityMode =
    operationStatus32 & OPERATION_STATUS_SECURITY_MODE_MASK;
  if (securityMode != OPERATION_STATUS_SECURITY_MODE_SEALED)
  {
    error = true;
    return true;
  }
  return false;
}

boolean ESAT_BatteryControllerClass::unseal()
{
  if (writeUnsealRegister())
  {
    return true;
  }
  byte operationStatus8[4];
  const boolean readError = read(OPERATION_STATUS_REGISTER,
                                 operationStatus8,
                                 sizeof(operationStatus8),
                                 MANUFACTURER_PROTOCOL);
  if (readError)
  {
    return true;
  }
  const unsigned long operationStatus32 =
    ESAT_Util.unsignedLong(operationStatus8[3],
                           operationStatus8[2],
                           operationStatus8[1],
                           operationStatus8[0]);
  const unsigned long securityMode =
    operationStatus32 & OPERATION_STATUS_SECURITY_MODE_MASK;
  if (securityMode != OPERATION_STATUS_SECURITY_MODE_FULL_ACCESS)
  {
    error = true;
    return true;
  }
  return false;
}

boolean ESAT_BatteryControllerClass::write(const word dataMemoryAddress,
                                           const byte dataMemory[],
                                           const byte dataMemoryLength)
{
  const byte numberOfFrames =
    ceil(float(dataMemoryLength) / TELECOMMAND_USER_DATA_MAX_LENGTH);
  // We fill the whole frame except the checksum byte.
  byte telecommandFrameBuffer[BM_COMMUNICATION_BUFFER_LENGTH - 1];
  ESAT_Buffer telecommandFrame(telecommandFrameBuffer,
                               sizeof(telecommandFrameBuffer));
  for (byte frame = 0; frame < numberOfFrames; frame++)
  {
    // Data memory address.
    const word actualDataMemoryAddress =
      dataMemoryAddress + frame * TELECOMMAND_USER_DATA_MAX_LENGTH;
    const byte dataMemoryAddressLow = actualDataMemoryAddress % 0x100;
    const byte dataMemoryAddressHigh = (actualDataMemoryAddress / 0x100) % 0x100;
    // Frame length.
    byte userDataLength;
    if (frame == (numberOfFrames - 1))
    {
      userDataLength =
        dataMemoryLength
        - (numberOfFrames - 1) * TELECOMMAND_USER_DATA_MAX_LENGTH;
    }
    else
    {
      userDataLength = TELECOMMAND_USER_DATA_MAX_LENGTH;
    }
    const byte packetDataLength =
      userDataLength + TELECOMMAND_MEMORY_ADDRESS_FIELD_LENGTH;
    telecommandFrame.write(ALTERNATE_MANUFACTURER_ACCESS_COMMAND_IDENTIFIER);
    telecommandFrame.write(packetDataLength);
    telecommandFrame.write(dataMemoryAddressLow);
    telecommandFrame.write(dataMemoryAddressHigh);
    for (byte index = 0; index < userDataLength; index++)
    {
      const word position = TELECOMMAND_USER_DATA_MAX_LENGTH * frame + index;
      telecommandFrame.write(dataMemory[position]);
    }
    if (writeFrame(telecommandFrameBuffer,
                   telecommandFrame.length(),
                   APPEND_CRC_BYTE))
    {
      return true;
    }
  }
  return false;
}

boolean ESAT_BatteryControllerClass::write(const word dataMemoryAddress)
{
  // We fill the whole frame except the checksum byte.
  byte telecommandFrameBuffer[BM_COMMUNICATION_BUFFER_LENGTH - 1];
  ESAT_Buffer telecommandFrame(telecommandFrameBuffer,
                               sizeof(telecommandFrameBuffer));
  // Data memory address.
  const byte dataMemoryAddressLow = dataMemoryAddress % 0x100;
  const byte dataMemoryAddressHigh = (dataMemoryAddress / 0x100) % 0x100;
  // Frame length.
  const byte userDataLength = 0;
  const byte packetDataLength =
    userDataLength + TELECOMMAND_MEMORY_ADDRESS_FIELD_LENGTH;
  telecommandFrame.write(ALTERNATE_MANUFACTURER_ACCESS_COMMAND_IDENTIFIER);
  telecommandFrame.write(packetDataLength);
  telecommandFrame.write(dataMemoryAddressLow);
  telecommandFrame.write(dataMemoryAddressHigh);
  return writeFrame(telecommandFrameBuffer,
                    telecommandFrame.length(),
                    APPEND_CRC_BYTE);
}

void ESAT_BatteryControllerClass::writeDelayBetweenCommunications(const byte delayInMilliseconds)
{
  delayMilliseconds = delayInMilliseconds;
}

boolean ESAT_BatteryControllerClass::writeFrame(const byte frame[],
                                                const byte frameLength,
                                                const CRCCommand command)
{
  Wire1.beginTransmission(ADDRESS);
  for (byte index = 0; index < frameLength; index++)
  {
    Wire1.write(frame[index]);
  }
  if (command == APPEND_CRC_BYTE)
  {
    byte CRCValue;
    const byte message[] = { ADDRESS << 1 };
    // We have to include the I2C address with the read/write bit to
    // calculate the CRC.
    CRCValue = CRC.read(message,1);
    CRCValue = CRC.add(CRCValue,frame,frameLength);
    Wire1.write(CRCValue);
  }
  const byte transmissionStatus = Wire1.endTransmission();
  delay(delayMilliseconds);
  if (transmissionStatus != 0)
  {
    error = true;
    return true;
  }
  else
  {
    return false;
  }
}

boolean ESAT_BatteryControllerClass::writeSealRegister()
{
  return write(SEAL_REGISTER);
}

boolean ESAT_BatteryControllerClass::writeUnsealRegister()
{
  for (byte index = 0;
       index < (sizeof(UNSEAL_REGISTERS) / 2);
       index++)
  {
    if (write(UNSEAL_REGISTERS[index]))
    {
      return true;
    }
  }
  for (byte index = 0;
       index < (sizeof(UNSEAL_FULL_ACCESS_REGISTERS) / 2);
       index++)
  {
    if (write(UNSEAL_FULL_ACCESS_REGISTERS[index]))
    {
      return true;
    }
  }
  return false;
}

ESAT_BatteryControllerClass ESAT_BatteryController;
