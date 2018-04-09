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
  chemicalId = 0;
  cycleCount = 0;
  designCapacity = 0;
  designVoltage = 0;
  desiredChargingCurrent = 0;
  desiredChargingVoltage = 0;
  deviceConfiguration = 0;
  enabledProtections = 0;
  for(byte index = 0; index < BM_FIRMWARE_VERSION_LENGTH; index ++)
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

void ESAT_BatteryControllerClass::read(word registerAddress, byte byteArray[], byte byteArraySize, Protocol theProtocol)
{
  switch(theProtocol)
  {
    case BLOCK_PROTOCOL:
      readWithBlockProtocol(registerAddress, byteArray, byteArraySize);
      break;
    case MANUFACTURER_PROTOCOL:
      readWithManufacturerProtocol(registerAddress, byteArray, byteArraySize);
      break;
    case WORD_PROTOCOL:
      readWithWordProtocol(registerAddress, byteArray, byteArraySize);
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
    previousReadingTime               = readingTime;
    balancingConfiguration            = readByte(BALANCING_CONFIGURATION_REGISTER, MANUFACTURER_PROTOCOL);
    battery1Voltage                   = readWord(BATTERY_1_VOLTAGE_REGISTER, WORD_PROTOCOL);
    battery2Voltage                   = readWord(BATTERY_2_VOLTAGE_REGISTER, WORD_PROTOCOL);
    batteryAbsoluteStateOfCharge      = readByte(ABSOLUTE_STATE_OF_CHARGE_REGISTER, WORD_PROTOCOL);
    batteryCurrent                    = readWord(BATTERY_CURRENT_REGISTER, WORD_PROTOCOL);
    batteryRelativeStateOfCharge      = readByte(RELATIVE_STATE_OF_CHARGE_REGISTER, WORD_PROTOCOL);
    batteryTemperature                = readWord(BATTERY_TEMPERATURE_REGISTER, WORD_PROTOCOL);
    cellOvervoltageRecoveryDelay      = readByte(CELL_OVERVOLTAGE_RECOVERY_DELAY_REGISTER, MANUFACTURER_PROTOCOL);
    cellOvervoltageRecoveryThreshold  = readWord(CELL_OVERVOLTAGE_RECOVERY_THRESHOLD_REGISTER, MANUFACTURER_PROTOCOL);
    cellOvervoltageThreshold          = readWord(CELL_OVERVOLTAGE_THRESHOLD_REGISTER, MANUFACTURER_PROTOCOL);
    cellUndervoltageRecoveryDelay     = readByte(CELL_UNDERVOLTAGE_RECOVERY_DELAY_REGISTER, MANUFACTURER_PROTOCOL);
    cellUndervoltageRecoveryThreshold = readWord(CELL_UNDERVOLTAGE_RECOVERY_THRESHOLD_REGISTER, MANUFACTURER_PROTOCOL);
    cellUndervoltageThreshold         = readWord(CELL_UNDERVOLTAGE_THRESHOLD_REGISTER, MANUFACTURER_PROTOCOL);
    chargingStatus                    = readUnsignedLong(CHARGING_STATUS_REGISTER, BLOCK_PROTOCOL);
    chemicalId                        = readWord(CHEMICAL_ID_REGISTER, MANUFACTURER_PROTOCOL);
    cycleCount                        = readWord(CYCLE_COUNT_REGISTER, WORD_PROTOCOL);
    designCapacity                    = readWord(DESIGN_CAPACITY_REGISTER, WORD_PROTOCOL);
    designVoltage                     = readWord(DESIGN_VOLTAGE_REGISTER, WORD_PROTOCOL);
    desiredChargingCurrent            = readWord(DESIRED_CHARGING_CURRENT_REGISTER, MANUFACTURER_PROTOCOL);
    desiredChargingVoltage            = readWord(DESIRED_CHARGING_VOLTAGE_REGISTER, MANUFACTURER_PROTOCOL);
    deviceConfiguration               = readByte(DEVICE_CONFIGURATION_REGISTER, MANUFACTURER_PROTOCOL);
    enabledProtections                = readUnsignedLong(ENABLED_PROTECTIONS_REGISTER, MANUFACTURER_PROTOCOL);
    read(FIRMWARE_VERSION_REGISTER, firmwareVersion, BM_FIRMWARE_VERSION_LENGTH, MANUFACTURER_PROTOCOL);
    manufacturingStatus               = readUnsignedLong(MANUFACTURING_STATUS_REGISTER, BLOCK_PROTOCOL);
    microcontrollerTemperature        = readWord(MICROCONTROLLER_TEMPERATURE_REGISTER, MANUFACTURER_PROTOCOL);
    operationStatus                   = readUnsignedLong(OPERATION_STATUS_REGISTER, MANUFACTURER_PROTOCOL);
    safetyStatus                      = readUnsignedLong(SAFETY_STATUS_REGISTER, BLOCK_PROTOCOL);
    serialNumber                      = readWord(SERIAL_NUMBER_REGISTER, WORD_PROTOCOL);
    totalBatteryVoltage               = readWord(TOTAL_BATTERY_VOLTAGE_REGISTER, WORD_PROTOCOL);
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

byte ESAT_BatteryControllerClass::readByte(word registerAddress, Protocol theProtocol)
{
  byte byteArray[1];
  read(registerAddress, byteArray, 1, theProtocol);
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
  readAll();
  return chemicalId;
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
  for(byte index = 0; index < BM_FIRMWARE_VERSION_LENGTH; index ++)
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

unsigned long ESAT_BatteryControllerClass::readUnsignedLong(word registerAddress, Protocol theProtocol)
{
  byte byteArray[4];
  read(registerAddress, byteArray, 4, theProtocol);
  return ((unsigned long)byteArray[3] << 24) |
         ((unsigned long)byteArray[2] << 16) |
         ((unsigned long)byteArray[1] << 8) |
         (unsigned long)byteArray[0];
}

void ESAT_BatteryControllerClass::readWithBlockProtocol(word registerAddress, byte byteArray[], byte byteArraySize)
{
  Wire1.beginTransmission(ADDRESS);
  Wire1.write((byte)registerAddress);
  byte writeStatus = Wire1.endTransmission();
  delay(DELAY_MILLIS);
  if (writeStatus != 0)
  {
    error = true;
    return;
  }
  // In the block protocol, the first sent byte is the parameter size
  byte bytesRead = Wire1.requestFrom((byte)ADDRESS, (byte)(1 + byteArraySize));
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
}

void ESAT_BatteryControllerClass::readWithManufacturerProtocol(word registerAddress, byte byteArray[], byte byteArraySize)
{
  byte nFrames = ceil((float)byteArraySize/TM_USER_DATA_MAX_LENGTH);
  word actualRegisterAddress;
  byte registerAddressLow;
  byte registerAddressHigh;
  byte receivedRegisterAddressLow;
  byte receivedRegisterAddressHigh;
  byte theStatus;
  byte userDataLength;
  byte TCFrame[BM_COMM_BUFFER - 1];
  byte TCFrameIndx;
  byte nBytesToRequest;
  byte receivedPacketDataLength;
  for(byte frame = 0; frame < nFrames; frame++)
  {
    // Data memory address.
    actualRegisterAddress = registerAddress + frame*TM_USER_DATA_MAX_LENGTH;
    registerAddressLow = actualRegisterAddress % 0x100;
    registerAddressHigh = (actualRegisterAddress / 0x100)%0x100;
    // Frame length.
    if(frame == (nFrames - 1))
    {
      userDataLength = byteArraySize - (nFrames-1)*TM_USER_DATA_MAX_LENGTH;
    }
    else
    {
      userDataLength = TM_USER_DATA_MAX_LENGTH;
    }
    nBytesToRequest = userDataLength +
                      TM_MEMORY_ADDRESS_FIELD_LENGTH +
                      TM_HEADER_LENGTH +
                      TM_FOOTER_LENGTH;
    // Send data memory address.
    write(actualRegisterAddress);
    if(error)
    {
      return;
    }
    delay(DELAY_MILLIS);
    // Send command to request the data.
    TCFrameIndx = 0;
    TCFrame[TCFrameIndx] = ALTERNATE_MANUFACTURER_ACCESS_COMMAND_ID;
    TCFrameIndx++;
    writeFrame(TCFrame, TCFrameIndx,DO_NOT_APPEND_CRC_BYTE);
    if(error)
    {
      return;
    }
    delay(DELAY_MILLIS);
    // request the data memory.
    byte nBytesReceived = Wire1.requestFrom(ADDRESS, nBytesToRequest);
    delay(DELAY_MILLIS);
    if(nBytesReceived != nBytesToRequest)
    {
      error = true;
      return;
    }
    receivedPacketDataLength = Wire1.read();
    receivedRegisterAddressLow = Wire1.read();
    receivedRegisterAddressHigh = Wire1.read();
    for(byte indx = 0; indx < userDataLength; indx++)
    {
      byteArray[(TM_USER_DATA_MAX_LENGTH*frame) + indx] = Wire1.read();
    }
    // We do noting with the last byte. We do not know if it is a CRC byte or
    // part of the user data.
    (void) Wire1.read();
    if(receivedPacketDataLength < (userDataLength +
                                   TM_MEMORY_ADDRESS_FIELD_LENGTH))
    {
      error = true;
      return;
    }
    if((receivedRegisterAddressLow != registerAddressLow) ||
      (receivedRegisterAddressHigh != registerAddressHigh))
    {
      error = true;
      return;
    }
  }
}

void ESAT_BatteryControllerClass::readWithWordProtocol(word registerAddress, byte byteArray[], byte byteArraySize)
{
  Wire1.beginTransmission(ADDRESS);
  Wire1.write((byte)registerAddress);
  byte writeStatus = Wire1.endTransmission();
  delay(DELAY_MILLIS);
  if (writeStatus != 0)
  {
    error = true;
    return;
  }
  byte bytesRead = Wire1.requestFrom((byte)ADDRESS, byteArraySize);
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
}

word ESAT_BatteryControllerClass::readWord(word registerAddress, Protocol theProtocol)
{
  byte byteArray[2];
  read(registerAddress, byteArray, 2, theProtocol);
  return word(byteArray[1], byteArray[0]);
}

void ESAT_BatteryControllerClass::seal()
{
  writeSealRegister();
  if(error)
  {
    return;
  }
  operationStatus = readUnsignedLong(OPERATION_STATUS_REGISTER, MANUFACTURER_PROTOCOL);
  if(error)
  {
    return;
  }
  unsigned long securityMode = operationStatus & OPERATION_STATUS_SECURITY_MODE_MASK;
  if(securityMode != OPERATION_STATUS_SECURITY_MODE_SEALED)
  {
    error = true;
  }
}

void ESAT_BatteryControllerClass::unseal()
{
  writeUnsealRegister();
  if(error)
  {
    return;
  }
  operationStatus = readUnsignedLong(OPERATION_STATUS_REGISTER, MANUFACTURER_PROTOCOL);
  if(error)
  {
    return;
  }
  unsigned long securityMode = operationStatus & OPERATION_STATUS_SECURITY_MODE_MASK;
  if ((securityMode != OPERATION_STATUS_SECURITY_MODE_UNSEALED) &&
      (securityMode != OPERATION_STATUS_SECURITY_MODE_FULL_ACCESS))
  {
    error = true;
  }
}

void ESAT_BatteryControllerClass::write(word dataMemoryAddress,
                                         byte dataMemory[],
                                         byte dataMemoryLength)
{
  byte nFrames;
  nFrames = ceil((float)dataMemoryLength/TC_USER_DATA_MAX_LENGTH);
  word actualDataMemoryAddress;
  byte dataMemoryAddressLow;
  byte dataMemoryAddressHigh;
  byte PacketDataLength;
  byte userDataLength;
  // We fill the whole frame except the checksum byte.
  byte TCFrame[BM_COMM_BUFFER - 1];
  byte TCFrameIndx;
  for(byte frame = 0; frame < nFrames; frame++)
  {
    // Data memory address.
    actualDataMemoryAddress = dataMemoryAddress + frame*TC_USER_DATA_MAX_LENGTH;
    dataMemoryAddressLow = actualDataMemoryAddress % 0x100;
    dataMemoryAddressHigh = (actualDataMemoryAddress / 0x100)%0x100;
    // Frame length.
    if(frame == (nFrames - 1))
    {
      userDataLength = dataMemoryLength - (nFrames-1)*TC_USER_DATA_MAX_LENGTH;
    }
    else
    {
      userDataLength = TC_USER_DATA_MAX_LENGTH;
    }
    PacketDataLength = userDataLength + TC_MEMORY_ADDRESS_FIELD_LENGTH;
    TCFrameIndx = 0;
    TCFrame[TCFrameIndx] = ALTERNATE_MANUFACTURER_ACCESS_COMMAND_ID;
    TCFrameIndx++;
    TCFrame[TCFrameIndx] = PacketDataLength;
    TCFrameIndx++;
    TCFrame[TCFrameIndx] = dataMemoryAddressLow;
    TCFrameIndx++;
    TCFrame[TCFrameIndx] = dataMemoryAddressHigh;
    TCFrameIndx++;
    for(byte indx = 0; indx < userDataLength; indx++)
    {
      TCFrame[TCFrameIndx] = dataMemory[(TC_USER_DATA_MAX_LENGTH*frame) + indx];
      TCFrameIndx++;
    }
    writeFrame(TCFrame, TCFrameIndx, APPEND_CRC_BYTE);
    if(error)
    {
      return;
    }
  }
}

void ESAT_BatteryControllerClass::write(word dataMemoryAddress)
{
  byte dataMemoryAddressLow;
  byte dataMemoryAddressHigh;
  byte PacketDataLength;
  byte userDataLength;
  // We fill the whole frame except the checksum byte.
  byte TCFrame[BM_COMM_BUFFER - 1];
  byte TCFrameIndx;
  // Data memory address.
  dataMemoryAddressLow = dataMemoryAddress % 0x100;
  dataMemoryAddressHigh = (dataMemoryAddress / 0x100)%0x100;
  // Frame length.
  userDataLength = 0;
  PacketDataLength = userDataLength + TC_MEMORY_ADDRESS_FIELD_LENGTH;
  TCFrameIndx = 0;
  TCFrame[TCFrameIndx] = ALTERNATE_MANUFACTURER_ACCESS_COMMAND_ID;
  TCFrameIndx++;
  TCFrame[TCFrameIndx] = PacketDataLength;
  TCFrameIndx++;
  TCFrame[TCFrameIndx] = dataMemoryAddressLow;
  TCFrameIndx++;
  TCFrame[TCFrameIndx] = dataMemoryAddressHigh;
  TCFrameIndx++;
  writeFrame(TCFrame, TCFrameIndx,APPEND_CRC_BYTE);
}

void ESAT_BatteryControllerClass::writeFrame(byte frame[],
                                              byte frameLength,
                                              CRCCommand command)
{
  Wire1.beginTransmission(ADDRESS);
  for(byte indx = 0; indx < frameLength; indx++)
  {
    Wire1.write(frame[indx]);
  }
  if(command == APPEND_CRC_BYTE)
  {
    byte CRCValue;
    byte message[1];
    message[0] = ADDRESS << 1;
    // We have to include the I2C address with the read/write bit to
    // calculate the CRC.
    CRCValue = CRC.read(message,1);
    CRCValue = CRC.add(CRCValue,frame,frameLength);
    Wire1.write(CRCValue);
  }
  if(Wire1.endTransmission())
  {
    error = true;
  }
  delay(DELAY_MILLIS);
}

void ESAT_BatteryControllerClass::writeSealRegister()
{
  write(SEAL_REGISTER);
}

void ESAT_BatteryControllerClass::writeUnsealRegister()
{
  for(byte indx = 0; indx < (sizeof(UNSEAL_REGISTERS)/2); indx++)
  {
    write(UNSEAL_REGISTERS[indx]);
    if(error)
    {
      return;
    }
  }
}

ESAT_BatteryControllerClass ESAT_BatteryController;
