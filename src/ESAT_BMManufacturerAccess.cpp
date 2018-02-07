/*
 * This file is part of Theia Space's ESAT EPS library.
 *
 * Theia Space's ESAT BM programmer is free software: you can
 * redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Theia Space's ESAT BM programmer is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's ESAT OBC library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "ESAT_BMManufacturerAccess.h"
#include <Wire.h>

ESAT_BMManufacturerAccessClass::ESAT_BMManufacturerAccessClass()
{
  CRC.begin(CRC_POLYNOMIAL);
}

byte ESAT_BMManufacturerAccessClass::readOperationStatus()
{
  return read(OPERATION_STATUS_MEMORY_ADDRESS,
              operationStatus,
              OPERATION_STATUS_LENGTH);
}

byte ESAT_BMManufacturerAccessClass::write(word dataMemoryAddress,
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
  byte theStatus;
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
    theStatus = writeFrame(TCFrame, TCFrameIndx, APPEND_CRC_BYTE);
    if(theStatus == STATUS_FAIL)
    {
      return STATUS_FAIL;
    }
  }
  return STATUS_SUCCESS;
}

byte ESAT_BMManufacturerAccessClass::write(word dataMemoryAddress)
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
  return writeFrame(TCFrame, TCFrameIndx,APPEND_CRC_BYTE);
}

byte ESAT_BMManufacturerAccessClass::writeFrame(byte frame[],
                                              byte frameLength,
                                              CRCCommand command)
{
  byte error;
  byte theStatus;
  Wire1.beginTransmission(BM_I2C_ADDRESS);
  for(byte indx = 0; indx < frameLength; indx++)
  {
    Wire1.write(frame[indx]);
  }
  if(command == APPEND_CRC_BYTE)
  {
    byte CRCValue;
    byte message[1];
    message[0] = BM_I2C_ADDRESS << 1;
    // We have to include the I2C address with the read/write bit to
    // calculate the CRC.
    CRCValue = CRC.read(message,1);
    CRCValue = CRC.add(CRCValue,frame,frameLength);
    Wire1.write(CRCValue);
  }
  error =  Wire1.endTransmission();
  delay(DELAY_MILLIS);
  // when we send the CRC byte, the BM knows that is the last byte,
  // and it responds with a NACK to end the transmission.
  switch(error)
  {
    case TWI_ERRROR_NO_ERROR:
      theStatus = STATUS_SUCCESS;
      break;
    default:
      theStatus = STATUS_FAIL;
      break;
  }
  return theStatus;
}

byte ESAT_BMManufacturerAccessClass::read(word dataMemoryAddress,
                                          byte dataMemory[],
                                          byte dataMemoryLength)
{
  byte nFrames = ceil((float)dataMemoryLength/TM_USER_DATA_MAX_LENGTH);
  word actualDataMemoryAddress;
  byte dataMemoryAddressLow;
  byte dataMemoryAddressHigh;
  byte receivedDataMemoryAddressLow;
  byte receivedDataMemoryAddressHigh;
  byte error;
  byte theStatus;
  byte userDataLength;
  byte TCFrame[BM_COMM_BUFFER - 1];
  byte TCFrameIndx;
  byte nBytesToRequest;
  byte receivedPacketDataLength;
  for(byte frame = 0; frame < nFrames; frame++)
  {
    // Data memory address.
    actualDataMemoryAddress = dataMemoryAddress + frame*TM_USER_DATA_MAX_LENGTH;
    dataMemoryAddressLow = actualDataMemoryAddress % 0x100;
    dataMemoryAddressHigh = (actualDataMemoryAddress / 0x100)%0x100;
    // Frame length.
    if(frame == (nFrames - 1))
    {
      userDataLength = dataMemoryLength - (nFrames-1)*TM_USER_DATA_MAX_LENGTH;
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
    theStatus = write(actualDataMemoryAddress);
    if(theStatus)
    {
      return STATUS_FAIL;
    }
    delay(DELAY_MILLIS);
    // Send command to request the data.
    TCFrameIndx = 0;
    TCFrame[TCFrameIndx] = ALTERNATE_MANUFACTURER_ACCESS_COMMAND_ID;
    TCFrameIndx++;
    error = writeFrame(TCFrame, TCFrameIndx,DO_NOT_APPEND_CRC_BYTE);
    if(error)
    {
      return error;
    }
    delay(DELAY_MILLIS);
    // request the data memory.
    byte nBytesReceived = Wire1.requestFrom(BM_I2C_ADDRESS, nBytesToRequest);
    delay(DELAY_MILLIS);
    if(nBytesReceived != nBytesToRequest)
    {
      return STATUS_FAIL;
    }
    receivedPacketDataLength = Wire1.read();
    receivedDataMemoryAddressLow = Wire1.read();
    receivedDataMemoryAddressHigh = Wire1.read();
    for(byte indx = 0; indx < userDataLength; indx++)
    {
      dataMemory[(TM_USER_DATA_MAX_LENGTH*frame) + indx] = Wire1.read();
    }
    // We do noting with the last byte. We do not know if it is a CRC byte or
    // part of the user data.
    (void) Wire1.read();
    if(receivedPacketDataLength < (userDataLength +
                                   TM_MEMORY_ADDRESS_FIELD_LENGTH))
    {
      return STATUS_FAIL;
    }
    if((receivedDataMemoryAddressLow != dataMemoryAddressLow) ||
      (receivedDataMemoryAddressHigh != dataMemoryAddressHigh))
    {
      return STATUS_FAIL;
    }
  }
  return STATUS_SUCCESS;
}

byte ESAT_BMManufacturerAccessClass::seal()
{
  if(readOperationStatus() == STATUS_FAIL)
  {
    return STATUS_FAIL;
  }
  if((operationStatus[1] & 0B11) == OPERATION_STATUS_SEALED)
  {
    return STATUS_SUCCESS;
  }
  if(seal() == STATUS_FAIL)
  {
    return STATUS_FAIL;
  }
  if(readOperationStatus() == STATUS_FAIL)
  {
    return STATUS_FAIL;
  }
  if((operationStatus[1] & 0B11) == OPERATION_STATUS_SEALED)
  {
    return STATUS_SUCCESS;
  }
  else
  {
    return STATUS_FAIL;
  }
}

byte ESAT_BMManufacturerAccessClass::unseal()
{
  if(readOperationStatus() == STATUS_FAIL)
  {
    return STATUS_FAIL;
  }
  if(((operationStatus[1] & 0B11) == OPERATION_STATUS_FULL_ACCESS) ||
     ((operationStatus[1] & 0B11) == OPERATION_STATUS_UNSEALED))
  {
    return STATUS_SUCCESS;
  }
  if(writeUnsealRegister() == STATUS_FAIL)
  {
    return STATUS_FAIL;
  }
  if(readOperationStatus() == STATUS_FAIL)
  {
    return STATUS_FAIL;
  }
  if(((operationStatus[1] & 0B11) == OPERATION_STATUS_FULL_ACCESS) ||
     ((operationStatus[1] & 0B11) == OPERATION_STATUS_UNSEALED))
  {
    return STATUS_SUCCESS;
  }
  else
  {
    return STATUS_FAIL;
  }
}

byte ESAT_BMManufacturerAccessClass::writeSealRegister()
{
  return write(SEAL_MEMORY_ADDRESS);
}

byte ESAT_BMManufacturerAccessClass::writeUnsealRegister()
{
  byte theStatus;
  for(byte indx = 0; indx < (sizeof(UNSEAL_MEMORY_ADDRESSES)/2); indx++)
  {
    theStatus = write(UNSEAL_MEMORY_ADDRESSES[indx]);
    if(theStatus == STATUS_FAIL)
    {
      return STATUS_FAIL;
    }
  }
  return STATUS_SUCCESS;
}
