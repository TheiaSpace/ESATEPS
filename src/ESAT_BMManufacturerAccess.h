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

#ifndef BM_COMMUNICATION_HANDLER_H
#define BM_COMMUNICATION_HANDLER_H

#include <Arduino.h>
#include "ESAT_CRC8.h"

// This handler allows to read and update the data flash of the bq40z60
// MCU (0x4000 to 0x5FFF data flash addresses). It uses the
// "alternate manufacturer access" (Command 0x44) to do it.
// It does not allow firmware programming.
class ESAT_BMManufacturerAccessClass
{
  public:
    // This method has to be called before calling any other of this class.
    ESAT_BMManufacturerAccessClass();

    // This method must be called after finishing the read/write operations.
    // It seals the MCU.
    // Return STATUS_SUCCESS when the MCU is successfully sealed, otherwise
    // return STATUS_FAIL.
    byte seal();

    // This method must be called before the read/write operations.
    // It unseals the MCU.
    // Return STATUS_SUCCESS when the MCU is successfully unsealed, otherwise
    // return STATUS_FAIL.
    byte unseal();

    // Write the dataMemory array in the MCU data flash starting in the
    // given dataMemoryAddress. It uses the "alternate manufacturer access"
    // and the CRC checksum.
    // Return STATUS_SUCCESS when the communcation with the MCU was successful,
    // otherwise return STATUS_FAIL.
    byte write(word dataMemoryAddress, byte dataMemory[], byte dataMemoryLength);

    // Write the data memory address using the "alternate manufacturer access"
    // and the CRC checksum. It is used when the data memory address is
    // actually a command.
    // Return STATUS_SUCCESS when the communcation with the MCU was successful,
    // otherwise return STATUS_FAIL.
    byte write(word dataMemoryAddress);

    // Read a data flash segment starting at "dataMemoryAddress"
    // and filling the "dataMemory" array.
    // Return STATUS_SUCCESS when the communcation with the MCU was successful,
    // otherwise return STATUS_FAIL.
    byte read(word dataMemoryAddress, byte dataMemory[], byte dataMemoryLength);

    enum statusValue
    {
      STATUS_SUCCESS = 0,
      STATUS_FAIL = 1
    };

    static const byte MEMORY_ADDRESS_FIELD_LENGTH = 2;

  private:
    enum CRCCommand
    {
      APPEND_CRC_BYTE,
      DO_NOT_APPEND_CRC_BYTE
    };

    // 7-bits BM I2C addresss.
    static const byte BM_I2C_ADDRESS = 0x0B;

    // length of the buffer used to comunicate with the BM.
    static const byte BM_COMM_BUFFER = 16;

    // SBS command used to access to the MCU data flash.
    static const byte ALTERNATE_MANUFACTURER_ACCESS_COMMAND_ID = 0x44;

    // milliseconds waited after any communication.
    static const byte DELAY_MILLIS = 5;

    // The operation status register is used to read the current security mode.
    static const word OPERATION_STATUS_MEMORY_ADDRESS = 0x0054;
    static const byte OPERATION_STATUS_LENGTH = 4;
    byte operationStatus[OPERATION_STATUS_LENGTH];
    static const byte OPERATION_STATUS_SEALED      = 0B11;
    static const byte OPERATION_STATUS_UNSEALED    = 0B10;
    static const byte OPERATION_STATUS_FULL_ACCESS = 0B01;

    // security modes passwords (or memory addresses).
    static const word SEAL_MEMORY_ADDRESS = 0x0030;
    static constexpr word UNSEAL_MEMORY_ADDRESSES[2] = {0x0414, 0x3672};

    // TC packet definition
    // 1. TC header.
    //      "command ID" field (1 byte).
    //      "Packet data length" field (1 byte).
    // 2. Packet data field.
    //      "memory address" field (2 bytes).
    //      "user data" fields (variable).
    // 3. TC footer.
    //      "CRC8 checksum" field (1 byte).
    static const byte TC_HEADER_LENGTH = 2;
    static const byte TC_MEMORY_ADDRESS_FIELD_LENGTH = MEMORY_ADDRESS_FIELD_LENGTH;
    static const byte TC_FOOTER_LENGTH = 1;
    static const byte TC_USER_DATA_MAX_LENGTH = BM_COMM_BUFFER
                                               - TC_HEADER_LENGTH
                                               - TC_MEMORY_ADDRESS_FIELD_LENGTH
                                               - TC_FOOTER_LENGTH;

    // TM packet definition
    // 1. TM header.
    //      "Packet data length" field (1 byte).
    // 2. Packet data field.
    //      "memory address" field (2 bytes).
    //      "user data" fields (variable).
    // 3. TM footer.
    //      "CRC8 checksum" field (1 byte).
    // The telemetry length depends on the telemetry type, you cannot specify
    // it, if it is higher than BM_COMM_BUFFER, something that usually happens,
    // you cannot read the CRC byte. In this case you are loosing the last byte,
    // which is actually part of the user data. To keep it simple, we just deal
    // with it. We do not check the CRC but we do not use it as user data.
    static const byte TM_HEADER_LENGTH = 1;
    static const byte TM_MEMORY_ADDRESS_FIELD_LENGTH = MEMORY_ADDRESS_FIELD_LENGTH;
    static const byte TM_FOOTER_LENGTH = 1;
    static const byte TM_USER_DATA_MAX_LENGTH =BM_COMM_BUFFER
                                               - TM_HEADER_LENGTH
                                               - TM_MEMORY_ADDRESS_FIELD_LENGTH
                                               - TM_FOOTER_LENGTH;

    // CRC used in the writeFrame method to append a CRC byte to the I2C frame.
    ESAT_CRC8Class CRC;

    // SM Bus CRC polinomial (x8+x2+x+1)
    static const byte CRC_POLYNOMIAL = 0b00000111;

    // It received the frame to send without the CRC byte.
    // It calculates the CRC and appends it to the frame when requested
    // with "command".
    // Return STATUS_SUCCESS when the communcation with the MCU was successful,
    // otherwise return STATUS_FAIL.
    byte writeFrame(byte frame[],byte frameLength, CRCCommand command);

    // Return STATUS_SUCCESS when the communcation with the MCU was successful,
    // otherwise return STATUS_FAIL.
    byte readOperationStatus();

    // Return STATUS_SUCCESS when the communication is successfully done,
    // otherwise return STATUS_FAIL.
    byte writeSealRegister();

    // Return STATUS_SUCCESS when the communication is successfully done,
    // otherwise return STATUS_FAIL.
    byte writeUnsealRegister();
};

#endif
