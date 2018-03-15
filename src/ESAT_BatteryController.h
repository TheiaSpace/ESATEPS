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

#ifndef ESAT_BatteryController_h
#define ESAT_BatteryController_h

#include <Arduino.h>
#include <ESAT_CRC8.h>
#include <Wire.h>

// An interface with the battery controller.
// Use the global instance ESAT_BatteryController.
//
// The underlying hardware is the BQ40Z60 programmable battery
// management unit from Texas Instruments.  Communications are done
// through the EPS I2C bus.
class ESAT_BatteryControllerClass
{
  public:
    enum Protocol
    {
      BLOCK_PROTOCOL,
      MANUFACTURER_PROTOCOL,
      WORD_PROTOCOL,
    };

    // True after a read error.  Must be reset manually.
    boolean error;

    // Number of bytes used by the BM MCU Firmware version.
    static const byte BM_FIRMWARE_VERSION_LENGTH = 11;

    // Instantiate a battery controller library.
    ESAT_BatteryControllerClass();

    // Read from "registerAddress" "contentSize" bytes and stores it in "content".
    // The protocol has to be selected among: BLOCK_PROTOCOL,
    // MANUFACTURER_PROTOCOL and WORD_PROTOCOL.
    // Set the error flag on error.
    void read(word registerAddress, byte content[], byte contentSize, Protocol theProtocol);

    // Read the battery balancing configuration.
    // Set the error flag on error.
    byte readBalancingConfiguration();

    // Read the voltage of battery number 1.
    // Set the error flag on error.
    word readBattery1Voltage();

    // Read the voltage of battery number 2.
    // Set the error flag on error.
    word readBattery2Voltage();

    // Read the absolute state of charge.
    // Set the error flag on error.
    byte readBatteryAbsoluteStateOfCharge();

    // Read the current flowing through the batteries.
    // Set the error flag on error.
    word readBatteryCurrent();

    // Read the state of charge.
    // Set the error flag on error.
    byte readBatteryRelativeStateOfCharge();

    // Read the temperature of the batteries.
    // Set the error flag on error.
    word readBatteryTemperature();

    // Read the battery charging status.
    // Set the error flag on error.
    unsigned long readChargingStatus();

    // Read the battery chemical ID.
    // Set the error flag on error.
    word readChemicalID();

    // Read the cell overvoltage recovery delay.
    // Set the error flag on error.
    byte readCellOvervoltageRecoveryDelay();

    // Read the cell overvoltage recovery threshold.
    // Set the error flag on error.
    word readCellOvervoltageRecoveryThreshold();

    // Read the cell overvoltage threshold.
    // Set the error flag on error.
    word readCellOvervoltageThreshold();

    // Read the cell undervoltage recovery delay.
    // Set the error flag on error.
    byte readCellUndervoltageRecoveryDelay();

    // Read the cell undervoltage recovery threshold.
    // Set the error flag on error.
    word readCellUndervoltageRecoveryThreshold();

    // Read the cell undervoltage threshold.
    // Set the error flag on error.
    word readCellUndervoltageThreshold();

    // Read the battery cycle count.
    // Set the error flag on error.
    word readCycleCount();

    // Read the design capacity of the battery.
    // Set the error flag on error.
    word readDesignCapacity();

    // Read the design voltage of the battery.
    // Set the error flag on error.
    word readDesignVoltage();

    // Read the desired charging current.
    // Set the error flag on error.
    word readDesiredChargingCurrent();

    // Read the desired charging voltage.
    // Set the error flag on error.
    word readDesiredChargingVoltage();

    // Read the device configuration. Corresponds to "DA Configuration".
    // Set the error flag on error.
    byte readDeviceConfiguration();

    // Read the enabled protections.
    // Set the error flag on error.
    unsigned long readEnabledProtections();

    // Read the firmware version and return it in firmwareVersion. The size of 
    // firmwareVersion has to be at least BM_FIRMWARE_VERSION_LENGTH.
    // Set the error flag on error.
    void readFirmwareVersion(byte firmwareVersion[]);

    // Read the manufacturing status.
    // Set the error flag on error.
    unsigned long readManufacturingStatus();

    // Read the microcontroller temperature.
    // Set the error flag on error.
    word readMicrocontrollerTemperature();

    // Read the operation status.
    // Set the error flag on error.
    unsigned long readOperationStatus();

    // Read the safety status.
    // Set the error flag on error.
    unsigned long readSafetyStatus();

    // Read the serial number.
    // Set the error flag on error.
    word readSerialNumber();

    // Read the total battery voltage.
    // Set the error flag on error.
    word readTotalBatteryVoltage();

    // This method must be called after finishing the write operations.
    // It seals the MCU.
    // Return STATUS_SUCCESS when the MCU is successfully sealed, otherwise
    // return STATUS_FAIL.
    byte seal();

    // This method must be called before the write operations.
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

  private:
    // I2C address of the battery controller.
    static const byte ADDRESS = 0x0B;

    // length of the buffer used to comunicate with the BM. It is get from Wire.h.
    static const byte BM_COMM_BUFFER = BUFFER_LENGTH;

    // SM Bus CRC polinomial (x8+x2+x+1)
    static const byte CRC_POLYNOMIAL = 0b00000111;

    // CRC used in the writeFrame method to append a CRC byte to the I2C frame.
    ESAT_CRC8 CRC;

    // SBS command used to access to the MCU data flash.
    static const byte ALTERNATE_MANUFACTURER_ACCESS_COMMAND_ID = 0x44;

    // The operation status register is used to read the current security mode.
    static const unsigned long OPERATION_STATUS_SECURITY_MODE_MASK        = (unsigned long)0B11 << 8;
    static const unsigned long OPERATION_STATUS_SECURITY_MODE_SEALED      = (unsigned long)0B11 << 8;
    static const unsigned long OPERATION_STATUS_SECURITY_MODE_UNSEALED    = (unsigned long)0B10 << 8;
    static const unsigned long OPERATION_STATUS_SECURITY_MODE_FULL_ACCESS = (unsigned long)0B01 << 8;

    enum CRCCommand
    {
      APPEND_CRC_BYTE,
      DO_NOT_APPEND_CRC_BYTE
    };
    
    enum statusValue
    {
      STATUS_SUCCESS = 0,
      STATUS_FAIL = 1
    };

    // Registers.
    static const word ABSOLUTE_STATE_OF_CHARGE_REGISTER = 0x0E;
    static const word BALANCING_CONFIGURATION_REGISTER = 0x4C5B;
    static const word BATTERY_1_VOLTAGE_REGISTER = 0x3F;
    static const word BATTERY_2_VOLTAGE_REGISTER = 0x3E;
    static const word BATTERY_CURRENT_REGISTER = 0x0A;
    static const word BATTERY_TEMPERATURE_REGISTER = 0x08;
    static const word CELL_OVERVOLTAGE_RECOVERY_DELAY_REGISTER = 0x485B;
    static const word CELL_OVERVOLTAGE_RECOVERY_THRESHOLD_REGISTER = 0x485C;
    static const word CELL_OVERVOLTAGE_THRESHOLD_REGISTER = 0x4859;
    static const word CELL_UNDERVOLTAGE_RECOVERY_DELAY_REGISTER = 0x484B;
    static const word CELL_UNDERVOLTAGE_RECOVERY_THRESHOLD_REGISTER = 0x484C;
    static const word CELL_UNDERVOLTAGE_THRESHOLD_REGISTER = 0x4849;
    static const word CHARGING_STATUS_REGISTER = 0x55;
    static const word CHEMICAL_ID_REGISTER = 0x0006;
    static const word CYCLE_COUNT_REGISTER = 0x17;
    static const word DESIGN_CAPACITY_REGISTER = 0x18;
    static const word DESIGN_VOLTAGE_REGISTER = 0x19;
    static const word DESIRED_CHARGING_CURRENT_REGISTER = 0x14;
    static const word DESIRED_CHARGING_VOLTAGE_REGISTER = 0x15;
    static const word DEVICE_CONFIGURATION_REGISTER = 0x495D;
    static const word ENABLED_PROTECTIONS_REGISTER = 0x4845;
    static const word FIRMWARE_VERSION_REGISTER = 0x0002;
    static const word MANUFACTURING_STATUS_REGISTER = 0x57;
    static const word MICROCONTROLLER_TEMPERATURE_REGISTER = 0x72;
    static const word OPERATION_STATUS_REGISTER = 0x54;
    static const word RELATIVE_STATE_OF_CHARGE_REGISTER = 0x0D;
    static const word SAFETY_STATUS_REGISTER = 0x51;
    static const word SEAL_REGISTER = 0x0030;
    static const word SERIAL_NUMBER_REGISTER = 0x1C;
    static const word TOTAL_BATTERY_VOLTAGE_REGISTER = 0x09;
    static constexpr word UNSEAL_REGISTERS[2] = {0x0414, 0x3672};

    // Readings may update up to once every PERIOD milliseconds.
    // The EPS cycle isn't fast enough to capture fast transients, and
    // the quasy-steady dynamics are slow, so measuring more often
    // would just waste processor time for no real benefit.
    static const unsigned long PERIOD = 1000;

    // milliseconds waited after any communication.
    static const byte DELAY_MILLIS = 2;

    // Latest readings.
    byte balancingConfiguration;
    word battery1Voltage;
    word battery2Voltage;
    byte batteryAbsoluteStateOfCharge;
    word batteryCurrent;
    byte batteryRelativeStateOfCharge;
    word batteryTemperature;
    byte cellOvervoltageRecoveryDelay;
    word cellOvervoltageRecoveryThreshold;
    word cellOvervoltageThreshold;
    byte cellUndervoltageRecoveryDelay;
    word cellUndervoltageRecoveryThreshold;
    word cellUndervoltageThreshold;
    unsigned long chargingStatus;
    word chemicalId;
    word cycleCount;
    word designCapacity;
    word designVoltage;
    word desiredChargingCurrent;
    word desiredChargingVoltage;
    byte deviceConfiguration;
    unsigned long enabledProtections;
    byte firmwareVersion[BM_FIRMWARE_VERSION_LENGTH];
    unsigned long manufacturingStatus;
    word microcontrollerTemperature;
    unsigned long operationStatus;
    unsigned long safetyStatus;
    word serialNumber;
    word totalBatteryVoltage;

    // True after a read error on the latest readAll() actual read
    // operation; false otherwise.
    boolean previousError;

    // System uptime at the previous readings.
    unsigned long previousReadingTime;

    static const byte MEMORY_ADDRESS_FIELD_LENGTH = 2;

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

    // If more than PERIOD milliseconds have ellapsed since
    // previousReadingTime, update all readings, set the error flag on
    // error and update previousError; otherwise don't update the
    // readings, but set error to previousError.
    void readAll();

    // Read a byte from the given register using the given protocol.
    // Set the error flag on error.
    byte readByte(word registerAddress, Protocol theProtocol);

    // Read a 16-bit integer from the given register using the given protocol.
    // Set the error flag on error.
    word readWord(word registerAddress, Protocol theProtocol);

    // Read a 32-bit integer from the given register using the given protocol.
    // Set the error flag on error.
    unsigned long readUnsignedLong(word registerAddress, Protocol theProtocol);

    // Read from "registerAddress" "contentSize" bytes and stores it in "content".
    // It use the block protocol.
    // Set the error flag on error.
    void readWithBlockProtocol(word registerAddress, byte byteArray[], byte byteArraySize);

    // Read from "registerAddress" "contentSize" bytes and stores it in "content".
    // It use the manufacturer protocol.
    // Set the error flag on error.
    void readWithManufacturerProtocol(word registerAddress, byte byteArray[], byte byteArraySize);

    // Read from "registerAddress" "contentSize" bytes and stores it in "content".
    // It use the word protocol.
    // Set the error flag on error.
    void readWithWordProtocol(word registerAddress, byte byteArray[], byte byteArraySize);

    // It received the frame to send without the CRC byte.
    // It calculates the CRC and appends it to the frame when requested
    // with "command".
    // Return STATUS_SUCCESS when the communcation with the MCU was successful,
    // otherwise return STATUS_FAIL.
    byte writeFrame(byte frame[],byte frameLength, CRCCommand command);

    // Return STATUS_SUCCESS when the communication is successfully done,
    // otherwise return STATUS_FAIL.
    byte writeSealRegister();

    // Return STATUS_SUCCESS when the communication is successfully done,
    // otherwise return STATUS_FAIL.
    byte writeUnsealRegister();
};

// Global instance of the battery controller library.
extern ESAT_BatteryControllerClass ESAT_BatteryController;

#endif /* ESAT_BatteryController_h */
