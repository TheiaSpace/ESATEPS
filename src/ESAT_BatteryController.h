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
#include <ESAT_BMManufacturerAccess.h>

// An interface with the battery controller.
// Use the global instance ESAT_BatteryController.
//
// The underlying hardware is the BQ40Z60 programmable battery
// management unit from Texas Instruments.  Communications are done
// through the EPS I2C bus.
class ESAT_BatteryControllerClass
{
  public:
    // True after a read error.  Must be reset manually.
    boolean error;

    // Number of bytes used by the BM MCU Firmware version.
    static const byte BM_FIRMWARE_VERSION_LENGTH = 11;

    // Instantiate a battery controller library.
    ESAT_BatteryControllerClass();

    // Read the absolute state of charge.
    // Set the error flag on error.
    byte readAbsoluteStateOfCharge();

    // Read the battery balancing configuration.
    // Set the error flag on error.
    byte readBalancingConfiguration();

    // Read the voltage of battery number 1.
    // Set the error flag on error.
    word readBattery1Voltage();

    // Read the voltage of battery number 2.
    // Set the error flag on error.
    word readBattery2Voltage();

    // Read the current flowing through the batteries.
    // Set the error flag on error.
    word readBatteryCurrent();

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

    // Read the operation status.
    // Set the error flag on error.
    unsigned long readOperationStatus();

    // Read the state of charge.
    // Set the error flag on error.
    byte readRelativeStateOfCharge();

    // Read the safety status.
    // Set the error flag on error.
    unsigned long readSafetyStatus();

    // Read the serial number.
    // Set the error flag on error.
    word readSerialNumber();

    // Read the total battery voltage.
    // Set the error flag on error.
    word readTotalBatteryVoltage();

    enum Protocol
    {
      BLOCK_PROTOCOL,
      MANUFACTURER_PROTOCOL,
      WORD_PROTOCOL,
    };

  private:
    // I2C address of the battery controller.
    static const byte ADDRESS = 0x0B;

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
    static const word OPERATION_STATUS_REGISTER = 0x54;
    static const word RELATIVE_STATE_OF_CHARGE_REGISTER = 0x0D;
    static const word SAFETY_STATUS_REGISTER = 0x51;
    static const word SERIAL_NUMBER_REGISTER = 0x1C;
    static const word TOTAL_BATTERY_VOLTAGE_REGISTER = 0x09;

    // Readings may update up to once every PERIOD milliseconds.
    // The EPS cycle isn't fast enough to capture fast transients, and
    // the quasy-steady dynamics are slow, so measuring more often
    // would just waste processor time for no real benefit.
    static const unsigned long PERIOD = 1000;

    // Latest readings.
    word battery1Voltage;
    word battery2Voltage;
    word batteryCurrent;
    word batteryTemperature;
    byte batteryStateOfCharge;
    word totalBatteryVoltage;

    // True after a read error on the latest readAll() actual read
    // operation; false otherwise.
    boolean previousError;
    
    // Manufacturer access handler
    ESAT_BMManufacturerAccessClass BMManufacturerAccess;

    // System uptime at the previous readings.
    unsigned long previousReadingTime;

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
    // Set the error flag on error.
    void read(word registerAddress, byte content[], byte contentSize, Protocol theProtocol);
};

// Global instance of the battery controller library.
extern ESAT_BatteryControllerClass ESAT_BatteryController;

#endif /* ESAT_BatteryController_h */
