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

// An interface with the battery controller.
// Use the global instance ESAT_BatteryController.
class ESAT_BatteryControllerClass
{
  public:
    // True after a read error.  Must be reset manually.
    boolean error;

    // Instantiate a battery controller library.
    ESAT_BatteryControllerClass();

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

    // Read the state of charge.
    // Set the error flag on error.
    byte readStateOfCharge();

    // Read the total battery voltage.
    // Set the error flag on error.
    word readTotalBatteryVoltage();


  private:
    // I2C address of the battery controller.
    static const byte ADDRESS = 0x0B;

    // Registers.
    static const byte BATTERY_1_VOLTAGE_REGISTER = 0x3E;
    static const byte BATTERY_2_VOLTAGE_REGISTER = 0x3F;
    static const byte BATTERY_TEMPERATURE_REGISTER = 0x08;
    static const byte BATTERY_CURRENT_REGISTER = 0x0A;
    static const byte STATE_OF_CHARGE_REGISTER = 0x0D;
    static const byte TOTAL_BATTERY_VOLTAGE_REGISTER = 0x09;

    // Readings may update up to once every PERIOD milliseconds.
    // The EPS cycle isn't fast enough to capture fast transients, and
    // the quasy-steady dynamics are slow, so measuring more often
    // would just waste processor time for no real benefit.
    static const unsigned long PERIOD = 1000;

    // Latest readings.
    word previousBattery1VoltageReading;
    word previousBattery2VoltageReading;
    word previousBatteryCurrentReading;
    word previousBatteryTemperatureReading;
    byte previousStateOfChargeReading;
    word previousTotalBatteryVoltageReading;

    // System uptime at the previous readings.
    unsigned long previousBattery1VoltageReadingTime;
    unsigned long previousBattery2VoltageReadingTime;
    unsigned long previousBatteryCurrentReadingTime;
    unsigned long previousBatteryTemperatureReadingTime;
    unsigned long previousStateOfChargeReadingTime;
    unsigned long previousTotalBatteryVoltageReadingTime;

    // Read a byte from the given register.
    // Set the error flag on error.
    byte readByte(byte registerNumber);

    // Read a 16-bit integer from the given register.
    // Set the error flag on error.
    word readWord(byte registerNumber);
};

// Global instance of the battery controller library.
extern ESAT_BatteryControllerClass ESAT_BatteryController;

#endif /* ESAT_BatteryController_h */
