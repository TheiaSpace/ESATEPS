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

#ifndef ESATBatteryController_h
#define ESATBatteryController_h

#include <Energia.h>

// An interface with the battery controller.
class ESATBatteryController
{
  public:
    // True after a read error.  Must be reset manually.
    boolean error;

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
    word readStateOfCharge();

    // Read the total battery voltage.
    // Set the error flag on error.
    word readTotalBatteryVoltage();


  private:
    // I2C address of the battery controller.
    static const byte address = 0x0b;

    // Registers.
    static const byte battery1VoltageRegister = 0x3e;
    static const byte battery2VoltageRegister = 0x3f;
    static const byte batteryTemperatureRegister = 0x08;
    static const byte batteryCurrentRegister = 0x0a;
    static const byte stateOfChargeRegister = 0x0d;
    static const byte totalBatteryVoltageRegister = 0x09;

    // Read a byte from the given register.
    // Set the error flag on error.
    byte readByte(byte registerNumber);

    // Read a 16-bit integer from the given register.
    // Set the error flag on error.
    word readWord(byte registerNumber);
};

extern ESATBatteryController BatteryController;

#endif /* ESATBatteryController_h */
