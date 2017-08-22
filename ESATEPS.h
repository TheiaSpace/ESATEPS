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

#ifndef ESATEPS_h
#define ESATEPS_h

#include <Energia.h>
#include <MspFlash.h>

#define EPS_IDENTIFIER_FLASH_SEGMENT SEGMENT_C
#define EPS_SOFTWARE_VERSION 1

class ESATEPS
{
  public:
    // Set up the EPS board.
    void begin();

    // Handle the next command of the command queue.
    void handleCommand();

    // Return true if there are pending commands;
    // otherwise return false.
    boolean pendingCommands();

    // Send a telemetry packet.
    void sendTelemetry();

    // Update the maximum power point tracking system.
    void updateMaximumPowerPointTracking();

    // Update the telemetry buffer.
    void updateTelemetry();

  private:
    // Command codes.
    enum CommandCode
    {
      SET_IDENTIFIER = 0,
      TOGGLE_5V_LINE = 1,
      TOGGLE_3V3_LINE = 2,
      MAXIMUM_POWER_POINT_TRACKING_MODE = 3,
      SWEEP_MODE = 4,
      FIXED_MODE = 5,
    };

    // Offsets of the telemetry fields.
    enum TelemetryFieldOffsets
    {
      STATUS_REGISTER_1_OFFSET = 0,
      STATUS_REGISTER_2_OFFSET = 1,
      CURRENT_5V_OFFSET = 2,
      VOLTAGE_5V_OFFSET = 3,
      CURRENT_3V3_OFFSET = 4,
      VOLTAGE_3V3_OFFSET = 5,
      INPUT_CURRENT_OFFSET = 6,
      INPUT_VOLTAGE_OFFSET = 7,
      PANEL_1_OUTPUT_CURRENT_OFFSET = 8,
      PANEL_1_VOLTAGE_OFFSET = 9,
      PANEL_1_INPUT_CURRENT_OFFSET = 10,
      PANEL_2_VOLTAGE_OFFSET = 11,
      PANEL_2_INPUT_CURRENT_OFFSET = 12,
      PANEL_2_OUTPUT_CURRENT_OFFSET = 13,
      TOTAL_BATTERY_VOLTAGE_OFFSET = 14,
      BATTERY_1_VOLTAGE_OFFSET = 15,
      BATTERY_2_VOLTAGE_OFFSET = 16,
      BATTERY_TEMPERATURE_OFFSET = 17,
      BATTERY_CURRENT_OFFSET = 18,
      STATE_OF_CHARGE_OFFSET = 24,
      PANEL_1_TEMPERATURE_OFFSET = 19,
      PANEL_2_TEMPERATURE_OFFSET = 20,
      DIRECT_ENERGY_TRANSFER_SYSTEM_CURRENT_OFFSET = 21,
      DIRECT_ENERGY_TRANSFER_SYSTEM_VOLTAGE_OFFSET = 22,
      DIRECT_ENERGY_TRANSFER_SYSTEM_SHUNT_VOLTAGE_OFFSET = 23,
    };

    enum StatusRegister1Offsets
    {
      SOFTWARE_VERSION_OFFSET = 3,
    };

    // Bit offsets of the status bits on status register 2.
    enum StatusRegister2Offsets
    {
      DIRECT_ENERGY_TRANSFER_SYSTEM_STATUS_OFFSET = 0,
      PANEL_2_THERMOMETER_STATUS_OFFSET = 1,
      PANEL_1_THERMOMETER_STATUS_OFFSET = 2,
      BATTERY_STATUS_OFFSET = 3,
      OVERCURRENT_3V3_OFFSET = 4,
      OVERCURRENT_5V_OFFSET = 5,
      SWITCH_3V3_ON_OFFSET = 6,
      SWITCH_5V_ON_OFFSET = 7,
    };

    // Size in words of the telemetry buffer.
    static const byte TELEMETRY_BUFFER_LENGTH = 25;

    // Command buffer structure.
    struct Command
    {
      byte commandCode;
      byte parameter;
      boolean pending;
    };

    // Command queue.
    volatile Command command;

    // Last received command code.
    byte commandCode;

    // Last received command parameter.
    byte commandParameter;

    // Identifier number of the EPS board.
    byte identifier;

    // Telemetry buffer.
    word telemetry[TELEMETRY_BUFFER_LENGTH];

    // Set the maximum power point tracking drivers in fixed mode.
    void handleFixedModeCommand();

    // Set the maximum power point tracking drivers in maximum power
    // point tracking mode.
    void handleMaximumPowerPointTrackingModeCommand();

    // Set and store the identifier number.
    void handleSetIdentifierCommand();

    // Set the maximum power point tracking drivers in sweep mode.
    void handleSweepModeCommand();

    // Toggle the 3V3 line.
    void handleToggle3V3LineCommand();

    // Toggle the 5V line.
    void handleToggle5VLineCommand();

    // Add a command to the command queue.
    void queueCommand(byte commandCode, byte parameter);

    // Queue incoming USB commands.
    void queueIncomingUSBCommands();

    // Response to incoming telecommands sent by the OBC.
    static void receiveEvent(int howMany);

    // Response when asked for telemetry by the OBC.
    static void requestEvent();

    // Update the battery fields of the telemetry buffer.
    void updateBatteryTelemetry();

    // Update the direct energy transfer system fields of the telemetry buffer.
    void updateDirectEnergyTransferSystemTelemetry();

    // Update the main fields of the telemetry buffer.
    void updateMainTelemetry();

    // Update the panel fields of the telemetry buffer.
    void updatePanelTelemetry();

    // Update the software version field of the telemetry buffer.
    void updateSoftwareVersionTelemetry();

    // Update the status registers of the telemetry buffer.
    void updateStatusTelemetry();
};

extern ESATEPS EPS;

#endif
