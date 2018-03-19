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

#ifndef ESAT_EPS_h
#define ESAT_EPS_h

#include <Arduino.h>
#include <ESAT_CCSDSPacket.h>
#include <ESAT_KISSStream.h>
#include <ESAT_SoftwareClock.h>
#include <ESAT_FlagContainer.h>

// Library with the functionality of the Electrical Power Subsystem
// (EPS) board.  Use the global instance EPS.
//
// * Set up the peripherals with ESAT_EPS.begin().
// * Retrieve a telecommand with ESAT_EPS.readTelecommand().
// * Handle a telecommand with ESAT_EPS.handleTelecommand().
// * Retrieve a telemetry packet with ESAT_EPS.readTelemetry().
// * Write the telemetry packet through the USB interface with
//   ESAT_EPS.writeTelemetry().
// * Update the telemetry measurements and do housekeeping work with
//   ESAT_EPS.update().
// * After ESAT_EPS.begin(), the board will respond to I2C messages
//   on the Wire (SCL_O, SDA_O) interface.
//
// See the example EPS program for a functional EPS loop.
class ESAT_EPSClass
{
  public:
    // Set up the EPS board.
    void begin();

    // Handle a telecommand.
    void handleTelecommand(ESAT_CCSDSPacket& packet);

    // Read an incomming telecommand and write it into a packet.
    // Return true if there was a valid telecommand available;
    // otherwise return false.
    // This sets pendingI2CTelecommand to false.
    boolean readTelecommand(ESAT_CCSDSPacket& packet);

    // Fill the telemetry packet with the next available telemetry vector.
    // Return true if there were new available telemetry;
    // otherwise return false.
    // This sets newTelemetryPacket to false.
    boolean readTelemetry(ESAT_CCSDSPacket& packet);

    // Update the EPS:
    // - Update the maximum point tracking system.
    // - Update the telemetry vector.
    void update();

    // Send a telemetry packet.
    void writeTelemetry(ESAT_CCSDSPacket& packet);

  private:
    // Command codes.
    enum CommandCode
    {
      SET_TIME = 0x00,
      SWITCH_3V3_LINE = 0x10,
      SWITCH_5V_LINE = 0x11,
      MAXIMUM_POWER_POINT_TRACKING_MODE = 0x20,
      SWEEP_MODE = 0x21,
      FIXED_MODE = 0x22,
      ACTIVATE_TELEMETRY_DELIVERY = 0x30,
      DEACTIVATE_TELEMETRY_DELIVERY = 0x31,
    };

    // Telemetry packet identifiers.
    enum TelemetryPacketIdentifier
    {
      HOUSEKEEPING = 0,
      BM_HOUSEKEEPING = 1,
    };

    // EPS subsystem identifier.
    static const word APPLICATION_PROCESS_IDENTIFIER = 1;

    // Software version number.
    static const byte MAJOR_VERSION_NUMBER = 2;
    static const byte MINOR_VERSION_NUMBER = 0;
    static const byte PATCH_VERSION_NUMBER = 1;

    // Set current time command size:
    // - Year (2 byte).
    // - Month (1 byte).
    // - Day (1 byte).
    // - Hours (1 byte).
    // - Minutes (1 byte).
    // - Seconds (1 byte).
    static const byte MAXIMUM_COMMAND_PARAMETER_LENGTH = 7;

    // Packet data length of telecommand packets.
    // - Secondary header.
    // - Longest command parameter.
    static const byte MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH =
      ESAT_CCSDSSecondaryHeader::LENGTH
      + MAXIMUM_COMMAND_PARAMETER_LENGTH;

    // Size of the "housekeeping" telemetry buffer:
    // - Secondary header.              (ESAT_CCSDSSecondaryHeader::LENGTH).
    // - EPS measurements.              3.3 V line current (2 bytes).
    // - EPS measurements.              3.3 V line voltage (2 bytes).
    // - EPS measurements.              5 V line current (2 bytes).
    // - EPS measurements.              5 V line voltage (2 bytes).
    // - EPS measurements.              Input line current (2 bytes).
    // - EPS measurements.              Input line voltage (2 bytes).
    // - EPS measurements.              Panel 1 input current (2 bytes).
    // - EPS measurements.              Panel 1 output current (2 bytes).
    // - EPS measurements.              Panel 1 voltage (2 bytes).
    // - EPS measurements.              Panel 2 input current (2 bytes).
    // - EPS measurements.              Panel 2 output current (2 bytes).
    // - EPS measurements.              Panel 2 voltage (2 bytes).
    // - Switches.                      3.3 V line switch state (1 byte).
    // - Switches.                      5 V line switch state (1 byte).
    // - Battery controller.            Battery current (2 bytes).
    // - Battery controller.            Total battery voltage (2 bytes).
    // - Battery controller.            Battery 1 voltage (2 bytes).
    // - Battery controller.            Battery 2 voltage (2 bytes).
    // - Battery controller.            Battery temperature (2 bytes).
    // - Battery controller.            State of charge (1 byte).
    // - Battery controller.            Error (1 byte).
    // - Panel thermometers.            Panel 1 temperature (2 bytes).
    // - Panel thermometers.            Panel 1 thermometer error (1 byte).
    // - Panel thermometers.            Panel 2 temperature (2 bytes).
    // - Panel thermometers.            Panel 2 thermometer error (1 byte).
    // - Maximum power point tracking.  Driver 1 mode (1 byte).
    // - Maximum power point tracking.  Driver 1 duty cycle (1 byte).
    // - Maximum power point tracking.  Driver 2 mode (1 byte).
    // - Maximum power point tracking.  Driver 2 duty cycle (1 byte).
    // - Direct energy transfer system. Current (2 bytes).
    // - Direct energy transfer system. Voltage (2 bytes).
    // - Direct energy transfer system. Shunt voltage (2 bytes).
    // - Direct energy transfer system. Error (1 byte).
    static const byte EPS_MEASUREMENTS_TELEMETRY_BUFFER_LENGTH = 2*12;
    static const byte SWITCHES_TELEMETRY_BUFFER_LENGTH = 1*2;
    static const byte BATTERY_CONTROLLER_TELEMETRY_BUFFER_LENGTH = 2*5 + 1*2;
    static const byte PANEL_THERMOMETERS_TELEMETRY_BUFFER_LENGTH = 2*2 + 1*2;
    static const byte MAXIMUM_POWER_POINT_TRACKING_TELEMETRY_BUFFER_LENGTH = 1*4;
    static const byte DIRECT_ENERGY_TRANSFER_SYSTEM_TELEMETRY_BUFFER_LENGTH = 2*3 + 1*1;
    static const byte HOUSEKEEPING_TELEMETRY_PACKET_DATA_LENGTH =
      ESAT_CCSDSSecondaryHeader::LENGTH
      + EPS_MEASUREMENTS_TELEMETRY_BUFFER_LENGTH
      + SWITCHES_TELEMETRY_BUFFER_LENGTH
      + BATTERY_CONTROLLER_TELEMETRY_BUFFER_LENGTH
      + PANEL_THERMOMETERS_TELEMETRY_BUFFER_LENGTH
      + MAXIMUM_POWER_POINT_TRACKING_TELEMETRY_BUFFER_LENGTH
      + DIRECT_ENERGY_TRANSFER_SYSTEM_TELEMETRY_BUFFER_LENGTH;

    // Size of the "BM housekeeping"  telemetry buffer:
    // - Secondary header.      (ESAT_CCSDSSecondaryHeader::LENGTH).
    // - BM status registers.   Operation status (4 bytes).
    // - BM status registers.   Charging status (4 bytes).
    // - BM status registers.   Manufacuring status (4 bytes).
    // - BM status registers.   Safety status (4 bytes).
    // - BM measurements.       Battery current (2 bytes).
    // - BM measurements.       Total battery voltage (2 bytes).
    // - BM measurements.       Battery 1 voltage (2 bytes).
    // - BM measurements.       Battery 2 voltage (2 bytes).
    // - BM measurements.       Battery temperature (2 bytes).
    // - BM measurements.       Microcontroller temperature (2 bytes).
    // - BM measurements.       Relative state of charge (1 byte).
    // - BM measurements.       Absolute state of charge (1 byte).
    // - BM measurements.       Desired charging current (2 byte).
    // - BM measurements.       Desired charging voltage (2 byte).
    // - BM measurements.       Error (1 byte).
    // - Battery configuration. Serial number (2 bytes).
    // - Battery configuration. Cycle count (2 bytes).
    // - Battery configuration. Design capacity (2 bytes).
    // - Battery configuration. Design voltage (2 bytes).
    // - Battery configuration. Enabled protections (4 bytes).
    // - Battery configuration. Device configuration (1 bytes).
    // - Battery configuration. Balancing configuration (1 bytes).
    // - Battery configuration. COV threshold (2 bytes).
    // - Battery configuration. COV recovery threshold (2 bytes).
    // - Battery configuration. COV recovery delay (1 bytes).
    // - Battery configuration. CUV threshold (2 bytes).
    // - Battery configuration. CUV recovery threshold (2 bytes).
    // - Battery configuration. CUV recovery delay (1 bytes).
    // - Battery configuration. Chemical ID (2 bytes).
    // - Battery configuration. Firmware version (11 bytes).
    static const byte BM_STATUS_REGISTERS_TELEMETRY_BUFFER_LENGTH = 4*4;
    static const byte BM_MEASUREMENTS_TELEMETRY_BUFFER_LENGTH = 2*8 + 1*3;
    static const byte BM_CONFIGURATION_TELEMETRY_BUFFER_LENGTH = 11*1+4*1+2*9+1*4;
    static const byte BM_HOUSEKEEPING_TELEMETRY_PACKET_DATA_LENGTH =
      ESAT_CCSDSSecondaryHeader::LENGTH
      + BM_STATUS_REGISTERS_TELEMETRY_BUFFER_LENGTH
      + BM_MEASUREMENTS_TELEMETRY_BUFFER_LENGTH
      + BM_CONFIGURATION_TELEMETRY_BUFFER_LENGTH;

    // Telemetry with the highest packet data length
    static const byte MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH = 
      BM_HOUSEKEEPING_TELEMETRY_PACKET_DATA_LENGTH;

    // Real time clock.
    // Useful for generating timestamps for telemetry packets.
    ESAT_SoftwareClock clock;

    // I2C packet buffers.
    byte i2cTelecommandPacketData[MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH];
    byte i2cTelemetryPacketData[MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH];

    // List of all the telemetry IDs that can be delivered.
    ESAT_FlagContainer AvailableTelemetry;

    // List of the telemetry IDs that have to be delivered every cycle.
    ESAT_FlagContainer ActiveTelemetry;

    // List of the telemetry IDs that are pending to be delivered in the current 
    // cycle. There is one for the telemetry sent through the USB bus and 
    // another one for the telemetry sent through the I2C bus.
    ESAT_FlagContainer UsbPendingTelemetry;
    ESAT_FlagContainer I2cPendingTelemetry;

    // Decode USB KISS frames with this stream.
    byte usbTelecommandBuffer[ESAT_CCSDSPrimaryHeader::LENGTH
                              + MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH];
    ESAT_KISSStream usbTelecommandDecoder;

    // Telemetry buffer.
    ESAT_CCSDSPacket telemetry;

    // Telemetry packet data buffer.
    byte telemetryPacketData[MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH];

    // Packet sequence count of the telemetry packets.
    // It grows by 1 every time we generate a new telemetry packet.
    word telemetryPacketSequenceCount;

    // Set the maximum power point tracking drivers in fixed mode.
    void handleFixedModeCommand(ESAT_CCSDSPacket& packet);

    // Set the maximum power point tracking drivers in maximum power
    // point tracking mode.
    void handleMaximumPowerPointTrackingModeCommand(ESAT_CCSDSPacket& packet);

    // Set the maximum power point tracking drivers in sweep mode.
    void handleSweepModeCommand(ESAT_CCSDSPacket& packet);

    // Switch the 3V3 line.
    void handleSwitch3V3LineCommand(ESAT_CCSDSPacket& packet);

    // Switch the 5V line.
    void handleSwitch5VLineCommand(ESAT_CCSDSPacket& packet);

    // Set the time of the real time clock.
    void handleSetTimeCommand(ESAT_CCSDSPacket& packet);
    
    // Set the BM housekeeping telemetry as pending
    void handleActivateTelemetryDelivery(ESAT_CCSDSPacket& packet);

    // Set the BM housekeeping telemetry as pending
    void handleDeactivateTelemetryDelivery(ESAT_CCSDSPacket& packet);

    // Queue incoming USB commands.
    void queueIncomingUSBCommands();

    // Read a telecommand from USB and write it into the given packet.
    // Return true on success; otherwise return false.
    boolean readTelecommandFromUSB(ESAT_CCSDSPacket& packet);

    // Update the maximum power point tracking system.
    void updateMaximumPowerPointTracking();

    // Update the I2C slave telemetry buffer.
    void updateI2CTelemetry();

    // Update the telemetry buffer.
    // This sets newTelemetryPacket to true.
    boolean updateTelemetry(byte ID);
};

// Global instance of the EPS library.
extern ESAT_EPSClass ESAT_EPS;

#endif /* ESAT_EPS_h */
