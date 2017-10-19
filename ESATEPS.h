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

#include <Arduino.h>
#include <ESATCCSDSPacket.h>
#include <ESATKISSStream.h>
#include <ESATSoftwareClock.h>

// Library with the functionality of the Electrical Power Subsystem
// (EPS) board.  Use the global instance EPS.
// Set up the peripherals with EPS.begin().
// Retrieve a telecommand with readTelecommand().
// Handle a telecommand with handleTelecommand().
// Retrieve a telemetry packet with readTelemetry().
// Write the telemetry packet through the USB interface with writeTelemetry().
// Update the telemetry measurements and do housekeeping work with update().
// After begin(), the board will respond to I2C messages on the Wire
// (SCL_O, SDA_O) interface.
// See the example EPS program for a functional EPS loop.
class ESATEPS
{
  public:
    // Set up the EPS board.
    // Accumulate incoming USB telecommands in the given buffer.
    void begin(byte buffer[], unsigned long bufferLength);

    // Handle a telecommand.
    void handleTelecommand(ESATCCSDSPacket& packet);

    // Read an incomming telecommand and write it into a packet.
    // Return true if there was a valid telecommand available;
    // otherwise return false.
    // This sets pendingI2CTelecommand to false.
    boolean readTelecommand(ESATCCSDSPacket& packet);

    // Fill the telemetry packet with the next available telemetry vector.
    // Return true if there were new available telemetry;
    // otherwise return false.
    // This sets newTelemetryPacket to false.
    boolean readTelemetry(ESATCCSDSPacket& packet);

    // Update the EPS:
    // - Update the maximum point tracking system.
    // - Update the telemetry vector.
    void update();

    // Send a telemetry packet.
    void writeTelemetry(ESATCCSDSPacket& packet);

  private:
    // Command codes.
    enum CommandCode
    {
      SET_CURRENT_TIME = 0x00,
      SWITCH_3V3_LINE = 0x10,
      SWITCH_5V_LINE = 0x11,
      MAXIMUM_POWER_POINT_TRACKING_MODE = 0x20,
      SWEEP_MODE = 0x21,
      FIXED_MODE = 0x22,
    };

    // Telemetry packet identifiers.
    enum TelemetryPacketIdentifier
    {
      HOUSEKEEPING = 0,
    };

    // EPS subsystem identifier.
    static const word APPLICATION_PROCESS_IDENTIFIER = 1;

    // Software version number.
    static const byte MAJOR_VERSION_NUMBER = 2;
    static const byte MINOR_VERSION_NUMBER = 0;
    static const byte PATCH_VERSION_NUMBER = 0;

    // The commands that have a 1-byte argument field.
    static const byte MINIMUM_COMMAND_PARAMETER_LENGTH = 1;

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
    // - Shortest command parameter.
    static const byte MINIMUM_TELECOMMAND_PACKET_DATA_LENGTH =
      ESATCCSDSSecondaryHeader::LENGTH
      + MINIMUM_COMMAND_PARAMETER_LENGTH;

    // Packet data length of telecommand packets.
    // - Secondary header.
    // - Longest command parameter.
    static const byte MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH =
      ESATCCSDSSecondaryHeader::LENGTH
      + MAXIMUM_COMMAND_PARAMETER_LENGTH;

    // Size of the telemetry buffer (EPS measurements):
    // - 3.3 V line current (2 bytes).
    // - 3.3 V line voltage (2 bytes).
    // - 5 V line current (2 bytes).
    // - 5 V line voltage (2 bytes).
    // - Input line current (2 bytes).
    // - Input line voltage (2 bytes).
    // - Panel 1 input current (2 bytes).
    // - Panel 1 output current (2 bytes).
    // - Panel 1 voltage (2 bytes).
    // - Panel 2 input current (2 bytes).
    // - Panel 2 output current (2 bytes).
    // - Panel 2 voltage (2 bytes).
    static const byte EPS_MEASUREMENTS_TELEMETRY_BUFFER_LENGTH = 2*12;

    // Size of the telemetry buffer (switches):
    // - 3.3 V line switch state (1 byte).
    // - 5 V line switch state (1 byte).
    static const byte SWITCHES_TELEMETRY_BUFFER_LENGTH = 1*2;

    // Size of the telemetry buffer (overcurrent):
    // - 3.3 V line overcurrent (1 byte).
    // - 5 V line overcurrent (1 byte);
    static const byte OVERCURRENT_TELEMETRY_BUFFER_LENGTH = 1*2;

    // Size of the telemetry buffer (battery controller):
    // - Battery current (2 bytes).
    // - Total battery voltage (2 bytes).
    // - Battery 1 voltage (2 bytes).
    // - Battery 2 voltage (2 bytes).
    // - Battery temperature (2 bytes).
    // - State of charge (1 byte).
    // - Error (1 byte).
    static const byte BATTERY_CONTROLLER_TELEMETRY_BUFFER_LENGTH = 2*5 + 1*2;

    // Size of the telemetry buffer (panel thermometers):
    // - Panel 1 temperature (2 bytes).
    // - Panel 1 thermometer error (1 byte).
    // - Panel 2 temperature (2 bytes).
    // - Panel 2 thermometer error (1 byte).
    static const byte PANEL_THERMOMETERS_TELEMETRY_BUFFER_LENGTH = 2*2 + 1*2;

    // Size of the telemetry buffer (maximum power point tracking):
    // - Driver 1 mode (1 byte).
    // - Driver 1 duty cycle (1 byte).
    // - Driver 2 mode (1 byte).
    // - Driver 2 duty cycle (1 byte).
    static const byte MAXIMUM_POWER_POINT_TRACKING_TELEMETRY_BUFFER_LENGTH = 1*4;

    // Size of the telemetry buffer (direct energy transfer system):
    // - Current (2 bytes).
    // - Voltage (2 bytes).
    // - Shunt voltage (2 bytes).
    // - Error (1 byte).
    static const byte DIRECT_ENERGY_TRANSFER_SYSTEM_TELEMETRY_BUFFER_LENGTH = 2*3 + 1*1;

    // Size of the telemetry packet data buffer (total):
    // - Secondary header.
    // - EPS measurements.
    // - Battery controller.
    // - Panel thermometers.
    // - Direct energy transfer system.
    // - Switches.
    static const byte TELEMETRY_PACKET_DATA_LENGTH =
      ESATCCSDSSecondaryHeader::LENGTH
      + EPS_MEASUREMENTS_TELEMETRY_BUFFER_LENGTH
      + SWITCHES_TELEMETRY_BUFFER_LENGTH
      + OVERCURRENT_TELEMETRY_BUFFER_LENGTH
      + BATTERY_CONTROLLER_TELEMETRY_BUFFER_LENGTH
      + MAXIMUM_POWER_POINT_TRACKING_TELEMETRY_BUFFER_LENGTH
      + PANEL_THERMOMETERS_TELEMETRY_BUFFER_LENGTH
      + DIRECT_ENERGY_TRANSFER_SYSTEM_TELEMETRY_BUFFER_LENGTH;

    // Real time clock.
    // Useful for generating timestamps for telemetry packets.
    ESATSoftwareClock clock;

    // I2C packet buffers.
    byte i2cTelecommandPacketData[MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH];
    byte i2cTelemetryPacketData[TELEMETRY_PACKET_DATA_LENGTH];

    // True when there is a new telemetry packet that was not
    // requested with readTelemetry():
    // - true after updateTelemetry()
    // - false after readTelemetry()
    boolean newTelemetryPacket;

    // Decode USB KISS frames with this stream.
    ESATKISSStream usbTelecommandDecoder;

    // Telemetry buffer.
    ESATCCSDSPacket telemetry;

    // Telemetry packet data buffer.
    byte telemetryPacketData[TELEMETRY_PACKET_DATA_LENGTH];

    // Packet sequence count of the telemetry packets.
    // It grows by 1 every time we generate a new telemetry packet.
    word telemetryPacketSequenceCount;

    // Set the maximum power point tracking drivers in fixed mode.
    void handleFixedModeCommand(ESATCCSDSPacket& packet);

    // Set the maximum power point tracking drivers in maximum power
    // point tracking mode.
    void handleMaximumPowerPointTrackingModeCommand(ESATCCSDSPacket& packet);

    // Set the maximum power point tracking drivers in sweep mode.
    void handleSweepModeCommand(ESATCCSDSPacket& packet);

    // Switch the 3V3 line.
    void handleSwitch3V3LineCommand(ESATCCSDSPacket& packet);

    // Switch the 5V line.
    void handleSwitch5VLineCommand(ESATCCSDSPacket& packet);

    // Set the time of the real time clock.
    void handleSetCurrentTimeCommand(ESATCCSDSPacket& packet);

    // Queue incoming USB commands.
    void queueIncomingUSBCommands();

    // Read a telecommand from USB and write it into the given packet.
    // Return true on success; otherwise return false.
    boolean readTelecommandFromUSB(ESATCCSDSPacket& packet);

    // Update the maximum power point tracking system.
    void updateMaximumPowerPointTracking();

    // Update the I2C slave telemetry buffer.
    void updateI2CTelemetry();

    // Update the telemetry buffer.
    // This sets newTelemetryPacket to true.
    void updateTelemetry();
};

// Global instance of the EPS library.
extern ESATEPS EPS;

#endif
