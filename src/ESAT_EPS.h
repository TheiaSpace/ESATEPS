/*
 * Copyright (C) 2017, 2018 Theia Space, Universidad Politécnica de Madrid
 *
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
#include <ESAT_CCSDSPacketFromKISSFrameReader.h>
#include <ESAT_CCSDSPacketToKISSFrameWriter.h>
#include <ESAT_CCSDSPacket.h>
#include <ESAT_CCSDSTelecommandPacketDispatcher.h>
#include <ESAT_CCSDSTelecommandPacketHandler.h>
#include <ESAT_CCSDSTelemetryPacketBuilder.h>
#include <ESAT_CCSDSTelemetryPacketContents.h>
#include <ESAT_FlagContainer.h>
#include <ESAT_KISSStream.h>
#include <ESAT_SoftwareClock.h>

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
    // Register a telecommand handler.
    void addTelecommand(ESAT_CCSDSTelecommandPacketHandler& telecommand);

    // Register a telemetry packet.
    // The telemetry packet will be enabled by default;
    // it can be disabled with disableTelemetry().
    void addTelemetry(ESAT_CCSDSTelemetryPacketContents& telemetry);

    // Set up the EPS board.
    void begin();

    // Disable the generation of the telemetry packet with the given
    // identifier.
    void disableTelemetry(byte identifier);

    // Enable the generation of the telemetry packet with the given
    // identifier.
    void enableTelemetry(byte identifier);

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

    // Set the time of the real-time clock.
    void setTime(ESAT_Timestamp timestamp);

    // Update the EPS:
    // - Update the maximum point tracking system.
    // - Update the telemetry vector.
    // - Respond to I2C reques
    // - Update the brightness of the heartbeat LED.
    void update();

    // Send a telemetry packet.
    void writeTelemetry(ESAT_CCSDSPacket& packet);

  private:
    // EPS subsystem identifier.
    static const word APPLICATION_PROCESS_IDENTIFIER = 1;

    // Software version number.
    static const byte MAJOR_VERSION_NUMBER = 2;
    static const byte MINOR_VERSION_NUMBER = 1;
    static const byte PATCH_VERSION_NUMBER = 0;

    // Set current time command size:
    // - Year (2 byte).
    // - Month (1 byte).
    // - Day (1 byte).
    // - Hours (1 byte).
    // - Minutes (1 byte).
    // - Seconds (1 byte).
    static const unsigned long MAXIMUM_COMMAND_PARAMETER_LENGTH = 7;

    // Packet data length of telecommand packets.
    // - Secondary header.
    // - Longest command parameter.
    static const unsigned long MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH =
      ESAT_CCSDSSecondaryHeader::LENGTH
      + MAXIMUM_COMMAND_PARAMETER_LENGTH;

    // Total length of telecommand packets:
    // - Primary header.
    // - Maximum telecommand packet data length.
    static const unsigned long MAXIMUM_TELECOMMAND_PACKET_LENGTH =
      ESAT_CCSDSPrimaryHeader::LENGTH
      + MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH;

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
    static const byte EPS_MEASUREMENTS_TELEMETRY_BUFFER_LENGTH =
      2*12;
    static const byte SWITCHES_TELEMETRY_BUFFER_LENGTH =
      1*2;
    static const byte BATTERY_CONTROLLER_TELEMETRY_BUFFER_LENGTH =
      2*5 + 1*2;
    static const byte PANEL_THERMOMETERS_TELEMETRY_BUFFER_LENGTH =
      2*2 + 1*2;
    static const byte MAXIMUM_POWER_POINT_TRACKING_TELEMETRY_BUFFER_LENGTH =
      1*4;
    static const byte DIRECT_ENERGY_TRANSFER_SYSTEM_TELEMETRY_BUFFER_LENGTH =
      2*3 + 1*1;
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
    static const byte BATTERY_MODULE_STATUS_REGISTERS_TELEMETRY_BUFFER_LENGTH =
      4*4;
    static const byte BATTERY_MODULE_MEASUREMENTS_TELEMETRY_BUFFER_LENGTH =
      2*8 + 1*3;
    static const byte BATTERY_MODULE_CONFIGURATION_TELEMETRY_BUFFER_LENGTH =
      11*1 + 4*1 + 2*9 + 1*4;
    static const byte BATTERY_MODULE_HOUSEKEEPING_TELEMETRY_PACKET_DATA_LENGTH =
      ESAT_CCSDSSecondaryHeader::LENGTH
      + BATTERY_MODULE_STATUS_REGISTERS_TELEMETRY_BUFFER_LENGTH
      + BATTERY_MODULE_MEASUREMENTS_TELEMETRY_BUFFER_LENGTH
      + BATTERY_MODULE_CONFIGURATION_TELEMETRY_BUFFER_LENGTH;

    // Telemetry with the highest packet data length.
    static const byte MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH =
      BATTERY_MODULE_HOUSEKEEPING_TELEMETRY_PACKET_DATA_LENGTH;

    // Maximum number of available telemetry packets.
    static const word MAXIMUM_NUMBER_OF_TELEMETRY_PACKETS = 16;

    // Real time clock.
    // Useful for generating timestamps for telemetry packets.
    ESAT_SoftwareClock clock;

    // Telecommand packet dispatcher.
    ESAT_CCSDSTelecommandPacketDispatcher telecommandPacketDispatcher =
      ESAT_CCSDSTelecommandPacketDispatcher(APPLICATION_PROCESS_IDENTIFIER);

    // Telemetry packet builder.
    ESAT_CCSDSTelemetryPacketBuilder telemetryPacketBuilder =
      ESAT_CCSDSTelemetryPacketBuilder(APPLICATION_PROCESS_IDENTIFIER,
                                       MAJOR_VERSION_NUMBER,
                                       MINOR_VERSION_NUMBER,
                                       PATCH_VERSION_NUMBER,
                                       clock);

    // Enabled telemetry list.
    ESAT_FlagContainer enabledTelemetry;

    // Pending telemetry list for I2C requests (active).
    ESAT_FlagContainer i2cPendingTelemetry;

    // Pending telemetry list for I2C requests
    // (buffer; copy this list into i2cPendingTelemetry
    // on pending telemetry list reset request).
    ESAT_FlagContainer i2cPendingTelemetryBuffer;

    // Pending telemetry list for USB output.
    ESAT_FlagContainer usbPendingTelemetry;

    // I2C packet buffers.
    byte i2cTelecommandPacketData[MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH];
    byte i2cTelemetryPacketData[MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH];

    // Use this read CCSDS packets from KISS frames coming from USB interface.
    ESAT_CCSDSPacketFromKISSFrameReader usbReader;

    // Use this write CCSDS packets in KISS frames to the USB interface.
    ESAT_CCSDSPacketToKISSFrameWriter usbWriter;

    // Use this buffer to accumulate incoming telecommands.
    byte usbTelecommandBuffer[MAXIMUM_TELECOMMAND_PACKET_LENGTH];

    // Configure the hardware.
    void beginHardware();

    // Configure the telecommand handlers.
    void beginTelecommands();

    // Configure the telemetry packets.
    void beginTelemetry();

    // Read a telecommand packet from the I2C interface.  Return true
    // on success; otherwise return false.
    boolean readTelecommandFromI2C(ESAT_CCSDSPacket& packet);

    // Read a telecommand packet from the USB interface.  Return true
    // on success; otherwise return false.
    boolean readTelecommandFromUSB(ESAT_CCSDSPacket& packet);

    // Respond to telemetry and telecommand requests coming from the I2C bus.
    void respondToI2CRequest();

    // Respond to a named-packet (of given identifier) telemetry
    // request coming from the I2C bus.
    void respondToNamedPacketTelemetryRequest(byte identifier);

    // Respond to a next-packet telecommand request coming from the
    // I2C bus.
    void respondToNextPacketTelecommandRequest();

    // Respond to a next-packet telemetry request coming from the I2C
    // bus.
    void respondToNextPacketTelemetryRequest();

    // Update the brightness of the heartbeat LED.
    void updateLEDBrightness();

    // Update the maximum power point tracking system.
    void updateMaximumPowerPointTracking();

    // Update the lists of pending telemetry.
    void updatePendingTelemetryLists();
};

// Global instance of the EPS library.
extern ESAT_EPSClass ESAT_EPS;

#endif /* ESAT_EPS_h */
