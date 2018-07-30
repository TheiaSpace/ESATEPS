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

#include "ESAT_EPS.h"
#include <ESAT_CCSDSPrimaryHeader.h>
#include <ESAT_I2CSlave.h>
#include <ESAT_KISSStream.h>
#include <ESAT_Timestamp.h>
#include <USBSerial.h>
#include <Wire.h>
#include "ESAT_BatteryModuleHousekeeping.h"
#include "ESAT_EPSHousekeeping.h"
#include "ESAT_EPSMeasurements.h"
#include "ESAT_MaximumPowerPointTrackingDriver.h"
#include "ESAT_PowerLineSwitch.h"
#include "ESAT_BatteryController.h"

void ESAT_EPSClass::addTelemetryPacket(ESAT_CCSDSPacketContents& contents)
{
  telemetryPacketBuilder.addPacketContents(contents);
}

void ESAT_EPSClass::begin()
{
  telemetryPacketBuilder =
    ESAT_CCSDSPacketBuilder(APPLICATION_PROCESS_IDENTIFIER,
                            MAJOR_VERSION_NUMBER,
                            MINOR_VERSION_NUMBER,
                            PATCH_VERSION_NUMBER,
                            clock);
  i2cTelemetryPacketBuilder = telemetryPacketBuilder;
  addTelemetryPacket(ESAT_EPSHousekeeping);
  addTelemetryPacket(ESAT_BatteryModuleHousekeeping);
  telemetryPacketBuilder.enablePacket(ESAT_EPSHousekeeping.packetIdentifier());
  telemetryPacketBuilder.disablePacket(ESAT_BatteryModuleHousekeeping.packetIdentifier());
  telemetry = ESAT_CCSDSPacket(telemetryPacketData,
                               MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH);
  usbTelecommandDecoder = ESAT_KISSStream(USB,
                                          usbTelecommandBuffer,
                                          sizeof(usbTelecommandBuffer));
  ESAT_EPSMeasurements.begin();
  ESAT_MaximumPowerPointTrackingDriver1.begin();
  ESAT_MaximumPowerPointTrackingDriver2.begin();
  ESAT_MaximumPowerPointTrackingDriver1.setMPPTMode();
  ESAT_MaximumPowerPointTrackingDriver2.setMPPTMode();
  ESAT_PowerLine5VSwitch.begin();
  ESAT_PowerLine5VSwitch.write(ESAT_PowerLine5VSwitch.ON);
  ESAT_PowerLine3V3Switch.begin();
  ESAT_PowerLine3V3Switch.write(ESAT_PowerLine3V3Switch.ON);
  Wire1.begin();
  ESAT_BatteryController.writeDelayBetweenCommunications((byte)2);
  Wire.begin(byte(APPLICATION_PROCESS_IDENTIFIER));
  USB.begin();
  ESAT_I2CSlave.begin(Wire,
                      i2cTelecommandPacketData,
                      MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH,
                      i2cTelemetryPacketData,
                      MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH);
}

void ESAT_EPSClass::handleTelecommand(ESAT_CCSDSPacket& packet)
{
  packet.rewind();
  const ESAT_CCSDSPrimaryHeader primaryHeader = packet.readPrimaryHeader();
  if (primaryHeader.applicationProcessIdentifier
      != APPLICATION_PROCESS_IDENTIFIER)
  {
    return;
  }
  if (primaryHeader.packetType != primaryHeader.TELECOMMAND)
  {
    return;
  }
  if (primaryHeader.packetDataLength < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return;
  }
  const ESAT_CCSDSSecondaryHeader secondaryHeader =
    packet.readSecondaryHeader();
  if (secondaryHeader.majorVersionNumber < MAJOR_VERSION_NUMBER)
  {
    return;
  }
  switch (secondaryHeader.packetIdentifier)
  {
    case SWITCH_5V_LINE:
      handleSwitch5VLineCommand(packet);
      break;
    case SWITCH_3V3_LINE:
      handleSwitch3V3LineCommand(packet);
      break;
    case MAXIMUM_POWER_POINT_TRACKING_MODE:
      handleMaximumPowerPointTrackingModeCommand(packet);
      break;
    case SWEEP_MODE:
      handleSweepModeCommand(packet);
      break;
    case FIXED_MODE:
      handleFixedModeCommand(packet);
      break;
    case SET_TIME:
      handleSetTimeCommand(packet);
      break;
    case ACTIVATE_TELEMETRY_DELIVERY:
      handleActivateTelemetryDelivery(packet);
      break;
    case DEACTIVATE_TELEMETRY_DELIVERY:
      handleDeactivateTelemetryDelivery(packet);
      break;
    default:
      break;
  }
}

void ESAT_EPSClass::handleFixedModeCommand(ESAT_CCSDSPacket& packet)
{
  const byte commandParameter = packet.readByte();
  const byte dutyCycle = constrain(commandParameter, 0, 255);
  ESAT_MaximumPowerPointTrackingDriver1.setFixedMode(dutyCycle);
  ESAT_MaximumPowerPointTrackingDriver2.setFixedMode(dutyCycle);
}

void ESAT_EPSClass::handleMaximumPowerPointTrackingModeCommand(ESAT_CCSDSPacket& packet)
{
  (void) packet;
  ESAT_MaximumPowerPointTrackingDriver1.setMPPTMode();
  ESAT_MaximumPowerPointTrackingDriver2.setMPPTMode();
}

void ESAT_EPSClass::handleSweepModeCommand(ESAT_CCSDSPacket& packet)
{
  (void) packet;
  ESAT_MaximumPowerPointTrackingDriver1.setSweepMode();
  ESAT_MaximumPowerPointTrackingDriver2.setSweepMode();
}

void ESAT_EPSClass::handleSwitch3V3LineCommand(ESAT_CCSDSPacket& packet)
{
  const byte commandParameter = packet.readByte();
  if (commandParameter > 0)
  {
    ESAT_PowerLine3V3Switch.write(ESAT_PowerLine3V3Switch.ON);
  }
  else
  {
    ESAT_PowerLine3V3Switch.write(ESAT_PowerLine3V3Switch.OFF);
  }
}

void ESAT_EPSClass::handleSwitch5VLineCommand(ESAT_CCSDSPacket& packet)
{
  const byte commandParameter = packet.readByte();
  if (commandParameter > 0)
  {
    ESAT_PowerLine5VSwitch.write(ESAT_PowerLine5VSwitch.ON);
  }
  else
  {
    ESAT_PowerLine5VSwitch.write(ESAT_PowerLine5VSwitch.OFF);
  }
}

void ESAT_EPSClass::handleSetTimeCommand(ESAT_CCSDSPacket& packet)
{
  const ESAT_Timestamp timestamp = packet.readTimestamp();
  clock.write(timestamp);
}

void ESAT_EPSClass::handleActivateTelemetryDelivery(ESAT_CCSDSPacket& packet)
{
  const byte identifier = packet.readByte();
  telemetryPacketBuilder.enablePacket(identifier);
}

void ESAT_EPSClass::handleDeactivateTelemetryDelivery(ESAT_CCSDSPacket& packet)
{
  const byte identifier = packet.readByte();
  telemetryPacketBuilder.disablePacket(identifier);
}

boolean ESAT_EPSClass::readTelecommand(ESAT_CCSDSPacket& packet)
{
  packet.flush();
  if (packet.capacity() < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return false;
  }
  boolean pendingTelecommand = ESAT_I2CSlave.readPacket(packet);
  if (!pendingTelecommand)
  {
    pendingTelecommand = readTelecommandFromUSB(packet);
  }
  if (!pendingTelecommand)
  {
    return false;
  }
  const ESAT_CCSDSPrimaryHeader primaryHeader = packet.readPrimaryHeader();
  if (primaryHeader.packetType != primaryHeader.TELECOMMAND)
  {
    return false;
  }
  if (primaryHeader.applicationProcessIdentifier
      != APPLICATION_PROCESS_IDENTIFIER)
  {
    return false;
  }
  if (primaryHeader.packetDataLength < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return false;
  }
  return true;
}

boolean ESAT_EPSClass::readTelecommandFromUSB(ESAT_CCSDSPacket& packet)
{
  const boolean gotFrame = usbTelecommandDecoder.receiveFrame();
  if (!gotFrame)
  {
    return false;
  }
  return packet.readFrom(usbTelecommandDecoder);
}

boolean ESAT_EPSClass::readTelemetry(ESAT_CCSDSPacket& packet)
{
  return telemetryPacketBuilder.buildNextTelemetryPacket(packet);
}

void ESAT_EPSClass::respondToI2CRequest()
{
  const int requestedPacket = ESAT_I2CSlave.requestedPacket();
  switch (requestedPacket)
  {
    case ESAT_I2CSlave.NO_PACKET_REQUESTED:
      break;
    case ESAT_I2CSlave.NEXT_TELEMETRY_PACKET_REQUESTED:
      respondToNextPacketTelemetryRequest();
      break;
    case ESAT_I2CSlave.NEXT_TELECOMMAND_PACKET_REQUESTED:
      respondToNextPacketTelecommandRequest();
      break;
    default:
      respondToNamedPacketTelemetryRequest(byte(requestedPacket));
      break;
  }
}

void ESAT_EPSClass::respondToNamedPacketTelemetryRequest(const byte identifier)
{
  byte packetData[MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH];
  ESAT_CCSDSPacket packet(packetData, sizeof(packetData));
  const boolean gotPacket =
    telemetryPacketBuilder.buildNamedTelemetryPacket(packet, identifier);
  if (gotPacket)
  {
    ESAT_I2CSlave.writePacket(packet);
  }
  else
  {
    ESAT_I2CSlave.rejectPacket();
  }
}

void ESAT_EPSClass::respondToNextPacketTelecommandRequest()
{
  ESAT_I2CSlave.rejectPacket();
}

void ESAT_EPSClass::respondToNextPacketTelemetryRequest()
{
  if (ESAT_I2CSlave.telemetryQueueResetReceived())
  {
    i2cTelemetryPacketBuilder = telemetryPacketBuilder;
    i2cTelemetryPacketBuilder.rewindPacketContentsQueue();
  }
  byte packetData[MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH];
  ESAT_CCSDSPacket packet(packetData, sizeof(packetData));
  const boolean gotPacket =
    i2cTelemetryPacketBuilder.buildNextTelemetryPacket(packet);
  if (gotPacket)
  {
    ESAT_I2CSlave.writePacket(packet);
  }
  else
  {
    ESAT_I2CSlave.rejectPacket();
  }
}

void ESAT_EPSClass::update()
{
  updateMaximumPowerPointTracking();
  updatePendingTelemetryList();
  respondToI2CRequest();
}

void ESAT_EPSClass::updateMaximumPowerPointTracking()
{
  ESAT_MaximumPowerPointTrackingDriver1.update();
  ESAT_MaximumPowerPointTrackingDriver2.update();
}

void ESAT_EPSClass::updatePendingTelemetryList()
{
  telemetryPacketBuilder.rewindPacketContentsQueue();
}

void ESAT_EPSClass::writeTelemetry(ESAT_CCSDSPacket& packet)
{
  packet.rewind();
  const unsigned long encoderBufferLength =
    ESAT_KISSStream::frameLength(packet.length());
  byte encoderBuffer[encoderBufferLength];
  ESAT_KISSStream encoder(USB, encoderBuffer, encoderBufferLength);
  (void) encoder.beginFrame();
  (void) packet.writeTo(encoder);
  (void) encoder.endFrame();
}

ESAT_EPSClass ESAT_EPS;
