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

#include "ESAT_EPS.h"
#include <ESAT_CCSDSPrimaryHeader.h>
#include <ESAT_I2CSlave.h>
#include <ESAT_Timestamp.h>
#include <Wire.h>
#include "ESAT_EPSMeasurements.h"
#include "ESAT_EPS-peripherals/ESAT_EPSLED.h"
#include "ESAT_EPS-telecommands/ESAT_EPSDisableTelemetryTelecommand.h"
#include "ESAT_EPS-telecommands/ESAT_EPSEnableTelemetryTelecommand.h"
#include "ESAT_EPS-telecommands/ESAT_EPSFixedModeTelecommand.h"
#include "ESAT_EPS-telecommands/ESAT_EPSMaximumPowerPointTrackingModeTelecommand.h"
#include "ESAT_EPS-telecommands/ESAT_EPSSetTimeTelecommand.h"
#include "ESAT_EPS-telecommands/ESAT_EPSSweepModeTelecommand.h"
#include "ESAT_EPS-telecommands/ESAT_EPSSwitch3V3LineTelecommand.h"
#include "ESAT_EPS-telecommands/ESAT_EPSSwitch5VLineTelecommand.h"
#include "ESAT_EPS-telemetry/ESAT_BatteryModuleHousekeepingTelemetry.h"
#include "ESAT_EPS-telemetry/ESAT_EPSHousekeepingTelemetry.h"
#include "ESAT_MaximumPowerPointTrackingDriver.h"
#include "ESAT_PowerLineSwitch.h"
#include "ESAT_BatteryController.h"

void ESAT_EPSClass::addTelecommand(ESAT_CCSDSPacketConsumer& telecommand)
{
  telecommandPacketHandler.add(telecommand);
}

void ESAT_EPSClass::addTelemetry(ESAT_CCSDSPacketContents& telemetry)
{
  telemetryPacketBuilder.add(telemetry);
  disableTelemetry(telemetry.packetIdentifier());
}

void ESAT_EPSClass::begin()
{
  telemetryPacketBuilder =
    ESAT_CCSDSTelemetryPacketBuilder(APPLICATION_PROCESS_IDENTIFIER,
                                     MAJOR_VERSION_NUMBER,
                                     MINOR_VERSION_NUMBER,
                                     PATCH_VERSION_NUMBER,
                                     clock);
  enabledTelemetry.clearAll();
  i2cPendingTelemetry.clearAll();
  i2cPendingTelemetryBuffer.clearAll();
  usbPendingTelemetry.clearAll();
  addTelemetry(ESAT_EPSHousekeepingTelemetry);
  enableTelemetry(ESAT_EPSHousekeepingTelemetry.packetIdentifier());
  addTelemetry(ESAT_BatteryModuleHousekeepingTelemetry);
  disableTelemetry(ESAT_BatteryModuleHousekeepingTelemetry.packetIdentifier());
  addTelecommand(ESAT_EPSSetTimeTelecommand);
  addTelecommand(ESAT_EPSSwitch3V3LineTelecommand);
  addTelecommand(ESAT_EPSSwitch5VLineTelecommand);
  addTelecommand(ESAT_EPSMaximumPowerPointTrackingModeTelecommand);
  addTelecommand(ESAT_EPSSweepModeTelecommand);
  addTelecommand(ESAT_EPSFixedModeTelecommand);
  addTelecommand(ESAT_EPSEnableTelemetryTelecommand);
  addTelecommand(ESAT_EPSDisableTelemetryTelecommand);
  usbReader = ESAT_CCSDSPacketFromKISSFrameReader(Serial,
                                                  usbTelecommandBuffer,
                                                  sizeof(usbTelecommandBuffer));
  usbWriter = ESAT_CCSDSPacketToKISSFrameWriter(Serial);
  ESAT_EPSMeasurements.begin();
  ESAT_MaximumPowerPointTrackingDriver1.begin();
  ESAT_MaximumPowerPointTrackingDriver2.begin();
  ESAT_MaximumPowerPointTrackingDriver1.setMPPTMode();
  ESAT_MaximumPowerPointTrackingDriver2.setMPPTMode();
  ESAT_PowerLine5VSwitch.begin();
  ESAT_PowerLine5VSwitch.write(ESAT_PowerLine5VSwitch.ON);
  ESAT_PowerLine3V3Switch.begin();
  ESAT_PowerLine3V3Switch.write(ESAT_PowerLine3V3Switch.ON);
  WireEPS.begin();
  ESAT_BatteryController.writeDelayBetweenCommunications(byte(2));
  ESAT_EPSLED.begin();
  WireOBC.begin(byte(APPLICATION_PROCESS_IDENTIFIER));
  Serial.begin();
  ESAT_I2CSlave.begin(WireOBC,
                      i2cTelecommandPacketData,
                      MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH,
                      i2cTelemetryPacketData,
                      MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH);
}

void ESAT_EPSClass::disableTelemetry(const byte identifier)
{
  enabledTelemetry.clear(identifier);
}

void ESAT_EPSClass::enableTelemetry(const byte identifier)
{
  enabledTelemetry.set(identifier);
}

void ESAT_EPSClass::handleTelecommand(ESAT_CCSDSPacket& packet)
{
  packet.rewind();
  (void) telecommandPacketHandler.handle(packet);
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
    pendingTelecommand = usbReader.read(packet);
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

boolean ESAT_EPSClass::readTelemetry(ESAT_CCSDSPacket& packet)
{
  if (usbPendingTelemetry.available() > 0)
  {
    const byte identifier = byte(usbPendingTelemetry.readNext());
    usbPendingTelemetry.clear(identifier);
    return telemetryPacketBuilder.build(packet, identifier);
  }
  else
  {
    return false;
  }
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
  const boolean enabled = enabledTelemetry.read(identifier);
  if (!enabled)
  {
    ESAT_I2CSlave.rejectPacket();
    return;
  }
  const boolean gotPacket =
    telemetryPacketBuilder.build(packet, identifier);
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
    i2cPendingTelemetry = i2cPendingTelemetryBuffer;
    i2cPendingTelemetryBuffer.clearAll();
  }
  i2cPendingTelemetry = i2cPendingTelemetry & enabledTelemetry;
  if (i2cPendingTelemetry.available() > 0)
  {
    const byte identifier = byte(i2cPendingTelemetry.readNext());
    i2cPendingTelemetry.clear(identifier);
    respondToNamedPacketTelemetryRequest(identifier);
  }
  else
  {
    ESAT_I2CSlave.rejectPacket();
  }
}

void ESAT_EPSClass::setTime(const ESAT_Timestamp timestamp)
{
  clock.write(timestamp);
}

void ESAT_EPSClass::updatePendingTelemetryLists()
{
  const ESAT_FlagContainer availableTelemetry =
    telemetryPacketBuilder.available();
  const ESAT_FlagContainer availableAndEnabledTelemetry =
    availableTelemetry & enabledTelemetry;
  usbPendingTelemetry =
    availableAndEnabledTelemetry;
  i2cPendingTelemetryBuffer =
    i2cPendingTelemetryBuffer | availableAndEnabledTelemetry;
 }

void ESAT_EPSClass::update()
{
  updateMaximumPowerPointTracking();
  updatePendingTelemetryLists();
  respondToI2CRequest();
  updateLEDBrightness();
}

void ESAT_EPSClass::updateLEDBrightness()
{
  const word milliseconds = millis() % 1000;
  const float brightness = (milliseconds / 1000.) * 100.;
  ESAT_EPSLED.write(brightness);
}

void ESAT_EPSClass::updateMaximumPowerPointTracking()
{
  ESAT_MaximumPowerPointTrackingDriver1.update();
  ESAT_MaximumPowerPointTrackingDriver2.update();
}

void ESAT_EPSClass::writeTelemetry(ESAT_CCSDSPacket& packet)
{
  (void) usbWriter.unbufferedWrite(packet);
}

ESAT_EPSClass ESAT_EPS;
