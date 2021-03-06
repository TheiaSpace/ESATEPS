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
#include "ESAT_EPS-hardware/ESAT_BatteryController.h"
#include "ESAT_EPS-hardware/ESAT_EPSLED.h"
#include "ESAT_EPS-hardware/ESAT_EPSMeasurements.h"
#include "ESAT_EPS-hardware/ESAT_MaximumPowerPointTrackingDriver.h"
#include "ESAT_EPS-hardware/ESAT_PowerLineSwitch.h"
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

void ESAT_EPSClass::addTelecommand(ESAT_CCSDSTelecommandPacketHandler& telecommand)
{
  telecommandPacketDispatcher.add(telecommand);
}

void ESAT_EPSClass::addTelemetry(ESAT_CCSDSTelemetryPacketContents& telemetry)
{
  telemetryPacketBuilder.add(telemetry);
  enableTelemetry(telemetry.packetIdentifier());
}

void ESAT_EPSClass::begin()
{
  beginTelemetry();
  beginTelecommands();
  beginHardware();
}

void ESAT_EPSClass::beginHardware()
{
  // We pass packets around the USB interface in KISS frames.
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
  Serial.begin();
  // We are a slave node at the OBC I2C interface.
  WireOBC.begin(byte(APPLICATION_PROCESS_IDENTIFIER));
  ESAT_I2CSlave.begin(WireOBC,
                      i2cTelecommandPacketData,
                      MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH,
                      i2cTelemetryPacketData,
                      MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH);
}

void ESAT_EPSClass::beginTelecommands()
{
  addTelecommand(ESAT_EPSSetTimeTelecommand);
  addTelecommand(ESAT_EPSSwitch3V3LineTelecommand);
  addTelecommand(ESAT_EPSSwitch5VLineTelecommand);
  addTelecommand(ESAT_EPSMaximumPowerPointTrackingModeTelecommand);
  addTelecommand(ESAT_EPSSweepModeTelecommand);
  addTelecommand(ESAT_EPSFixedModeTelecommand);
  addTelecommand(ESAT_EPSEnableTelemetryTelecommand);
  addTelecommand(ESAT_EPSDisableTelemetryTelecommand);
}

void ESAT_EPSClass::beginTelemetry()
{
  addTelemetry(ESAT_EPSHousekeepingTelemetry);
  enableTelemetry(ESAT_EPSHousekeepingTelemetry.packetIdentifier());
  addTelemetry(ESAT_BatteryModuleHousekeepingTelemetry);
  disableTelemetry(ESAT_BatteryModuleHousekeepingTelemetry.packetIdentifier());
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
  // We leave the complexity of handling telecommands to the
  // telecommand packet dispatcher.
  (void) telecommandPacketDispatcher.dispatch(packet);
}

boolean ESAT_EPSClass::readTelecommand(ESAT_CCSDSPacket& packet)
{
  // There are two sources of telecommands:
  // - the I2C interface;
  // - the USB interface.
  if (readTelecommandFromI2C(packet))
  {
    return true;
  }
  else
  {
    return readTelecommandFromUSB(packet);
  }
}

boolean ESAT_EPSClass::readTelecommandFromI2C(ESAT_CCSDSPacket& packet)
{
  const boolean gotPacket = ESAT_I2CSlave.readPacket(packet);
  if (gotPacket && packet.isTelecommand())
  {
    return true;
  }
  else
  {
    return false;
  }
}

boolean ESAT_EPSClass::readTelecommandFromUSB(ESAT_CCSDSPacket& packet)
{
  const boolean gotPacket = usbReader.read(packet);
  if (gotPacket && packet.isTelecommand())
  {
    return true;
  }
  else
  {
    return false;
  }
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
  // We try to satisfy named-packet telemetry requests, but only for
  // enabled telemetry packets.  If the requested packet isn't enabled
  // or we don't recognise it, we reject the request.
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
  // We don't generate telecommands.
  ESAT_I2CSlave.rejectPacket();
}

void ESAT_EPSClass::respondToNextPacketTelemetryRequest()
{
  // We only update the list of pending I2C telemetry packets
  // with the contents of the buffered list when we receive
  // a command to reset the telemetry queue.
  if (ESAT_I2CSlave.telemetryQueueResetReceived())
  {
    i2cPendingTelemetry = i2cPendingTelemetryBuffer;
    i2cPendingTelemetryBuffer.clearAll();
  }
  // Some pending I2C telemetry packet might have been disabled
  // since the last I2C request.
  i2cPendingTelemetry = i2cPendingTelemetry & enabledTelemetry;
  // We try to satisfy requests until we run out of packets.
  // Then, we just reject the request.
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
  // We have two pending telemetry lists to update:
  // - The USB pending telemetry list, which is consumed
  //   by readTelemetry().  This list contains the telemetry
  //   packets that are currently available and enabled.
  // - The I2C penging telemetry list buffer, which isn't
  //   consumed by I2C requests directly, but goes to the
  //   list of pending I2C telemetry when we receive a
  //   telemetry queue reset command.  This list accumulates
  //   the telemetry packets that are available and enabled
  //   on each cycle.
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
