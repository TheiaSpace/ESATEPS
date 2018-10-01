/*
 * Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_EPSDisableTelemetryTelecommand_h
#define ESAT_EPSDisableTelemetryTelecommand_h

#include <Arduino.h>
#include <ESAT_CCSDSPacketConsumer.h>
#include <ESAT_SemanticVersionNumber.h>

// Telecommand handler for EPS_DISABLE_TELEMETRY.
// Used by ESAT_EPSSubsystem.
class ESAT_EPSDisableTelemetryTelecommandClass: public ESAT_CCSDSPacketConsumer
{
  public:
    // Handle a telecommand packet.
    // Return true on success; otherwise return false.
    boolean consume(ESAT_CCSDSPacket packet);

  private:
    // Identifier of the EPS_DOWNLOAD_STORED_TELEMETRY telecommand.
    static const byte EPS_DISABLE_TELEMETRY = 0x31;

    // with a version number that is backwards-compatible with
    // this version number.
    static const ESAT_SemanticVersionNumber INTERFACE_VERSION_NUMBER;

    // Return true if this telecommand handler accepts the
    // packet with the given secondary header; otherwise
    // return false.
    boolean accept(ESAT_CCSDSSecondaryHeader secondaryHeader) const;

    // Handle the telecommand packet (given with the read/write pointer
    // at the start of the user data field).
    // Return true on success; otherwise return false.
    boolean handle(ESAT_CCSDSPacket packet) const;
};

// Global instance of ESAT_EPSDisableTelemetryTelecommandClass.
// Used by ESAT_EPSSubsystem.
extern ESAT_EPSDisableTelemetryTelecommandClass ESAT_EPSDisableTelemetryTelecommand;

#endif /* ESAT_EPSDisableTelemetryTelecommand_h */
