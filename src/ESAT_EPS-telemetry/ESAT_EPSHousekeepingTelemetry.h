/*
 * Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid
 *
 * This file is part of Theia Space's ESAT Util library.
 *
 * Theia Space's ESAT Util library is free software: you can
 * redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Theia Space's ESAT Util library is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's ESAT Util library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef ESAT_EPSHousekeepingTelemetry_h
#define ESAT_EPSHousekeepingTelemetry_h

#include <Arduino.h>
#include <ESAT_CCSDSTelemetryPacketContents.h>

// EPS housekeeping telemetry packet contents.
// Use the global instance ESAT_EPSHousekeepingTelemetry.
class ESAT_EPSHousekeepingTelemetryClass: public ESAT_CCSDSTelemetryPacketContents
{
  public:
    // Return true (EPS housekeeping telemetry packets are always available).
    boolean available();

    // Return the packet identifier of EPS housekeeping telemetry.
    byte packetIdentifier()
    {
      return 0x00;
    }

    // Fill the user data section of the provided packet
    // with EPS housekeeping telemetry.
    // Return true on success; otherwise return false.
    boolean fillUserData(ESAT_CCSDSPacket& packet);
};

// Global instance of the ESAT_EPSHousekeepingTelemetry library.
extern ESAT_EPSHousekeepingTelemetryClass ESAT_EPSHousekeepingTelemetry;

#endif /* ESAT_EPSHousekeepingTelemetry */
