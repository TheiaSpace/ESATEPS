/*
 * Copyright (C) 2018, 2019 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_BatteryModuleHousekeepingTelemetry_h
#define ESAT_BatteryModuleHousekeepingTelemetry_h

#include <Arduino.h>
#include <ESAT_CCSDSTelemetryPacketContents.h>

// Battery module housekeeping telemetry packet contents.
// Use the global instance ESAT_BatteryModuleHousekeepingTelemetry.
class ESAT_BatteryModuleHousekeepingTelemetryClass: public ESAT_CCSDSTelemetryPacketContents
{
  public:
    // Return true when a new telemetry packet is available; otherwise
    // return false;
    boolean available();

    // Return the packet identifier of battery module housekeeping
    // telemetry.
    byte packetIdentifier()
    {
      return 0x01;
    }

    // Fill the user data section of the provided packet
    // with battery module housekeeping telemetry.
    // Return true on success; otherwise return false.
    boolean fillUserData(ESAT_CCSDSPacket& packet);

  private:
    // Generate packets up to once every PERIOD milliseconds.
    // The EPS cycle isn't fast enough to capture fast transients, and
    // the quasi-steady dynamics are slow, so measuring more often
    // would just waste time for no real benefit.
    static const unsigned long PERIOD = 1000;

    // System uptime at the previous packet.
    unsigned long previousPacketTime = 0;
};

// Global instance of the ESAT_BatteryModuleHousekeepingTelemetry library.
extern ESAT_BatteryModuleHousekeepingTelemetryClass ESAT_BatteryModuleHousekeepingTelemetry;

#endif /* ESAT_BatteryModuleHousekeepingTelemetry */
