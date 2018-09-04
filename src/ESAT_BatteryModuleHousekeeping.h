/*
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

#ifndef ESAT_BatteryModuleHousekeeping_h
#define ESAT_BatteryModuleHousekeeping_h

#include <Arduino.h>
#include <ESAT_CCSDSPacketContents.h>

// Battery module housekeeping packet contents.
// Use the global instance ESAT_BatteryModuleHousekeeping.
class ESAT_BatteryModuleHousekeepingClass: public ESAT_CCSDSPacketContents
{
  public:
    // Return true when a new packet is available, which is once every
    // second; otherwise return false.
    boolean available();

    // Return the packet identifier of battery module housekeeping
    // telemetry.
    byte packetIdentifier();

    // Fill the user data section of the provided packet
    // with battery module housekeeping.
    // Return true on success; otherwise return false.
    boolean fillUserData(ESAT_CCSDSPacket& packet);

  private:
    // Packet identifier of battery module housekeeping telemetry.
    static const byte PACKET_IDENTIFIER = 1;

    // Generate packets up to once every PERIOD milliseconds.
    // The EPS cycle isn't fast enough to capture fast transients, and
    // the quasi-steady dynamics are slow, so measuring more often
    // would just waste time for no real benefit.
    static const unsigned long PERIOD = 1000;

    // System uptime at the previous packet.
    unsigned long previousPacketTime = 0;
};

// Global instance of the ESAT_BatteryModuleHousekeeping library.
extern ESAT_BatteryModuleHousekeepingClass ESAT_BatteryModuleHousekeeping;

#endif /* ESAT_BatteryModuleHousekeeping */
