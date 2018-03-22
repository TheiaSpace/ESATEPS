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

#ifndef ESAT_EPSHousekeeping_h
#define ESAT_EPSHousekeeping_h

#include <Arduino.h>
#include <ESAT_CCSDSPacketContents.h>

// EPS housekeeping packet contents.
// Use the global instance ESAT_EPSHousekeeping.
class ESAT_EPSHousekeepingClass: public ESAT_CCSDSPacketContents
{
  public:
    // Return true (EPS housekeeping packets are always available).
    boolean available();

    // Return the packet identifier of EPS housekeeping telemetry.
    byte packetIdentifier();

    // Fill the user data section of the provided packet
    // with EPS housekeeping.
    // Return true on success; otherwise return false.
    boolean fillUserData(ESAT_CCSDSPacket& packet);

  private:
    // Packet identifier of EPS housekeeping telemetry.
    static const byte PACKET_IDENTIFIER = 0;
};

// Global instance of the ESAT_EPSHousekeeping library.
extern ESAT_EPSHousekeepingClass ESAT_EPSHousekeeping;

#endif /* ESAT_EPSHousekeeping */
