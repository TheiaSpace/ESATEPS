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

#include <ESAT_EPS.h>

// Main program of the EPS board: respond to telecommands,
// do housekeeping work and send telemetry.

// After a call to EPS.begin(), the board will respond to I2C messages
// on the Wire (SCL_O, SDA_O) interface through the interrupt handling
// code of the ESATI2CSlave module (part of the ESATUtil library): it
// will send telemetry when requested and it will queue telecommands
// for later retrieval on the main loop with readTelecommand().

// Maximum packet data length we will handle.
const word PACKET_DATA_BUFFER_LENGTH = 256;

// Start the peripherals and do some initial bookkeeping work.
void setup()
{
  ESAT_EPS.begin();
}

// Body of the main loop of the program:
// - Retrieve the incomming telecommands (from the I2C interface
//   and from the USB interface).
// - Handle the incoming telecommands.
// - Update the telemetry measurements and do housekeeping work
//   (update the maximum power point tracking drivers, respond to I2C
//   telemetry requests and update the brightness of the heartbeat
//   LED).
// - Retrieve the telemetry packets.
// - Write the telemetry packets through the USB interface.
// The packet data buffer used for telecommand and telemetry
// operations must be big enough for holding all the packet
// data; if the buffer is too small, the packets will be dropped.
void loop()
{
  byte buffer[PACKET_DATA_BUFFER_LENGTH];
  ESAT_CCSDSPacket packet(buffer, sizeof(buffer));
  while (ESAT_EPS.readTelecommand(packet))
  {
    ESAT_EPS.handleTelecommand(packet);
  }
  ESAT_EPS.update();
  while (ESAT_EPS.readTelemetry(packet))
  {
    ESAT_EPS.writeTelemetry(packet);
  }
}
