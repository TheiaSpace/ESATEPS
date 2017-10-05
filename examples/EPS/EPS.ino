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

#include <ESATEPS.h>

void setup()
{
  EPS.begin();
}

void loop()
{
  const word bufferLength = 256;
  byte buffer[bufferLength];
  ESATCCSDSPacket packet(buffer, bufferLength);
  while (EPS.readTelecommand(packet))
  {
    EPS.handleTelecommand(packet);
  }
  EPS.update();
  while (EPS.readTelemetry(packet))
  {
    EPS.sendTelemetry(packet);
  }
}
