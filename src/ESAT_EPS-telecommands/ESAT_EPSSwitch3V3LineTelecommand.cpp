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

#include "ESAT_EPS-telecommands/ESAT_EPSSwitch3V3LineTelecommand.h"
#include "ESAT_EPS-peripherals/ESAT_PowerLineSwitch.h"

boolean ESAT_EPSSwitch3V3LineTelecommandClass::handleUserData(ESAT_CCSDSPacket packet)
{
  const byte commandParameter = packet.readByte();
  if (packet.triedToReadBeyondLength())
  {
    return false;
  }
  else
  {
    if (commandParameter > 0)
    {
      ESAT_PowerLine3V3Switch.write(ESAT_PowerLine3V3Switch.ON);
    }
    else
    {
      ESAT_PowerLine3V3Switch.write(ESAT_PowerLine3V3Switch.OFF);
    }
    return true;
  }
}

ESAT_EPSSwitch3V3LineTelecommandClass ESAT_EPSSwitch3V3LineTelecommand;
