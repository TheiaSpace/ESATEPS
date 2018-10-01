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

#include "ESAT_EPS-telecommands/ESAT_EPSSwitch5VLineTelecommand.h"
#include "ESAT_EPS-peripherals/ESAT_PowerLineSwitch.h"

const ESAT_SemanticVersionNumber ESAT_EPSSwitch5VLineTelecommandClass::INTERFACE_VERSION_NUMBER(2, 0, 0);

boolean ESAT_EPSSwitch5VLineTelecommandClass::accept(const ESAT_CCSDSSecondaryHeader secondaryHeader) const
{
  if (!INTERFACE_VERSION_NUMBER.isForwardCompatibleWith(secondaryHeader.majorVersionNumber,
                                                        secondaryHeader.minorVersionNumber,
                                                        secondaryHeader.patchVersionNumber))
  {
    return false;
  }
  if (secondaryHeader.packetIdentifier != EPS_SWITCH_5V_LINE)
  {
    return false;
  }
  return true;
}

boolean ESAT_EPSSwitch5VLineTelecommandClass::consume(ESAT_CCSDSPacket packet)
{
  const ESAT_CCSDSSecondaryHeader secondaryHeader =
    packet.readSecondaryHeader();
  if (accept(secondaryHeader))
  {
    return handle(packet);
  }
  else
  {
    return false;
  }
}

boolean ESAT_EPSSwitch5VLineTelecommandClass::handle(ESAT_CCSDSPacket packet) const
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
      ESAT_PowerLine5VSwitch.write(ESAT_PowerLine5VSwitch.ON);
    }
    else
    {
      ESAT_PowerLine5VSwitch.write(ESAT_PowerLine5VSwitch.OFF);
    }
    return true;
  }
}

ESAT_EPSSwitch5VLineTelecommandClass ESAT_EPSSwitch5VLineTelecommand;
