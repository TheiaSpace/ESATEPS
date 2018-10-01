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

#include "ESAT_EPS-telecommands/ESAT_EPSFixedModeTelecommand.h"
#include "ESAT_MaximumPowerPointTrackingDriver.h"

const ESAT_SemanticVersionNumber ESAT_EPSFixedModeTelecommandClass::INTERFACE_VERSION_NUMBER(2, 0, 0);

boolean ESAT_EPSFixedModeTelecommandClass::accept(const ESAT_CCSDSSecondaryHeader secondaryHeader) const
{
  if (!INTERFACE_VERSION_NUMBER.isForwardCompatibleWith(secondaryHeader.majorVersionNumber,
                                                        secondaryHeader.minorVersionNumber,
                                                        secondaryHeader.patchVersionNumber))
  {
    return false;
  }
  if (secondaryHeader.packetIdentifier != EPS_FIXED_MODE)
  {
    return false;
  }
  return true;
}

boolean ESAT_EPSFixedModeTelecommandClass::consume(ESAT_CCSDSPacket packet)
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

boolean ESAT_EPSFixedModeTelecommandClass::handle(ESAT_CCSDSPacket packet) const
{
  const byte parameter = packet.readByte();
  if (packet.triedToReadBeyondLength())
  {
    (void) parameter; // Ignored.
    return false;
  }
  else
  {
    const byte dutyCycle = constrain(parameter, 0, 255);
    ESAT_MaximumPowerPointTrackingDriver1.setFixedMode(dutyCycle);
    ESAT_MaximumPowerPointTrackingDriver2.setFixedMode(dutyCycle);
    return true;
  }
}

ESAT_EPSFixedModeTelecommandClass ESAT_EPSFixedModeTelecommand;
