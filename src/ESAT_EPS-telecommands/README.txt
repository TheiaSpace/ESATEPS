Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid

This file is part of Theia Space's ESAT EPS library.

Theia Space's ESAT EPS library is free software: you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation, either
version 3 of the License, or (at your option) any later version.

Theia Space's ESAT EPS library is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Theia Space's ESAT EPS library.  If not, see
<http://www.gnu.org/licenses/>.


EPS telecommand handlers provide telecommand responses.  Each
telecommand handler implements the ESAT_CCSDSPacketConsumer interface
from ESATUtil and responds to one or several telecommands.  They are
used by ESAT_EPS.


#ESAT_EPSDisableTelemetryTelecommand

Telecommand handler for EPS_DISABLE_TELEMETRY (0x31): disable the
generation of an EPS telemetry packet.


# ESAT_EPSEnableTelemetryTelecommand

Telecommand handler for EPS_ENABLE_TELEMETRY (0x30): enable the
generation of an EPS telemetry packet.


# ESAT_EPSFixedModeTelecommand

Telecommand handler for EPS_FIXED_MODE (0x22): set the maximum power
point tracking drivers in fixed mode.


# ESAT_EPSMaximum Power Point TrackingModeTelecommand

Telecommand handler for EPS_MAXIMUM POWER POINT TRACKING_MODE (0x20):
set the maximum power point tracking drivers in maximum power point
tracking mode.


# ESAT_EPSSetTimeTelecommand

Telecommand handler for EPS_SET_TIME (0x00): set the time of the
real-time clock of the ESAT EPS board.


# ESAT_EPSSweepModeTelecommand

Telecommand handler for EPS_SWEEP_MODE (0x21): set the maximum power
point tracking drivers in sweep mode.


# ESAT_EPSSwitch3V3LineModeTelecommand

Telecommand handler for EPS_SWITCH_3V3_LINE (0x10): switch the 3V3
power line.


# ESAT_EPSSwitch5VLineModeTelecommand

Telecommand handler for EPS_SWITCH_5V_LINE (0x11): switch the 5V
power line.
