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


EPS telemetry packet contents objects fill telemetry packets.  Each
telemetry packet contents object implements the
ESAT_CCSDSPacketContents interface from ESATUtil and fills one
telemetry packet.  They are used by ESAT_EPS.


# ESAT_EPSHousekeepingTelemetry

Fill the EPS_HOUSEKEEPING (0x00) telemetry packet.


# ESAT_BatteryModuleHouseekeepingTelemetry

Fill the BATTERY_MODULE_HOUSEKEEEPING (0x01) telemetry packet.
