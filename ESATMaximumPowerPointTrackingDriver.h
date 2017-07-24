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

#ifndef ESATMaximumPowerPointTrackingDriver_h
#define ESATMaximumPowerPointTrackingDriver_h

#include <Energia.h>

// Maximum power point tracking (MPPT) driver.
class ESATMaximumPowerPointTrackingDriver
{
  public:
    // Duty cycle relative to 255.
    byte dutyCycle;

    // Instantiate an MPPT driver that takes input from a sensor pin
    // and drives an output pin with a PWM signal.
    ESATMaximumPowerPointTrackingDriver(int sensorPin, int outputPin);

    // Configure the pins of the MPPT driver.
    void begin();

    // Set the mode of operation to fixed mode.
    void setFixedMode();

    // Set the mode of operation to MPPT mode.
    void setMPPTMode();

    // Set the mode of operation to sweep mode.
    void setSweepMode();

    // Update the MPPT driver.
    void update();

  private:
    // Modes of operation.
    enum Mode
    {
      FIXED_MODE,
      MPPT_MODE,
      SWEEP_MODE,
    };

    // Default duty cycle.
    static const byte defaultDutyCycle = 50;

    // Current mode of operation.
    Mode mode;

    // Previous sensor reading.
    int previousReading;

    // Increment to the output setting.
    int dutyCycleIncrement;

    // Read the current from this pin.
    const int sensorPin;

    // Write a PWM signal to this pin.
    const int outputPin;

    // Compute the sign of setpointIncrement for increasing the
    // delivered power.  Helper function for updateMPPTMode().
    int gradientDirection();

    // Update the MPPT driver in fixed mode.
    void updateFixedMode();

    // Update the MPPT driver in MPPT mode.
    void updateMPPTMode();

    // Update the MPPT driver in sweep mode.
    void updateSweepMode();
};

#endif /* ESATMaximumPowerPointTrackingDriver_h */
