/****************************************************************************//*
 * Copyright (C) 2022 Marek M. Cel
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

#include <mcsim/env/AtmosphereUS76.h>

#include <algorithm>
#include <cmath>

#include <mcutils/misc/Log.h>
#include <mcutils/physics/Constants.h>
#include "AtmosphereUS76.h"

namespace mc {

// US Standard Atmosphere 1976, Table 3, p.3
const double AtmosphereUS76::kMi[] = {
    28.0134  ,
    31.9988  ,
    39.948   ,
    44.00995 ,
    20.183   ,
    4.0026   ,
    83.8     ,
    131.3    ,
    16.04303 ,
    2.01594
};

// US Standard Atmosphere 1976, Table 3, p.3
const double AtmosphereUS76::kFi[] = {
    0.78084     ,
    0.209476    ,
    0.00934     ,
    0.000314    ,
    0.00001818  ,
    0.00000524  ,
    0.00000114  ,
    0.000000087 ,
    0.000002    ,
    0.0000005
};

// US Standard Atmosphere 1976, Table 4, p.3
const double AtmosphereUS76::kHb[] = {
    11000.0 ,
    20000.0 ,
    32000.0 ,
    47000.0 ,
    51000.0 ,
    71000.0 ,
    84852.0
};

// US Standard Atmosphere 1976, Table I, p.50-73
const double AtmosphereUS76::kPb[] = {
    101325.0   ,
     22632.0   ,
      5474.8   ,
       868.01  ,
       110.9   ,
        66.938 ,
         3.9564
};

// US Standard Atmosphere 1976, Table I, p.50-73
const double AtmosphereUS76::kTb[] = {
    288.15 ,
    216.65 ,
    216.65 ,
    228.65 ,
    270.65 ,
    270.65 ,
    214.65
};

// US Standard Atmosphere 1976, Table 4, p.3
const double AtmosphereUS76::kLb[] = {
    -6.5e-3 ,
     0.0    ,
     1.0e-3 ,
     2.8e-3 ,
     0.0    ,
    -2.8e-3 ,
    -2.0e-3
};

// [m/s^2] standard gravitional acceleration
const double AtmosphereUS76::kG = 9.80665;

// [kg/kmol] mean molecular weight, US Standard Atmosphere 1976, p.9
const double AtmosphereUS76::kM =
        ( AtmosphereUS76::kMi[ AtmosphereUS76::N2  ] * AtmosphereUS76::kFi[ AtmosphereUS76::N2  ]
        + AtmosphereUS76::kMi[ AtmosphereUS76::O2  ] * AtmosphereUS76::kFi[ AtmosphereUS76::O2  ]
        + AtmosphereUS76::kMi[ AtmosphereUS76::Ar  ] * AtmosphereUS76::kFi[ AtmosphereUS76::Ar  ]
        + AtmosphereUS76::kMi[ AtmosphereUS76::CO2 ] * AtmosphereUS76::kFi[ AtmosphereUS76::CO2 ]
        + AtmosphereUS76::kMi[ AtmosphereUS76::Ne  ] * AtmosphereUS76::kFi[ AtmosphereUS76::Ne  ]
        + AtmosphereUS76::kMi[ AtmosphereUS76::He  ] * AtmosphereUS76::kFi[ AtmosphereUS76::He  ]
        + AtmosphereUS76::kMi[ AtmosphereUS76::Kr  ] * AtmosphereUS76::kFi[ AtmosphereUS76::Kr  ]
        + AtmosphereUS76::kMi[ AtmosphereUS76::Xe  ] * AtmosphereUS76::kFi[ AtmosphereUS76::Xe  ]
        + AtmosphereUS76::kMi[ AtmosphereUS76::CH4 ] * AtmosphereUS76::kFi[ AtmosphereUS76::CH4 ]
        + AtmosphereUS76::kMi[ AtmosphereUS76::H2  ] * AtmosphereUS76::kFi[ AtmosphereUS76::H2  ] )
        /
        ( AtmosphereUS76::kFi[ AtmosphereUS76::N2  ]
        + AtmosphereUS76::kFi[ AtmosphereUS76::O2  ]
        + AtmosphereUS76::kFi[ AtmosphereUS76::Ar  ]
        + AtmosphereUS76::kFi[ AtmosphereUS76::CO2 ]
        + AtmosphereUS76::kFi[ AtmosphereUS76::Ne  ]
        + AtmosphereUS76::kFi[ AtmosphereUS76::He  ]
        + AtmosphereUS76::kFi[ AtmosphereUS76::Kr  ]
        + AtmosphereUS76::kFi[ AtmosphereUS76::Xe  ]
        + AtmosphereUS76::kFi[ AtmosphereUS76::CH4 ]
        + AtmosphereUS76::kFi[ AtmosphereUS76::H2  ] );

const double AtmosphereUS76::kR     = 8.31432e3;        // US Standard Atmosphere 1976, Table 2, p.2
const double AtmosphereUS76::kS     = 110.0;            // US Standard Atmosphere 1976, Table 2, p.2
const double AtmosphereUS76::kBeta  = 1.458e-6;         // US Standard Atmosphere 1976, Table 2, p.2
const double AtmosphereUS76::kGamma = 1.4;              // US Standard Atmosphere 1976, Table 2, p.2

const double AtmosphereUS76::kAltMax = 86000.0;

const double AtmosphereUS76::kStdSlPress = 101325.0;   // US Standard Atmosphere 1976, Table 2, p.2
const double AtmosphereUS76::kStdSlTemp  = 288.15;     // US Standard Atmosphere 1976, Table 2, p.2
const double AtmosphereUS76::kStdSlRho   = 1.225;
const double AtmosphereUS76::kStdSlSoS   = 340.293;
const double AtmosphereUS76::kStdSlMu    = 1.79118e-05;
const double AtmosphereUS76::kStdSlNu    = 1.46218e-05;

const double AtmosphereUS76::kSlTempMax = 343.15;
const double AtmosphereUS76::kSlTempMin = 203.15;

const double AtmosphereUS76::kSlPressMax = 110000.0;
const double AtmosphereUS76::kSlPressMin = 90000.0;

double AtmosphereUS76::GetDensityAltitude(double pressure, double temperature,
                                          double altitude)
{
    double result = altitude;
    if ( altitude < kHb[0] )
    {
        static double b = (-kLb[0]*kR) / (kG*kM + kLb[0]*kR);
        double a = (pressure / kStdSlPress ) / ( temperature / kStdSlTemp);
        result = -(kStdSlTemp / kLb[0] ) * ( 1.0 - pow(a, b));
    }

    return result;
}

void AtmosphereUS76::Update(double altitude)
{
    double alt_limited = std::min(altitude, kAltMax);
    AltConstants ac = GetAltitudeConstants(alt_limited);
    double delta_h = alt_limited - ac.Hb;
    
    temperature_    = ComputeTemperature(delta_h, ac.Tb, ac.Lb);
    pressure_       = ComputePressure(delta_h, ac.Tb, ac.Lb, ac.Pb, altitude, temperature_);
    density_        = ComputeDensity(temperature_, pressure_);
    speed_of_sound_ = ComputeSpeedOfSound(temperature_);
    dyn_viscosity_  = ComputeDynamicViscosity(temperature_);
    kin_viscosity_  = ComputeKinematicViscosity(dyn_viscosity_, density_);
}

void AtmosphereUS76::set_sl_pressure(double press)
{
    if ( press > kSlPressMin && press < kSlPressMax )
    {
        sl_pressure_ = press;
    }
    else
    {
        Log::Warning("Atmosphere wrong value of sea level pressure.");
    }
}

void AtmosphereUS76::set_sl_temperature(double temp)
{
    if ( temp > kSlTempMin && temp < kSlTempMax )
    {
        sl_temperature_ = temp;
    }
    else
    {
        Log::Warning("Atmosphere wrong value of sea level temperature.");
    }
}

int AtmosphereUS76::GetAltitudeIndex(double altitude)
{
    if ( altitude < kHb[0] )
    {
        return 0;
    }
    else
    {
        for ( int i = 1; i < 7; ++i )
        {
            if ( altitude < kHb[i] )
            {
                return i;
                break;
            }
        }

        if ( altitude > kHb[6] )
        {
            Log::Warning("Atmosphere altitude above valid range.");
        }
    }

    return kAltTabsIndexMax;
}

AtmosphereUS76::AltConstants AtmosphereUS76::GetAltitudeConstants(double altitude)
{
    AltConstants result;

    int alt_index = GetAltitudeIndex(altitude);

    if ( alt_index == 0 )
    {
        result.Hb = 0.0;
        result.Pb = kPb[0];
        result.Tb = sl_temperature_;
        result.Lb = -(sl_temperature_ - kTb[1]) / kHb[0];
    }
    else
    {
        result.Hb = kHb[alt_index - 1];
        result.Pb = kPb[alt_index];
        result.Tb = kTb[alt_index];
        result.Lb = kLb[alt_index];
    }

    return result;
}

double AtmosphereUS76::ComputeTemperature(double delta_h, double Tb, double Lb)
{
    // [K] temperature, US Standard Atmosphere 1976, p.10
    return Tb + Lb * delta_h;
}

double AtmosphereUS76::ComputePressure(double delta_h, double Tb, double Lb, double Pb,
                                       double altitude, double temperature)
{
    double pressure = kPb[0];

    // [Pa] pressure, US Standard Atmosphere 1976, p.12
    if (fabs(Lb) < 1.0e-6)
    {
        pressure = Pb * exp(-(kG * kM * delta_h) / (kR * Tb));
    }
    else
    {
        pressure = Pb * pow(Tb / temperature, (kG * kM) / (kR * Lb));

        if (altitude < kHb[0])
        {
            double c_h = (kHb[0] - altitude) / kHb[0];
            double c_p = (c_h * sl_pressure_ + (1.0 - c_h) * kStdSlPress) / kStdSlPress;
            pressure = pressure * c_p;
        }
    }

    return pressure;
}

double AtmosphereUS76::ComputeDensity(double temperature, double pressure)
{
    // [kg/m^3] density, US Standard Atmosphere 1976, p.15
    return (pressure * kM) / (kR * temperature);
}

double AtmosphereUS76::ComputeSpeedOfSound(double temperature)
{
    // [m/s] speed of sound, US Standard Atmosphere 1976, p.18
    return sqrt((kGamma * kR * temperature) / kM);
}

double AtmosphereUS76::ComputeDynamicViscosity(double temperature)
{
    // [Pa*s] dynamic viscosity, US Standard Atmosphere 1976, p.19
    return kBeta * pow(temperature, 3.0 / 2.0) / (temperature + kS);
}

double AtmosphereUS76::ComputeKinematicViscosity(double dyn_viscosity, double density)
{
    // [m^2/s] kinematic viscosity, US Standard Atmosphere 1976, p.19
    return dyn_viscosity / density;
}

} // namespace mc
