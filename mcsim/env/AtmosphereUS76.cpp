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

#include <cmath>

#include <mcutils/misc/Log.h>
#include <mcutils/physics/Physics.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

// US Standard Atmosphere 1976, Table 3, p.3
const double AtmosphereUS76::kM_i[] = {
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
const double AtmosphereUS76::kF_i[] = {
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
const double AtmosphereUS76::kH_b[] = {
    11000.0 ,
    20000.0 ,
    32000.0 ,
    47000.0 ,
    51000.0 ,
    71000.0 ,
    84852.0
};

// US Standard Atmosphere 1976, Table I, p.50-73
const double AtmosphereUS76::kP_b[] = {
    101325.0   ,
     22632.0   ,
      5474.8   ,
       868.01  ,
       110.9   ,
        66.938 ,
         3.9564
};

// US Standard Atmosphere 1976, Table I, p.50-73
const double AtmosphereUS76::kT_b[] = {
    288.15 ,
    216.65 ,
    216.65 ,
    228.65 ,
    270.65 ,
    270.65 ,
    214.65
};

// US Standard Atmosphere 1976, Table 4, p.3
const double AtmosphereUS76::kL_b[] = {
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
        ( AtmosphereUS76::kM_i[ AtmosphereUS76::N2  ] * AtmosphereUS76::kF_i[ AtmosphereUS76::N2  ]
        + AtmosphereUS76::kM_i[ AtmosphereUS76::O2  ] * AtmosphereUS76::kF_i[ AtmosphereUS76::O2  ]
        + AtmosphereUS76::kM_i[ AtmosphereUS76::Ar  ] * AtmosphereUS76::kF_i[ AtmosphereUS76::Ar  ]
        + AtmosphereUS76::kM_i[ AtmosphereUS76::CO2 ] * AtmosphereUS76::kF_i[ AtmosphereUS76::CO2 ]
        + AtmosphereUS76::kM_i[ AtmosphereUS76::Ne  ] * AtmosphereUS76::kF_i[ AtmosphereUS76::Ne  ]
        + AtmosphereUS76::kM_i[ AtmosphereUS76::He  ] * AtmosphereUS76::kF_i[ AtmosphereUS76::He  ]
        + AtmosphereUS76::kM_i[ AtmosphereUS76::Kr  ] * AtmosphereUS76::kF_i[ AtmosphereUS76::Kr  ]
        + AtmosphereUS76::kM_i[ AtmosphereUS76::Xe  ] * AtmosphereUS76::kF_i[ AtmosphereUS76::Xe  ]
        + AtmosphereUS76::kM_i[ AtmosphereUS76::CH4 ] * AtmosphereUS76::kF_i[ AtmosphereUS76::CH4 ]
        + AtmosphereUS76::kM_i[ AtmosphereUS76::H2  ] * AtmosphereUS76::kF_i[ AtmosphereUS76::H2  ] )
        /
        ( AtmosphereUS76::kF_i[ AtmosphereUS76::N2  ]
        + AtmosphereUS76::kF_i[ AtmosphereUS76::O2  ]
        + AtmosphereUS76::kF_i[ AtmosphereUS76::Ar  ]
        + AtmosphereUS76::kF_i[ AtmosphereUS76::CO2 ]
        + AtmosphereUS76::kF_i[ AtmosphereUS76::Ne  ]
        + AtmosphereUS76::kF_i[ AtmosphereUS76::He  ]
        + AtmosphereUS76::kF_i[ AtmosphereUS76::Kr  ]
        + AtmosphereUS76::kF_i[ AtmosphereUS76::Xe  ]
        + AtmosphereUS76::kF_i[ AtmosphereUS76::CH4 ]
        + AtmosphereUS76::kF_i[ AtmosphereUS76::H2  ] );

const double AtmosphereUS76::kR     = 8.31432e3;        // US Standard Atmosphere 1976, Table 2, p.2
const double AtmosphereUS76::kS     = 110.0;            // US Standard Atmosphere 1976, Table 2, p.2
const double AtmosphereUS76::kBeta  = 1.458e-6;         // US Standard Atmosphere 1976, Table 2, p.2
const double AtmosphereUS76::kGamma = 1.4;              // US Standard Atmosphere 1976, Table 2, p.2

const double AtmosphereUS76::kStdSL_press = 101325.0;   // US Standard Atmosphere 1976, Table 2, p.2
const double AtmosphereUS76::kStdSL_temp  = 288.15;     // US Standard Atmosphere 1976, Table 2, p.2
const double AtmosphereUS76::kStdSL_rho   = 1.225;
const double AtmosphereUS76::kStdSL_sos   = 340.293;
const double AtmosphereUS76::kStdSL_mu    = 1.79118e-05;
const double AtmosphereUS76::kStdSL_nu    = 1.46218e-05;

////////////////////////////////////////////////////////////////////////////////

double AtmosphereUS76::GetDensityAltitude(double pressure, double temperature,
                                          double altitude)
{
    static double b = ( -kL_b[0]*kR  ) / ( kG*kM + kL_b[0]*kR  );

    double result = altitude;

    if ( altitude < kH_b[0] )
    {
        double a = ( pressure / kStdSL_press ) / ( temperature / kStdSL_temp );
        result = -( kStdSL_temp / kL_b[0] ) * ( 1.0 - pow(a, b) );
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////

void AtmosphereUS76::Update(double altitude)
{
    double h_b = kH_b[ 5 ];
    double p_b = kP_b[ 6 ];
    double t_b = kT_b[ 6 ];
    double l_b = 0.0;

    if ( altitude < kH_b[ 0 ] )
    {
        h_b = 0.0;
        p_b = kP_b[ 0 ];
        t_b = sl_temperature_;
        l_b = -( sl_temperature_ - kT_b[ 1 ] ) / kH_b[ 0 ];
    }
    else
    {
        for ( int i = 1; i < 7; ++i )
        {
            if ( altitude < kH_b[i] )
            {
                h_b = kH_b[i - 1];
                p_b = kP_b[i];
                t_b = kT_b[i];
                l_b = kL_b[i];

                break;
            }
        }

        if ( altitude > kH_b[6] )
        {
            Log::Warning( "Atmosphere altitude above valid range." );
        }
    }

    double delta_h = altitude - h_b;

    // [K] temperature, US Standard Atmosphere 1976, p.10
    temperature_ = t_b + l_b * delta_h;

    // [Pa] pressure, US Standard Atmosphere 1976, p.12
    if ( fabs( l_b ) < 1.0e-6 )
    {
        pressure_ = p_b * exp( -( kG * kM * delta_h ) / ( kR * t_b ) );
    }
    else
    {
        pressure_ = p_b * pow(t_b/temperature_, (kG * kM) / (kR*l_b));

        if ( altitude < kH_b[0] )
        {
            double c_h = ( kH_b[0] - altitude ) / kH_b[0];
            double c_p = ( c_h * sl_pressure_ + (1.0 - c_h) * kStdSL_press ) / kStdSL_press;
            pressure_ = pressure_ * c_p;
        }
    }

    // [kg/m^3] density, US Standard Atmosphere 1976, p.15
    density_ = ( pressure_ * kM ) / ( kR * temperature_ );

    // [m/s] speed of sound, US Standard Atmosphere 1976, p.18
    speed_of_sound_ = sqrt( ( kGamma * kR * temperature_ ) / kM );

    // [Pa*s] dynamic viscosity, US Standard Atmosphere 1976, p.19
    dyn_viscosity_ = kBeta * pow(temperature_, 3.0/2.0) / ( temperature_ + kS );

    // [m^2/s] kinematic viscosity, US Standard Atmosphere 1976, p.19
    kin_viscosity_ = dyn_viscosity_ / density_;
}

////////////////////////////////////////////////////////////////////////////////

void AtmosphereUS76::set_sl_pressure(double press)
{
    if ( press > 90000.0 && press < 110000.0 )
    {
        sl_pressure_ = press;
    }
    else
    {
        Log::Warning( "Atmosphere wrong value of sea level pressure." );
    }
}

////////////////////////////////////////////////////////////////////////////////

void AtmosphereUS76::set_sl_temperature(double temp)
{
    if ( temp > 173.15 && temp < 373.15 )
    {
        sl_temperature_ = temp;
    }
    else
    {
        Log::Warning( "Atmosphere wrong value of sea level temperature." );
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
