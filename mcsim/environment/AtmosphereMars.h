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
#ifndef LIBMCSIM_ENVIRONMENT_ATMOSPHEREMARS_H
#define LIBMCSIM_ENVIRONMENT_ATMOSPHEREMARS_H

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Mars atmosphere model class.
 *
 * @see NASA Engineering Modelsof the Mars Atmosphere for Entry Vehicle Design, NASA, TN-D-2525, 1964
 */
class MCSIMEXPORT AtmosphereMars
{
public:

    /** @brief Constructor. */
    AtmosphereMars();

    /** @brief Destructor. */
    ~AtmosphereMars();

    /**
     * @brief Updates atmosphere due to altitude.
     * @param altitude [m] altitude above reference level
     */
    void update( double altitude );

    /**
     * @brief Sets reference level air pressure value.
     * @param p_sl [Pa] reference level air pressure
     */
    void setPressureRL( double p_rl );

    /**
     * @brief Sets reference level air temperature value.
     * @param t_sl [K] reference level air temperature
     */
    void setTemperatureRL( double t_rl );

    inline double getTemperature  () const { return _temperature;  }
    inline double getPressure     () const { return _pressure;     }
    inline double getDensity      () const { return _density;      }
    inline double getSpeedOfSound () const { return _speedOfSound; }
    inline double getDynViscosity () const { return _dynViscosity; }
    inline double getKinViscosity () const { return _kinViscosity; }

private:

    double _temperature_0;      ///< [K] reference level air temperature
    double _pressure_0;         ///< [Pa] reference level air pressure

    double _temperature;        ///< [K] air temperature
    double _pressure;           ///< [Pa] air static pressure
    double _density;            ///< [kg/m^3] air density
    double _speedOfSound;       ///< [m/s] speed of sound
    double _dynViscosity;       ///< [Pa*s] dynamic viscosity
    double _kinViscosity;       ///< [m^2/s] kinematic viscosity
};

using AtmosphereMarsPtrS = std::shared_ptr < AtmosphereMars >;
using AtmosphereMarsPtrU = std::unique_ptr < AtmosphereMars >;
using AtmosphereMarsPtrW = std::weak_ptr   < AtmosphereMars >;

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_ENVIRONMENT_ATMOSPHEREMARS_H
