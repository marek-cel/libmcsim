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
#ifndef MCSIM_ENV_ATMOSPHEREUS76_H_
#define MCSIM_ENV_ATMOSPHEREUS76_H_

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief US76 Standard Atmosphere model class.
 *
 * This class is used to compute altitude depend atmospheric data. It is
 * based on U.S. Standard Atmosphere 1976 extended by user defined sea level
 * conditions. User defined sea level conditions affect only the lowest layer
 * up to 11,000 m above mean sea level.
 *
 * Model is valid up to 84,852 meters above mean sea level.
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li><a href="https://ntrs.nasa.gov/citations/19770009539">US Standard Atmosphere 1976, NASA-TM-X-74335</a></li>
 * </ul>
 */
class MCSIMAPI AtmosphereUS76
{
public:

    /** Gas species indeces. */
    enum GasSpeciesIndeces
    {
        N2 = 0,                         ///< index of Nitrogen       (N2)  in tables _m_i and _f_i
        O2,                             ///< index of Oxygen         (O2)  in tables _m_i and _f_i
        Ar,                             ///< index of Argon          (Ar)  in tables _m_i and _f_i
        CO2,                            ///< index of Carbon Dioxide (C02) in tables _m_i and _f_i
        Ne,                             ///< index of Neon           (Ne)  in tables _m_i and _f_i
        He,                             ///< index of Helium         (He)  in tables _m_i and _f_i
        Kr,                             ///< index of Krypton        (Kr)  in tables _m_i and _f_i
        Xe,                             ///< index of Xenon          (Xe)  in tables _m_i and _f_i
        CH4,                            ///< index of Methane        (CH4) in tables _m_i and _f_i
        H2                              ///< index of Hydrogen       (H2)  in tables _m_i and _f_i
    };

    static const double _m_i[ 10 ];     ///< [kg/kmol] molecular weight
    static const double _f_i[ 10 ];     ///< [-]       fractional volume

    static const double _h_b[ 7 ];      ///< [m]   altitude values
    static const double _p_b[ 7 ];      ///< [Pa]  pressure values
    static const double _t_b[ 7 ];      ///< [K]   temperature values
    static const double _l_b[ 7 ];      ///< [K/m] temperature gradients

    static const double _g;             ///< [m/s^2]         standard gravitional acceleration
    static const double _m;             ///< [kg/kmol]        mean molecular weight
    static const double _r;             ///< [J/(kmol*K)]     universal gas constant
    static const double _s;             ///< [K]              Sutherland constant
    static const double _beta;          ///< [kg/(s*m*K^0.5)] a constant used in computing dynamic viscosity
    static const double _gamma;         ///< [-]              a constant taken to represent the ratio of specific heat at constant pressure to the specific heat at constant volume (cp/cv)

    static const double _std_sl_t;      ///< [K]      standard sea level temperature (288.15 K or 15 deg C)
    static const double _std_sl_p;      ///< [Pa]     standard sea level pressure (1013.25 hPa)
    static const double _std_sl_rho;    ///< [kg/m^3] standard sea level density (1.225 kg/m^3)
    static const double _std_sl_c;      ///< [m/s]    standard sea level speed of sound (340.293 m/s)
    static const double _std_sl_mu;     ///< [Pa*s]   standard sea level dynamic viscosity (1.79118e-05 Pa*s)
    static const double _std_sl_nu;     ///< [m^2/s]  standard sea level kinematic viscosity (1.46218e-05 m^2/s)

    /**
     * @brief Computes density altitude.
     * @param pressure [Pa] outside pressure
     * @param temperature [K] outside temperature
     * @param altitude [m] altitude above sea level
     * @return [m] density altitude
     */
    static double getDensityAltitude( double pressure, double temperature,
                                      double altitude );

    /**
     * @brief Updates atmosphere due to altitude.
     * @param altitude [m] altitude above sea level
     */
    void update( double altitude );

    /**
     * @brief Sets sea level air pressure value.
     * @param p_sl [Pa] sea level air pressure
     */
    void setPressureSL( double p_sl );

    /**
     * @brief Sets sea level air temperature value.
     * @param t_sl [K] sea level air temperature
     */
    void setTemperatureSL( double t_sl );

    inline double getTemperature  () const { return _temperature;  }
    inline double getPressure     () const { return _pressure;     }
    inline double getDensity      () const { return _density;      }
    inline double getSpeedOfSound () const { return _speedOfSound; }
    inline double getDynViscosity () const { return _dynViscosity; }
    inline double getKinViscosity () const { return _kinViscosity; }

private:

    double _temperature_0 { _std_sl_t   };  ///< [K]      sea level air temperature
    double _pressure_0    { _std_sl_p   };  ///< [Pa]     sea level air pressure

    double _temperature   { _std_sl_t   };  ///< [K]      air temperature
    double _pressure      { _std_sl_p   };  ///< [Pa]     air static pressure
    double _density       { _std_sl_rho };  ///< [kg/m^3] air density
    double _speedOfSound  { _std_sl_c   };  ///< [m/s]    speed of sound
    double _dynViscosity  { _std_sl_mu  };  ///< [Pa*s]   dynamic viscosity
    double _kinViscosity  { _std_sl_nu  };  ///< [m^2/s]  kinematic viscosity
};

using AtmosphereUS76PtrS = std::shared_ptr < AtmosphereUS76 >;
using AtmosphereUS76PtrU = std::unique_ptr < AtmosphereUS76 >;
using AtmosphereUS76PtrW = std::weak_ptr   < AtmosphereUS76 >;

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_ENV_ATMOSPHEREUS76_H_
