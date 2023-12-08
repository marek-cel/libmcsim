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

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief US76 Standard Atmosphere model class.
 *
 * \par This class is used to compute altitude depend atmospheric data. It is
 * based on U.S. Standard Atmosphere 1976 extended by user defined sea level
 * conditions. User defined sea level conditions affect only the lowest layer
 * up to 11,000 m above mean sea level.
 *
 * \par Model is valid up to 84,852 meters above mean sea level.
 *
 * ### Refernces:
 * - [US Standard Atmosphere 1976, NASA-TM-X-74335](https://ntrs.nasa.gov/citations/19770009539)
 */
class MCSIMAPI AtmosphereUS76
{
public:

    /** Altitude dependent constants. */
    struct AltConstants
    {
        double Hb = 0.0;    ///< [m]   geopotential height
        double Pb = 0.0;    ///< [Pa]  pressure
        double Tb = 0.0;    ///< [K]   temperature
        double Lb = 0.0;    ///< [K/m] temperature gradient
    };

    /** Gas species indeces. */
    enum GasSpeciesIndeces
    {
        N2 = 0, ///< index of Nitrogen       (N2)  in tables kMi and kFi
        O2,     ///< index of Oxygen         (O2)  in tables kMi and kFi
        Ar,     ///< index of Argon          (Ar)  in tables kMi and kFi
        CO2,    ///< index of Carbon Dioxide (C02) in tables kMi and kFi
        Ne,     ///< index of Neon           (Ne)  in tables kMi and kFi
        He,     ///< index of Helium         (He)  in tables kMi and kFi
        Kr,     ///< index of Krypton        (Kr)  in tables kMi and kFi
        Xe,     ///< index of Xenon          (Xe)  in tables kMi and kFi
        CH4,    ///< index of Methane        (CH4) in tables kMi and kFi
        H2      ///< index of Hydrogen       (H2)  in tables kMi and kFi
    };

    static constexpr int kGasTabsIndexMax = H2; ///< gas species data tables max index
    static constexpr int kAltTabsIndexMax = 6;  ///< altitude breakpoint values tables max index

    static constexpr int kGasTabsSize = kGasTabsIndexMax + 1;   ///< gas species data tables size
    static constexpr int kAltTabsSize = kAltTabsIndexMax + 1;   ///< altitude breakpoint values tables size

    static const double kMi[kGasTabsSize];  ///< [kg/kmol] molecular weight
    static const double kFi[kGasTabsSize];  ///< [-]       fractional volume

    static const double kHb[kAltTabsSize];  ///< [m]   geopotential height values
    static const double kPb[kAltTabsSize];  ///< [Pa]  pressure values
    static const double kTb[kAltTabsSize];  ///< [K]   temperature values
    static const double kLb[kAltTabsSize];  ///< [K/m] temperature gradients

    static const double kG;             ///< [m/s^2]          standard gravitional acceleration
    static const double kM;             ///< [kg/kmol]        mean molecular weight
    static const double kR;             ///< [J/(kmol*K)]     universal gas constant
    static const double kS;             ///< [K]              Sutherland constant
    static const double kBeta;          ///< [kg/(s*m*K^0.5)] a constant used in computing dynamic viscosity
    static const double kGamma;         ///< [-]              a constant taken to represent the ratio of specific heat at constant pressure to the specific heat at constant volume (cp/cv)

    static const double kStdSlTemp;     ///< [K]      standard sea level temperature (288.15 K or 15 deg C)
    static const double kStdSlPress;    ///< [Pa]     standard sea level pressure (1013.25 hPa)
    static const double kStdSlRho;      ///< [kg/m^3] standard sea level density (1.225 kg/m^3)
    static const double kStdSlSoS;      ///< [m/s]    standard sea level speed of sound (340.293 m/s)
    static const double kStdSlMu;       ///< [Pa*s]   standard sea level dynamic viscosity (1.79118e-05 Pa*s)
    static const double kStdSlNu;       ///< [m^2/s]  standard sea level kinematic viscosity (1.46218e-05 m^2/s)

    /**
     * @brief Computes density altitude.
     * @param pressure [Pa] outside pressure
     * @param temperature [K] outside temperature
     * @param altitude [m] altitude above sea level
     * @return [m] density altitude
     */
    static double GetDensityAltitude(double pressure, double temperature,
                                     double altitude);

    /**
     * @brief Updates atmosphere due to altitude.
     * @param altitude [m] altitude above sea level
     */
    void Update(double altitude);

    inline double temperature() const { return temperature_; }
    inline double pressure() const { return pressure_; }
    inline double density() const { return density_; }
    inline double speed_of_sound() const { return speed_of_sound_; }
    inline double dyn_viscosity() const { return dyn_viscosity_; }
    inline double kin_viscosity() const { return kin_viscosity_; }

    /**
     * @brief Sets sea level air pressure value.
     * @param press [Pa] sea level air pressure
     */
    void set_sl_pressure(double press);

    /**
     * @brief Sets sea level air temperature value.
     * @param temp [K] sea level air temperature
     */
    void set_sl_temperature(double temp);

private:

    double sl_temperature_ = kStdSlTemp;   ///< [K]      sea level air temperature
    double sl_pressure_    = kStdSlPress;  ///< [Pa]     sea level air pressure

    double temperature_    = kStdSlTemp;   ///< [K]      air temperature
    double pressure_       = kStdSlPress;  ///< [Pa]     air static pressure
    double density_        = kStdSlRho;    ///< [kg/m^3] air density
    double speed_of_sound_ = kStdSlSoS;    ///< [m/s]    speed of sound
    double dyn_viscosity_  = kStdSlMu;     ///< [Pa*s]   dynamic viscosity
    double kin_viscosity_  = kStdSlNu;     ///< [m^2/s]  kinematic viscosity

    int GetAltitudeIndex(double altitude);

    AltConstants GetAltitudeConstants(double altitude);

    double ComputeTemperature(double delta_h, double Lb, double Tb);
    double ComputePressure(double delta_h, double Lb, double Tb, double Pb,
                           double altitude, double temperature);
    double ComputeDensity(double pressure, double temperature);
    double ComputeSpeedOfSound(double temperature);
    double ComputeDynamicViscosity(double temperature);
    double ComputeKineticViscosity(double dyn_viscosity, double density);
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_ENV_ATMOSPHEREUS76_H_
