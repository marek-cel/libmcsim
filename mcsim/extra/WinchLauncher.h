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
#ifndef MCSIM_EXTERNAL_WINCHLAUNCHER_H_
#define MCSIM_EXTERNAL_WINCHLAUNCHER_H_

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcutils/math/Matrix3x3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Sailplane winch launcher model class.
 */
class MCSIMAPI WinchLauncher
{
public:

    /**
     * @brief Winch launcher data struct.
     */
    struct Data
    {
        Vector3 r_a_bas;            ///< [m] winch cable attachment point expressed in BAS

        double f_max = 0.0;         ///< [N]   maximum cable force
        double l_max = 0.0;         ///< [m]   maximum cable length
        double e_max = 0.0;         ///< [rad] maximum elevation
        double v_max = 0.0;         ///< [m/s] maximum cable velocity

        double tc_f = 0.0;          ///< [s] force time constant
        double tc_v = 0.0;          ///< [s] velocity time constant

        double stiffness = 0.0;     ///< [N/m^2] cable stiffness
    };

    // LCOV_EXCL_START
    WinchLauncher() = default;
    WinchLauncher(const WinchLauncher&) = delete;
    WinchLauncher(WinchLauncher&&) = default;
    WinchLauncher& operator=(const WinchLauncher&) = delete;
    WinchLauncher& operator=(WinchLauncher&&) = default;
    virtual ~WinchLauncher() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes force and moment.
     * @param wgs2bas matrix of rotation from WGS to BAS
     * @param pos_wgs [m] aircraft position expressed in WGS
     */
    virtual void ComputeForceAndMoment(const Matrix3x3& wgs2bas,
                                       const Vector3& pos_wgs);

    /**
     * @brief Update winch model.
     * @param dt [s] time step
     * @param bas2wgs matrix of rotation from BAS to WGS
     * @param wgs2ned matrix of rotation from WGS to NED
     * @param pos_wgs [m] aircraft position expressed in WGS
     * @param altitude_agl [m] altitude above ground level
     */
    virtual void Update(double dt,
                        const Matrix3x3& bas2wgs,
                        const Matrix3x3& wgs2ned,
                        const Vector3& pos_wgs,
                        double altitude_agl);

    inline const std::shared_ptr<Data> data() const { return data_; }

    inline const Vector3& f_bas() const { return f_bas_; }
    inline const Vector3& m_bas() const { return m_bas_; }

protected:

    std::shared_ptr<Data> data_;    ///< winch launcher data struct

    Vector3 f_bas_;                 ///< [N] total force vector expressed in BAS
    Vector3 m_bas_;                 ///< [N*m] total moment vector expressed in BAS

    Vector3 pos_wgs_;               ///< [m] winch position expressed in WGS

    double f_ = 0.0;                ///< [N]   current cable force
    double v_ = 0.0;                ///< [m/s] current cable velocity
    double l_ = 0.0;                ///< [m]   current cable length

    bool active_ = true;            ///< specify if winch is active
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_EXTERNAL_WINCHLAUNCHER_H_
