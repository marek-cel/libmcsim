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
#ifndef MCSIM_EXTERNAL_WINGRUNNER_H_
#define MCSIM_EXTERNAL_WINGRUNNER_H_

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Sailplane wing runner model class.
 * This class is intended to model forces applied to a sailplane by a wingrunner
 * (a person that holds sailplane wing in the initial phase of take-off).
 */
class MCSIMAPI WingRunner
{
public:

    /**
     * @brief Wing runner data struct.
     */
    struct Data
    {
        Vector3 r_wt_bas;   ///< [m] wing tip coordinates expressed in BAS

        double h_wt = 0.0;  ///< [m] wing tip desired position above ground

        double k = 0.0;     ///< [N/m] stiffness (linear spring) coefficient
        double c = 0.0;     ///< [N/(m/s)] damping coefficient

        double v_max = 0.0; ///< [m/s] maximum speed
    };

    // LCOV_EXCL_START
    WingRunner() = default;
    WingRunner(const WingRunner&) = delete;
    WingRunner(WingRunner&&) = default;
    WingRunner& operator=(const WingRunner&) = delete;
    WingRunner& operator=(WingRunner&&) = default;
    virtual ~WingRunner() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes force and moment.
     * @param vel_bas [m/s] aircraft linear velocity expressed in BAS
     * @param omg_bas [rad/s] aircraft angular velocity expressed in BAS
     * @param r_c_bas [m] contact point coordinates expressed in BAS
     * @param n_c_bas [-] contact point normal vector expressed in BAS
     */
    virtual void ComputeForceAndMoment(const Vector3& vel_bas,
                                       const Vector3& omg_bas,
                                       const Vector3& r_c_bas,
                                       const Vector3& n_c_bas);

    /**
     * @brief Update wing runner model.
     * @param dt [s] time step
     * @param vel_bas [m/s] aircraft velocity expressed in BAS
     * @param on_ground specifies if aircraft is on the ground
     */
    virtual void Update(double dt, const Vector3& vel_bas, bool on_ground);

    inline const std::shared_ptr<Data> data() const { return data_; }

    inline const Vector3& f_bas() const { return f_bas_; }
    inline const Vector3& m_bas() const { return m_bas_; }

protected:

    std::shared_ptr<Data> data_;    ///< wing runner data struct

    Vector3 f_bas_;                 ///< [N] total force vector expressed in BAS
    Vector3 m_bas_;                 ///< [N*m] total moment vector expressed in BAS

    bool active_ = true;            ///< specify if wing runner is active
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_EXTERNAL_WINGRUNNER_H_
