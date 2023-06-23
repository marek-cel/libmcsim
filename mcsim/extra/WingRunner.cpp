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

#include <mcsim/extra/WingRunner.h>

#include <mcutils/misc/String.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void WingRunner::ComputeForceAndMoment(const Vector3& vel_bas,
                                       const Vector3& omg_bas,
                                       const Vector3& r_c_bas,
                                       const Vector3& n_c_bas)
{
    f_bas_.Zeroize();
    m_bas_.Zeroize();

    if ( active_ )
    {
        double delta = n_c_bas * ( r_c_bas - data().r_wt_bas ) - data().h_wt;

        // wing tip velocities components
        Vector3 v_c_bas = vel_bas + ( omg_bas % data().r_wt_bas );
        double v_norm = n_c_bas * v_c_bas;

        // normal force
        double for_norm = data().k * delta - data().c * v_norm;

        // resulting forces
        f_bas_ = for_norm * n_c_bas;
        m_bas_ = r_c_bas % f_bas_;

        if ( !f_bas_.IsValid() || !m_bas_.IsValid() )
        {
            f_bas_.Zeroize();
            m_bas_.Zeroize();

            // TODO
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void WingRunner::Update(double dt, const Vector3 &vel_bas, bool on_ground)
{
    if ( active_ )
    {
        if ( dt > 0.0 )
        {
            if ( vel_bas.GetLength() > data().v_max || ( !on_ground ) )
            {
                active_ = false;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
