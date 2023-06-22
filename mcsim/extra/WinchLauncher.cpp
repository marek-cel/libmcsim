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

#include <mcsim/extra/WinchLauncher.h>

#include <algorithm>
#include <cmath>

#include <mcutils/misc/String.h>
#include <mcutils/physics/Physics.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void WinchLauncher::ComputeForceAndMoment( const Matrix3x3& wgs2bas,
                                           const Vector3& pos_wgs )
{
    f_bas_.Zeroize();
    m_bas_.Zeroize();

    if ( active_ )
    {
        Vector3 pos_bas = wgs2bas * ( pos_wgs_ - pos_wgs );

        double x = std::max(0.0, pos_bas.GetLength() - l_);

        f_bas_ = std::max(f_, fabs(x) * x * data_.stiffness) * pos_bas.GetNormalized();
        m_bas_ = data_.r_a_bas % f_bas_;

        if ( !f_bas_.IsValid() || !m_bas_.IsValid() )
        {
            // TODO
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void WinchLauncher::Update(double timeStep,
                           const Matrix3x3& bas2wgs,
                           const Matrix3x3& wgs2ned,
                           const Vector3& pos_wgs,
                           double altitude_agl)
{
    if ( active_ )
    {
        if ( timeStep > 0.0 )
        {
            f_ = Physics::inertia(data_.f_max, f_, timeStep, data_.tc_f);
            v_ = Physics::inertia(data_.v_max, v_, timeStep, data_.tc_v);
            l_ = l_ - timeStep * v_;

            Vector3 pos_ned = wgs2ned * ( pos_wgs - pos_wgs_ );

            if ( atan2( -pos_ned.z(), pos_ned.GetLengthXY() ) > data_.e_max )
            {
                active_ = false;
            }
        }
        else
        {
            if ( altitude_agl < 0.0 )
            {
                pos_wgs_ = pos_wgs + bas2wgs * Vector3(data_.l_max, 0.0, 0.0);
                l_ = data_.l_max;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
