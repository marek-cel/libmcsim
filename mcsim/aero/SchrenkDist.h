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
#ifndef MCSIM_AERO_SCHRENKDIST_H_
#define MCSIM_AERO_SCHRENKDIST_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Wing Schrenk approximation class.
 *
 * ### Refernces:
 * - [A Simple Approximation Method for Obtaining the Spanwise Lift Distribution, NACA-TM-948](https://ntrs.nasa.gov/citations/19930094469)
 * - Corke TC.: Design of Aircraft, 2003, p.215 
 * - Galinski C.: Wybrane zagadnienia projektowania samolotow, 2016, p.118. [in Polish]
 */
class MCSIMAPI SchrenkDist
{
public:

    /**
     * @brief Computes equivalent elliptic wing chord at the given spanwise coordinate. 
     * @param y [m] spanwise coordinate where 0: wing root (plane of symmetry)
     * @return [m] equivalent elliptic wing chord
     */
    double GetEllipticWingChord(double y) const;

    /**
     * @brief Computes approximation of lift coefficient.
     *
     * @par Computes Schrenk approximation of normalized spanwise lift coefficient
     * distribution.
     *
     * @param y [m] spanwise coordinate where 0: wing root (plane of symmetry)
     * @return [-] spanwise lift coefficient
     */
    double GetLiftCoefDist(double y) const;

    /**
     * @brief Sets wing area.
     * @param area [m^2] wing area
     */
    void set_area(double area);

    /**
     * @brief Sets wing span.
     * @param span [m] wing span
     */
    void set_span(double span);

    /**
     * @brief Sets wing chord.
     * @param chord [m] wing chord vs [m] spanwise coordinate
     */
    void set_chord(const Table& chord);

protected:

    double area_ = 0.0;     ///< [m^2] wing area
    double span_ = 0.0;     ///< [m] wing span

    Table chord_;           ///< [m] wing chord vs [m] spanwise coordinate
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_AERO_SCHRENKDIST_H_
