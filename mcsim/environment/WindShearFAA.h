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
#ifndef LIBMCSIM_ENVIRONMENT_WINDSHEARFAA_H
#define LIBMCSIM_ENVIRONMENT_WINDSHEARFAA_H

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcsim/defs.h>

#include <mcutil/math/Vector3.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Windshear model class.
 *
 * @see Windshear Training Aid, Volume 2 - Substantiating Data, FAA, 1990, p. 4.3-144 - 4.3-153
 */
class MCSIMEXPORT WindShearFAA
{
public:

    /**
     * @brief Gets windshear model 1
     * @see FAA Windshear Training Aid Vol. 2, Reference Wind Model #1, p. 4.3-146
     * @param distance [m]
     * @return [m/s] wind speed vector expressed in Tailwind-Crosswind-Downdraft convention
     */
    static Vector3 getWindModel1( double distance );

    /**
     * @brief Gets windshear model 2
     * @see FAA Windshear Training Aid Vol. 2, Reference Wind Model #2, p. 4.3-148
     * @param distance [m]
     * @return [m/s] wind speed vector expressed in Tailwind-Crosswind-Downdraft convention
     */
    static Vector3 getWindModel2( double distance );

    /**
     * @brief Gets windshear model 3
     * @see FAA Windshear Training Aid Vol. 2, Reference Wind Model #3, p. 4.3-150
     * @param distance [m]
     * @return [m/s] wind speed vector expressed in Tailwind-Crosswind-Downdraft convention
     */
    static Vector3 getWindModel3( double distance );

    /**
     * @brief Gets windshear model 4
     * @see FAA Windshear Training Aid Vol. 2, Reference Wind Model #4, p. 4.3-152
     * @param distance [m]
     * @return [m/s] wind speed vector expressed in Tailwind-Crosswind-Downdraft convention
     */
    static Vector3 getWindModel4( double distance );
};

using WindShearFAAPtrS = std::shared_ptr < WindShearFAA >;
using WindShearFAAPtrU = std::unique_ptr < WindShearFAA >;
using WindShearFAAPtrW = std::weak_ptr   < WindShearFAA >;

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_ENVIRONMENT_WINDSHEARFAA_H
