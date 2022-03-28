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
#ifndef MCSIM_PROP_GOVERNOR_H_
#define MCSIM_PROP_GOVERNOR_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcsim/defs.h>

#include <mcutils/math/Table.h>
#include <mcutils/xml/XmlNode.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Propeller governor base class.
 *
 * XML configuration file format:
 * @code
 * <governor>
 * TODO
 * </governor>
 * @endcode
 */
class MCSIMEXPORT Governor
{
public:

    /** @brief Constructor. */
    Governor();

    /** @brief Destructor. */
    virtual ~Governor();

    /**
     * @brief Reads data.
     * @param dataNode XML node
     */
    virtual void readData( XmlNode &dataNode );

    virtual void update( double propellerLever, double rpm );

    inline double getPitch() const { return _pitch; }

protected:

    Table _prop_rpm;        ///< [rpm] propeller setpoint RPM vs [-] propeller RPM lever position

    double _gain_1;         ///< first order gain
    double _gain_2;         ///< second order gain

    double _pitch;          ///< <0.0;1.0> normalized propeller pitch
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_PROP_GOVERNOR_H_
