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
#ifndef LIBMCSIM_STATEINDEX_H
#define LIBMCSIM_STATEINDEX_H

////////////////////////////////////////////////////////////////////////////////

#include <cstdint>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

struct MCSIMEXPORT StateIndex
{
    static const uint8_t x  {  0 };     ///< index of aircraft location x-coordinate (origin of BAS axis system) expressed in ECF axis system
    static const uint8_t y  {  1 };     ///< index of aircraft location y-coordinate (origin of BAS axis system) expressed in ECF axis system
    static const uint8_t z  {  2 };     ///< index of aircraft location z-coordinate (origin of BAS axis system) expressed in ECF axis system
    static const uint8_t e0 {  3 };     ///< index of aircraft attitude quaternion e0-coordinate
    static const uint8_t ex {  4 };     ///< index of aircraft attitude quaternion ex-coordinate
    static const uint8_t ey {  5 };     ///< index of aircraft attitude quaternion ey-coordinate
    static const uint8_t ez {  6 };     ///< index of aircraft attitude quaternion ez-coordinate
    static const uint8_t u  {  7 };     ///< index of aircraft linear velocity x-coordinate expressed in BAS axis system
    static const uint8_t v  {  8 };     ///< index of aircraft linear velocity y-coordinate expressed in BAS axis system
    static const uint8_t w  {  9 };     ///< index of aircraft linear velocity z-coordinate expressed in BAS axis system
    static const uint8_t p  { 10 };     ///< index of aircraft angular velocity x-coordinate expressed in BAS axis system
    static const uint8_t q  { 11 };     ///< index of aircraft angular velocity y-coordinate expressed in BAS axis system
    static const uint8_t r  { 12 };     ///< index of aircraft angular velocity z-coordinate expressed in BAS axis system
};

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_STATEINDEX_H
