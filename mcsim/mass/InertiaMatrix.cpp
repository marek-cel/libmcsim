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

#include <mcsim/mass/InertiaMatrix.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

Matrix6x6 getInertiaMatrix( double mass, Vector3 s_bas, Matrix3x3 i_bas )
{
    Matrix6x6 mi_bas;

    mi_bas(0,0) =  mass;
    mi_bas(0,1) =  0.0;
    mi_bas(0,2) =  0.0;
    mi_bas(0,3) =  0.0;
    mi_bas(0,4) =  s_bas.z();
    mi_bas(0,5) = -s_bas.y();

    mi_bas(1,0) =  0.0;
    mi_bas(1,1) =  mass;
    mi_bas(1,2) =  0.0;
    mi_bas(1,3) = -s_bas.z();
    mi_bas(1,4) =  0.0;
    mi_bas(1,5) =  s_bas.x();

    mi_bas(2,0) =  0.0;
    mi_bas(2,1) =  0.0;
    mi_bas(2,2) =  mass;
    mi_bas(2,3) =  s_bas.y();
    mi_bas(2,4) = -s_bas.x();
    mi_bas(2,5) =  0.0;

    mi_bas(3,0) =  0.0;
    mi_bas(3,1) = -s_bas.z();
    mi_bas(3,2) =  s_bas.y();
    mi_bas(3,3) =  i_bas.xx();
    mi_bas(3,4) =  i_bas.xy();
    mi_bas(3,5) =  i_bas.xz();

    mi_bas(4,0) =  s_bas.z();
    mi_bas(4,1) =  0.0;
    mi_bas(4,2) = -s_bas.x();
    mi_bas(4,3) =  i_bas.yx();
    mi_bas(4,4) =  i_bas.yy();
    mi_bas(4,5) =  i_bas.yz();

    mi_bas(5,0) = -s_bas.y();
    mi_bas(5,1) =  s_bas.x();
    mi_bas(5,2) =  0.0;
    mi_bas(5,3) =  i_bas.zx();
    mi_bas(5,4) =  i_bas.zy();
    mi_bas(5,5) =  i_bas.zz();

    return mi_bas;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
