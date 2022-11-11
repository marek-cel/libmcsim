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
#ifndef MCSIM_DEFS_H_
#define MCSIM_DEFS_H_

////////////////////////////////////////////////////////////////////////////////

#if defined(_MSC_VER)
#   if defined(MCSIM_DLL_EXPORTS)
#       define MCSIM_DLL_SPEC __declspec(dllexport)
#   else
#       define MCSIM_DLL_SPEC __declspec(dllimport)
#   endif
#else
#   define MCSIM_DLL_SPEC
#endif

#if defined(__cplusplus)
#   define MCSIMAPI MCSIM_DLL_SPEC
#endif

#if !defined(MCSIMAPI)
#   define MCSIMAPI
#endif

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_DEFS_H_
