/*
    MessyBsp. BSP collision and loading example code.
    Copyright (C) 2014 Richard Maxwell <jodi.the.tigger@gmail.com>
    This file is part of MessyBsp
    MessyBsp is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU Affero General Public License for more details.
    You should have received a copy of the GNU Affero General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#pragma once

#include "Geometry.hpp"
#include <cmath>

// ///////////////////
// Operators
// ///////////////////
inline Vec4& operator+=(Vec4& lhs, const Vec4& rhs)
{
    lhs.data[0] += rhs.data[0];
    lhs.data[1] += rhs.data[1];
    lhs.data[2] += rhs.data[2];
    lhs.data[3] += rhs.data[3];
    return lhs;
}

inline Vec4& operator-=(Vec4& lhs, const Vec4& rhs)
{
    lhs.data[0] -= rhs.data[0];
    lhs.data[1] -= rhs.data[1];
    lhs.data[2] -= rhs.data[2];
    lhs.data[3] -= rhs.data[3];
    return lhs;
}

inline Vec4& operator*=(Vec4& lhs, const Vec4& rhs)
{
    lhs.data[0] *= rhs.data[0];
    lhs.data[1] *= rhs.data[1];
    lhs.data[2] *= rhs.data[2];
    lhs.data[3] *= rhs.data[3];
    return lhs;
}

inline Vec4& operator/=(Vec4& lhs, const Vec4& rhs)
{
    lhs.data[0] /= rhs.data[0];
    lhs.data[1] /= rhs.data[1];
    lhs.data[2] /= rhs.data[2];
    lhs.data[3] /= rhs.data[3];
    return lhs;
}

inline constexpr Vec4 operator-(const Vec4& lhs)
{
    return Vec4
    {
        -lhs.data[0],
        -lhs.data[1],
        -lhs.data[2],
        -lhs.data[3],
    };
}

inline Vec4 operator+(Vec4 lhs, const Vec4& rhs){ lhs += rhs;  return lhs; }
inline Vec4 operator-(Vec4 lhs, const Vec4& rhs){ lhs -= rhs;  return lhs; }
inline Vec4 operator*(Vec4 lhs, const Vec4& rhs){ lhs *= rhs;  return lhs; }
inline Vec4 operator/(Vec4 lhs, const Vec4& rhs){ lhs /= rhs;  return lhs; }

inline Vec4& operator+=(Vec4& lhs, float rhs)
{
    lhs.data[0] += rhs;
    lhs.data[1] += rhs;
    lhs.data[2] += rhs;
    lhs.data[3] += rhs;
    return lhs;
}

inline Vec4& operator-=(Vec4& lhs, float rhs)
{
    lhs.data[0] -= rhs;
    lhs.data[1] -= rhs;
    lhs.data[2] -= rhs;
    lhs.data[3] -= rhs;
    return lhs;
}

inline Vec4& operator*=(Vec4& lhs, float rhs)
{
    lhs.data[0] *= rhs;
    lhs.data[1] *= rhs;
    lhs.data[2] *= rhs;
    lhs.data[3] *= rhs;
    return lhs;
}

inline Vec4& operator/=(Vec4& lhs, float rhs)
{
    return lhs *= 1.0f / rhs;
}

inline Vec4 operator+(Vec4 lhs, float rhs){ lhs += rhs;  return lhs; }
inline Vec4 operator-(Vec4 lhs, float rhs){ lhs -= rhs;  return lhs; }
inline Vec4 operator*(Vec4 lhs, float rhs){ lhs *= rhs;  return lhs; }
inline Vec4 operator/(Vec4 lhs, float rhs){ lhs /= rhs;  return lhs; }

// ///////////////////
// Vector Return Maths
// ///////////////////
inline constexpr Vec4 Sqrt(const Vec4& lhs)
{
    return Vec4
    {
        std::sqrt(lhs.data[0]),
        std::sqrt(lhs.data[1]),
        std::sqrt(lhs.data[2]),
        std::sqrt(lhs.data[3])
    };
}

/// If it uses the SIMD invsqrt, then it will be less precision
/// than explicitly doing 1.0f/Sqrt(lhs)
inline constexpr Vec4 InvSqrt(const Vec4& lhs)
{
    return
    {
        1.0f / std::sqrt(lhs.data[0]),
        1.0f / std::sqrt(lhs.data[1]),
        1.0f / std::sqrt(lhs.data[2]),
        1.0f / std::sqrt(lhs.data[3])
    };
}

inline constexpr Vec4 Absolute(const Vec4& lhs)
{
    return Vec4
    {
        std::fabs(lhs.data[0]),
        std::fabs(lhs.data[1]),
        std::fabs(lhs.data[2]),
        std::fabs(lhs.data[3])
    };
}

inline Vec4 Dot(const Vec4& lhs, const Vec4& rhs)
{
    // If this compiler is too dumb to do a decent DOT4, then do this instead:
    /*
    // http://www.gamedev.net/topic/617959-c-dot-product-vs-sse-dot-product/
    //__m128 m = _mm_mul_ps(v1, v2);
    //__m128 t = _mm_add_ps(m, _mm_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1)));
    //__m128 result = _mm_add_ps(t, _mm_shuffle_ps(t, t, _MM_SHUFFLE(1, 0, 3, 2)));

    auto multiply = lhs * rhs;
    auto shuffle1 = Vec4
    {
            multiply.data[1],
            multiply.data[0],
            multiply.data[3],
            multiply.data[2],
    };

    // x = x + y
    // y = y + x
    // z = z + w
    // w = w + z
    auto first = multiply + shuffle1;

    auto shuffle2 = Vec4
    {
           first.data[2],
           first.data[3],
           first.data[0],
           first.data[1],
    };

    // x = x + y + (z + w)
    // y = y + x + (w + z)
    // z = z + w + (x + y)
    // w = w + z + (y + x)
    return first + shuffle2;
    */

    // hope the compiler picks up on this pattern and recognises it as a Dot.
    auto mult = lhs * rhs;

    return Vec4
    {
        mult.data[0] + mult.data[1] + mult.data[2] + mult.data[3],
        mult.data[0] + mult.data[1] + mult.data[2] + mult.data[3],
        mult.data[0] + mult.data[1] + mult.data[2] + mult.data[3],
        mult.data[0] + mult.data[1] + mult.data[2] + mult.data[3],
    };
}

// Cross product doesn't exist for Vec4, only Vector3 and Vector7.

inline Vec4 Square(const Vec4& lhs)
{
    return Dot(lhs, lhs);
}

inline Vec4 Magnitude(const Vec4& lhs)
{
    return Sqrt(Square(lhs));
}

inline Vec4N Normalise(const Vec4& lhs)
{
    auto length = Magnitude(lhs);

    if (length.data[0] > 0.0f)
    {
        auto norm = lhs / length;
        return
        {
            norm.data[0],
            norm.data[1],
            norm.data[2],
            norm.data[3],
        };
    }

    // Put an assert here, and stuff will start going
    // wrong as vector is too small. But for now,
    // just assume the vector is normalised in the
    // x direction.
    Vec4N result;
    result.data[0] = 1.0f;
    return result;
}

inline constexpr Vec4 Lerp(const Vec4& lhs, const Vec4& rhs, float scale)
{
    return Vec4
    {
        lhs.data[0] + (rhs.data[0] - lhs.data[0]) * scale,
        lhs.data[1] + (rhs.data[1] - lhs.data[1]) * scale,
        lhs.data[2] + (rhs.data[2] - lhs.data[2]) * scale,
        lhs.data[3] + (rhs.data[3] - lhs.data[3]) * scale
    };
}

inline constexpr Vec4 Max(const Vec4& lhs, const Vec4& rhs)
{
    return Vec4
    {
        lhs.data[0] > rhs.data[0] ? lhs.data[0] : rhs.data[0],
        lhs.data[1] > rhs.data[1] ? lhs.data[1] : rhs.data[1],
        lhs.data[2] > rhs.data[2] ? lhs.data[2] : rhs.data[2],
        lhs.data[3] > rhs.data[3] ? lhs.data[3] : rhs.data[3]
    };
}

inline constexpr Vec4 Min(const Vec4& lhs, const Vec4& rhs)
{
    return Vec4
    {
        lhs.data[0] < rhs.data[0] ? lhs.data[0] : rhs.data[0],
        lhs.data[1] < rhs.data[1] ? lhs.data[1] : rhs.data[1],
        lhs.data[2] < rhs.data[2] ? lhs.data[2] : rhs.data[2],
        lhs.data[3] < rhs.data[3] ? lhs.data[3] : rhs.data[3]
    };
}

inline constexpr Vec4 Clamp(const Vec4& lhs, float min = 0.0f, float max = 1.0f)
{
    return Max(Min({max}, lhs),{min});
}

// ///////////////////
// Scalar Return Maths
// ///////////////////
// Avoid these as they convert from Vectors to floats which
// apparently is a performance penalty, especially if you then
// use the value in more vector calculations.
// http://www.gamasutra.com/view/feature/132636/designing_fast_crossplatform_simd_.php?print=1
inline constexpr float DotF(const Vec4& lhs, const Vec4& rhs)
{
    return
            (lhs.data[0] * rhs.data[0]) +
            (lhs.data[1] * rhs.data[1]) +
            (lhs.data[2] * rhs.data[2]) +
            (lhs.data[3] * rhs.data[3]);
}

/// Returns the area of the square formed with one corner
/// at origin and the other at the point lhs.
inline constexpr float SquareF(const Vec4& lhs)
{
    return DotF(lhs, lhs);
}
