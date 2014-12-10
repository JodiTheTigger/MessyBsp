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

// ///////////////////
// Simple Maths
// ///////////////////
inline Vector4& operator+=(Vector4& lhs, const Vector4& rhs)
{
    lhs.values[0] += rhs.values[0];
    lhs.values[1] += rhs.values[1];
    lhs.values[2] += rhs.values[2];
    lhs.values[3] += rhs.values[3];
    return lhs;
}

inline Vector4& operator-=(Vector4& lhs, const Vector4& rhs)
{
    lhs.values[0] -= rhs.values[0];
    lhs.values[1] -= rhs.values[1];
    lhs.values[2] -= rhs.values[2];
    lhs.values[3] -= rhs.values[3];
    return lhs;
}

inline Vector4& operator*=(Vector4& lhs, const Vector4& rhs)
{
    lhs.values[0] *= rhs.values[0];
    lhs.values[1] *= rhs.values[1];
    lhs.values[2] *= rhs.values[2];
    lhs.values[3] *= rhs.values[3];
    return lhs;
}

inline Vector4& operator/=(Vector4& lhs, const Vector4& rhs)
{
    lhs.values[0] /= rhs.values[0];
    lhs.values[1] /= rhs.values[1];
    lhs.values[2] /= rhs.values[2];
    lhs.values[3] /= rhs.values[3];
    return lhs;
}

inline constexpr Vector4 operator-(const Vector4& lhs)
{
    return Vector4
    {
        -lhs.values[0],
        -lhs.values[1],
        -lhs.values[2],
        -lhs.values[3],
    };
}

inline Vector4 operator+(Vector4 lhs, const Vector4& rhs){ lhs += rhs;  return lhs; }
inline Vector4 operator-(Vector4 lhs, const Vector4& rhs){ lhs -= rhs;  return lhs; }
inline Vector4 operator*(Vector4 lhs, const Vector4& rhs){ lhs *= rhs;  return lhs; }
inline Vector4 operator/(Vector4 lhs, const Vector4& rhs){ lhs /= rhs;  return lhs; }

inline Vector4& operator*=(Vector4& lhs, float rhs)
{
    lhs.values[0] *= rhs;
    lhs.values[1] *= rhs;
    lhs.values[2] *= rhs;
    lhs.values[3] *= rhs;
    return lhs;
}

inline Vector4& operator/=(Vector4& lhs, float rhs)
{
    return lhs *= 1.0f / rhs;
}

inline Vector4 operator*(Vector4 lhs, float rhs){ lhs *= rhs;  return lhs; }
inline Vector4 operator/(Vector4 lhs, float rhs){ lhs /= rhs;  return lhs; }

// ///////////////////
// Complicated Maths (vector return)
// ///////////////////
inline constexpr Vector4 Sqrt(const Vector4& lhs)
{
    return Vector4
    {
        std::sqrt(lhs.values[0]),
        std::sqrt(lhs.values[1]),
        std::sqrt(lhs.values[2]),
        std::sqrt(lhs.values[3])
    };
}

inline constexpr Vector4 Absolute(const Vector4& lhs)
{
    return Vector4
    {
        std::fabs(lhs.values[0]),
        std::fabs(lhs.values[1]),
        std::fabs(lhs.values[2]),
        std::fabs(lhs.values[3])
    };
}

inline Vector4 Dot(const Vector4& lhs, const Vector4& rhs)
{
    // If this compiler is too dumb to do a decent DOT4, then do this instead:
    /*
    // http://www.gamedev.net/topic/617959-c-dot-product-vs-sse-dot-product/
    //__m128 m = _mm_mul_ps(v1, v2);
    //__m128 t = _mm_add_ps(m, _mm_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1)));
    //__m128 result = _mm_add_ps(t, _mm_shuffle_ps(t, t, _MM_SHUFFLE(1, 0, 3, 2)));

    auto multiply = lhs * rhs;
    auto shuffle1 = Vector4
    {
            multiply.values[1],
            multiply.values[0],
            multiply.values[3],
            multiply.values[2],
    };

    // x = x + y
    // y = y + x
    // z = z + w
    // w = w + z
    auto first = multiply + shuffle1;

    auto shuffle2 = Vector4
    {
           first.values[2],
           first.values[3],
           first.values[0],
           first.values[1],
    };

    // x = x + y + (z + w)
    // y = y + x + (w + z)
    // z = z + w + (x + y)
    // w = w + z + (y + x)
    return first + shuffle2;
    */

    // hope the compiler picks up on this pattern and recognises it as a Dot.
    auto mult = lhs * rhs;

    return Vector4
    {
        mult.values[0] + mult.values[1] + mult.values[2] + mult.values[3],
        mult.values[0] + mult.values[1] + mult.values[2] + mult.values[3],
        mult.values[0] + mult.values[1] + mult.values[2] + mult.values[3],
        mult.values[0] + mult.values[1] + mult.values[2] + mult.values[3],
    };
}

// Cross product doesn't exist for Vector4, only Vector3 and Vector7.

inline constexpr Vec4 Square(const Vec4& lhs)
{
    return Dot(lhs, lhs);
}

inline constexpr Vec4 Magnitude(const Vec4& lhs)
{
    return Sqrt(Square(lhs));
}

inline constexpr Vec4N Normalise(const Vec4& lhs)
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

inline constexpr Vector4 Lerp(const Vector4& lhs, const Vector4& rhs, float scale)
{
    return Vector4
    {
        lhs.values[0] + (rhs.values[0] - lhs.values[0]) * scale,
        lhs.values[1] + (rhs.values[1] - lhs.values[1]) * scale,
        lhs.values[2] + (rhs.values[2] - lhs.values[2]) * scale,
        lhs.values[3] + (rhs.values[3] - lhs.values[3]) * scale
    };
}

inline constexpr Vector4 Max(const Vector4& lhs, const Vector4& rhs)
{
    return Vector4
    {
        lhs.values[0] > rhs.values[0] ? lhs.values[0] : rhs.values[0],
        lhs.values[1] > rhs.values[1] ? lhs.values[1] : rhs.values[1],
        lhs.values[2] > rhs.values[2] ? lhs.values[2] : rhs.values[2],
        lhs.values[3] > rhs.values[3] ? lhs.values[3] : rhs.values[3]
    };
}

inline constexpr Vector4 Min(const Vector4& lhs, const Vector4& rhs)
{
    return Vector4
    {
        lhs.values[0] < rhs.values[0] ? lhs.values[0] : rhs.values[0],
        lhs.values[1] < rhs.values[1] ? lhs.values[1] : rhs.values[1],
        lhs.values[2] < rhs.values[2] ? lhs.values[2] : rhs.values[2],
        lhs.values[3] < rhs.values[3] ? lhs.values[3] : rhs.values[3]
    };
}

// ///////////////////
// Complicated Maths (single return)
// ///////////////////
// Avoid these as they convert from Vectors to floats which
// apparently is a performance penalty, especially if you then
// use the value in more vector calculations.
// http://www.gamasutra.com/view/feature/132636/designing_fast_crossplatform_simd_.php?print=1
inline constexpr float DotF(const Vector4& lhs, const Vector4& rhs)
{
    return
            (lhs.values[0] * rhs.values[0]) +
            (lhs.values[1] * rhs.values[1]) +
            (lhs.values[2] * rhs.values[2]) +
            (lhs.values[3] * rhs.values[3]);
}
