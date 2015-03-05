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
inline Vec3& operator+=(Vec3& lhs, const Vec3& rhs)
{
    lhs.data[0] += rhs.data[0];
    lhs.data[1] += rhs.data[1];
    lhs.data[2] += rhs.data[2];
    return lhs;
}

inline Vec3& operator-=(Vec3& lhs, const Vec3& rhs)
{
    lhs.data[0] -= rhs.data[0];
    lhs.data[1] -= rhs.data[1];
    lhs.data[2] -= rhs.data[2];
    return lhs;
}

inline Vec3& operator*=(Vec3& lhs, const Vec3& rhs)
{
    lhs.data[0] *= rhs.data[0];
    lhs.data[1] *= rhs.data[1];
    lhs.data[2] *= rhs.data[2];
    return lhs;
}

inline Vec3& operator/=(Vec3& lhs, const Vec3& rhs)
{
    lhs.data[0] /= rhs.data[0];
    lhs.data[1] /= rhs.data[1];
    lhs.data[2] /= rhs.data[2];
    return lhs;
}

inline msvc_constexpr Vec3 operator-(const Vec3& lhs)
{
    return
    {
        -lhs.data[0],
        -lhs.data[1],
        -lhs.data[2]
    };
}

inline Vec3 operator+(Vec3 lhs, const Vec3& rhs){ lhs += rhs;  return lhs; }
inline Vec3 operator-(Vec3 lhs, const Vec3& rhs){ lhs -= rhs;  return lhs; }
inline Vec3 operator*(Vec3 lhs, const Vec3& rhs){ lhs *= rhs;  return lhs; }
inline Vec3 operator/(Vec3 lhs, const Vec3& rhs){ lhs /= rhs;  return lhs; }

inline Vec3& operator+=(Vec3& lhs, float rhs)
{
    lhs.data[0] += rhs;
    lhs.data[1] += rhs;
    lhs.data[2] += rhs;
    return lhs;
}

inline Vec3& operator-=(Vec3& lhs, float rhs)
{
    lhs.data[0] -= rhs;
    lhs.data[1] -= rhs;
    lhs.data[2] -= rhs;
    return lhs;
}

inline Vec3& operator*=(Vec3& lhs, float rhs)
{
    lhs.data[0] *= rhs;
    lhs.data[1] *= rhs;
    lhs.data[2] *= rhs;
    return lhs;
}

inline Vec3& operator/=(Vec3& lhs, float rhs)
{
    return lhs *= 1.0f / rhs;
}

inline Vec3 operator+(Vec3 lhs, float rhs){ lhs += rhs;  return lhs; }
inline Vec3 operator-(Vec3 lhs, float rhs){ lhs -= rhs;  return lhs; }
inline Vec3 operator*(Vec3 lhs, float rhs){ lhs *= rhs;  return lhs; }
inline Vec3 operator/(Vec3 lhs, float rhs){ lhs /= rhs;  return lhs; }

// ///////////////////
// Vector Return Maths
// ///////////////////
inline msvc_constexpr Vec3 Sqrt(const Vec3& lhs)
{
    return
    {
        std::sqrt(lhs.data[0]),
        std::sqrt(lhs.data[1]),
        std::sqrt(lhs.data[2])
    };
}

/// If it uses the SIMD invsqrt, then it will be less precision
/// than explicitly doing 1.0f/Sqrt(lhs)
inline msvc_constexpr Vec3 InvSqrt(const Vec3& lhs)
{
    return
    {
        1.0f / std::sqrt(lhs.data[0]),
        1.0f / std::sqrt(lhs.data[1]),
        1.0f / std::sqrt(lhs.data[2])
    };
}

inline msvc_constexpr Vec3 Absolute(const Vec3& lhs)
{
    return
    {
        std::fabs(lhs.data[0]),
        std::fabs(lhs.data[1]),
        std::fabs(lhs.data[2])
    };
}

inline Vec3 Dot(const Vec3& lhs, const Vec3& rhs)
{
    // If this compiler is too dumb to do a decent DOT4,
    // then use the Vector4 version instead.
    auto mult = lhs * rhs;

    return
    {
        mult.data[0] + mult.data[1] + mult.data[2],
        mult.data[0] + mult.data[1] + mult.data[2],
        mult.data[0] + mult.data[1] + mult.data[2],
    };
}

inline msvc_constexpr Vec3 Cross(const Vec3& lhs, const Vec3& rhs)
{
    return
    {
        lhs.data[1] * rhs.data[2] - lhs.data[2] * rhs.data[1],
        lhs.data[2] * rhs.data[0] - lhs.data[0] * rhs.data[2],
        lhs.data[0] * rhs.data[1] - lhs.data[1] * rhs.data[0]
    };
}

/// Returns the area of the square formed with one corner
/// at origin and the other at the point lhs.
inline Vec3 Square(const Vec3& lhs)
{
    return Dot(lhs, lhs);
}

inline Vec3 Magnitude(const Vec3& lhs)
{
    return Sqrt(Square(lhs));
}

inline Vec3N Normalise(const Vec3& lhs)
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
    Vec3N result;
    result.data[0] = 1.0f;
    return result;
}

inline Vec3 Rotate(Vec3 lhs, const Vec3& wAxis, Radians rotation)
{
    // Stole this from  bullet3's btVec3

    auto o = wAxis * Dot(wAxis, lhs);
    auto _x = lhs - o;
    auto _y = Cross(wAxis, lhs);

    return (o + _x * std::cos(rotation.data) + _y * std::sin(rotation.data));
}

inline msvc_constexpr Vec3 Lerp(const Vec3& lhs, const Vec3& rhs, float scale)
{
    return
    {
        lhs.data[0] + (rhs.data[0] - lhs.data[0]) * scale,
        lhs.data[1] + (rhs.data[1] - lhs.data[1]) * scale,
        lhs.data[2] + (rhs.data[2] - lhs.data[2]) * scale
    };
}

inline msvc_constexpr Vec3 Max(const Vec3& lhs, const Vec3& rhs)
{
    return
    {
        lhs.data[0] > rhs.data[0] ? lhs.data[0] : rhs.data[0],
        lhs.data[1] > rhs.data[1] ? lhs.data[1] : rhs.data[1],
        lhs.data[2] > rhs.data[2] ? lhs.data[2] : rhs.data[2]
    };
}

inline msvc_constexpr Vec3 Min(const Vec3& lhs, const Vec3& rhs)
{
    return
    {
        lhs.data[0] < rhs.data[0] ? lhs.data[0] : rhs.data[0],
        lhs.data[1] < rhs.data[1] ? lhs.data[1] : rhs.data[1],
        lhs.data[2] < rhs.data[2] ? lhs.data[2] : rhs.data[2]
    };
}

inline msvc_constexpr Vec3 Clamp(const Vec3& lhs, float min = 0.0f, float max = 1.0f)
{
    return Max(Min({max}, lhs),{min});
}

// ///////////////////
// Scalar Return Maths
// ///////////////////
inline constexpr float DotF(const Vec3& lhs, const Vec3& rhs)
{
    return
         (lhs.data[0] * rhs.data[0]) +
         (lhs.data[1] * rhs.data[1]) +
         (lhs.data[2] * rhs.data[2]);
}

/// Returns the area of the square formed with one corner
/// at origin and the other at the point lhs.
inline constexpr float SquareF(const Vec3& lhs)
{
    return DotF(lhs, lhs);
}
