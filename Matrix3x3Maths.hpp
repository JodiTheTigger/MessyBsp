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
#include "VectorMaths3.hpp"

// ///////////////////
// Operators
// ///////////////////
inline Matrix3x3& operator+=(Matrix3x3& lhs, const Matrix3x3& rhs)
{
    lhs.values[0] += rhs.values[0];
    lhs.values[1] += rhs.values[1];
    lhs.values[2] += rhs.values[2];
    return lhs;
}

inline Matrix3x3& operator-=(Matrix3x3& lhs, const Matrix3x3& rhs)
{
    lhs.values[0] -= rhs.values[0];
    lhs.values[1] -= rhs.values[1];
    lhs.values[2] -= rhs.values[2];
    return lhs;
}

inline Matrix3x3& operator*=(Matrix3x3& lhs, const Matrix3x3& rhs)
{
    auto& l         = lhs.values;
    const auto& r   = rhs.values;

    auto result = Matrix3x3
    {
            (l[0].values[0] * r[0].values[0]) + (l[0].values[1] * r[1].values[0]) + (l[0].values[2] * r[2].values[0]),
            (l[0].values[0] * r[0].values[1]) + (l[0].values[1] * r[1].values[1]) + (l[0].values[2] * r[2].values[1]),
            (l[0].values[0] * r[0].values[2]) + (l[0].values[1] * r[1].values[2]) + (l[0].values[2] * r[2].values[2]),

            (l[1].values[0] * r[0].values[0]) + (l[1].values[1] * r[1].values[0]) + (l[1].values[2] * r[2].values[0]),
            (l[1].values[0] * r[0].values[1]) + (l[1].values[1] * r[1].values[1]) + (l[1].values[2] * r[2].values[1]),
            (l[1].values[0] * r[0].values[2]) + (l[1].values[1] * r[1].values[2]) + (l[1].values[2] * r[2].values[2]),

            (l[2].values[0] * r[0].values[0]) + (l[2].values[1] * r[1].values[0]) + (l[2].values[2] * r[2].values[0]),
            (l[2].values[0] * r[0].values[1]) + (l[2].values[1] * r[1].values[1]) + (l[2].values[2] * r[2].values[1]),
            (l[2].values[0] * r[0].values[2]) + (l[2].values[1] * r[1].values[2]) + (l[2].values[2] * r[2].values[2]),
    };

    lhs = result;
    return lhs;
}

inline constexpr Matrix3x3 operator-(const Matrix3x3& lhs)
{
    return
    {
        -lhs.values[0],
        -lhs.values[1],
        -lhs.values[2],
    };
}

inline Matrix3x3 operator+(Matrix3x3 lhs, const Matrix3x3& rhs){ lhs += rhs;  return lhs; }
inline Matrix3x3 operator-(Matrix3x3 lhs, const Matrix3x3& rhs){ lhs -= rhs;  return lhs; }
inline Matrix3x3 operator*(Matrix3x3 lhs, const Matrix3x3& rhs){ lhs *= rhs;  return lhs; }

// ///////////////////
// Conversions
// ///////////////////
inline Matrix3x3 ToMatrix3x3(const Quaternion& rotation)
{
    auto xx      = rotation.values[0] * rotation.values[0];
    auto xy      = rotation.values[0] * rotation.values[1];
    auto xz      = rotation.values[0] * rotation.values[2];
    auto xw      = rotation.values[0] * rotation.values[3];

    auto yy      = rotation.values[1] * rotation.values[1];
    auto yz      = rotation.values[1] * rotation.values[2];
    auto yw      = rotation.values[1] * rotation.values[3];

    auto zz      = rotation.values[2] * rotation.values[2];
    auto zw      = rotation.values[2] * rotation.values[3];

    return Matrix3x3
    {{{
        {
            1.0f - 2.0f * ( yy + zz ),
                   2.0f * ( xy - zw ),
                   2.0f * ( xz + yw )
        },
        {
                    2.0f * ( xy + zw ),
             1.0f - 2.0f * ( xx + zz ),
                    2.0f * ( yz - xw )
        },
        {
                    2.0f * ( xz - yw ),
                    2.0f * ( yz + xw ),
             1.0f - 2.0f * ( xx + yy )
        }
    }}};
}

// ///////////////////
// Vector Maths.
// ///////////////////
inline Vec3 operator*(const Matrix3x3& lhs, const Vec3& rhs)
{
    return
    {
            DotF(lhs.values[0], rhs),
            DotF(lhs.values[1], rhs),
            DotF(lhs.values[2], rhs)
    };
}

inline Vec3 operator*(const Vec3& lhs, const Matrix3x3& rhs)
{
    return
    {
            rhs.values[0].values[0] * lhs.values[0] + rhs.values[1].values[0] * lhs.values[1] + rhs.values[2].values[0] * lhs.values[2],
            rhs.values[0].values[1] * lhs.values[0] + rhs.values[1].values[1] * lhs.values[1] + rhs.values[2].values[1] * lhs.values[2],
            rhs.values[0].values[2] * lhs.values[0] + rhs.values[1].values[2] * lhs.values[1] + rhs.values[2].values[2] * lhs.values[2]
    };
}
