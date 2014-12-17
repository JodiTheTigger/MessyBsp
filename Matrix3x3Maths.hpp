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
    lhs.data[0] += rhs.data[0];
    lhs.data[1] += rhs.data[1];
    lhs.data[2] += rhs.data[2];
    return lhs;
}

inline Matrix3x3& operator-=(Matrix3x3& lhs, const Matrix3x3& rhs)
{
    lhs.data[0] -= rhs.data[0];
    lhs.data[1] -= rhs.data[1];
    lhs.data[2] -= rhs.data[2];
    return lhs;
}

inline Matrix3x3& operator*=(Matrix3x3& lhs, const Matrix3x3& rhs)
{
    auto& l         = lhs.data;
    const auto& r   = rhs.data;

    auto result = Matrix3x3
    {
            (l[0].data[0] * r[0].data[0]) + (l[0].data[1] * r[1].data[0]) + (l[0].data[2] * r[2].data[0]),
            (l[0].data[0] * r[0].data[1]) + (l[0].data[1] * r[1].data[1]) + (l[0].data[2] * r[2].data[1]),
            (l[0].data[0] * r[0].data[2]) + (l[0].data[1] * r[1].data[2]) + (l[0].data[2] * r[2].data[2]),

            (l[1].data[0] * r[0].data[0]) + (l[1].data[1] * r[1].data[0]) + (l[1].data[2] * r[2].data[0]),
            (l[1].data[0] * r[0].data[1]) + (l[1].data[1] * r[1].data[1]) + (l[1].data[2] * r[2].data[1]),
            (l[1].data[0] * r[0].data[2]) + (l[1].data[1] * r[1].data[2]) + (l[1].data[2] * r[2].data[2]),

            (l[2].data[0] * r[0].data[0]) + (l[2].data[1] * r[1].data[0]) + (l[2].data[2] * r[2].data[0]),
            (l[2].data[0] * r[0].data[1]) + (l[2].data[1] * r[1].data[1]) + (l[2].data[2] * r[2].data[1]),
            (l[2].data[0] * r[0].data[2]) + (l[2].data[1] * r[1].data[2]) + (l[2].data[2] * r[2].data[2]),
    };

    lhs = result;
    return lhs;
}

inline constexpr Matrix3x3 operator-(const Matrix3x3& lhs)
{
    return
    {
        -lhs.data[0],
        -lhs.data[1],
        -lhs.data[2],
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
    auto xx      = rotation.data[0] * rotation.data[0];
    auto xy      = rotation.data[0] * rotation.data[1];
    auto xz      = rotation.data[0] * rotation.data[2];
    auto xw      = rotation.data[0] * rotation.data[3];

    auto yy      = rotation.data[1] * rotation.data[1];
    auto yz      = rotation.data[1] * rotation.data[2];
    auto yw      = rotation.data[1] * rotation.data[3];

    auto zz      = rotation.data[2] * rotation.data[2];
    auto zw      = rotation.data[2] * rotation.data[3];

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
            DotF(lhs.data[0], rhs),
            DotF(lhs.data[1], rhs),
            DotF(lhs.data[2], rhs)
    };
}

inline Vec3 operator*(const Vec3& lhs, const Matrix3x3& rhs)
{
    return
    {
            rhs.data[0].data[0] * lhs.data[0] + rhs.data[1].data[0] * lhs.data[1] + rhs.data[2].data[0] * lhs.data[2],
            rhs.data[0].data[1] * lhs.data[0] + rhs.data[1].data[1] * lhs.data[1] + rhs.data[2].data[1] * lhs.data[2],
            rhs.data[0].data[2] * lhs.data[0] + rhs.data[1].data[2] * lhs.data[1] + rhs.data[2].data[2] * lhs.data[2]
    };
}
