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
#include "VectorMaths4.hpp"

// *** ROW MAJOR ***

// ///////////////////
// Operators
// ///////////////////
inline Matrix4x4& operator+=(Matrix4x4& lhs, const Matrix4x4& rhs)
{
    lhs.data[0] += rhs.data[0];
    lhs.data[1] += rhs.data[1];
    lhs.data[2] += rhs.data[2];
    lhs.data[3] += rhs.data[3];
    return lhs;
}

inline Matrix4x4& operator-=(Matrix4x4& lhs, const Matrix4x4& rhs)
{
    lhs.data[0] -= rhs.data[0];
    lhs.data[1] -= rhs.data[1];
    lhs.data[2] -= rhs.data[2];
    lhs.data[3] -= rhs.data[3];
    return lhs;
}

inline Matrix4x4& operator*=(Matrix4x4& lhs, const Matrix4x4& rhs)
{
    auto& l         = lhs.data;
    const auto& r   = rhs.data;

    auto result = Matrix4x4
    {
            (l[0].data[0] * r[0].data[0]) + (l[0].data[1] * r[1].data[0]) + (l[0].data[2] * r[2].data[0]) + (l[0].data[3] * r[3].data[0]),
            (l[0].data[0] * r[0].data[1]) + (l[0].data[1] * r[1].data[1]) + (l[0].data[2] * r[2].data[1]) + (l[0].data[3] * r[3].data[1]),
            (l[0].data[0] * r[0].data[2]) + (l[0].data[1] * r[1].data[2]) + (l[0].data[2] * r[2].data[2]) + (l[0].data[3] * r[3].data[2]),
            (l[0].data[0] * r[0].data[3]) + (l[0].data[1] * r[1].data[3]) + (l[0].data[2] * r[2].data[3]) + (l[0].data[3] * r[3].data[3]),

            (l[1].data[0] * r[0].data[0]) + (l[1].data[1] * r[1].data[0]) + (l[1].data[2] * r[2].data[0]) + (l[1].data[3] * r[3].data[0]),
            (l[1].data[0] * r[0].data[1]) + (l[1].data[1] * r[1].data[1]) + (l[1].data[2] * r[2].data[1]) + (l[1].data[3] * r[3].data[1]),
            (l[1].data[0] * r[0].data[2]) + (l[1].data[1] * r[1].data[2]) + (l[1].data[2] * r[2].data[2]) + (l[1].data[3] * r[3].data[2]),
            (l[1].data[0] * r[0].data[3]) + (l[1].data[1] * r[1].data[3]) + (l[1].data[2] * r[2].data[3]) + (l[1].data[3] * r[3].data[3]),

            (l[2].data[0] * r[0].data[0]) + (l[2].data[1] * r[1].data[0]) + (l[2].data[2] * r[2].data[0]) + (l[2].data[3] * r[3].data[0]),
            (l[2].data[0] * r[0].data[1]) + (l[2].data[1] * r[1].data[1]) + (l[2].data[2] * r[2].data[1]) + (l[2].data[3] * r[3].data[1]),
            (l[2].data[0] * r[0].data[2]) + (l[2].data[1] * r[1].data[2]) + (l[2].data[2] * r[2].data[2]) + (l[2].data[3] * r[3].data[2]),
            (l[2].data[0] * r[0].data[3]) + (l[2].data[1] * r[1].data[3]) + (l[2].data[2] * r[2].data[3]) + (l[2].data[3] * r[3].data[3]),

            (l[3].data[0] * r[0].data[0]) + (l[3].data[1] * r[1].data[0]) + (l[3].data[2] * r[2].data[0]) + (l[3].data[3] * r[3].data[0]),
            (l[3].data[0] * r[0].data[1]) + (l[3].data[1] * r[1].data[1]) + (l[3].data[2] * r[2].data[1]) + (l[3].data[3] * r[3].data[1]),
            (l[3].data[0] * r[0].data[2]) + (l[3].data[1] * r[1].data[2]) + (l[3].data[2] * r[2].data[2]) + (l[3].data[3] * r[3].data[2]),
            (l[3].data[0] * r[0].data[3]) + (l[3].data[1] * r[1].data[3]) + (l[3].data[2] * r[2].data[3]) + (l[3].data[3] * r[3].data[3]),
    };

    lhs = result;
    return lhs;
}

inline constexpr Matrix4x4 operator-(const Matrix4x4& lhs)
{
    return
    {
        -lhs.data[0],
        -lhs.data[1],
        -lhs.data[2],
        -lhs.data[3],
    };
}

inline Matrix4x4 operator+(Matrix4x4 lhs, const Matrix4x4& rhs){ lhs += rhs;  return lhs; }
inline Matrix4x4 operator-(Matrix4x4 lhs, const Matrix4x4& rhs){ lhs -= rhs;  return lhs; }
inline Matrix4x4 operator*(Matrix4x4 lhs, const Matrix4x4& rhs){ lhs *= rhs;  return lhs; }

inline Matrix4x4& operator*=(Matrix4x4& lhs, float rhs)
{
    lhs.data[0] *= rhs;
    lhs.data[1] *= rhs;
    lhs.data[2] *= rhs;
    lhs.data[3] *= rhs;
    return lhs;
}

inline Matrix4x4 operator*(Matrix4x4 lhs, float rhs){ lhs *= rhs;  return lhs; }

// ///////////////////
// Matrix Maths.
// ///////////////////
inline Matrix4x4 Transpose(const Matrix4x4& lhs)
{
    return
    {
        lhs.data[0].data[0],
        lhs.data[1].data[0],
        lhs.data[2].data[0],
        lhs.data[3].data[0],

        lhs.data[0].data[1],
        lhs.data[1].data[1],
        lhs.data[2].data[1],
        lhs.data[3].data[1],

        lhs.data[0].data[2],
        lhs.data[1].data[2],
        lhs.data[2].data[2],
        lhs.data[3].data[2],

        lhs.data[0].data[3],
        lhs.data[1].data[3],
        lhs.data[2].data[3],
        lhs.data[3].data[3],
    };
}

inline constexpr Matrix4x4 Translation(const Vec3& lhs)
{
    return
    {
        1.0f, 0.0f, 0.0f, lhs.data[0],
        0.0f, 1.0f, 0.0f, lhs.data[1],
        0.0f, 0.0f, 0.0f, lhs.data[2],
        0.0f, 0.0f, 0.0f, 1.0f,
    };
}

Matrix4x4 Inverse(const Matrix4x4& lhs)
{
    // Modified from
    // http://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
    Matrix4x4 inverse = {0};

    inverse.data[0].data[0] =
        lhs.data[1].data[1]  * lhs.data[2].data[2] * lhs.data[3].data[3] -
        lhs.data[1].data[1]  * lhs.data[2].data[3] * lhs.data[3].data[2] -
        lhs.data[2].data[1]  * lhs.data[1].data[2]  * lhs.data[3].data[3] +
        lhs.data[2].data[1]  * lhs.data[1].data[3]  * lhs.data[3].data[2] +
        lhs.data[3].data[1] * lhs.data[1].data[2]  * lhs.data[2].data[3] -
        lhs.data[3].data[1] * lhs.data[1].data[3]  * lhs.data[2].data[2];

    inverse.data[1].data[0] =
        -lhs.data[1].data[0]  * lhs.data[2].data[2] * lhs.data[3].data[3] +
        lhs.data[1].data[0]  * lhs.data[2].data[3] * lhs.data[3].data[2] +
        lhs.data[2].data[0]  * lhs.data[1].data[2]  * lhs.data[3].data[3] -
        lhs.data[2].data[0]  * lhs.data[1].data[3]  * lhs.data[3].data[2] -
        lhs.data[3].data[0] * lhs.data[1].data[2]  * lhs.data[2].data[3] +
        lhs.data[3].data[0] * lhs.data[1].data[3]  * lhs.data[2].data[2];

    inverse.data[2].data[0] =
        lhs.data[1].data[0]  * lhs.data[2].data[1] * lhs.data[3].data[3] -
        lhs.data[1].data[0]  * lhs.data[2].data[3] * lhs.data[3].data[1] -
        lhs.data[2].data[0]  * lhs.data[1].data[1] * lhs.data[3].data[3] +
        lhs.data[2].data[0]  * lhs.data[1].data[3] * lhs.data[3].data[1] +
        lhs.data[3].data[0] * lhs.data[1].data[1] * lhs.data[2].data[3] -
        lhs.data[3].data[0] * lhs.data[1].data[3] * lhs.data[2].data[1];

    inverse.data[3].data[0] =
        -lhs.data[1].data[0]  * lhs.data[2].data[1] * lhs.data[3].data[2] +
        lhs.data[1].data[0]  * lhs.data[2].data[2] * lhs.data[3].data[1] +
        lhs.data[2].data[0]  * lhs.data[1].data[1] * lhs.data[3].data[2] -
        lhs.data[2].data[0]  * lhs.data[1].data[2] * lhs.data[3].data[1] -
        lhs.data[3].data[0] * lhs.data[1].data[1] * lhs.data[2].data[2] +
        lhs.data[3].data[0] * lhs.data[1].data[2] * lhs.data[2].data[1];

    inverse.data[0].data[1] =
        -lhs.data[0].data[1]  * lhs.data[2].data[2] * lhs.data[3].data[3] +
        lhs.data[0].data[1]  * lhs.data[2].data[3] * lhs.data[3].data[2] +
        lhs.data[2].data[1]  * lhs.data[0].data[2] * lhs.data[3].data[3] -
        lhs.data[2].data[1]  * lhs.data[0].data[3] * lhs.data[3].data[2] -
        lhs.data[3].data[1] * lhs.data[0].data[2] * lhs.data[2].data[3] +
        lhs.data[3].data[1] * lhs.data[0].data[3] * lhs.data[2].data[2];

    inverse.data[1].data[1] =
        lhs.data[0].data[0]  * lhs.data[2].data[2] * lhs.data[3].data[3] -
        lhs.data[0].data[0]  * lhs.data[2].data[3] * lhs.data[3].data[2] -
        lhs.data[2].data[0]  * lhs.data[0].data[2] * lhs.data[3].data[3] +
        lhs.data[2].data[0]  * lhs.data[0].data[3] * lhs.data[3].data[2] +
        lhs.data[3].data[0] * lhs.data[0].data[2] * lhs.data[2].data[3] -
        lhs.data[3].data[0] * lhs.data[0].data[3] * lhs.data[2].data[2];

    inverse.data[2].data[1] =
        -lhs.data[0].data[0]  * lhs.data[2].data[1] * lhs.data[3].data[3] +
        lhs.data[0].data[0]  * lhs.data[2].data[3] * lhs.data[3].data[1] +
        lhs.data[2].data[0]  * lhs.data[0].data[1] * lhs.data[3].data[3] -
        lhs.data[2].data[0]  * lhs.data[0].data[3] * lhs.data[3].data[1] -
        lhs.data[3].data[0] * lhs.data[0].data[1] * lhs.data[2].data[3] +
        lhs.data[3].data[0] * lhs.data[0].data[3] * lhs.data[2].data[1];

    inverse.data[3].data[1] =
        lhs.data[0].data[0]  * lhs.data[2].data[1] * lhs.data[3].data[2] -
        lhs.data[0].data[0]  * lhs.data[2].data[2] * lhs.data[3].data[1] -
        lhs.data[2].data[0]  * lhs.data[0].data[1] * lhs.data[3].data[2] +
        lhs.data[2].data[0]  * lhs.data[0].data[2] * lhs.data[3].data[1] +
        lhs.data[3].data[0] * lhs.data[0].data[1] * lhs.data[2].data[2] -
        lhs.data[3].data[0] * lhs.data[0].data[2] * lhs.data[2].data[1];

    inverse.data[0].data[2] =
        lhs.data[0].data[1]  * lhs.data[1].data[2] * lhs.data[3].data[3] -
        lhs.data[0].data[1]  * lhs.data[1].data[3] * lhs.data[3].data[2] -
        lhs.data[1].data[1]  * lhs.data[0].data[2] * lhs.data[3].data[3] +
        lhs.data[1].data[1]  * lhs.data[0].data[3] * lhs.data[3].data[2] +
        lhs.data[3].data[1] * lhs.data[0].data[2] * lhs.data[1].data[3] -
        lhs.data[3].data[1] * lhs.data[0].data[3] * lhs.data[1].data[2];

    inverse.data[1].data[2] =
        -lhs.data[0].data[0]  * lhs.data[1].data[2] * lhs.data[3].data[3] +
        lhs.data[0].data[0]  * lhs.data[1].data[3] * lhs.data[3].data[2] +
        lhs.data[1].data[0]  * lhs.data[0].data[2] * lhs.data[3].data[3] -
        lhs.data[1].data[0]  * lhs.data[0].data[3] * lhs.data[3].data[2] -
        lhs.data[3].data[0] * lhs.data[0].data[2] * lhs.data[1].data[3] +
        lhs.data[3].data[0] * lhs.data[0].data[3] * lhs.data[1].data[2];

    inverse.data[2].data[2] =
        lhs.data[0].data[0]  * lhs.data[1].data[1] * lhs.data[3].data[3] -
        lhs.data[0].data[0]  * lhs.data[1].data[3] * lhs.data[3].data[1] -
        lhs.data[1].data[0]  * lhs.data[0].data[1] * lhs.data[3].data[3] +
        lhs.data[1].data[0]  * lhs.data[0].data[3] * lhs.data[3].data[1] +
        lhs.data[3].data[0] * lhs.data[0].data[1] * lhs.data[1].data[3] -
        lhs.data[3].data[0] * lhs.data[0].data[3] * lhs.data[1].data[1];

    inverse.data[3].data[2] =
        -lhs.data[0].data[0]  * lhs.data[1].data[1] * lhs.data[3].data[2] +
        lhs.data[0].data[0]  * lhs.data[1].data[2] * lhs.data[3].data[1] +
        lhs.data[1].data[0]  * lhs.data[0].data[1] * lhs.data[3].data[2] -
        lhs.data[1].data[0]  * lhs.data[0].data[2] * lhs.data[3].data[1] -
        lhs.data[3].data[0] * lhs.data[0].data[1] * lhs.data[1].data[2] +
        lhs.data[3].data[0] * lhs.data[0].data[2] * lhs.data[1].data[1];

    inverse.data[0].data[3] =
        -lhs.data[0].data[1] * lhs.data[1].data[2] * lhs.data[2].data[3] +
        lhs.data[0].data[1] * lhs.data[1].data[3] * lhs.data[2].data[2] +
        lhs.data[1].data[1] * lhs.data[0].data[2] * lhs.data[2].data[3] -
        lhs.data[1].data[1] * lhs.data[0].data[3] * lhs.data[2].data[2] -
        lhs.data[2].data[1] * lhs.data[0].data[2] * lhs.data[1].data[3] +
        lhs.data[2].data[1] * lhs.data[0].data[3] * lhs.data[1].data[2];

    inverse.data[1].data[3] =
        lhs.data[0].data[0] * lhs.data[1].data[2] * lhs.data[2].data[3] -
        lhs.data[0].data[0] * lhs.data[1].data[3] * lhs.data[2].data[2] -
        lhs.data[1].data[0] * lhs.data[0].data[2] * lhs.data[2].data[3] +
        lhs.data[1].data[0] * lhs.data[0].data[3] * lhs.data[2].data[2] +
        lhs.data[2].data[0] * lhs.data[0].data[2] * lhs.data[1].data[3] -
        lhs.data[2].data[0] * lhs.data[0].data[3] * lhs.data[1].data[2];

    inverse.data[2].data[3] =
        -lhs.data[0].data[0] * lhs.data[1].data[1] * lhs.data[2].data[3] +
        lhs.data[0].data[0] * lhs.data[1].data[3] * lhs.data[2].data[1] +
        lhs.data[1].data[0] * lhs.data[0].data[1] * lhs.data[2].data[3] -
        lhs.data[1].data[0] * lhs.data[0].data[3] * lhs.data[2].data[1] -
        lhs.data[2].data[0] * lhs.data[0].data[1] * lhs.data[1].data[3] +
        lhs.data[2].data[0] * lhs.data[0].data[3] * lhs.data[1].data[1];

    inverse.data[3].data[3] =
        lhs.data[0].data[0] * lhs.data[1].data[1] * lhs.data[2].data[2] -
        lhs.data[0].data[0] * lhs.data[1].data[2] * lhs.data[2].data[1] -
        lhs.data[1].data[0] * lhs.data[0].data[1] * lhs.data[2].data[2] +
        lhs.data[1].data[0] * lhs.data[0].data[2] * lhs.data[2].data[1] +
        lhs.data[2].data[0] * lhs.data[0].data[1] * lhs.data[1].data[2] -
        lhs.data[2].data[0] * lhs.data[0].data[2] * lhs.data[1].data[1];

    auto determinant =
            lhs.data[0].data[0] * inverse.data[0].data[0] +
            lhs.data[0].data[1] * inverse.data[1].data[0] +
            lhs.data[0].data[2] * inverse.data[2].data[0] +
            lhs.data[0].data[3] * inverse.data[3].data[0];

    // TODO: Deal with 0 determinant.

    auto inverseDeterminant = 1.0f / determinant;

    return inverse * inverseDeterminant;
}

// ///////////////////
// Vector Maths.
// ///////////////////
inline Vec4 operator*(const Matrix4x4& lhs, const Vec4& rhs)
{
    return
    {
        DotF(lhs.data[0], rhs),
        DotF(lhs.data[1], rhs),
        DotF(lhs.data[2], rhs),
        DotF(lhs.data[3], rhs)
    };
}

inline Vec4 operator*(const Vec4& lhs, const Matrix4x4& rhs)
{
    return
    {
        rhs.data[0].data[0] * lhs.data[0] + rhs.data[1].data[0] * lhs.data[1] + rhs.data[2].data[0] * lhs.data[2] + rhs.data[3].data[0] * lhs.data[3],
        rhs.data[0].data[1] * lhs.data[0] + rhs.data[1].data[1] * lhs.data[1] + rhs.data[2].data[1] * lhs.data[2] + rhs.data[3].data[1] * lhs.data[3],
        rhs.data[0].data[2] * lhs.data[0] + rhs.data[1].data[2] * lhs.data[1] + rhs.data[2].data[2] * lhs.data[2] + rhs.data[3].data[2] * lhs.data[3],
        rhs.data[0].data[3] * lhs.data[0] + rhs.data[1].data[3] * lhs.data[1] + rhs.data[2].data[3] * lhs.data[2] + rhs.data[3].data[3] * lhs.data[3],
    };
}

// ///////////////////
// Conversions
// ///////////////////
inline constexpr Matrix4x4 ToOpenGL(const Matrix4x4& lhs)
{
    return Transpose(lhs);
}

inline constexpr Matrix4x4 ToDirectX(const Matrix4x4& lhs)
{
    return lhs;
}
