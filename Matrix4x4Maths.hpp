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

// *** ROW MAJOR STORAGE ***

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

inline msvc_constexpr Matrix4x4 operator+(const Matrix4x4& lhs, const Matrix4x4& rhs)
{
    return
    {
        lhs.data[0] + rhs.data[0],
        lhs.data[1] + rhs.data[1],
        lhs.data[2] + rhs.data[2],
        lhs.data[3] + rhs.data[3],
    };
}

inline msvc_constexpr Matrix4x4 operator-(const Matrix4x4& lhs, const Matrix4x4& rhs)
{
    return
    {
        lhs.data[0] - rhs.data[0],
        lhs.data[1] - rhs.data[1],
        lhs.data[2] - rhs.data[2],
        lhs.data[3] - rhs.data[3],
    };
}

inline Matrix4x4 operator*(const Matrix4x4& lhs, const Matrix4x4& rhs)
{
    auto temp = lhs;
    temp *= rhs;
    return temp;
}

inline Matrix4x4& operator*=(Matrix4x4& lhs, float rhs)
{
    lhs.data[0] *= rhs;
    lhs.data[1] *= rhs;
    lhs.data[2] *= rhs;
    lhs.data[3] *= rhs;
    return lhs;
}

inline msvc_constexpr Matrix4x4 operator*(const Matrix4x4& lhs, float rhs)
{
    return
    {
        lhs.data[0] * rhs,
        lhs.data[1] * rhs,
        lhs.data[2] * rhs,
        lhs.data[3] * rhs,
    };
}

// ///////////////////
// Matrix Maths.
// ///////////////////
inline msvc_constexpr Matrix4x4 Transpose(const Matrix4x4& lhs)
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

inline msvc_constexpr Matrix4x4 Translation(const Vec3& lhs)
{
    return
    {
        1.0f, 0.0f, 0.0f, lhs.data[0],
        0.0f, 1.0f, 0.0f, lhs.data[1],
        0.0f, 0.0f, 1.0f, lhs.data[2],
        0.0f, 0.0f, 0.0f, 1.0f,
    };
}

Matrix4x4 Inverse(const Matrix4x4& lhs)
{
    // Modified from
    // http://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
    // Note that this doesn't care is it's row or column major storage, maths
    // still works out.
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

/// Calculate the view matrix when the camera at \a eyePosition is looking
/// at the point \a target, orientatied so that straigt up is \a up.
/// RH Means we are using the right hand coordinate system so that the positive
/// z axis points out of the screen when up is y and right is x.
Matrix4x4 LookAtRH(
        const Vec3& eyePosition,
        const Vec3& target,
        const Vec3N& up = {0.0f, 1.0f, 0.0f})
{
    auto direction  = Normalise(target - eyePosition);
    auto right      = Normalise(Cross(direction, up));
    auto newUp      = Normalise(Cross(right, direction));

    return
    {
        right.data[0],     right.data[1],     right.data[2],     -DotF(right, eyePosition),
        newUp.data[0],     newUp.data[1],     newUp.data[2],     -DotF(newUp, eyePosition),
        direction.data[0], direction.data[1], direction.data[2], -DotF(direction, eyePosition),
        0.0f,              0.0f,              0.0f,              1.0f,
    };
}

Matrix4x4 LookAtRH(
        const Vec3& eyePosition,
        Radians yaw,
        Radians pitch)
{
    float cosYaw = std::cos(yaw.data);
    float sinYaw = std::sin(yaw.data);
    float cosPitch = std::cos(pitch.data);
    float sinPitch = std::sin(pitch.data);

    Vec3 NewX =
    {
        cosYaw,
        0,
        -sinYaw
    };

    Vec3 NewY =
    {
        sinYaw*sinPitch,
        cosPitch,
        cosYaw*sinPitch
    };

    Vec3 NewZ =
    {
        sinYaw*cosPitch,
        -sinPitch,
        cosYaw*cosPitch
    };

    return
    {
        NewX.data[0], NewX.data[1], NewX.data[2], -DotF(NewX, eyePosition),
        NewY.data[0], NewY.data[1], NewY.data[2], -DotF(NewY, eyePosition),
        NewZ.data[0], NewZ.data[1], NewZ.data[2], -DotF(NewZ, eyePosition),
        0.0f,         0.0f,         0.0f,         1.0f,
    };
}

Matrix4x4 Rx(Radians theta)
{
    auto c = std::cos(theta.data);
    auto s = std::sin(theta.data);

    return
    {
        1.0f, 0.0f, 0.0f,   0.0f,
        0.0f, c,    -s,     0.0f,
        0.0f, s,    c,      0.0f,
        0.0f, 0.0f, 0.0f,   1.0f,
    };
}

Matrix4x4 Ry(Radians theta)
{
    auto c = std::cos(theta.data);
    auto s = std::sin(theta.data);

    return
    {
        c,      0.0f, s,    0.0f,
        0.0f,   1.0f, 0.0f, 0.0f,
        -s,     0.0f, c,    0.0f,
        0.0f,   0.0f, 0.0f, 1.0f,
    };
}

Matrix4x4 Rz(Radians theta)
{
    auto c = std::cos(theta.data);
    auto s = std::sin(theta.data);

    return
    {
        c,      -s,     0.0f, 0.0f,
        s,      c,      0.0f, 0.0f,
        0.0f,   0.0f,   1.0f, 0.0f,
        0.0f,   0.0f,   0.0f, 1.0f,
    };
}

Matrix4x4 ProjectionMatrix(
    Radians fieldOfView,
    float aspect,
    float nearDistance,
    float farDistance)
{
    //
    // General form of the Projection Matrix
    //
    float f           = 1.0f / std::tan(0.5f * fieldOfView.data);
    float Zd          = farDistance - nearDistance;
    float Znid        = 1.0f / -Zd;
    float twoNearFar  = 2 * farDistance * nearDistance;

    return Matrix4x4
    {
        f / aspect, 0.0f,   0.0f,                                0.0f,
        0.0f,       f,      0.0f,                                0.0f,
        0.0f,       0.0f,   (farDistance + nearDistance) * Znid, twoNearFar * Znid,
        0.0f,       0.0f,   -1.0f,                               0.0f,
    };
}

// ///////////////////
// Vector Maths.
// ///////////////////
inline msvc_constexpr Vec4 operator*(const Matrix4x4& lhs, const Vec4& rhs)
{
    return
    {
        DotF(lhs.data[0], rhs),
        DotF(lhs.data[1], rhs),
        DotF(lhs.data[2], rhs),
        DotF(lhs.data[3], rhs)
    };
}

inline msvc_constexpr Vec4 operator*(const Vec4& lhs, const Matrix4x4& rhs)
{
    return
    {
        rhs.data[0].data[0] * lhs.data[0] + rhs.data[1].data[0] * lhs.data[1] + rhs.data[2].data[0] * lhs.data[2] + rhs.data[3].data[0] * lhs.data[3],
        rhs.data[0].data[1] * lhs.data[0] + rhs.data[1].data[1] * lhs.data[1] + rhs.data[2].data[1] * lhs.data[2] + rhs.data[3].data[1] * lhs.data[3],
        rhs.data[0].data[2] * lhs.data[0] + rhs.data[1].data[2] * lhs.data[1] + rhs.data[2].data[2] * lhs.data[2] + rhs.data[3].data[2] * lhs.data[3],
        rhs.data[0].data[3] * lhs.data[0] + rhs.data[1].data[3] * lhs.data[1] + rhs.data[2].data[3] * lhs.data[2] + rhs.data[3].data[3] * lhs.data[3],
    };
}
