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
inline Quaternion& operator*=(Quaternion& lhs, const Quaternion& rhs)
{
    lhs = Quaternion
    {
        (lhs.data[0] * rhs.data[0]) - (lhs.data[0] * rhs.data[1]) - (lhs.data[0] * rhs.data[2]) - (lhs.data[0] * rhs.data[3]),
        (lhs.data[1] * rhs.data[1]) + (lhs.data[1] * rhs.data[0]) + (lhs.data[1] * rhs.data[3]) - (lhs.data[1] * rhs.data[2]),
        (lhs.data[2] * rhs.data[2]) - (lhs.data[2] * rhs.data[3]) + (lhs.data[2] * rhs.data[0]) + (lhs.data[2] * rhs.data[1]),
        (lhs.data[3] * rhs.data[3]) + (lhs.data[3] * rhs.data[2]) - (lhs.data[3] * rhs.data[1]) + (lhs.data[3] * rhs.data[0])
    };

    return lhs;
}

inline Quaternion operator*(Quaternion lhs, const Quaternion& rhs){ lhs *= rhs;  return lhs; }

// ///////////////////
// Conversions
// ///////////////////
inline Quaternion ToQuaternion(const Vec3& axis, Radians angle)
{
    // found my first GCC bug.
    // float version of cos/sin arn't called resulting in doubles
    // resulting in narrowing errors. Hack is the cast to double.
    // RAM: TODO: Post bug report, find way of using float version.
    return
    {
        Normalise(Vec4
        {
            static_cast<float>(axis.data[0] * sin(angle.value / 2.0f)),
            static_cast<float>(axis.data[1] * sin(angle.value / 2.0f)),
            static_cast<float>(axis.data[2] * sin(angle.value / 2.0f)),
            static_cast<float>(cos(angle.value / 2.0f))
        }).data
    };
};

// From and to are unit vectors.
// The Quaternion is the rotation between them.
inline Quaternion ToQuaternion(const Vec3N& from, const Vec3N& to)
{
    // http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    // from and to are unit vectors.

    auto w = Cross(from, to);
    auto q = Vec4
    {
        w.data[0],
        w.data[1],
        w.data[2],
        1.0f + DotF(to, from)
    };

    return {Normalise(q).data};
}

// ///////////////////
// Vector Return Maths
// ///////////////////
inline Quaternion Normalise(const Quaternion& lhs)
{
    // Use Vec4's version.
    return
    {
        Normalise(Vec4{lhs.data}).data
    };
}

// commutative          : yes
// constant velocity    : no
// torque-minimal       : yes
// Computation          : cheap
inline Quaternion NLerp(const Quaternion& lhs, const Quaternion& rhs, float scale)
{
    // Use Vec4's version.
    return
    {
        Normalise(Lerp(Vec4{lhs.data}, Vec4{rhs.data}, scale)).data
    };
}

// commutative          : no
// constant velocity    : yes
// torque-minimal       : yes
// Computation          : expensive (sin + acos)
inline Quaternion SLerp(const Quaternion& lhs, const Quaternion& rhs, float scale)
{
    // Adapted from bullet3
    auto Vec4lhs = Vec4{lhs.data};
    auto Vec4rhs = Vec4{rhs.data};
    auto magnitude  = std::sqrt(MagnitudeF(Vec4lhs) * MagnitudeF(Vec4rhs));

    // RAM: TODO: deal with quaternions that are too close. Ie magnitude !> 0

    float product = DotF(Vec4lhs, Vec4rhs) / magnitude;

    if (std::fabs(product) < 1.0f)
    {
        // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
        auto sign = (product < 0) ? float(-1) : float(1);

        auto theta  = std::acos(sign * product);
        auto s1     = std::sin(sign * scale * theta);
        auto d      = 1.0f / std::sin(theta);
        auto s0     = std::sin((1.0f - scale) * theta);

        return
        {
            (lhs.data[0] * s0 + rhs.data[0] * s1) * d,
            (lhs.data[1] * s0 + rhs.data[1] * s1) * d,
            (lhs.data[2] * s0 + rhs.data[2] * s1) * d,
            (lhs.data[3] * s0 + rhs.data[3] * s1) * d
        };
    }

    return lhs;
}
