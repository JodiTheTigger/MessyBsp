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
// Conversions
// ///////////////////
inline constexpr Quaternion ToQuaternion()
{
    return {0.0f, 0.0f, 0.0f, 1.0f};
}

inline Quaternion ToQuaternion(const Vector3& axis, Radians angle)
{
    // found my first GCC bug.
    // float version of cos/sin arn't called resulting in doubles
    // resulting in narrowing errors. Hack is the cast to double.
    // RAM: TODO: Post bug report, find way of using float version.
    return
    {
        Normalise(Vector4
        {
            static_cast<float>(axis.values[0] * sin(angle.value / 2.0f)),
            static_cast<float>(axis.values[1] * sin(angle.value / 2.0f)),
            static_cast<float>(axis.values[2] * sin(angle.value / 2.0f)),
            static_cast<float>(cos(angle.value / 2.0f))
        }).values
    };
};

// From and to are unit vectors.
// The Quaternion is the rotation between them.
inline Quaternion ToQuaternion(const Vector3& from, const Vector3& to)
{
    // http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    // from and to are unit vectors.

    auto w = Cross(from, to);
    auto q = Vector4{w.values[0], w.values[1], w.values[2], 1.0f + DotF(to, from)};

    return {Normalise(q).values};
}

// ///////////////////
// Comparison Operators
// ///////////////////
// Taken from http://stackoverflow.com/questions/4421706/operator-overloading/4421719
// However all "inclass" operators changed to out of class.
inline constexpr bool operator==(const Quaternion& lhs, const Quaternion& rhs)
{
 return  (
             (lhs.values[0]==rhs.values[0]) &&
             (lhs.values[1]==rhs.values[1]) &&
             (lhs.values[2]==rhs.values[2]) &&
             (lhs.values[3]==rhs.values[3])
         );
}

inline constexpr bool operator!=(const Quaternion& lhs, const Quaternion& rhs){return  !operator==(lhs,rhs);}

// ///////////////////
// Simple Maths
// ///////////////////
inline Quaternion& operator*=(Quaternion& lhs, const Quaternion& rhs)
{
    lhs = Quaternion
    {
        (lhs.values[0] * rhs.values[0]) - (lhs.values[0] * rhs.values[1]) - (lhs.values[0] * rhs.values[2]) - (lhs.values[0] * rhs.values[3]),
        (lhs.values[1] * rhs.values[1]) + (lhs.values[1] * rhs.values[0]) + (lhs.values[1] * rhs.values[3]) - (lhs.values[1] * rhs.values[2]),
        (lhs.values[2] * rhs.values[2]) - (lhs.values[2] * rhs.values[3]) + (lhs.values[2] * rhs.values[0]) + (lhs.values[2] * rhs.values[1]),
        (lhs.values[3] * rhs.values[3]) + (lhs.values[3] * rhs.values[2]) - (lhs.values[3] * rhs.values[1]) + (lhs.values[3] * rhs.values[0])
    };

    return lhs;
}

inline Quaternion operator*(Quaternion lhs, const Quaternion& rhs){ lhs *= rhs;  return lhs; }

// ///////////////////
// Complicated Maths (vector return)
// ///////////////////
inline Quaternion Normalise(const Quaternion& lhs)
{
    // Use Vector4's version.
    return Quaternion{Normalise(Vector4{lhs.values}).values};
}

// commutative          : yes
// constant velocity    : no
// torque-minimal       : yes
// Computation          : cheap
inline Quaternion NLerp(const Quaternion& lhs, const Quaternion& rhs, float scale)
{
    // Use Vector4's version.
    return Quaternion{Normalise(Lerp(Vector4{lhs.values}, Vector4{rhs.values}, scale)).values};
}

// commutative          : no
// constant velocity    : yes
// torque-minimal       : yes
// Computation          : expensive (sin + acos)
inline Quaternion SLerp(const Quaternion& lhs, const Quaternion& rhs, float scale)
{
    // Adapted from bullet3
    auto vector4lhs = Vector4{lhs.values};
    auto vector4rhs = Vector4{rhs.values};
    auto magnitude  = std::sqrt(MagnitudeF(vector4lhs) * MagnitudeF(vector4rhs));

    // RAM: TODO: deal with quaternions that are too close. Ie magnitude !> 0

    float product = DotF(vector4lhs, vector4rhs) / magnitude;

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
            (lhs.values[0] * s0 + rhs.values[0] * s1) * d,
            (lhs.values[1] * s0 + rhs.values[1] * s1) * d,
            (lhs.values[2] * s0 + rhs.values[2] * s1) * d,
            (lhs.values[3] * s0 + rhs.values[3] * s1) * d
        };
    }

    return lhs;
}
