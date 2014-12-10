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

#include <type_traits>

// /////////////////////
// Stucts and Enums
// /////////////////////
struct alignas(16) Vec3
{
    float data[3];
};

static_assert(std::is_pod<Vec3>::value, "Vec3 is not a POD");
static_assert(std::is_standard_layout<Vec3>::value, "Vec3 doesn't ahve standard layout");
static_assert(std::is_trivial<Vec3>::value, "Vec3 isn't trivial");

// /////////////////////
// Helpers
// /////////////////////
Vec3 inline Add(const Vec3& a, const Vec3& b)
{
    return
    {
        a.data[0] + b.data[0],
        a.data[1] + b.data[1],
        a.data[2] + b.data[2],
    };
}

Vec3 inline Add(const Vec3& a, float b)
{
    return
    {
        a.data[0] + b,
        a.data[1] + b,
        a.data[2] + b,
    };
}

Vec3 inline Subtract(const Vec3& a, const Vec3& b)
{
    return
    {
        a.data[0] - b.data[0],
        a.data[1] - b.data[1],
        a.data[2] - b.data[2],
    };
}

Vec3 inline Multiply(const Vec3& a, float b)
{
    return
    {
        a.data[0] * b,
        a.data[1] * b,
        a.data[2] * b,
    };
}

Vec3 inline Mins(const Vec3&a, const Vec3&b)
{
    return
    {
        a.data[0] < b.data[0] ? a.data[0] : b.data[0],
        a.data[1] < b.data[1] ? a.data[1] : b.data[1],
        a.data[2] < b.data[2] ? a.data[2] : b.data[2],
    };
}

Vec3 inline Maxs(const Vec3&a, const Vec3&b)
{
    return
    {
        a.data[0] > b.data[0] ? a.data[0] : b.data[0],
        a.data[1] > b.data[1] ? a.data[1] : b.data[1],
        a.data[2] > b.data[2] ? a.data[2] : b.data[2],
    };
}

float inline DotProduct(const Vec3& a, const float(& b)[3])
{
    return  (a.data[0] * b[0]) +
            (a.data[1] * b[1]) +
            (a.data[2] * b[2]);
}

float inline DotProduct(const Vec3& a, const Vec3& b)
{
    return  (a.data[0] * b.data[0]) +
            (a.data[1] * b.data[1]) +
            (a.data[2] * b.data[2]);
}

float inline Square(const Vec3& a)
{
    return  (a.data[0] * b.data[0]) +
            (a.data[1] * b.data[1]) +
            (a.data[2] * b.data[2]);
}

Vec3 inline Lerp(const Vec3& start, const Vec3& end, float fraction)
{
    return
    {
        start.data[0] + fraction * (end.data[0] - start.data[0]),
        start.data[1] + fraction * (end.data[1] - start.data[1]),
        start.data[2] + fraction * (end.data[2] - start.data[2]),
    };
}
