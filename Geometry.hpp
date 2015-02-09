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

struct alignas(16) Vec3
{
    float data[3];
};

struct alignas(16) Vec4
{
    float data[4];
};

struct alignas(16) Quaternion
{
    float data[4];
};

/// Normalised Vector, ie Length == 1.0
struct alignas(16) Vec3N
{
    float data[3];

    constexpr operator Vec3() const
    {
        return
        {
            data[0],
            data[1],
            data[2],
        };
    }
};

struct Vec3U
{
    float data[3];

    constexpr operator Vec3() const
    {
        return
        {
            data[0],
            data[1],
            data[2],
        };
    }
};

struct alignas(16) Vec4N
{
    float data[4];

    constexpr operator Vec4() const
    {
        return
        {
            data[0],
            data[1],
            data[2],
            data[3],
        };
    }
};

struct Plane
{
    Vec3U normal;
    float distance;
};

struct Radians
{
    float data;
};

struct alignas(16) Matrix3x3
{
    Vec3 data[3];
};

struct alignas(16) Matrix4x4
{
    Vec4 data[4];
};
