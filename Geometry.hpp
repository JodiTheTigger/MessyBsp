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
    float data[3];
};

struct alignas(16) Quaternion
{
    float data[3];
};

/// Normalised Vector, ie Length == 1.0
struct alignas(16) Vec3N
{
    float data[3];
};

struct alignas(16) Vec4N
{
    float data[3];
};

struct alignas(16) Plane
{
    Vec3N normal;
    float distance;
};

struct Radians
{
    float data;
};

// Gotya if using brace initilisation.
// {x,y,z,a,b,c,d,e,f} will fail as each row
// is actually 4 along, not 3 (for alignment/speed reasons)
// so you'll need to remember to use a padding 0.0f, or use:
// {Vector3{}, Vector3{}, Vector{}}
// Or just use ToMatrix3x3(float, float, float,...).
// RAM: TODO: Fix.
struct alignas(16) Matrix3x3
{
    std::array<Vector3, 3> values;
};


