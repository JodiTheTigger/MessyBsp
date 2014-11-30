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

#include "Vec3.hpp"

// /////////////////////
// Forward Declarations
// /////////////////////
namespace Bsp
{
    class CollisionBsp;
    class Plane;
}

struct Bounds
{
    // Path to test
    Vec3    start;
    Vec3    end;

    // Trace type:
    // Ray      : boxMin == boxMax == {0, 0, 0}, sphereRadius == 0
    // Sphere   : boxMin == boxMax == {0, 0, 0}, sphereRadius > 0
    // Box      : boxMin != {0, 0, 0}, boxMax != {0, 0, 0}, sphereRadius == 0
    float   sphereRadius;
    Vec3    boxMin;
    Vec3    boxMax;
};

enum class PathInfo
{
    OutsideSolid,
    StartsInsideEndsOutsideSolid,
    InsideSolid,

    Count
};

struct TraceResult
{
    const Bsp::Plane* collisionPlane;

    /// 0 - 1.0f
    /// 0 == collision straight away, 1.0 means no collision at all.
    /// 0.5 means a collision 1/2 way thought the path, etc.
    float pathFraction;

    PathInfo info;
};

// /////////////////////
// Trace
// /////////////////////
// NOTE: Haven't actually numerically verified this function
//       So you'll to assume it's wrong somehow. Guess I better
//       write some unit tests.
TraceResult Trace(
        const Bsp::CollisionBsp& bsp,
        const Bounds& bounds);
