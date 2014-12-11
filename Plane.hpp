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

#include <vector>

// /////////////////////
// Helpers
// /////////////////////
bool inline PointBehindPlanes(
        const std::vector<Plane>& planes,
        const Vec3& point,
        float epislon = 0.0f)
{
    for (const auto& plane : planes)
    {
        auto distance =
                DotF(plane.direction, point) +
                plane.distance -
                epislon;

        if (distance > 0.0f)
        {
            return false;
        }
    }

    return true;
}
