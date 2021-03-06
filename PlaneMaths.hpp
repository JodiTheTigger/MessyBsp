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

#include <cmath>
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
                DotF(plane.normal, point) +
                plane.distance -
                epislon;

        if (distance > 0.0f)
        {
            return false;
        }
    }

    return true;
}

std::vector<Vec3> inline VerticiesFromIntersectingPlanes(
        const std::vector<Plane> planes)
{
    const auto planeCount = planes.size();
    std::vector<Vec3> result;

    // Ugh, brute force.
    for (unsigned i = 0; i < planeCount; ++i)
    {
        const auto& n1 = planes[i];

        for (unsigned j = i + 1; j < planeCount; ++j)
        {
            const auto& n2 = planes[j];

            for (unsigned k = j + 1; k < planeCount; ++k)
            {
                const auto& n3 = planes[k];

                // From http://geomalgorithms.com/a05-_intersect-1.html
                // P0 = -d1(n2 x n3) - d2(n3 x n1) - d3(n1 x n2)
                //      ----------------------------------------
                //                n1 . (n2 x n3)
                // Where . == dot product, and x = cross product.

                auto n2n3 = Cross(n2.normal, n3.normal);
                auto n3n1 = Cross(n3.normal, n1.normal);
                auto n1n2 = Cross(n1.normal, n2.normal);

                // Don't bother if the lengths are too small.
                if  (
                        ( SquareF(n2n3) < 0.0001f) ||
                        ( SquareF(n3n1) < 0.0001f) ||
                        ( SquareF(n1n2) < 0.0001f)
                    )
                {
                    continue;
                }

                auto quotient = DotF(n2n3, n1.normal);

                // There is only a 3 plane intersection if
                // (n1 . (n2 x n3)) != 0.
                if (std::abs(quotient) <= 0.000001f)
                {
                    continue;
                }

                // Make the quotent -ve to turn
                // all the -d1, -d2, -d3 to -(d1+d2+d3)
                quotient = -1.0f / quotient;

                auto d1n2n3 = n2n3 * n1.distance;
                auto d2n3n1 = n3n1 * n2.distance;
                auto d3n1n2 = n1n2 * n3.distance;

                auto point = d2n3n1 + d3n1n2;
                point = point + d1n2n3;
                point = point * quotient;

                if (!PointBehindPlanes(planes, point, 0.01f))
                {
                    continue;
                }

                result.push_back(point);
            }
        }
    }

    return result;
}
