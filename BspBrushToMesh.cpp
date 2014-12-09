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

#include "BspBrushToMesh.hpp"
#include "Bsp.hpp"

namespace Bsp {

std::vector<Mesh> GetBrushMeshes(CollisionBsp &bsp)
{
    std::vector<Mesh> result;

    // For each brush that's solid, get all the plane equations
    // and then get all the intersection points between all the planes.
    // Once you have those, get the convex hull for those points, then
    // turn it into a triangle mesh. Return.

    for (const auto& leaf : bsp.leaves)
    {
        const auto* leafBrushes = &bsp.leafBrushes[leaf.firstLeafBrushIndex];

        for (auto i = 0; i < leaf.leafBrushCount; ++i)
        {
            auto index = leafBrushes[i].brushIndex;

            const auto& brush = bsp.brushes[index].brush;

            // RAM: TODO: setting textureindex to -1 to prevent doing
            // brush multiple times, figure out how to do that here.
            if (brush.textureIndex < 0)
            {
                continue;
            }

            if (!(bsp.textures[brush.textureIndex].contentFlags & 1))
            {
                continue;
            }

            // RAM: TODO: Set brush as done.

            std::vector<Plane> planes;
            const auto* brushSides = &bsp.brushSides[brush.firstBrushSideIndex];

            for (auto j = 0; j < brush.sideCount; ++j)
            {
                const auto& brushSide = brushSides[j];

                auto plane = bsp.planes[brushSide.planeIndex];
                plane.distance = -plane.distance;

                planes.push_back(plane);
            }

            if (!planes.empty())
            {
                // Get verticies from plane equations.

                // TODO: Move this function into it's own file.
                // plane equations to verticies function.
            }
        }
    }

    return result;
}

} // namespace
