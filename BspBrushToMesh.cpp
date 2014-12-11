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
#include "PlaneMaths.hpp"
#include "third-party/ConvexHull/hull.h"

namespace Bsp {

std::vector<Mesh> GetBrushMeshes(const CollisionBsp &bsp)
{
    std::vector<Mesh> result;
    std::vector<bool> flags(bsp.brushes.size());
    HullLibrary mrHull;

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

            // Avoid repeating brushes.
            if (flags[index])
            {
                continue;
            }
            flags[index] = true;

            const auto& brush = bsp.brushes[index].brush;

            if (!(bsp.textures[brush.textureIndex].contentFlags & 1))
            {
                continue;
            }

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
                // According to the spec, alignment can be ignored.
                auto verts =
                        VerticiesFromIntersectingPlanes(planes);

                unsigned stride = verts.size() > 1 ?
                        verts[1].data - verts[0].data :
                        sizeof(Vec3);

                HullResult  hResult;
                HullDesc    hullInfo
                (
                    QF_TRIANGLES,
                    verts.size(),
                    verts[0].data,
                    stride
                );

                mrHull.CreateConvexHull(hullInfo, hResult);

                // RAM: TODO: Asserts.
                // RAM: TODO: convert indicies to 16 or 32 bit.
                Mesh mesh;
                mesh.verticies = std::vector<float>(hResult.mOutputVertices, hResult.mOutputVertices + hResult.mNumOutputVertices);
                mesh.indicies = std::vector<uint16_t>(hResult.mIndices, hResult.mIndices + hResult.mNumIndices);

                result.push_back(mesh);

                mrHull.ReleaseResult(hResult);
            }
        }
    }

    return result;
}

} // namespace
