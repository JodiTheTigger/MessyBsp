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
#include "PlaneMaths.hpp"
#include "third-party/ConvexHull/hull.h"

namespace Bsp {

std::vector<float> BrushMeshesAsTriangleListWithNormals(
        const CollisionBsp &bsp,
        unsigned maxBrushCount)
{
    std::vector<float> result;
    std::vector<bool> flags(bsp.brushes.size());

    // For each brush that's solid, get all the plane equations
    // and then get all the intersection points between all the planes.
    // Once you have those, get the convex hull for those points, then
    // turn it into a triangle mesh. Return.
    unsigned count = 0;
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

            auto mesh = MeshFromBrush(bsp, brush);

            if (!mesh.empty())
            {
                for (const auto& v : mesh)
                {
                    result.push_back(v.data[0]);
                    result.push_back(v.data[1]);
                    result.push_back(v.data[2]);

                    count++;

                    if (count >= maxBrushCount)
                    {
                        break;
                    }
                }
            }
        }        

        if (count >= maxBrushCount)
        {
            break;
        }
    }

    return result;
}

std::vector<Vec3> MeshFromBrush(
        const Bsp::CollisionBsp& bsp,
        const Bsp::Brush& brush)
{
    std::vector<Vec3> mesh;
    std::vector<Plane> planes;

    // Q3 stores plane distance as the distance from the origin along the normal
    // But my maths assume it's D from Ax + By + Cz + D = 0, so I need to invert
    // the distance.
    //
    // Secondly, the Q3 bsp assumes the Z axis is "up", so swap z with y.
    auto convertD = [] (Plane p)
    {
        return Plane
        {
            Vec3U{
                p.normal.data[0],
                p.normal.data[2],
                p.normal.data[1]
            },
            -p.distance
        };
    };

    for (int i = 0; i < brush.sideCount; ++i)
    {
        const auto brushSide = bsp.brushSides[brush.firstBrushSideIndex + i];
        int planeIndex = brushSide.planeIndex;
        planes.push_back(convertD(bsp.planes[planeIndex]));
    }

    const auto verts = VerticiesFromIntersectingPlanes(planes);

    if (!verts.empty())
    {
        const auto& firstVert = verts.data()[0];

        HullDesc hullInfo;

        hullInfo.mFlags        = QF_TRIANGLES;
        hullInfo.mVcount       = verts.size();
        hullInfo.mVertexStride = sizeof(Vec3);
        hullInfo.mVertices     = reinterpret_cast<const float*>(firstVert.data);

        HullResult result;
        HullLibrary library;

        auto createResult = library.CreateConvexHull(hullInfo, result);

        if (createResult == QE_OK)
        {
            for (unsigned face = 0 ; face < result.mNumFaces; ++face)
            {
                auto index = result.mIndices[face * 3 + 0] * 3;

                Vec3 a =
                {
                    result.mOutputVertices[index + 0],
                    result.mOutputVertices[index + 1],
                    result.mOutputVertices[index + 2],
                };

                index = result.mIndices[face * 3 + 1] * 3;

                Vec3 b =
                {
                    result.mOutputVertices[index + 0],
                    result.mOutputVertices[index + 1],
                    result.mOutputVertices[index + 2],
                };

                index = result.mIndices[face * 3 + 2] * 3;

                Vec3 c =
                {
                    result.mOutputVertices[index + 0],
                    result.mOutputVertices[index + 1],
                    result.mOutputVertices[index + 2],
                };

                auto normal = Normalise(Cross(b-a, c-a));

                mesh.push_back(a);
                mesh.push_back(normal);

                mesh.push_back(b);
                mesh.push_back(normal);

                mesh.push_back(c);
                mesh.push_back(normal);
            }
        }

        library.ReleaseResult(result);
    }

    return mesh;
}

} // namespace
