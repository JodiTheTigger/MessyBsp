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

// Note:
// This code was written based on BSP collision dection explination article
// written by Nathan Ostgard and too much time browsing the Quake3 source code.
//
// It's written under C++14 (mainly for the complete brace initilisation of
// return statements).
//
// http://openzone.googlecode.com/git-history/f73bb8dfe8e6a16c13d39aba1c8f6537ee263d07/doc/Quake3BSP.html

#include "Trace.hpp"
#include "Bsp.hpp"
#include "rAssert.hpp"
#include "VectorMaths3.hpp"

// for std::abs(float)
#include <cmath>


// /////////////////////
// Constants
// /////////////////////

// Taken from Quake3's CM_TraceThroughBrush:
// keep 1/8 unit away to keep the position valid before network snapping
// and to avoid various numeric issues
static const float EPSILON = 0.125f;

// /////////////////////
// Structs
// /////////////////////
struct TraceBounds
{
    Bounds  bounds;
    Vec3    aabbMin;
    Vec3    aabbMax;
};

// /////////////////////
// Helpers
// /////////////////////
inline float Clamp0To1(float toClamp)
{
    return toClamp > 0.0f ? (toClamp < 1.0f ? toClamp : 1.0f) : 0.0f;
}

bool inline AabbDontIntersect(
        const Vec3& mina,
        const Vec3& maxa,
        const Vec3& minb,
        const Vec3& maxb)
{
    return
        (mina.data[0] > (maxb.data[0] + EPSILON)) ||
        (mina.data[1] > (maxb.data[1] + EPSILON)) ||
        (mina.data[2] > (maxb.data[2] + EPSILON)) ||
        (maxa.data[0] < (minb.data[0] - EPSILON)) ||
        (maxa.data[1] < (minb.data[1] - EPSILON)) ||
        (maxa.data[2] < (minb.data[2] - EPSILON));

}

// /////////////////////
// Trace Functions
// /////////////////////
TraceResult CheckBrush(
        const Bsp::CollisionBsp& bsp,
        const Bsp::Brush& brush,
        const Bounds& bounds,
        const TraceResult& currentResult)
{
    float startFraction         = -1.0f;
    float endFraction           = 1.0f;
    bool startsOut              = false;
    bool endsOut                = false;
    const Plane* collisionPlane = nullptr;

    // NOTE: In Q3 CM_TestBoundingBoxInCapsule
    // Seems to skip the first 6 sides of a brush
    // due to some sort of AABB thing. Find out why
    // they could do that.
    for (int i = 0; i < brush.sideCount; ++i)
    {
        const auto& brushSide   = bsp.brushSides[brush.firstBrushSideIndex + i];
        const auto& plane       = bsp.planes[brushSide.planeIndex];

        Vec3 offset =
        {
            plane.normal.data[0] < 0 ? bounds.boxMax.data[0] : bounds.boxMin.data[0],
            plane.normal.data[1] < 0 ? bounds.boxMax.data[1] : bounds.boxMin.data[1],
            plane.normal.data[2] < 0 ? bounds.boxMax.data[2] : bounds.boxMin.data[2],
        };

        // Ray is just a Sphere with a sphereRadius of 0, and a box offset of 0.
        // A sphere has a box offset of 0 as well.
        // A box just has a sphereRadius, like the ray, of 0.
        float startDistance =
                DotF(bounds.start + offset, plane.normal) -
                (bounds.sphereRadius + plane.distance);

        float endDistance =
                DotF(bounds.end + offset, plane.normal) -
                (bounds.sphereRadius + plane.distance);

        if (startDistance > 0)
        {
            startsOut = true;
        }

        if (endDistance > 0)
        {
            endsOut = true;
        }

        // make sure the trace isn't completely on one side of the brush
        // NOTE: Q3 does an episilone compare here. Dunno if we need to.
        // if (d1 > 0 && ( d2 >= SURFACE_CLIP_EPSILON || d2 >= d1 )  )
        if (startDistance > 0 && endDistance > 0)
        {
            // both are in front of the plane, its outside of this brush
            return
            {
                nullptr,
                1.0f,
                PathInfo::OutsideSolid
            };
        }

        if (startDistance <= 0 && endDistance <= 0)
        {
            // both are behind this plane, it will get clipped by another one
            continue;
        }

        if (startDistance > endDistance)
        {
            // line is entering into the brush
            float fraction =
                (startDistance - EPSILON) / (startDistance - endDistance);

            if (fraction > startFraction)
            {
                startFraction = fraction;
                collisionPlane = &plane;
            }
        }
        else
        {
            // line is leaving the brush
            float fraction =
                (startDistance + EPSILON) / (startDistance - endDistance);

            if (fraction < endFraction)
            {
                endFraction = fraction;
            }
        }
    }

    if (startsOut == false)
    {
        return
        {
            currentResult.collisionPlane,
            currentResult.pathFraction,
            endsOut ?
                PathInfo::StartsInsideEndsOutsideSolid :
                PathInfo::InsideSolid
        };
    }

    if (startFraction < endFraction)
    {
        if (startFraction > -1 && startFraction < currentResult.pathFraction)
        {
            return
            {
                collisionPlane,
                Clamp0To1(startFraction),
                PathInfo::OutsideSolid
            };
        }
    }

    // No collision. Do nothing.
    return currentResult;
}

TraceResult CheckNode(
    int nodeIndex,
    float startFraction,
    float endFraction,

    const Vec3& start,
    const Vec3& end,
    const Vec3& extents,
    const TraceBounds& boundsAabb,

    TraceResult result,
    const Bsp::CollisionBsp& bsp)
{
    if (result.pathFraction <= startFraction)
    {
        // already hit something nearer
        return result;
    }

    if (nodeIndex < 0)
    {
        // this is a leaf
        const auto& leaf = bsp.leaves[-(nodeIndex + 1)];

        for (int i = 0; i < leaf.leafBrushCount; i++)
        {
            const auto& brush =
                    bsp.brushes[bsp.leafBrushes[leaf.firstLeafBrushIndex + i].brushIndex];

            // Don't even bother if there are no brush sides.
            if (brush.brush.sideCount <= 0)
            {
                continue;
            }

            // Only test solid brushes
            // 1 == CONTENTS_SOLID
            if (!(bsp.textures[brush.brush.textureIndex].contentFlags & 1))
            {
                continue;
            }

            // Early exit if the AABB doesn't collide.
            if (AabbDontIntersect(
                        boundsAabb.aabbMin,
                        boundsAabb.aabbMax,
                        brush.aabbMin,
                        brush.aabbMax))
            {
                continue;
            }

            result = CheckBrush(bsp, brush.brush, boundsAabb.bounds, result);
        }

        // don't have to do anything else for leaves
        return result;
    }

    // this is a node
    const auto& node = bsp.nodes[nodeIndex];
    const auto& plane = bsp.planes[node.planeIndex];

    float startDistance = DotF(start, plane.normal) - plane.distance;
    float endDistance   = DotF(end, plane.normal) - plane.distance;

    // Offset used for non-ray tests.
    const auto& bounds = boundsAabb.bounds;
    float offset = bounds.sphereRadius;

    // extents are zero for ray or sphere tests.
    offset +=
        std::abs(extents.data[0] * plane.normal.data[0]) +
        std::abs(extents.data[1] * plane.normal.data[1]) +
        std::abs(extents.data[2] * plane.normal.data[2]);

    if (startDistance >= offset && endDistance >= offset)
    {
        // both points are in front of the plane
        // so check the front child
        return CheckNode(
            node.childIndex[0],
            startFraction,
            endFraction,
            start,
            end,
            extents,
            boundsAabb,
            result,
            bsp);
    }

    if (startDistance < -offset && endDistance < -offset)
    {
        // both points are behind the plane
        // so check the back child
        return CheckNode(
            node.childIndex[1],
            startFraction,
            endFraction,
            start,
            end,
            extents,
            boundsAabb,
            result,
            bsp);
    }

    // the line spans the splitting plane
    // Default values assume startDistance == endDistance.
    int side = 0;
    float fraction1 = 1.0f;
    float fraction2 = 0.0f;

    // split the segment into two
    if (startDistance < endDistance)
    {
        // back
        side = 1;
        float inverseDistance = 1.0f / (startDistance - endDistance);
        fraction1 = (startDistance - offset + EPSILON) * inverseDistance;
        fraction2 = (startDistance + offset + EPSILON) * inverseDistance;
    }

    if (endDistance < startDistance)
    {
        // front
        float inverseDistance = 1.0f / (startDistance - endDistance);
        fraction1 = (startDistance + offset + EPSILON) * inverseDistance;
        fraction2 = (startDistance - offset - EPSILON) * inverseDistance;
    }

    // make sure the numbers are valid
    fraction1 = Clamp0To1(fraction1);
    fraction2 = Clamp0To1(fraction2);

    // calculate the middle point for the first side
    {
        auto middleFraction =
                startFraction + (endFraction - startFraction) * fraction1;

        auto middle = Lerp(start, end, fraction1);

        // check the first side
        result = CheckNode(
            node.childIndex[side],
            startFraction,
            middleFraction,
            start,
            middle,
            extents,
            boundsAabb,
            result,
            bsp);
    }

    // calculate the middle point for the second side
    {
        auto middleFraction =
                startFraction + (endFraction - startFraction) * fraction2;

        auto middle = Lerp(start, end, fraction2);

        // check the second side
        result = CheckNode(
            node.childIndex[!side],
            middleFraction,
            endFraction,
            middle,
            end,
            extents,
            boundsAabb,
            result,
            bsp);
    }

    return result;
}

// /////////////////////
// Trace
// /////////////////////
TraceResult Trace(
        const Bsp::CollisionBsp &bsp,
        const Bounds &bounds)
{
    // TODO: Deal with point tests (ray with length of 0).

    // Find the maximum distance per axis from the bounds.
    Vec3 extents =
    {
        std::abs(bounds.boxMin.data[0]) > std::abs(bounds.boxMax.data[0]) ?
        std::abs(bounds.boxMin.data[0]) :
        std::abs(bounds.boxMax.data[0]),

        std::abs(bounds.boxMin.data[1]) > std::abs(bounds.boxMax.data[1]) ?
        std::abs(bounds.boxMin.data[1]) :
        std::abs(bounds.boxMax.data[1]),

        std::abs(bounds.boxMin.data[2]) > std::abs(bounds.boxMax.data[2]) ?
        std::abs(bounds.boxMin.data[2]) :
        std::abs(bounds.boxMax.data[2]),
    };    

    // Create an Axis Aligned Bounding Box (AABB)
    // along the path of the trace, taking into
    // consideration the sphere radius and the
    // bounds extents.
    auto aabbMin = Min(bounds.start, bounds.end);
    aabbMin = aabbMin + -bounds.sphereRadius;
    aabbMin = aabbMin - extents;

    auto aabbMax = Max(bounds.start, bounds.end);
    aabbMax = aabbMax + bounds.sphereRadius;
    aabbMax = aabbMax + extents;

    return CheckNode(
                0,
                0.0f,
                1.0f,
                bounds.start,
                bounds.end,
                extents,
                {
                    bounds,
                    aabbMin,
                    aabbMax,
                },
                {
                    nullptr,
                    1.0f,
                    PathInfo::OutsideSolid
                },
                bsp);
}
