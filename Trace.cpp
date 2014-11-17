#include "Q3Loader.h"

// Quake 3 defines it as 0.03125f, or 1/32.
static const float EPSILON = 0.03125f;

// RAM: Or does it? Taken from CM_TraceThroughBrush
// keep 1/8 unit away to keep the position valid before network snapping
// and to avoid various numeric issues
#define	SURFACE_CLIP_EPSILON	(0.125)

// Passing stuff into poitners
// http://en.wikipedia.org/wiki/X86_calling_conventions#Microsoft_x64_calling_convention
// http://msdn.microsoft.com/en-us/library/zthk2dkh.aspx
//  bits:           how:
//  32 - stdcall    all vai stack (right to left)
//  32 - fastcall   first two on left via register, rest via stack (right to left)
//  64              first 4 ints via reg, first 4 floats via XMM reg, rest via stack (right to left), returns via regs.

void rAssert(bool) {};

struct Vec3
{
    float data[3];
};

// Watch out, floating point is haaaaaard.
inline bool IsZero(const Vec3& vec)
{
    return
        vec.data[0] == 0.0f &&
        vec.data[1] == 0.0f &&
        vec.data[2] == 0.0f;
}

Vec3 inline Add(const Vec3& a, const Vec3& b)
{
    return
    {
        a.data[0] + b.data[0],
        a.data[1] + b.data[1],
        a.data[2] + b.data[2],
    };
}

Vec3 inline Negative(const Vec3& a)
{
    return
    {
        -a.data[0],
        -a.data[1],
        -a.data[2],
    };
}

float inline DotProduct(const Vec3& a, const float* b)
{
    return  (a.data[0] * b[0]) +
            (a.data[1] * b[1]) +
            (a.data[2] * b[2]);
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

inline float Clamp0To1(float toClamp)
{
    return toClamp > 0.0f ? (toClamp < 1.0f ? toClamp : 1.0f) : 0.0f;
}

struct Bounds
{
    float sphereRadius;
    Vec3* boxMin;
    Vec3* boxMax;
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
    /// 0 - 1.0f
    /// 0 == collision straight away, 1.0 means no collision at all.
    /// 0.5 means a collision 1/2 way thought the path, etc.
    float pathFraction;

    PathInfo info;
};

// Set sphereRadius to 0 for ray tests over sphere tests.
TraceResult CheckBrush(
        const TMapQ3& bsp,
        const TBrush& brush,
        const Vec3& start,
        const Vec3& end,
        const Bounds& bounds)
{
    float startFraction     = -1.0f;
    float endFraction       = 1.0f;
    bool startsOut          = false;
    bool endsOut            = false;

    for (int i = 0; i < brush.mNbBrushSides; ++i)
    {
        const auto& brushSide   = bsp.mBrushSides[brush.mBrushSide + i];
        const auto& plane       = bsp.mPlanes[brushSide.mPlaneIndex];

        Vec3 offset =
        {
            0.0f,
            0.0f,
            0.0f,
        };

        if (bounds.boxMin && bounds.boxMax)
        {
            offset =
            {
                plane.mNormal[0] < 0 ? bounds.boxMax->data[0] : bounds.boxMin->data[0],
                plane.mNormal[1] < 0 ? bounds.boxMax->data[1] : bounds.boxMin->data[1],
                plane.mNormal[2] < 0 ? bounds.boxMax->data[2] : bounds.boxMin->data[2],
            };
        }

        // Ray is just a Sphere with a sphereRadius of 0, and a box offset of 0.
        // A sphere has a box offset of 0 as well.
        // A box just has a sphereRadius, like the ray, of 0.
        float startDistance   = DotProduct(Add(start, offset), plane.mNormal) - (bounds.sphereRadius + plane.mDistance);
        float endDistance     = DotProduct(Add(end,   offset), plane.mNormal) - (bounds.sphereRadius + plane.mDistance);

        if (startDistance > 0)
        {
            startsOut = true;
        }

        if (endDistance > 0)
        {
            endsOut = true;
        }

        // make sure the trace isn't completely on one side of the brush
        if (startDistance > 0 && endDistance > 0)
        {
            // both are in front of the plane, its outside of this brush
            return
            {
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
            float fraction = (startDistance - EPSILON) / (startDistance - endDistance);
            if (fraction > startFraction)
            {
                startFraction = fraction;
            }
        }
        else
        {
            // line is leaving the brush
            float fraction = (startDistance + EPSILON) / (startDistance - endDistance);
            if (fraction < endFraction)
            {
                endFraction = fraction;
            }
        }
    }

    // Does it start (and end) inside a solid?
    if (!startsOut)
    {
        if (endsOut)
        {
            return
            {
                1.0f,
                PathInfo::StartsInsideEndsOutsideSolid,
            };
        }

        return
        {
            0.0f,
            PathInfo::InsideSolid
        };
    }

    // So, is it an actual collision?
    if (startFraction < endFraction)
    {
        return
        {
            Clamp0To1(startFraction),
            PathInfo::OutsideSolid
        };
    }

    // No collision
    return
    {
        1.0,
        PathInfo::OutsideSolid
    };
}

// RAM: TODO: Return the surface normal at the collision as well please.
TraceResult CheckNode(
    const TMapQ3& bsp,
    int nodeIndex,
    float startFraction,
    float endFraction,
    const Vec3& start,
    const Vec3& end,
    const Vec3& originalStart,
    const Vec3& originalEnd,
    TraceResult result,
    const Bounds& bounds)
{
    if (nodeIndex < 0)
    {
        // this is a leaf
        const auto& leaf = bsp.mLeaves[-(nodeIndex + 1)];

        for (int i = 0; i < leaf.mNbLeafBrushes; i++)
        {
            const auto& brush = bsp.mBrushes[bsp.mLeafBrushes[leaf.mLeafBrush + i].mBrushIndex];

            if  (
                    (brush.mNbBrushSides > 0) //&&
                    // RAM: What's the equilivant?! TODO!(bsp.shaders[brush->shaderIndex].contentFlags & 1)
                )
            {
                auto test = CheckBrush(
                            bsp,
                            brush,
                            originalStart,
                            originalEnd,
                            bounds);

                // RAM: TODO: Don't alter the fraction if the collision starts inside a solid.
                // RAM: TODO: Don't alter the collsion type if it's already != OutsideSolid.
                if (test.pathFraction < result.pathFraction)
                {
                    result = test;
                }
            }
        }

        // don't have to do anything else for leaves
        return result;
    }

    // this is a node
    const auto& node = bsp.mNodes[nodeIndex];
    const auto& plane = bsp.mPlanes[node.mPlane];

    float startDistance = DotProduct(start, plane.mNormal) - plane.mDistance;
    float endDistance   = DotProduct(end, plane.mNormal) - plane.mDistance;

    // Used for volumns, for now, assume ray == 0.
    float offset = 0;

    if (startDistance >= offset && endDistance >= offset)
    {
        // both points are in front of the plane
        // so check the front child
        return CheckNode(
            bsp,
            node.mChildren[0],
            startFraction,
            endFraction,
            start,
            end,
            originalStart,
            originalEnd,
            result,
            bounds);
    }

    if (startDistance < -offset && endDistance < -offset)
    {
        // both points are behind the plane
        // so check the back child
        return CheckNode(
            bsp,
            node.mChildren[1],
            startFraction,
            endFraction,
            start,
            end,
            originalStart,
            originalEnd,
            result,
            bounds);
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
        auto middleFraction = startFraction + (endFraction - startFraction) * fraction1;
        auto middle = Lerp(start, end, fraction1);

        // check the first side
        result = CheckNode(
            bsp,
            node.mChildren[side],
            startFraction,
            middleFraction,
            start,
            middle,
            originalStart,
            originalEnd,
            result,
            bounds);
    }

    // calculate the middle point for the second side
    {
        auto middleFraction = startFraction + (endFraction - startFraction) * fraction2;
        auto middle = Lerp(start, end, fraction2);

        // check the second side
        result = CheckNode(
            bsp,
            node.mChildren[!side],
            middleFraction,
            endFraction,
            middle,
            end,
            originalStart,
            originalEnd,
            result,
            bounds);
    }

    return result;
}

// Trace type if for bounds:
// Ray      : boxMin == boxMax == nullptr, sphereRadius == 0
// Sphere   : boxMin == boxMax == nullptr, sphereRadius > 0
// Box      : boxMin != nullptr, boxMax != nullptr, sphereRadius == 0
TraceResult Trace(
        const TMapQ3& bsp,
        const Vec3& start,
        const Vec3& end,
        const Bounds& bounds)
{
    rAssert(
                (!bounds.boxMin && !bounds.boxMax) ||
                (bounds.boxMin && bounds.boxMax && !bounds.sphereRadius)
           );

    return CheckNode(
                bsp,
                0,
                0.0f,
                1.0f,
                start,
                end,
                start,
                end,
                {
                    1.0f,
                    PathInfo::OutsideSolid
                },
                bounds);
}

#if PSEUDO_CODE
float outputFraction;
vector outputEnd;
boolean outputStartsOut;
boolean outputAllSolid;

#define TT_RAY 0
#define TT_SPHERE 1
#define TT_BOX 2

int traceType;
float sphereRadius;
vector traceMins;
vector traceMaxs;
vector traceExtents;

void TraceRay( vector inputStart, vector inputEnd )
{
    traceType = TT_RAY;
    Trace( inputStart, inputEnd );
}

void TraceSphere( vector inputStart, vector inputEnd, float inputRadius )
{
    traceType = TT_SPHERE;
    sphereRadius = inputRadius;
    Trace( inputStart, inputEnd );
}

void TraceBox( vector inputStart, vector inputEnd, vector inputMins, vector inputMaxs )
{
    if (inputMins[0] == 0 && inputMins[1] == 0 && inputMins[2] == 0 &&
        inputMaxs[0] == 0 && inputMaxs[1] == 0 && inputMaxs[2] == 0)
    {	// the user TraceBox, but this is actually a ray
        TraceRay( inputStart, inputEnd );
    }
    else
    {	// setup for a box
        traceType = TT_BOX;
        traceMins = inputMins;
        traceMaxs = inputMaxs;
        traceExtents[0] = -traceMins[0] > traceMaxs[0] ? -traceMins[0] : traceMaxs[0];
        traceExtents[1] = -traceMins[1] > traceMaxs[1] ? -traceMins[1] : traceMaxs[1];
        traceExtents[2] = -traceMins[2] > traceMaxs[2] ? -traceMins[2] : traceMaxs[2];
        Trace( inputStart, inputEnd );
    }
}

void Trace( vector inputStart, vector inputEnd )
{
    outputStartsOut = true;
    outputAllSolid = false;
    outputFraction = 1.0f;

    // walk through the BSP tree
    CheckNode( 0, 0.0f, 1.0f, inputStart, inputEnd );

    if (outputFraction == 1.0f)
    {	// nothing blocked the trace
        outputEnd = inputEnd;
    }
    else
    {	// collided with something
        for (i = 0; i < 3; i++)
        {
            outputEnd[i] = inputStart[i] + outputFraction * (inputEnd[i] - inputStart[i]);
        }
    }
}

void CheckNode( int nodeIndex, float startFraction, float endFraction, vector start, vector end )
{
    if (nodeIndex < 0)
    {	// this is a leaf
        auto* leaf = &BSP.leaves[-(nodeIndex + 1)];
        for (i = 0; i < leaf->numLeafBrushes; i++)
        {
            auto* brush = &BSP.brushes[BSP.leafBrushes[leaf->firstLeafBrush + i]];
            if (brush->numSides > 0 &&
                (BSP.shaders[brush->shaderIndex].contentFlags & 1))
            {
                CheckBrush( brush );
            }
        }

        // don't have to do anything else for leaves
        return;
    }

    // this is a node

    auto* node = &BSP.nodes[nodeIndex];
    auto* plane = &BSP.planes[node->planeIndex];

    float startDistance, endDistance, offset;
    startDistance = DotProduct( start, plane->normal ) - plane->distance;
    endDistance = DotProduct( end, plane->normal ) - plane->distance;

    if (traceType == TT_RAY)
    {
        offset = 0;
    }
    else if (traceType == TT_SPHERE)
    {
        offset = sphereRadius;
    }
    else if (traceType == TT_BOX)
    {
        // this is just a dot product, but we want the absolute values
        offset = (float)(fabs( traceExtents[0] * plane->normal[0] ) +
                         fabs( traceExtents[1] * plane->normal[1] ) +
                         fabs( traceExtents[2] * plane->normal[2] ) );
    }

    if (startDistance >= offset && endDistance >= offset)
    {	// both points are in front of the plane
        // so check the front child
        CheckNode( node->children[0], startFraction, endFraction, start, end );
    }
    else if (startDistance < -offset && endDistance < -offset)
    {	// both points are behind the plane
        // so check the back child
        CheckNode( node->children[1], startFraction, endFraction, start, end );
    }
    else
    {	// the line spans the splitting plane
        int side;
        float fraction1, fraction2, middleFraction;
        vector middle;

        // split the segment into two
        if (startDistance < endDistance)
        {
            side = 1; // back
            float inverseDistance = 1.0f / (startDistance - endDistance);
            fraction1 = (startDistance - offset + EPSILON) * inverseDistance;
            fraction2 = (startDistance + offset + EPSILON) * inverseDistance;
        }
        else if (endDistance < startDistance)
        {
            side = 0; // front
            float inverseDistance = 1.0f / (startDistance - endDistance);
            fraction1 = (startDistance + offset + EPSILON) * inverseDistance;
            fraction2 = (startDistance - offset - EPSILON) * inverseDistance;
        }
        else
        {
            side = 0; // front
            fraction1 = 1.0f;
            fraction2 = 0.0f;
        }

        // make sure the numbers are valid
        if (fraction1 < 0.0f) fraction1 = 0.0f;
        else if (fraction1 > 1.0f) fraction1 = 1.0f;
        if (fraction2 < 0.0f) fraction2 = 0.0f;
        else if (fraction2 > 1.0f) fraction2 = 1.0f;

        // calculate the middle point for the first side
        middleFraction = startFraction + (endFraction - startFraction) * fraction1;
        for (i = 0; i < 3; i++)
            middle[i] = start[i] + fraction1 * (end[i] - start[i]);

        // check the first side
        CheckNode( node->children[side], startFraction, middleFraction, start, middle );

        // calculate the middle point for the second side
        middleFraction = startFraction + (endFraction - startFraction) * fraction2;
        for (i = 0; i < 3; i++)
            middle[i] = start[i] + fraction2 * (end[i] - start[i]);

        // check the second side
        CheckNode( node->children[!side], middleFraction, endFraction, middle, end );
    }
}

void CheckBrush( auto* brush )
{
    float startFraction = -1.0f;
    float endFraction = 1.0f;
    boolean startsOut = false;
    boolean endsOut = false;

    for (int i = 0; i < brush->numSides; i++)
    {
        auto* brushSide = &BSP.brushSides[brush->firstSide + i];
        auto* plane = &BSP.planes[brushSide->planeIndex];

        float startDistance, endDistance;
        if (traceType == TT_RAY)
        {
            startDistance = DotProduct( inputStart, plane->normal ) - plane->distance;
            endDistance = DotProduct( inputEnd, plane->normal ) - plane->distance;
        }
        else if (traceType == TT_SPHERE)
        {
            startDistance = DotProduct( inputStart, plane->normal ) - (plane->distance + sphereRadius);
            endDistance = DotProduct( inputEnd, plane->normal ) - (plane->distance + sphereRadius);
        }
        else if (traceType == TT_BOX)
        {
            vector offset;
            for (int j = 0; j < 3; j++)
            {
                if (plane->normal[j] < 0)
                    offset[j] = traceMaxs[j];
                else
                    offset[j] = traceMins[j];
            }

            startDistance = (inputStart[0] + offset[0]) * plane->normal[0] +
                            (inputStart[1] + offset[1]) * plane->normal[1] +
                            (inputStart[2] + offset[2]) * plane->normal[2] - plane->distance;
            endDistance = (inputEnd[0] + offset[0]) * plane->normal[0] +
                          (inputEnd[1] + offset[1]) * plane->normal[1] +
                          (inputEnd[2] + offset[2]) * plane->normal[2] - plane->distance;
        }

        if (startDistance > 0)
            startsOut = true;
        if (endDistance > 0)
            endsOut = true;

        // make sure the trace isn't completely on one side of the brush
        if (startDistance > 0 && endDistance > 0)
        {   // both are in front of the plane, its outside of this brush
            return;
        }
        if (startDistance <= 0 && endDistance <= 0)
        {   // both are behind this plane, it will get clipped by another one
            continue;
        }

        if (startDistance > endDistance)
        {   // line is entering into the brush
            float fraction = (startDistance - EPSILON) / (startDistance - endDistance);
            if (fraction > startFraction)
                startFraction = fraction;
        }
        else
        {   // line is leaving the brush
            float fraction = (startDistance + EPSILON) / (startDistance - endDistance);
            if (fraction < endFraction)
                endFraction = fraction;
        }
    }

    if (startsOut == false)
    {
        outputStartOut = false;
        if (endsOut == false)
            outputAllSolid = true;
        return;
    }

    if (startFraction < endFraction)
    {
        if (startFraction > -1 && startFraction < outputFraction)
        {
            if (startFraction < 0)
                startFraction = 0;
            outputFraction = startFraction;
        }
    }
}
#endif
