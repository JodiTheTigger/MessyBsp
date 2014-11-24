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

// Information taken from:
// http://www.mralligator.com/q3/
// http://www.flipcode.com/archives/Simple_Quake3_BSP_Loader.shtml

#pragma once

#include <vector>
#include <cstdint>
#include <string>

namespace Bsp {

enum Lumps
{
    Entities = 0,
    Textures,
    Planes,
    Nodes,
    Leaves,
    LeafFaces,
    LeafBrushes,
    Models,
    Brushes,
    BrushSides,
    Vertexes,
    Meshverts,
    Effects,
    Faces,
    Lightmaps,
    Lightvols,
    Visdata,

    Count,
};

const unsigned cQuake3BspVersion = 0x2e;

struct Lump
{
    int32_t offsetInBytesFromStartOfFile;

    /// Always a multiple of 4.
    int32_t byteCount;
};

struct Header
{
    // Always "IBSP"
    char magicString[4];

    // 0x2e for the BSP files distributed with Quake 3.
    int32_t version;
    Lump lumps[Lumps::Count];
};

struct Texture
{
    char    name[64];
    int32_t surfaceFlags;
    int32_t contentFlags;
};

/// Note that planes are paired. The pair of planes with indices i and i ^ 1
/// are coincident planes with opposing normals.
struct alignas(16) Plane
{
    float normal[3];

    /// Distance from origin to plane along normal.
    float distance;
};

struct Node
{
    int32_t planeIndex;

    /// Children indices. Negative numbers are leaf indices: -(leaf+1).
    int32_t childIndex[2];

    int32_t boundsMin[3];
    int32_t boundsMax[3];
};

struct Leaf
{
    int32_t visdataClusterIndex;
    int32_t areaPortal;
    int32_t boundsMin[3];
    int32_t boundsMax[3];
    int32_t firstLeafFaceIndex;
    int32_t leafFaceCount;
    int32_t firstLeafBrushIndex;
    int32_t leafBrushCount;
};

struct LeafBrush
{
    int32_t brushIndex;
};

struct Brush
{
    int32_t firstBrushSideIndex;
    int32_t sideCount;
    int32_t textureIndex;
};

struct BrushAabb
{
    Brush brush;
    float aabbMin[3];
    float aabbMax[3];
};

struct BrushSide
{
    int32_t planeIndex;
    int32_t textureIndex;
};

struct CollisionBsp
{
    Header                  header;
    std::vector<Texture>    textures;
    std::vector<Plane>      planes;
    std::vector<Node>       nodes;
    std::vector<Leaf>       leaves;
    std::vector<LeafBrush>  leafBrushes;
    std::vector<BrushAabb>  brushes;
    std::vector<BrushSide>  brushSides;
};

void GetCollisionBsp(const std::string& filePath, CollisionBsp& bsp);

} // namespace
