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

#include "Bsp.hpp"

#include <cstdlib>

namespace Bsp {

/// This is bare bones, with no nice error reporting
/// or proper exception safe cleanup. C++ makes it too ugly.
void GetCollisionBsp(
        const std::string &filePath,
        CollisionBsp &bsp)
{
    auto fileHandle = fopen(filePath.c_str(), "r+b");

    // Check if the file exists.
    if (!fileHandle)
    {
        return;
    }

    // Using do once + continue in leiu of scoped_exit
    do
    {
        // Read the header.
        if (!fread(&bsp.header, 1, sizeof(Header), fileHandle))
        {
            continue;
        }

        // basic checks
        if  (
                (bsp.header.version != cQuake3BspVersion) ||
                (bsp.header.magicString[0] != 'I') ||
                (bsp.header.magicString[1] != 'B') ||
                (bsp.header.magicString[2] != 'S') ||
                (bsp.header.magicString[3] != 'P')
            )
        {
            continue;
        }

        // Count
        const auto& lumps = bsp.header.lumps;
        size_t Counts[Lumps::Count] =
        {
            0,
            lumps[Textures].byteCount       / sizeof(Texture),
            lumps[Planes].byteCount         / sizeof(Plane),
            lumps[Nodes].byteCount          / sizeof(Node),
            lumps[Leaves].byteCount         / sizeof(Leaf),
            0,
            lumps[LeafBrushes].byteCount    / sizeof(LeafBrush),
            0,
            lumps[Brushes].byteCount        / sizeof(Brush),
            lumps[BrushSides].byteCount     / sizeof(BrushSide),
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        };

        // Reserve
        bsp.textures.reserve(Counts[Textures]);
        bsp.planes.reserve(Counts[Planes]);
        bsp.nodes.reserve(Counts[Nodes]);
        bsp.leaves.reserve(Counts[Leaves]);
        bsp.leafBrushes.reserve(Counts[LeafBrushes]);
        bsp.brushes.reserve(Counts[Brushes]);
        bsp.brushSides.reserve(Counts[BrushSides]);

        // My first generic lambda. DRY FTW!
        auto readTypes = [&] (Lumps lumpEnum, auto& vector, size_t typeSize)
        {
            fseek(
                fileHandle,
                bsp.header.lumps[lumpEnum].offsetInBytesFromStartOfFile,
                SEEK_SET);

            for (unsigned i = 0; i < Counts[lumpEnum]; ++i)
            {
                vector.push_back({});

                if (!fread(&vector.back(), 1, typeSize, fileHandle))
                {
                    return false;
                }

            }

            return true;
        };

        // Grrr, not DRY enough (how to not repeat the enum and type?)
        if (!readTypes(Textures,    bsp.textures,       sizeof(Texture)))  continue;
        if (!readTypes(Planes,      bsp.planes,         sizeof(Plane)))     continue;
        if (!readTypes(Nodes,       bsp.nodes,          sizeof(Node)))      continue;
        if (!readTypes(Leaves,      bsp.leaves,         sizeof(Leaf)))      continue;
        if (!readTypes(LeafBrushes, bsp.leafBrushes,    sizeof(LeafBrush))) continue;
        if (!readTypes(Brushes,     bsp.brushes,        sizeof(Brush)))     continue;
        if (!readTypes(BrushSides,  bsp.brushSides,     sizeof(BrushSide))) continue;

    } while(false);

    fclose(fileHandle);
}

} // namespace
