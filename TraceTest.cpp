/*
    MesyBsp. BSP collision and loading example code.
    Copyright (C) 2014 Richard Maxwell <jodi.the.tigger@gmail.com>
    This file is part of Game-in-a-box
    Game-in-a-box is free software: you can redistribute it and/or modify
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

#include "TraceTest.hpp"
#include "Trace.hpp"

#include <iostream>
#include <vector>
#include <random>

std::chrono::microseconds TestBspCollision(
        const TMapQ3& bsp,
        unsigned collisionsToTest)
{
    // First build array of points to test.
    std::vector<Bounds> testArray;

    testArray.reserve(collisionsToTest);

    // build the array.

    // Test the array.
    auto start = std::chrono::high_resolution_clock::now();
    for(const auto& bounds : testArray)
    {
        // Ignore the result.
        Trace(bsp, bounds);
    }
    auto end = std::chrono::high_resolution_clock::now();

    return std::chrono::duration_cast<std::chrono::microseconds>(end - start);
}
