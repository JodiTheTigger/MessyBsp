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

#include "TraceTest.hpp"
#include "Trace.hpp"

#include <iostream>
#include <vector>
#include <random>

std::chrono::microseconds TimeBspCollision(
        const Bsp::CollisionBsp& bsp,
        unsigned collisionsToTest)
{
    // First build array of points to test.
    std::vector<Bounds> testArray;

    testArray.reserve(collisionsToTest);

    // build the array.
    {
        unsigned seed = 1;

        // just range between -1000 to 1000.
        auto e = std::default_random_engine{seed};
        auto d = std::uniform_real_distribution<float>{-1000, 1000};

        for (unsigned i = 0; i < collisionsToTest; ++i)
        {
            Bounds bounds =
            {
                Vec3
                {
                    d(e),
                    d(e),
                    d(e)
                },

                Vec3
                {
                    d(e),
                    d(e),
                    d(e)
                },

                {0,0,0},
                {0,0,0},
				0.0f,
            };

            auto typeTest = d(e);

            if (typeTest > 333.0f)
            {
                // Just use a player size(ish) for the box bounds.
                bounds.boxMin =
                {
                    -20,
                    -90,
                    -20,
                };

                bounds.boxMax =
                {
                    20,
                    90,
                    20,
                };
            }

            if (typeTest < -333.0f)
            {
                // use a 10cm sphere.
                bounds.sphereRadius = 5;
            }

            testArray.push_back(bounds);
        }
    }

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
