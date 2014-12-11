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

#include <BspBrushToMesh.hpp>
#include <gtest/gtest.h>

namespace Bsp {

// Class definition!
class TestBspBrushToMesh : public ::testing::Test
{
public:
    virtual void SetUp()
    {
        // TODO.
    }

protected:
    Bsp::CollisionBsp bsp;
};

TEST_F(TestBspBrushToMesh, Todo)
{
}


} // namespace
