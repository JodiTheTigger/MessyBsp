#include "TraceTest.hpp"
#include <cstdlib>

// RAM: Testing.
#include "Bsp.hpp"

int main(int, char**)
{
    Bsp::CollisionBsp bsp;

    Bsp::GetCollisionBsp("final.bsp", bsp);

    auto result = TimeBspCollision(bsp, 1000000);

    printf("Trace Took %ld microseconds\n", result.count());
	
	return 0;
};
