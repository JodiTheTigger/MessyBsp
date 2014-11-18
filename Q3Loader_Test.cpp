#include "Q3Loader.h"
#include "TraceTest.hpp"
#include <cstdlib>

int main(int, char**)
{
	TMapQ3	lMap;
	
	readMap("final.bsp", lMap);

    auto result = TimeBspCollision(lMap, 1000000);

    printf("Trace Took %ld microseconds\n", result.count());

//	FILE*	lFile = fopen("final_debug.txt", "w+");
//	debugInformations(lMap, lFile);
//	fclose(lFile);

	freeMap(lMap);
	
	return 0;
};
