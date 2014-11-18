#include "Q3Loader.h"
#include "TraceTest.hpp"
#include <cstdlib>

int main(int, char**)
{
	TMapQ3	lMap;
	
	readMap("final.bsp", lMap);

    auto result = TimeBspCollision(lMap, 10000);

    printf("Trace Took %d microseconds\n", result);

//	FILE*	lFile = fopen("final_debug.txt", "w+");
//	debugInformations(lMap, lFile);
//	fclose(lFile);

	freeMap(lMap);
	
	return 0;
};
