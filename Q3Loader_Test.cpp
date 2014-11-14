#include "Q3Loader.h"

#include <cstdlib>

int main(int pArgc, char** pArgv)
{
	TMapQ3	lMap;
	
	readMap("final.bsp", lMap);

	FILE*	lFile = fopen("final_debug.txt", "w+");	
	debugInformations(lMap, lFile);
	fclose(lFile);

	freeMap(lMap);
	
	return 0;
};
