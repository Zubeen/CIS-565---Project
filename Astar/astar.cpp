#include "astar.h"
#include "cutil_math.h"


// utilities and system includes
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <vector_types.h>
#include <cstdio>

#define X 16
#define Y 16
#define N 5 //number of a-star searches taking plane in parallel

extern "C" void astar(int* map, int mapX, int mapY, int n);


int
main( int argc, char** argv) 
{
 	int map[X][Y];

	for(int i=0;i<X;i++) //randomly populating the map
	{
		for(int j=0;j<Y;j++)
		{
			if(rand()%10==0) // 1 in 10 blocks on the amp will be an obstacle
			{
				map[i][j]=1;
			}
			else
			{
				map[i][j]=0;
			}
		}
	}

	astar(&map[0][0],X,Y,N);

	return 0;
}
