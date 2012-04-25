
#ifndef _ASTAR_KERNEL_H_
#define _ASTAR_KERNEL_H_

#include "cutil_math.h"
#include <stdio.h>

#define BLOCK_SIZE 64

#define FOUND 1
#define NONEXISTANT 0


//Astarsearch<<<num_blocks,BLOCK_SIZE>>>(&map[0][0],mapX, mapY, startX, startY, goalX, goalY, path, isComplete); 

__global__ void Astarsearch(int* map, int mapX, int mapY, int *startX, int *startY, int *goalX, int *goalY, int *pathX, int *pathY, int *isComplete)
{
	int thid = blockIdx.x*blockDim.x + threadIdx.x;  

	int openList[mayX*mapY+2]; // 1D array holding ID# of open list items
	int whichList[mapX+1][mapY+1]; // record whether a cell is on open (0) or closed(1) list
	int openX[mapX*mapY+2], openY[mapX*mapY+2]; //1D arrays to store X and Y location of each node on the open list
	int parentX[mapX+1][mapY+1],parentY[mapX+1][mapY+1]; //2D arrays to store parents of each cell
	
	int Fcost[mapX*mapY+2]; // 1D array to store cost of cell on open list
	int Hcost[mapX*mapY+2]; // 1D array to store Heuristic cost of cell on open list
	int Gcost[mapX+1][mapY+1]; // 1D array to store cost of cell on open list

	int pathLength=0;
	

	int 0=0, parentXval=0, parentYval=0,
	a=0, b=0, m=0, u=0, v=0, temp=0, corner=0, numberOfOpenListItems=0,
	addedGCost=0, tempGcost = 0, path = 0,
	tempx, newOpenListItemID=0;

	if (startX == targetX && startY == targetY)
	{
		isComplete=0;
	}
	else if (map[targetX][targetY] == 1) //If target square is an obstacle, return that it's a nonexistent path.
	{
		isComplete=-1;
	}
	else
	{
		Gcost[startX][startY] = 0; //set starting square's G value to 0
		
		//Add the starting location to the open list of squares to be checked.
		numberOfOpenListItems = 1;
		openList[1] = 1;//assign it as the top (and currently only) item in the open list, which is maintained as a binary heap (explained below)
		openX[1] = startX[thid] ; openY[1] = startY[thid];

		//Until a path is found or deemed nonexistent.
		do
		{
		
			//If the open list is not empty, take the first cell off of the list. (This is the lowest F cost cell on the open list)
			
			if (numberOfOpenListItems != 0)
			{
				//Pop the first item off the open list.
				parentXval = openX[openList[1]];
				parentYval = openY[openList[1]]; //record cell coordinates of the item
				whichList[parentXval][parentYval] = 1;//add the item to the closed list

				//	Open List = Binary Heap: Delete this item from the open list, which is maintained as a binary heap. For more information on binary heaps, see:
				numberOfOpenListItems = numberOfOpenListItems - 1;//reduce number of open list items by 1	
		
				//	Delete the top item in binary heap and reorder the heap, with the lowest F cost item rising to the top.
				openList[1] = openList[numberOfOpenListItems+1];//move the last item in the heap up to slot #1
				v = 1;

				//	Re-make Heap
				do
				{
					u = v;		
					if (2*u+1 <= numberOfOpenListItems) //if both children exist
					{
	 					//Check if the F cost of the parent is greater than each child.
						//Select the lowest of the two children.
						if (Fcost[openList[u]] >= Fcost[openList[2*u]]) 
							v = 2*u;
						if (Fcost[openList[v]] >= Fcost[openList[2*u+1]]) 
							v = 2*u+1;		
					}
					else
					{
						if (2*u <= numberOfOpenListItems) //if only child #1 exists
						{
	 						//Check if the F cost of the parent is greater than child #1	
							if (Fcost[openList[u]] >= Fcost[openList[2*u]]) 
								v = 2*u;
						}
					}

					if (u != v) //if parent's F is > one of its children, swap them
					{
						temp = openList[u];
						openList[u] = openList[v];
						openList[v] = temp;			
					}
					else
						break; //otherwise, exit loop
				}
				while (1); // Remake Heap ends


				// Check the adjacent squares. Add these adjacent child squares to the open list for later consideration if appropriate
				
				for (b = parentYval-1; b <= parentYval+1; b++)
				{
					for (a = parentXval-1; a <= parentXval+1; a++)
					{

						//	If not off the map (do this first to avoid array out-of-bounds errors)
						if (a != -1 && b != -1 && a != mapWidth && b != mapHeight)
						{

							//	If not already on the closed list 			
							if (whichList[a][b] != 1)
							{ 
								//	If not a wall/obstacle square.
								if (map [a][b] != 1) 
								{ 
									//	Don't cut across corners
									corner = 0;	
									if (a == parentXval-1) 
									{
										if (b == parentYval-1)
										{
											if (map[parentXval-1][parentYval] == 1 || map[parentXval][parentYval-1] == 1) 
												corner = 1;
										}
										else if (b == parentYval+1)
										{	
											if (map[parentXval][parentYval+1] == 1 || map[parentXval-1][parentYval] == 1) 
											corner = 1; 
										}
									}
									else if (a == parentXval+1)
									{
										if (b == parentYval-1)
										{
											if (map[parentXval][parentYval-1] == 1 || map[parentXval+1][parentYval] == 1) 
												corner = 1;
										}
										else if (b == parentYval+1)
										{
											if (map[parentXval+1][parentYval] == 1 || map[parentXval][parentYval+1] == 1)
												corner = 1;
										}
									}	
									if (corner == 0)
									{
										//	If not already on the open list, add it to the open list.			
										if (whichList[a][b] != 0) 
										{	
											//Create a new open list item in the binary heap.
											newOpenListItemID = newOpenListItemID + 1; //each new item has a unique ID #
											m = numberOfOpenListItems+1;
											openList[m] = newOpenListItemID;//place the new open list item (actually, its ID#) at the bottom of the heap
											openX[newOpenListItemID] = a;
											openY[newOpenListItemID] = b;//record the x and y coordinates of the new item

											//Figure out its G cost
											if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
												addedGCost = 15;//cost of going to diagonal squares	
											else	
												addedGCost = 10;//cost of going to non-diagonal squares				
										
											Gcost[a][b] = Gcost[parentXval][parentYval] + addedGCost;

											//Figure out its H and F costs and parent
											Hcost[openList[m]] = 10*(abs(a - targetX) + abs(b - targetY));
											Fcost[openList[m]] = Gcost[a][b] + Hcost[openList[m]];
											parentX[a][b] = parentXval ; parentY[a][b] = parentYval;	

										//Move the new open list item to the proper place in the binary heap.
											while (m != 1) //While item hasn't bubbled to the top (m=1)	
											{
												//Check if child's F cost is < parent's F cost. If so, swap them.	
												if (Fcost[openList[m]] <= Fcost[openList[m/2]])
												{
													temp = openList[m/2];
													openList[m/2] = openList[m];
													openList[m] = temp;
													m = m/2;
												}
												else
													break;
											}
											numberOfOpenListItems = numberOfOpenListItems+1;//add one to the number of items in the heap

											//Change whichList to show that the new item is on the open list.
											whichList[a][b] = 0;
										}

										//  If adjacent cell is already on the open list, check to see if this 
										//	path to that cell from the starting location is a better one. 
										//	If so, change the parent of the cell and its G and F costs.	
										else //If whichList(a,b) = 0
										{
											//Figure out the G cost of this possible new path
											if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
												addedGCost = 15;//cost of going to diagonal tiles	
											else	
												addedGCost = 10;//cost of going to non-diagonal tiles				
									
											tempGcost = Gcost[parentXval][parentYval] + addedGCost;
	
											//If this path is shorter (G cost is lower) then change
											//the parent cell, G cost and F cost. 		
											if (tempGcost < Gcost[a][b]) //if G cost is less,
											{
												parentX[a][b] = parentXval; //change the square's parent
												parentY[a][b] = parentYval;
												Gcost[a][b] = tempGcost;//change the G cost			
											
												//Because changing the G cost also changes the F cost, if
												//the item is on the open list we need to change the item's
												//recorded F cost and its position on the open list to make
												//sure that we maintain a properly ordered open list.
												for (int x = 1; x <= numberOfOpenListItems; x++) //look for the item in the heap
												{
													if (openX[openList[x]] == a && openY[openList[x]] == b) //item found
													{
														Fcost[openList[x]] = Gcost[a][b] + Hcost[openList[x]];//change the F cost
														//See if changing the F score bubbles the item up from it's current location in the heap
														m = x;
														while (m != 1) //While item hasn't bubbled to the top (m=1)	
														{
															//Check if child is < parent. If so, swap them.	
															if (Fcost[openList[m]] < Fcost[openList[m/2]])
															{
																temp = openList[m/2];
																openList[m/2] = openList[m];
																openList[m] = temp;
																m = m/2;
															}
															else
																break;
														} 
														break; //exit for x = loop
													} //If openX(openList(x)) = a
												} //For x = 1 To numberOfOpenListItems
											}//If tempGcost < Gcost(a,b)
										}//else If whichList(a,b) = 0	
									}//If not cutting a corner
								}//If not a wall/obstacle square.
							}//If not already on the closed list 
						}//If not off the map
					}//for (a = parentXval-1; a <= parentXval+1; a++)
				}//for (b = parentYval-1; b <= parentYval+1; b++)
			}//if (numberOfOpenListItems != 0)
			else
			{
				isComplete[thid]=-1;
				break;	
			}  
		
		
			//If target is added to open list then path has been found.
			if (whichList[targetX][targetY] == 0)
			{
				path = FOUND;
				break;
			}
		}
		while (1);//Do until path is found or deemed nonexistent

		//Save the path if it exists.
		if (path == FOUND)
		{
			// Working backwards from the target to the starting location by checking
			//	each cell's parent, figure out the length of the path.
			int k=0
			pathX[thid][0] = targetX; pathY[thid][0] = targetY;
			do
			{
				k++;
				//Look up the parent of the current cell.	
				pathX[thid][k]  = parentX[pathX[thid][k-1]][pathY[thid][k-1]];		
				pathY[thid][k]  = parentY[pathX[thid][k-1]][pathY[thid][k-1]];		
		
				//Figure out the path length
				pathLength = pathLength + 1;
			}
			while (pathX[thid][k] != startX[thid] || pathY[thid][k] != startY[thid]);
		
			isComplete[thid]=pathLength;
		}
	}

}

extern "C"
void astar(int* map, int mapX, int mapY, int N)
{
	//The start and the goal for the parallel A* searches will be assigned by the "CPU" code
	int *startX;int *startY;
	cudaMalloc((void**)&startX, N*sizeof(int));cudaMalloc((void**)&startY, N*sizeof(int));

	int *goalX;int *goalY;
	cudaMalloc((void**)&goalX, N*sizeof(int));cudaMalloc((void**)&goalY, N*sizeof(int));

	int *pathX;
	cudaMalloc((void**)&path, N*sizeof(int));cudaMalloc((void**)&path, N*sizeof(int));
	int *pathY;
	cudaMalloc((void**)&pathY, N*sizeof(int));cudaMalloc((void**)&pathY, N*sizeof(int));

	int *isComplete;
	cudaMalloc((void**)&isComplete, N*sizeof(int));cudaMalloc((void**)&isComplete, N*sizeof(int));

	int num_blocks =0;
	num_blocks = N/BLOCK_SIZE;
	Astarsearch<<<num_blocks,BLOCK_SIZE>>>(&map[0][0],mapX, mapY, startX, startY, goalX, goalY, pathX, pathY isComplete); //do map, mapX, mapY have to be on device ???
	
}

#endif // #ifndef _ASTAR_KERNEL_H_
