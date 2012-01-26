/* 
 * THIS CLASS IS UNSTABLE
 * THIS CLASS SPECIFICALLY FOR THE MAZE SOLVING ROBOT!!!!! will be renamed Maze soon
 * Given input from the maze robot this will map the maze properly, TESTED
 * path planning TESTED works properly
 * issues: 		
 *				 isUnknown() only returns false (might stay that way)
				 walkable(Point) needs to be updated for the new representation
				 calcCost(Point) needs to be updated to take into account turning
 * 
 * The Maze class is here to represent the maze that an automatic robot
 * will operate on. The physical layout of the maze is a black surface
 * with white lines.
 * The white lines will form a maze of sqares with the possibility that 
 * there will be a line at 45 degrees connecting two sides of a square.
 * 
 * The maze class here will give a deterimination of what is on the maze
 * and provide an A* method to give a route from one maze Point to another.
 * The route will include diagonal moves if there is a connection
 *
 * The original code was written by James Lumsden, Yusuf Syaid, & Mike Hamer.
 * Tidied, converted into OO and continued by Todd Hurst & Zachary Oliver.
 * Started 10:23 23/12/2010
 * Last Updated 10:03 24/12/2010
 
 A centre is defined when every square in a maze is being represented by a 3x3 square and not 2x2.
 
 Understanding binary arithmatic statements:
 every point is defined by 1 byte, initialised as 1111 1111.
 As an example the statement:
 (assuming maze[x][y] = 1010 1111)
 maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gWESTWARDS));
            = (1010 1111 & 1111 0000)   + ((1010 1111 & 0000 1111)   - (1010 1111 & 0000 0011))
			= (1010 0000)               + (0000 1111 - 0000 0011)
			= 1010 1100 (set to gNORTH, gSOUTH and gEASTWARD)
Done this way because if maze[x][y] = 1010 1101:
 maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gWESTWARDS));
            = (1010 1101 & 1111 0000)   + ((1010 1101 & 0000 1111)   - (1010 1101 & 0000 0011))
			= (1010 0000)               + (0000 1101 - 0000 0001)
			= 1010 1100
			using (maze[x][y] & gWESTWARDS) stops us from accidentally removing a bit that is already moved!
& gSTRAIGHTS = only high 4 bits
& gDIAGONALS = only low 4 gits
& "any direction" = if "direction" bit is not set, don't include it in the calculation
Lastly assuming maze[x][y] = 1010 1100
  maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gWESTWARDS));
            = (1010 1100 & 1111 0000)   + ((1010 1100 & 0000 1111)   - (1010 1100 & 0000 0011))
			= (1010 0000)               + (0000 1100 - 0000 0000)
			= 1010 1100
 */

#include "Maze.h"

// Constructor 
Maze::Maze(unsigned short int newXLength, unsigned short int newYLength)
{
  xLength = newXLength;
  yLength = newYLength;
//Check requested range
// As it's unsigned it cannot be less than zero.
  if (xLength > MAZERANGE)
  {
    xLength = MAZERANGE;
  }
  if (yLength > MAZERANGE)
  {
    yLength = MAZERANGE;
  }
 
  for (unsigned short int x = 0; x < xLength; x++)
  {
    for (unsigned short int y = 0; y < yLength; y++)
    {
      maze[x][y] = gUNKNOWN;
    }
  }
}

/*
\|/\|/\|/      \|/\|/\|/
-N--N--N-      -N--N--N-
/|\/|\/|\      /|\/|\/|\
\|/\|/\|/      \|/   \|/
-N--C--N-  to  -N- X -N-
/|\/|\/|\      /|\   /|\
\|/\|/\|/      \|/\|/\|/
-N--N--N-      -N--N--N-
/|\/|\/|\      /|\/|\/|\
*/
void Maze::initAllCentresUnpassable()
{
	for(int ii = 1; ii < xLength; ii += 2)
	{
		for(int jj = 1; jj < yLength; jj += 2)
		{
			maze[ii][jj] = 0x00;
		}
	}
}

/*
\|/\|/\|/      \|/\|/\|/
-N--N--N-      -N--N--N-
/|\/|\/|\      /|\/ \/|\
\|/\|/\|/      \|/\|/\|/
-N--C--N-  to  -N -C- N-
/|\/|\/|\      /|\/|\/|\
\|/\|/\|/      \|/\ /\|/
-N--N--N-      -N--N--N-
/|\/|\/|\      /|\/|\/|\
*/
void Maze::initCutAllCardinalConnectionsToCentres()
{
  //Cut all cardinal (N,S,E,W) connections to centre nodes
  //Does 2 sets of for loops to account for mazes that aren't square 
  for(int ii = 0; ii < xLength; ii += 2)
  {
      for(int jj = 1; jj < yLength; jj += 2)
      {
        maze[ii][jj] -= (gEAST + gWEST);
      }
  }
  for(int ii = 0; ii < yLength; ii += 2)
  {
      for(int jj = 1; jj < xLength; jj += 2)
      {
		maze[jj][ii] -= (gNORTH + gSOUTH);
      }
  }
}

/*
\|/\|/\|/      
-N--N--N-       N--N--N
/|\/|\/|\       |\/|\/|
\|/\|/\|/       |/\|/\|
-N--C--N-  to   N--C--N
/|\/|\/|\       |\/|\/|
\|/\|/\|/       |/\|/\|
-N--N--N-       N--N--N
/|\/|\/|\       
*/
void Maze::initCutAllConnectionsOutOfRange()
{
	for(int ii = 0; ii < xLength; ii++)
	{
		maze[ii][0] = maze[ii][0] - ((maze[ii][0] & gNORTH) + (maze[ii][0] & gNORTHWARDS));
		maze[ii][yLength - 1] = maze[ii][yLength - 1] - ((maze[ii][yLength - 1] & gSOUTH) + (maze[ii][yLength - 1] & gSOUTHWARDS));	
	}
	for(int jj = 0; jj < yLength; jj++)
	{
		maze[0][jj] = maze[0][jj] - ((maze[0][jj] & gWEST) + (maze[0][jj] & gWESTWARDS));
		maze[xLength - 1][jj] = maze[xLength - 1][jj] - ((maze[xLength - 1][jj] & gEAST) + (maze[xLength - 1][jj] & gEASTWARDS));
	}
}

// Sets a node in the array.
//PASSABLE kills all bridge connections (diagonal connections) to and from the node
//if it is UNPASSABLE then we kill the node and all connections to it
void Maze::setTerrain(Point point, Terrain terrain)
{
  // Only set something if it's within our range.
  if (point.x < xLength && point.y < yLength && point.x >= 0 && point.y >= 0)
  {
    short int x = point.x;
    short int y = point.y;
//PASSABLE kills all bridge connections to and from the node
    if(terrain == PASSABLE)
    {
		maze[x][y] = (maze[x][y] & gSTRAIGHTS);
		x += 1; y -= 1;
		if(x < xLength && y >= 0)
			maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSW));
		x = point.x - 1; y = point.y - 1;
		if(x >= 0 && y >= 0)
			maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSE));
        x = point.x +1; y = point.y + 1;
		if(x < xLength && y < yLength)
			maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNW));
        x = point.x - 1; y = point.y + 1;
		if (y < yLength && x >= 0)
			maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNE));
	}			
	else if((terrain == UNPASSABLE)) //if it is UNPASSABLE then we kill the node and all connections to it
	{
		maze[x][y] = 0x00;
		maze[x][y-1] -= (maze[x][y-1] & gSOUTH);
		maze[x][y+1] -= (maze[x][y+1] & gNORTH);
		maze[x-1][y] -= (maze[x-1][y] & gEAST);
		maze[x+1][y] -= (maze[x+1][y] & gWEST);
		maze[x+1][y+1] -= (maze[x+1][y+1] & gNW);
		maze[x+1][y-1] -= (maze[x+1][y-1] & gSW);
		maze[x-1][y+1] -= (maze[x-1][y+1] & gNE);
		maze[x-1][y-1] -= (maze[x-1][y-1] & gSE);		
        }
    }
}

//sets all possible bridges in the direction given for the input point
//cuts all other bridges in the square thus ASSUMES ONLY 1 BRIGE PER SQUUARE
/*
eg. (2,1), EAST, oneBridgePerSquare. "Set node A to EAST and assume only one bridge per square":
-N--N--N--N--N-       -N--N--N--N--N-
/| / \ | / \ |\       /| /   | /   |\
\|/   \|/   \|/       \|/    |/    |/
-N     A     N-       -N     A     N-
/|\   /|\   /|\       /|\    |\    |\
\| \ / | \ / |/       \| \   | \   |/
-N--N--N--N--N-       -N--N--N--N--N-

eg. (2,1), EAST, NOT onBridgePerSquare. "Set node A to EAST but there may be more than 1 bridge per square":
-N--N--N--N--N-       -N--N--N--N--N-
/| / \ | / \ |\       /| /   | / \ |\
\|/   \|/   \|/       \|/    |/   \|/
-N     A     N-       -N     A     N-
/|\   /|\   /|\       /|\    |\   /|\
\| \ / | \ / |/       \| \   | \ / |/
-N--N--N--N--N-       -N--N--N--N--N-
*/
void Maze::setTerrain(Point point, Direction direction, bool oneBridgePerSquare)
{
	short int x = point.x;
	short int y = point.y;
	//if the point is within the maze
	if (point.x < xLength && point.y < yLength && point.x >= 0 && point.y >= 0)
    {
		unsigned char setTo;
		switch(direction)
		{
			case NORTH:
				maze[x][y] = (maze[x][y] & gSTRAIGHTS) + (maze[x][y] & gNORTHWARDS);
				x = point.x + 1; y = point.y + 1;
				if(x < xLength && y >= 0)
					maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNW));
				x = point.x - 1; y = point.y + 1;
				if(x >= 0 && y >= 0)
					maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNE));
				if(oneBridgePerSquare)
				{
					x = point.y + 1; y = point.y = 1;
					if(x < xLength && y >= 0)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNW));
					x = point.x - 1; y = point.y - 1;
					if(x >= 0 && y >= 0)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNE));
					x = point.x; y = point.y - 2;
					if(y >= 0)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSOUTHWARDS));
				}
				break;
			case SOUTH:
				maze[x][y] = (maze[x][y] & gSTRAIGHTS) + (maze[x][y] & gSOUTHWARDS);
				x = point.x + 1; y = point.y - 1;
				if(x < xLength && y >= 0)
					maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSW));
				x = point.x - 1; y = point.y - 1;
				if(x >= 0 && y >= 0)
					maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSE));
				if(oneBridgePerSquare)
				{				
					x = point.x + 1; y = point.y + 1;
					if(x < xLength && y < yLength)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSW));
					x = point.x - 1; y = point.y + 1;
					if (y < yLength && x >= 0)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSE));
					x = point.x; y = point.y + 2;
					if (y < yLength)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNORTHWARDS));
				}
				break;
			case EAST:
				maze[x][y] = (maze[x][y] & gSTRAIGHTS) + (maze[x][y] & gEASTWARDS);
				x = point.x - 1; y = point.y -1;
				if(x >= 0 && y >= 0)
					maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSE));
				x = point.x - 1; y = point.y + 1;
				if(x >= 0 && y < yLength)
					maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNE));				
				if(oneBridgePerSquare)
				{
					x = point.x + 1; y = point.y - 1;
					if(x < xLength && y >= 0)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSE));
					x = point.x + 1; y = point.y + 1;
					if(x < xLength && y < yLength)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNE));
					x = point.x + 2; y = point.y;
					if(x < xLength)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gWESTWARDS));
				}
				break;
			case WEST:
				maze[x][y] = (maze[x][y] & gSTRAIGHTS) + (maze[x][y] & gWESTWARDS);
				x = point.x + 1; y = point.y -1;
				if(x < xLength && y >= 0)
					maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSW));
				x = point.x + 1; y = point.y + 1;
				if(x < xLength && y < yLength)
					maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNW));
				if(oneBridgePerSquare)
				{
					x = point.x -1; y = point.y - 1;
					if(x >= 0 && y >= 0)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gSW));
					x = point.x - 1; y = point.y + 1;
					if (y < yLength && x >= 0)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gNW));
					x = point.x - 2; y = point.y;
					if(x >= 0)
						maze[x][y] = (maze[x][y] & gSTRAIGHTS) + ((maze[x][y] & gDIAGONALS) - (maze[x][y] & gEASTWARDS));
				}
				break;
        }
	}
}

//getting too complex, need to do it out properly
/*
void Maze::removeSingleDirectionConnections()
{
	byte listOfConnections[8] = {gNORTH,gEAST,gSOUTH,gWEST,gNE,gSE,gSW,gNW};
	byte adjacentConnections = {};
	Point adjacentPoints[8];
	for(int mm = 0; mm < 8; mm++)
	{
		for(int ii = -1; ii <= 1; ii++)
		{
			for(int jj = -1; jj <= 1; jj++)
			{
				if((ii != 0) && (jj != 0))
				{
					adjacentPoints[mm].x = ii;
					adjacentPoints[mm].y = jj;
				}
			}
		}
	}
	for(int ii = 0; ii < xLength; ii++)
	{
		for(int jj = 0; jj < yLength; jj++);
		{
			for(int kk = 0; kk < 8; kk++)
			{
				if(((maze[ii][jj] & listOfConnections[kk]) == listOfConnections[kk]) && (((getTerrain(adjacentPoints[kk]) ^ (gUNKNOWN) & adjacentConnections[kk]) != adjacentConnections[kk])				
}*/

// Gets the terrain from the maze.
// Will return UNKNOWN if it's outside of our range.
byte Maze::getTerrain(Point point)
{
  // Only get something if we're within our range.
  // As it's unsigned, it cannot be less than zero.
  if (point.x < xLength && point.y < yLength)
  {
    short int x = point.x;
    short int y = point.y;
    return maze[x][y];
  }
  return gUNKNOWN;
}

unsigned short int Maze::getXLength()
{
  return xLength;
}

unsigned short int Maze::getYLength()
{
  return yLength;
}

// Return the generated route.
// The route will be generated using the A* pathfinding algorithm
Path Maze::generateRoute(Point start, Point goal)
{
  ListNode openClosedList[MAZERANGE][MAZERANGE];

  // Create the openClosed list.
  for (int x = 0; x < MAZERANGE; x++)
  {
    for (int y = 0; y < MAZERANGE; y++)
    {
      ListNode currentNode = {{-1,-1}, -1, -1, -1, NONE};
      openClosedList[x][y] = currentNode;
    }
  }

  // Initialise starting node.
  openClosedList[start.x][start.y].listType = OPEN;
  openClosedList[start.x][start.y].movementCost = 0;
  openClosedList[start.x][start.y].heuristic = calcHeuristic(start, goal);
  openClosedList[start.x][start.y].sum = calcHeuristic(start, goal);

  // Initialise goal node.
  openClosedList[goal.x][goal.y].listType = GOAL;
  openClosedList[goal.x][goal.y].movementCost = 0;
  openClosedList[goal.x][goal.y].heuristic = 0;
  openClosedList[goal.x][goal.y].sum = 0;

  // Return Control Flags
  bool openListEmpty = false; //  If openListEmpty, then we have attempted a path.
  bool atGoal = false;        //  If atGoal, then we have a valid path.
  bool partialPath = false;   //  Else If openListEmpty && partialPath, then we have a partial path.
  //  Else there's no valid path.

  Point parentOfFirstUnknownPoint;

  while (!(openListEmpty || atGoal)) // if either openListEmpty or atGoal, then finish loop.
  {
    Point current;

    bool foundOpen = false;
    bool firstFound = false;
    // Search through the open list to get the lowest sum node. Set it to current.
    for (int x = 0; x < xLength; x++)
    {
      for (int y = 0; y < yLength; y++)
      {
        if (openClosedList[x][y].listType == OPEN)
        {
          foundOpen = true;
          if (!firstFound)
          {
            current.x = x;
            current.y = y;
            firstFound = true;
          }          
          if (openClosedList[x][y].sum < openClosedList[current.x][current.y].sum)
          {
            current.x = x;
            current.y = y;
          }
        }
      }
    }

    if (!foundOpen)
    {
      openListEmpty = true;
    }

    // Add the current node to the closed list.
    openClosedList[current.x][current.y].listType = CLOSED;

    unsigned short int currentCost = openClosedList[current.x][current.y].movementCost;
    // Get adjacent nodes to current. Calculate values. Allows diagonal movement.
    for (short int xDelta = -1; xDelta <= 1; xDelta++)
    {
      for (short int yDelta = -1; yDelta <= 1; yDelta++)
      {
		/*  if(atGoal)
			  break;*/
          unsigned short int heuristic;
          unsigned short int movementCost;
          Point newPoint = {(current.x + xDelta), (current.y + yDelta)};
// Check if outside range AND if newPoint can be reached from current, if everything is satisfied, continue
          if(pointsJoined(current, newPoint) && ! (newPoint.x < 0 || newPoint.x >= xLength || newPoint.y < 0 || newPoint.y >= yLength))
		  {
            ListNode* newNode = &openClosedList[newPoint.x][newPoint.y];
            switch (newNode->listType)
            {
              case OPEN:
                movementCost = calcCost(newPoint, current) + currentCost;
                heuristic = newNode->heuristic; 
                // If this path is shorter to get to from current rather than a previous
                if (movementCost < newNode->movementCost)
                {
                  // Store parent and cost to openClosedList array if the new way is quicker.
                  newNode->pathParent = current;
                  newNode->sum = movementCost + heuristic;
                  newNode->movementCost = movementCost;
                  newNode->heuristic = heuristic;
                }
                break;
              case NONE:
// Already checked to see if we can walk it.
				   if(isUnknown(newPoint))
					{
						partialPath = true;
						parentOfFirstUnknownPoint = current;
					}
					else
					{
					// It's new (not closed, not open), it's walkable, so calculate cost.  
					heuristic = calcHeuristic(goal, newPoint);
					movementCost = calcCost(newPoint, current) + currentCost;

					// Store cost and parent back to the openClosedList array.
					newNode->pathParent = current;
					newNode->sum = movementCost + heuristic;
					newNode->movementCost = movementCost;
					newNode->heuristic = heuristic;
					newNode->listType = OPEN;
					}
					break;
              case GOAL:
                // We're close enough. Finish loop with valid path.
                atGoal = true;
                newNode->pathParent = current;
				break;
            }//switch
		  }//if
      }//for
    }//for
  }//while

  // Work backward once to determine path length. from goal to generate chosen path.
  Point tempPoint;

  // Set the tempPoint to the end.
  if (atGoal)
  {
    tempPoint = goal;
  }
  else if (openListEmpty && partialPath)
  {
    tempPoint = parentOfFirstUnknownPoint;
  }
  else
  {
    tempPoint = start;
  }

  short int pathLength = 0;
  while (!pointsEqual(tempPoint, start))
  {
    tempPoint = openClosedList[tempPoint.x][tempPoint.y].pathParent;
    pathLength++;
  }

  // Set the tempPoint to the end.
  if (atGoal)
  {
    tempPoint = goal;
  }
  else if (openListEmpty && partialPath)
  {
    tempPoint = parentOfFirstUnknownPoint;
  }  
  else
  {
    tempPoint = start;
  }

  Path path;
  path.length = pathLength;
  for (int x = pathLength - 1; x >= 0; x--)
  {
    path.path[x] = tempPoint;
    tempPoint = openClosedList[tempPoint.x][tempPoint.y].pathParent;
  }
  return path;
}

// UNKNOWN terrain returns false.
// Does not check for connections.
// CURRENTLY ALWAYS RETURNS TRUE
bool Maze::walkable(Point point)
{
  return ! isUnknown(point);
}

// calcCost will return a value calculated based on the terrain.
// This should ordinarily be used between two adjacent nodes.
// We have written this code so that a cost could be calculated between
// nodes a distance apart.
short int Maze::calcCost(Point from, Point to)
{
  short int deltaY = abs(from.y - to.y);
  short int deltaX = abs(from.x - to.x);
  short int ortho = min(deltaY, deltaX); 
  short int diag = max(deltaY, deltaX) - ortho;

  // ORTHOGONAL default value = 5, DIAGONAL default value = 7
  if(isUnknown(to))
    return UNWALKABLEWEIGHT * ((ORTHOGONAL * ortho) + (DIAGONAL * diag));
  else
    return ((ORTHOGONAL * ortho) + (DIAGONAL * diag));
}

//TODO: Appropriate value for unpassable terrain needs to be decided. Using 100.
/*short int Maze::mazeWeight(Point mazeRef)
{
  // {UNKNOWN, BLOCK, ONEWEIGHT, TWOWEIGHT, FOURWEIGHT, UNPASSABLE, OUTOFRANGE}
  switch (maze[mazeRef.x][mazeRef.y])
  {
    case ONEWEIGHT:
      return 1;
    case TWOWEIGHT:
      return 2;
    case FOURWEIGHT:
      return 4;
    case UNKNOWN:
    case BLOCK:
    case UNPASSABLE:
    case OUTOFRANGE:
      return UNWALKABLEWEIGHT;
  }
}*/

// Calculates the heuristic between two points.
short int Maze::calcHeuristic(Point one, Point two)
{
  // The distance in the x dimension plus the distance in the y dimension.
  return HEURISTICWEIGHT * (abs(one.x - two.x) + abs(one.y - two.y));
}

bool Maze::pointsEqual(Point one, Point two)
{
  return ((one.x == two.x) && (one.y == two.y));
}

//returns true if there is a connection from pt1 to pt2 AND from pt2 to pt1 AND they are adjacent
bool Maze::pointsJoined(Point pt1, Point pt2)
{
	bool isConnection = false;
	short int x1 = pt1.x;
	short int y1 = pt1.y;
	short int x2 = pt2.x;
	short int y2 = pt2.y;
	//if the points are adjacent
	if(abs(x1-x2) <= 1 && abs(y1-y2) <= 1)
	{
		switch(x2-x1)
		{
			case 0:
				switch(y2-y1)
				{
					case 0:
						isConnection = true;
						break;
					case 1:
						isConnection = (((maze[x1][y1] & gSOUTH) == gSOUTH) && ((maze[x2][y2] & gNORTH) == gNORTH));
						break;
					default:
						isConnection = (((maze[x1][y1] & gNORTH) == gNORTH) && ((maze[x2][y2] & gSOUTH) == gSOUTH));
						break;
				}
				break;
			case 1:
				switch(y2-y1)
				{
					case 0:
						isConnection = (((maze[x1][y1] & gEAST) == gEAST) && ((maze[x2][y2] & gWEST) == gWEST));
						break;
					case 1:
						isConnection = (((maze[x1][y1] & gSE) == gSE) && ((maze[x2][y2] & gNW) == gNW));
						break;
					case -1:
						isConnection = (((maze[x1][y1] & gNE) == gNE) && ((maze[x2][y2] & gSW) == gSW));
						break;
				}
				break;			
			case -1:
				switch(y2-y1)
				{
					case 0:
						isConnection = (((maze[x1][y1] & gWEST) == gWEST) && ((maze[x2][y2] & gEAST) == gEAST));
						break;
					case 1:
						isConnection = (((maze[x1][y1] & gSW) == gSW) && ((maze[x2][y2] & gNE) == gNE));
						break;
					default:
						isConnection = (((maze[x1][y1] & gNW) == gNW) && ((maze[x2][y2] & gSE) == gSE));
						break;
				}
				break;	
        }		
	}
    else
        isConnection = false;
	return isConnection;
}

//returns true if more than 1 diagonal is set for the point 
//(ie, the node hasn't been resolved to just one/zero diagonal and therefore we don't have a complete picture of it)
//this assumes only one bridge per square
//CURRENTLY ONLY RETURNS FALSE, 
//there is a bug in there somewhere, possibly now solved by changing:
//if(maze[x][y] & check[ii] == check[ii]) to if((maze[x][y] & check[ii]) == check[ii])
//not yet tested, logic needs to be changed too, Toby thinks
bool Maze::isUnknown(Point point)
{
	/*byte check[11] = {gDIAGONALS,gNW+gNE+gSW,gNW+gNE+gSE,gNW+gSE+gSW,gNE+gSE+gSW,gEASTWARDS,gWESTWARDS,gNORTHWARDS,gSOUTHWARDS, gNE+gSW, gNW+gSE};
	bool nodeIsUnknown = false;
	short unsigned int x = point.x;
	short unsigned int y = point.y;
	for(int ii = 0; ii < 11; ii++)
	{
		if((maze[x][y] & check[ii]) == check[ii])
		{
			nodeIsUnknown = true;
			break;
		}
	}*/
	return false;//nodeIsUnknown;
}

//prints out picture of the current maze, good for debugging.  Prints out a "picture" of characters: (3 * yLength) by (3 * xLength)
//Only works via USB, it would be good to do over Xbee too
void Maze::printMazeOverSerial()
{
	byte terrain;
	for(int ii = 0; ii < yLength; ii++)
	{
		for(int kk = 0; kk < 3; kk++)
		{
			for(int jj = 0; jj < xLength; jj++)
			{
				terrain = maze[jj][ii];
				if(kk == 0)
				{
					if((terrain & gNW) == gNW)
						Serial.print("\\");
					else
						Serial.print(" ");
					if((terrain & gNORTH) == gNORTH)
						Serial.print("|");
					else
						Serial.print(" ");
					if((terrain & gNE) == gNE)
						Serial.print("/");
					else
						Serial.print(" ");
				}
				else if(kk == 1)
				{
					if((terrain & gWEST) == gWEST)
						Serial.print("-");
					else
						Serial.print(" ");
					Serial.print("N");
					if((terrain & gEAST) == gEAST)
						Serial.print("-");
					else
						Serial.print(" ");

				}
				else if(kk == 2)
				{
					if((terrain & gSW) == gSW)
						Serial.print("/");
					else
						Serial.print(" ");
					if((terrain & gSOUTH) == gSOUTH)
						Serial.print("|");
					else
						Serial.print(" ");
					if((terrain & gSE) == gSE)
						Serial.print("\\");
					else
						Serial.print(" ");
				}				
			}
	Serial.println();
		}
	}
}












