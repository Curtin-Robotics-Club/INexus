/* 
 * issues: 		
 *				 isUnknown() only returns false (might stay that way)
 *				 walkable(Point) needs to be updated for the new representation
 *   			 calcCost(Point) needs to be updated to take into account turning
 *				 there is no "weight" (all points take the same time to cross)
 *
 * The Maze class is here to represent the maze that an automatic robot
 * will operate on. The physical layout of the maze is a black surface
 * with white lines.
 * Any point may connect to any of the 8 adjacent points, all points are equidistant
 * 
 * The maze class here will give a deterimination of what is on the maze
 * and provide an A* method to give a route from one maze point to another.
 * The route can either be only along the x and y axis OR can include the
 * ability to go diagonally.
 *
 * The original code was written by James Lumsden, Yusuf Syaid, & Mike Hamer.
 * Tidied, converted into OO and continued by Todd Hurst & Zachary Oliver.
 * Changed to a binary array representation and incorporated diagonals by Toby Scantlebury.
 * Started 10:23 23/12/2010
 * Last Updated 11:21 07/11/2011
 */

#ifndef Maze_h
#define Maze_h

// The maximum range we want on the x and y dimensions is 16 (so every coordinate is one byte), gives 256 bytes for the maze max.
#define MAZERANGE 16

// Cost of travel. Todd has set these to 5 and 7 just to shut his brother up.
#define ORTHOGONAL 5
#define DIAGONAL 7

// Used in mazeWeight
#define UNWALKABLEWEIGHT 100
#define HEURISTICWEIGHT 1

// TODO iostream is just here for testing 
//#include <iostream>
#include <WProgram.h> 
#include "struct.h"

using namespace std;


// This is the terrain of the maze we will be driving across
// We may turn this into a character and define to save memory.
enum Terrain {UNKNOWN, BLOCK, ONEWEIGHT, TWOWEIGHT, FOURWEIGHT, UNPASSABLE, OUTOFRANGE, PASSABLE};

//Holds directions that you can access from nodes

#define gNORTH 0x80
#define gEAST 0x40
#define gSOUTH 0x20
#define gWEST 0x10
#define gNE 0x08
#define gSE 0x04
#define gSW 0x02
#define gNW 0x01
#define gUNKNOWN 0xFF
#define gDIAGONALS 0x0F
#define gSTRAIGHTS 0xF0
#define gEASTWARDS 0x0C
#define gWESTWARDS 0x03
#define gNORTHWARDS 0x09
#define gSOUTHWARDS 0x06

// Path structure is here to return the path that the bot should take.
typedef struct
{
  Point path[(MAZERANGE * MAZERANGE)];
  short int length;
} Path;

enum ListType {OPEN, CLOSED, NONE, GOAL};

// ListNode is to represent bot the open list and closed list for generateRoute().
// We have created it like this to save memory.
// We may go further and turn 'enum ListType' into a character and define. 
typedef struct
{
  Point pathParent;
  short int sum; // F
  short int movementCost; // G
  short int heuristic; // H
  ListType listType;
} ListNode;
 
 
class Maze
{
  public:
    // Constructor
    // we will not allow an xLength or yLength of larger than MAZERANGE.
    // (So we can mark one point as a character. Yay)
    Maze(unsigned short int newXLength, unsigned short int newYLength);
	
	//Initialising the maze is modular, can be adapted to different situations
	//check Maze.cpp for more detailed desriptions of these
	void initAllCentresUnpassable();
	void initCutAllCardinalConnectionsToCentres();
	void initCutAllConnectionsOutOfRange();
 
    // Sets the terrain in the maze.
    void setTerrain(Point point, Terrain terrain);
	void setTerrain(Point point, Direction direction, bool oneBridgePerSquare);
 
    //currently just returns the Maze representation of a node
    byte getTerrain(Point point);
 
    // Return the generated route.
    // It will generate a route without going diagonal.
    // Todd wants to go diagonal. The code can be altered easily. It will be
    // done when we get our robots to move diagonally. That is a bit harder.
    Path generateRoute(Point start, Point end);
    // Check to see if the points are equal.
    bool pointsEqual(Point one, Point two);
    
    unsigned short int getXLength();
    unsigned short int getYLength();
 
	//prints out picture of the current maze, good for debugging.  Prints out a "picture" (3 * yLength) by (3 * xLength) characters
	//Only works via USB, it would be good to do over Xbee too
	void printMazeOverSerial();
	
  private:
    
	byte maze[MAZERANGE][MAZERANGE];
    unsigned short xLength;
    unsigned short yLength;
 
    //returns true if there is a connection from pt1 to pt2 AND from pt2 to pt1 AND they are adjacent
	bool pointsJoined(Point pt1, Point pt2);
    // This will return a heuristic for calculating the route.
    short int calcHeuristic(Point one, Point two);
    // This will return a cost based on the terrain between two nodes.
    short int calcCost(Point one, Point two);
   
   // This will return the cost of the terrain on a node.
//	NOT IMPLEMENTED
//    short int mazeWeight(Point mazeRef);
 
 // Returns if it's walkable. CURRENTLY ONLY TRUE
    bool walkable(Point point);
//returns true if more than 1 diagonal direction is set CURRENTLY ONLY FALSE
	bool isUnknown(Point point);
    
  protected:
 
};

#endif
