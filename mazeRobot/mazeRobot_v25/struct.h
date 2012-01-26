/*
 * THIS MODIFIED FOR MAZE ROBOT
 * The original code was written by James Lumsden, Yusuf Syaid, & Mike Hamer.
 * Tidied, converted into OO and continued by Todd Hurst & Zachary Oliver.
 * Started 10:23 23/12/2010
 * Last Updated 10:28 24/12/2010
 */

#ifndef struct_h
#define struct_h

// NORTH is facing away from the barrier between the manual robots start point and the grid.
enum Direction {NORTH, NEAST, EAST, SEAST, SOUTH, SWEST, WEST, NWEST, NOT_NS, NOT_EW};
// This is just a structure of an x and y position on the grid
typedef struct
{
  short int x;
  short int y;
} Point;
/*
bool pointEquals(Point one, Point two)
{
  return (one.x == two.x && one.y == two.y);
} 
*/
#endif
