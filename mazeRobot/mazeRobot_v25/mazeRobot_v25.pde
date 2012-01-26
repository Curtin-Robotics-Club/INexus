/************************************************************************
** Search for "competition" to find all variables to set for competion **
** Search for "testing" to find all tests (delays and testing loops    **
************************************************************************/
/****************Map maze and return***************
Idea:
    From starting position:
    Give the rebot a point from our list(pointList[] => goalPoint), which can be reached by going in a straight line
    The robot decides where the first node it should go to is and sets that as nextNode
    on its way to nextNode:
        if it sees something on its left it sets boolean variable leftBridgeFound
        if it sees something on its right it sets boolean variable rightBridgeFound
        when it sees something on both sensors then its gotten to nextNode (Logic for this doesn't work because there might be 2 bridges)
        it decides if the node is passable or not, sets bridges based on right/leftBridgeFound
        then sets nextNode to the next node it needs to go in order to reach goalPoint
    when it reaches the goalPoint we give it another point and it repeats*/
//So we can use things
#include "Robot.h"
#include <PololuQTRSensors.h>
#include <PololuWheelEncoders.h>
#include "Maze.h"
#include "struct.h"

//Name all the switches (the box at the front)
#define BOTPIN 52
#define CALPIN 50
#define SPEEDONEPIN 48
#define SPEEDTWOPIN 46
#define SPEEDTHREEPIN 44
#define SPEEDFOURPIN 42
#define RESTARTPIN 40
#define DEBUGPIN 38

//Name things to make life easier/less confusing
#define LEDPIN 2//the big red LED for signalling info
#define PUSHPIN 3//The left-most reset button for starting calibration and for starting the program
#define XBee Serial3//for convenience, XBee is connected to Serial3 (used for wireless comunication)

//These are to make it easier to understand the sensor readings
//sensors as seen from birdseye view (ie, robot's perspective), left is robot's left etc
#define vvvR 0 //[v|v|v|v|v|v|v|R] "rightmost sensor"
#define vvRv 1 //[v|v|v|v|v|v|R|v]
#define vRvv 2 //[v|v|v|v|v|R|v|v]
#define Rvvv 3 //[v|v|v|v|R|v|v|v] "right centre sensor"
#define vvvL 4 //[v|v|v|L|v|v|v|v] "left centre sensor"
#define vvLv 5 //[v|v|L|v|v|v|v|v]
#define vLvv 6 //[v|L|v|v|v|v|v|v]
#define Lvvv 7 //[L|v|v|v|v|v|v|v] "Leftmost sensor"
#define MIN_BLACK 250//min value for a black reading, less than this is white
#define MIN_BLACKISH (MIN_BLACK * 4)/5

//inititalise sensors etc...
PololuQTRSensorsRC mazeSensors((unsigned char[]) {22,23,24,25,26,27,28,29}, 8, 4000, 30);
PololuQTRSensorsRC blockSensors((unsigned char[]) {22,23,24,25,26,27,28,29}, 8, 4000, 32);
PololuWheelEncoders wheelEnc;
Robot robot = Robot(mazeSensors, mazeSensors, wheelEnc);//the robot

/************* Set up maze *******************/
#define MAZESIZE 7 //5x5:9//set 15 for competition, this is for 5x5 arena
Maze maze = Maze(MAZESIZE,MAZESIZE);//the maze
#define ONE_BRIDGE_PER_SQUARE true


#define EXPECTED_TICKS 100//amount of ticks expected to reach one cross from another. This number may not be accurate.
#define MAX_EXPECTED_TICKS  (EXPECTED_TICKS * 6)/5//upper range of EXPECTED_TICKS
#define MIN_EXPECTED_TICKS  (EXPECTED_TICKS * 4)/5//lower range of EXPECTED_TICKS
enum Direction myDirection; //stores robot's orientation
Point myLocation;//keep track of our position
Point goal;//used in path planning and moving from Todd's stuff. Currently doesn't need to be global
Point startPoint;//defines the starting point for the robot (14,8) for competition
Path myPath;//used in path planning and moving from Todd's stuff. Currently doesn't need to be global

void setup()
{
//Configure pins
    pinMode(LEDPIN, OUTPUT);  
    pinMode(PUSHPIN, INPUT);
    pinMode(BOTPIN, INPUT);
    pinMode(DEBUGPIN, INPUT);
    pinMode(SPEEDONEPIN, INPUT);
    pinMode(SPEEDTWOPIN, INPUT);
    pinMode(SPEEDTHREEPIN, INPUT);
    pinMode(SPEEDFOURPIN, INPUT);
    pinMode(RESTARTPIN, INPUT);
    pinMode(CALPIN, INPUT);
  
//not sure if we actually need any of these...
    //turn on pull-ups
    digitalWrite(PUSHPIN, HIGH);
    digitalWrite(BOTPIN, HIGH);
    digitalWrite(DEBUGPIN, HIGH);
    digitalWrite(SPEEDONEPIN, HIGH);
    digitalWrite(SPEEDTWOPIN, HIGH);
    digitalWrite(SPEEDTHREEPIN, HIGH);
    digitalWrite(SPEEDFOURPIN, HIGH);
    digitalWrite(RESTARTPIN, HIGH);
    digitalWrite(CALPIN, HIGH);

//For debugging/communications
    // Set up the serial connections
    Serial.begin(9600);
    XBee.begin(9600);

//initialise motors and servos  
    ServoControl.begin(9600);
    MotorControl.begin(38400);
    MotorControl.write(0xAA);
  
//Initialise maze
    maze.initAllCentresUnpassable();//Cut all directions leaving the centres
    maze.initCutAllCardinalConnectionsToCentres();//this preserves diagonals entering the centres from nodes because those are used to check if the nodes have been visited
    maze.initCutAllConnectionsOutOfRange();//get rid of any connections that lead out of the maze's range

// Pause to be pushed for calibration of maze QTR
    digitalWrite(LEDPIN, HIGH);
    while(digitalRead(PUSHPIN) == HIGH)
    {
        delay(1000); 
    }
    digitalWrite(LEDPIN, LOW);

    robot.openClaws();
    robot.calibrateGrid();

    int error;//testing
// Pause to be pushed for starting
    digitalWrite(LEDPIN, HIGH);
    while(digitalRead(PUSHPIN) == HIGH)
    {
        error = robot.errorAdjustment();//testing
        robot.gridSensorInfo();//testing
        delay(1000); 
    }
    digitalWrite(LEDPIN, LOW);
}

// turnToPoint turns to orientation. It is to be used so that orientation
// is only one point away from myLocation.
//Currently used with nextNode because nextNode is only different in the x or y direction, not both.  Tested and it works.
// Code copied from Todd's openDay code
void turnToPoint(Point orientation);

//generates path and navigates to the Point "goal"
void moveTo();//goal and myLocation are currently global, hence no import

//returns stepValue for mapping the maze, used to set where nextNode is
short int getStepValue(short int inPointCoord, short int inMyLocCoord);

//returns direction to pass to maze.setTerrain() if there is a connection on our right
Direction getRightConnection(Direction inDir);

//returns direction to pass to maze.setTerrain() if there is a connection on our left
Direction getLeftConnection(Direction inDir);

//returns bridgeNode in order to set bridge locations 
Point getBridgeNode(bool inXDirection, Point nextNode, int stepValue);

//returns if the sensors might be seeing a bridge on the right
bool isPotentialRightBridge(unsigned int* sensors);

//returns if the sensors might be seeing a bridge on the left
bool isPotentialLeftBridge(unsigned int* sensors);

//returns true if sensors are [O|O| | | | |O|O]
bool onCross(unsigned int* sensors);

//Check if any of the four centre sensors are black (intensity of black adjusted for noise)
bool isUnpassable();

//Checks if we've gone overtime, if so then navigates to startPoint and returns true
bool overTime(unsigned long startTimeInMillis, int loopCount, const int NUMPOINTS);

//go backwards while adjusting for error
void reverse();

//checks if the ticks after startTicks are within a range divided by scale, scale = 1 for a cross, scale = 2 for bridges
bool withinExpectedTicks(int startTicks, int scale);

//maps maze and signals completion, will not go overtime
void createMapOfMaze();

//loops through testing points to ensure mapping was done correctly and test navigation
void loopThroughTestingPointsIndefinitely();

void loop()
{
    createMapOfMaze();
    //loopThroughTestingPointsIndefinitely();
    testDeliverBlock();
}

Point blockPickup = Point {14,6};
Point blockDropoff = Point {0,14};
void testDeliverBlock()
{
  goal = blockPickup;
  moveTo();
  robot.closeClaws();
  goal = blockDropoff;
  delay(1);
  moveTo();
  robot.turnTo(SEAST);
  delay(1);
  robot.moveTicks(DIAGONALTICKS, STRAIGHTSPEED);
  delay(1);
  robot.openClaws();
  delay(1);
  robot.moveTicks(-DIAGONALTICKS, STRAIGHTSPEED);
  goal = startPoint;
  moveTo();
}

//maps maze and signals completion, will not go overtime
void createMapOfMaze()
{
/*************** START setup *****************/

/******* START set up points to iterate for loop ********/
    // (for 8x8) (competition) This visits 3 sides of every square, that should be sufficient for mapping
    // short int coordList[NUMPOINTS] = {0,8,0,6,14,6,14,4,0,4,0,2,14,2,14,0,0,0,0,14,14,14,14,12,0,12,0,10,14,10,14,0,10,0,10,14,6,14,6,0,2,0,2,14,14,14,14,8};//used to set the list of points
    //const int NUMPOINTS = 24 // this used to hold the size of the list of points for mapping the maze
    //const int COORDS = 48;// this is used to set up the pointList    

    //(for 5x5)
    //short int coordList[COORDS] = {8,0,0,0,0,8,2,8,2,0,4,0,4,8,6,8,6,0,8,0,8,2,0,2,0,4,8,4,8,6,0,6,0,8,8,8,8,4};//testing coords
    //const int NUMPOINTS = 19;//this used to hold the size of the list of points for mapping the maze
    //const int COORDS = 38; //this is used to set up the pointList    

    //(for 3x3)
    const int NUMPOINTS = 13;//this used to hold the size of the list of points for mapping the maze
    const int COORDS = 26; //this is used to set up the pointList    
    short int coordList[COORDS] = {6,0,0,0,0,6,2,6,2,0,4,0,4,6,6,6,6,4,0,4,0,2,6,2,6,4};//testing coords

    Point pointList[NUMPOINTS];//List of points for mapping maze
    
	//set up pointList using coordList
    for(int ii; ii < COORDS; ii+=2)
    {
        Point tempPoint;
        tempPoint.x = coordList[ii];
        tempPoint.y = coordList[ii+1];
        pointList[ii/2] = tempPoint;
    }
/********* END set up points to iterate for loop *********/

    const int DELAY_TICKS = 15; //used to delay so we don't read the same point of interest twice
    const int DELAY_TO_CHECK_ANGLE = 30;//used to ensure we don't preemptively call a cross a bridge, we use this to wait then check again, just in case
    const int DELAY_TO_CHECK_READING = 30;//used to ensure a reading from the sensors isn't just a fluke

    unsigned int sensors[8];//holds sensor info to see where the line is
    Point goalPoint;//holds the point in the for loop, this is the point we will have to turn
    Point nextNode; //for iteration of the mapping for loop, it's the nextNode to go to until we reach goalPoint
	enum Direction connection;//used to set where bridges connect
    Point bridgeNode;//used to set bridges
    short int stepValue; //size and direction of the step needed to set the nextNode
    bool inXDirection; //used to determine if we are moving in the x direction or y direction
    bool leftBridgeFound; //used to tell if we found a bridge on the left before we arrived at the node
    bool rightBridgeFound; //used to tell if we found a bridge on the right before we arrived at the node
    int startTicks;//holds number of ticks the wheels had, used to compare with current ticks to check if we've gone too far without seeing anything
    int lastError = 0;//keep track of the last error
    bool haveReversed = false;//Used to keep track of whether or not we've already gone back to to recheck our readings

    //set our initial position
    startPoint.x = 6;//5x5:8//set 14 for competition
    startPoint.y = 4;//5x5:4//set 8 for competition
    myLocation = startPoint;

    unsigned long startTimeInMillis = millis();//used to tell how long we've been mapping incase we're going overtime
/************** END setup ****************/

    //24 points(in competition), for each point we move in a straight line and scan until we reach it
    for(int ii = 0; ii < NUMPOINTS; ii++)
    {
        goalPoint = pointList[ii];
        if(myLocation.x - goalPoint.x == 0)//Check if we moving in y direction
        {
            inXDirection = false;
            nextNode.x = myLocation.x;
            stepValue = getStepValue(goalPoint.y,myLocation.y);
            nextNode.y = myLocation.y + stepValue;
        }
        else//must be moving in x direction
        {
            inXDirection = true;
            nextNode.y = myLocation.y;
            stepValue = getStepValue(goalPoint.x,myLocation.x);
            nextNode.x = myLocation.x + stepValue;
        }
        turnToPoint(nextNode);
        startTicks = robot.getTicks();
        robot.moveTicks(DELAY_TICKS,STRAIGHTSPEED);//Get off the cross, if we're on it.
        robot.motorForward(STRAIGHTSPEED,0);
        while((inXDirection && (myLocation.x != goalPoint.x))||( ! inXDirection && (myLocation.y != goalPoint.y)))//while haven't reached goalPoint
        {
            robot.gridSensors.readCalibrated(sensors,QTR_EMITTERS_ON);
/*            if((robot.getTicks() - startTicks) > MAX_EXPECTED_TICKS)
            {
                reverse();
                haveReversed = true;
            }
*/
            if(onCross(sensors) && (withinExpectedTicks(startTicks,1) || haveReversed))
            {
                delay(DELAY_TO_CHECK_READING);
                robot.gridSensors.readCalibrated(sensors,QTR_EMITTERS_ON);
                if(onCross(sensors))
                {                            
                    robot.motorStop();//stop for a bit, just incase
                    delay(10);
                    startTicks = robot.getTicks();

                    if(isUnpassable())
                        maze.setTerrain(nextNode,UNPASSABLE);
                    else //[ | | | X?|X?| | | ]
                        maze.setTerrain(nextNode,PASSABLE);
						
                    if(!leftBridgeFound && !rightBridgeFound)//cut bridge connections
                    {
                        bridgeNode = getBridgeNode(inXDirection, nextNode, stepValue);
                        maze.setTerrain(bridgeNode,PASSABLE);
                    }
                    else if( ! rightBridgeFound)
                    {
                        myDirection = robot.getMyDirection();                
                        bridgeNode = getBridgeNode(inXDirection, nextNode, stepValue);
                        connection = getLeftConnection(myDirection);
                        maze.setTerrain(bridgeNode,connection, ONE_BRIDGE_PER_SQUARE);
                    }
                    else if( ! leftBridgeFound)
                    {
                        myDirection = robot.getMyDirection();
                        bridgeNode = getBridgeNode(inXDirection, nextNode, stepValue);
                        connection = getRightConnection(myDirection);
                        maze.setTerrain(bridgeNode,connection, ONE_BRIDGE_PER_SQUARE);
                    }//If both left and right bridges are found, we do nothing.
					
                    myLocation = nextNode; //we got to the next node, need to update our position
					
                    if((inXDirection && (myLocation.x != goalPoint.x)) || ( ! inXDirection && (myLocation.y != goalPoint.y)))
                    {
                        if(inXDirection)
                            nextNode.x += stepValue;
                        else
                            nextNode.y += stepValue;
                        robot.moveTicks(DELAY_TICKS,STRAIGHTSPEED);//pause scanning until the robot moves past this point of interest, then resume
                    }
					
                    leftBridgeFound = false;
                    rightBridgeFound = false;
                    haveReversed = false;
                    startTicks = robot.getTicks();
                }
            }
            else if ( ! leftBridgeFound && ( ! (lastError > 5)) && isPotentialLeftBridge(sensors) && withinExpectedTicks(startTicks,2))
            {
                delay(DELAY_TO_CHECK_READING);
                robot.gridSensors.readCalibrated(sensors,QTR_EMITTERS_ON);        
                if(isPotentialLeftBridge(sensors))
                {
                    delay(DELAY_TO_CHECK_ANGLE);//wait then check again, we might have hit a straight line on an angle
                    robot.gridSensors.readCalibrated(sensors,QTR_EMITTERS_ON);
                    if( ! onCross(sensors))
                    {
                        robot.motorStop();//testing
                        delay(1000); //testing
                        leftBridgeFound = true;
                    }
                }
            }
            else if( ! rightBridgeFound && ( ! (lastError < -5)) && isPotentialRightBridge(sensors) && withinExpectedTicks(startTicks,2))
            {
                delay(DELAY_TO_CHECK_READING);
                robot.gridSensors.readCalibrated(sensors,QTR_EMITTERS_ON);            
                if(isPotentialRightBridge(sensors))            
                {
                    delay(DELAY_TO_CHECK_ANGLE);//wait then check again, we might have hit a straight line on an angle
                    robot.gridSensors.readCalibrated(sensors,QTR_EMITTERS_ON);
                    if( ! onCross(sensors))
                    {            
                        robot.motorStop();//testing
                        delay(1000); //testing
                        rightBridgeFound = true;
                    }
                }
            }
            else//need to correct error so that we stay on the line
                robot.motorForward(STRAIGHTSPEED,lastError=robot.errorAdjustment());
        }//end while
        
        robot.motorStop();//reached goal point, need to stop

        if(overTime(startTimeInMillis, ii,NUMPOINTS))
            break;//we're out of time, we need to stop mapping, overTime() has already taken us back to the start!!
            
    }//end for

    //signal we're done to the judges:
    robot.closeClaws();
    delay(100);
    robot.openClaws();

    maze.printMazeOverSerial();//testing, display the map the robot created
}

void loopThroughTestingPointsIndefinitely()
{
/************* Testing *****************/
    myLocation.x = 6;  myLocation.y = 4;

    digitalWrite(PUSHPIN,HIGH);
    digitalWrite(LEDPIN, HIGH);
    while(digitalRead(PUSHPIN) == HIGH)
    {
        delay(1000); 
    }
    digitalWrite(LEDPIN, LOW);

    while(true)
    {
    //first point
        goal.x = 0;  goal.y = 6;
        
        Serial.print(goal.x);
        Serial.print(goal.y);
        Serial.println();
        
        digitalWrite(PUSHPIN,HIGH);
        digitalWrite(LEDPIN, HIGH);
        while(digitalRead(PUSHPIN) == HIGH)
        {
            delay(1000); 
        }
        digitalWrite(LEDPIN, LOW);
        
        moveTo();//goal is global

    //second point    
        goal.x = 6;  goal.y = 0;
        
        Serial.print(goal.x);
        Serial.print(goal.y);
        Serial.println();
        
        digitalWrite(PUSHPIN,HIGH);
        digitalWrite(LEDPIN, HIGH);
        while(digitalRead(PUSHPIN) == HIGH)
        {
            delay(1000); 
        }
        digitalWrite(LEDPIN, LOW);
        
        moveTo();//goal is global

    //third point
        goal.x = 0;  goal.y = 4;
        
        Serial.print(goal.x);
        Serial.print(goal.y);
        Serial.println();
        
        digitalWrite(PUSHPIN,HIGH);
        digitalWrite(LEDPIN, HIGH);
        while(digitalRead(PUSHPIN) == HIGH)
        {
            delay(1000); 
        }
        digitalWrite(LEDPIN, LOW);
        
        moveTo();//goal is global

    //fourth point
        goal.x = 6;  goal.y = 4;
        
        Serial.print(goal.x);
        Serial.print(goal.y);
        Serial.println();
        
        digitalWrite(PUSHPIN,HIGH);
        digitalWrite(LEDPIN, HIGH);
        while(digitalRead(PUSHPIN) == HIGH)
        {
            delay(1000); 
        }
        digitalWrite(LEDPIN, LOW);
        
        moveTo();//goal is global
    }// end while(true)
/************ End testing ***************/
}//end loopThroughTestingPointsIndefinitely()


/****************Waiting for and delivering blocks***************
{
//move to block pickup zone, already facing the right way
robot.movePoints(1);

//  LOOP:
while(true)
{
    while(block has not arrived)
    {
        ////    wait then rescan
    }
    
    delay(WAIT_FOR_GRID_ROBOT_TO_MOVE);
    
    //pick up block:
    robot.movePoints(1);
    servoClose();
    
////     determine type
////    set goal to correct block deposite area
    moveTo();//goal is global, no import needed
    robot.dropBlock();
////backup so as not to disturb the block(not over an UNPASSABLE node!)
//    set goal to block pick up area DON"T ACCIDENTALLY DRIVE THROUGH BLOCK, FACE RIGHT WAY
    goal.x = startPoint.x;  goal.y = startPoint.y;
    moveTo();
}//    END LOOP
}
*/

// turnToPoint turns to orientation. It is to be used so that orientation
// is only one point away from myLocation.
// Code copied from Todd's openDay code and modified (y direction was inversed)
void turnToPoint(Point orientation)
{
  short int xDelta = orientation.x - myLocation.x;
  short int yDelta = orientation.y - myLocation.y;

  if ((xDelta > 0) && (yDelta == 0))
  {
    robot.turnTo(EAST);
  }
  else if ((xDelta < 0) && (yDelta == 0))
  {
    robot.turnTo(WEST);
  }
  else if ((yDelta < 0) && (xDelta == 0))
  {
    robot.turnTo(NORTH);
  }
  else if ((yDelta > 0) && (xDelta == 0))
  {
    robot.turnTo(SOUTH);
  }
  else if ((xDelta > 0) && (yDelta < 0))
  {
    robot.turnTo(NEAST);
  }
  else if ((xDelta < 0) && (yDelta < 0))
  {
    robot.turnTo(NWEST);
  }
  else if ((xDelta > 0) && (yDelta > 0))
  {
    robot.turnTo(SEAST);
  }
  else if ((xDelta < 0) && (yDelta > 0))
  {
    robot.turnTo(SWEST);
  }
}

//generates path and navigates to the Point "goal"
void moveTo()//goal and myLocation are currently global, hence no import
{
    bool useTwoPoints;
//Get path to start:
    myPath = maze.generateRoute(myLocation, goal);
//follow path:
    for (int ii = 0; ii < myPath.length; ii++)
    {
        if((ii != myPath.length - 1))//don't check if this is the final point
        {
            //if either of the coordinates of the target point are an odd number (if the point isn't a cross):
            if((myPath.path[ii].y % 2 != 0) || (myPath.path[ii].x % 2 != 0)) 
            {
                //if the coordintates of the point AFTER the target point are even (if the point AFTER the target point IS a cross)
                //AND there is no change in either the x direction, the y direction or both (we are not going to cross a bridge):
                if(((myPath.path[ii+1].y % 2 == 0) && (myPath.path[ii+1].x % 2 == 0)) && (((myLocation.x - myPath.path[ii].x) == 0) || ((myLocation.y - myPath.path[ii].y) == 0)))
                    ii++; //skip this point (go to the next one)
            }
        }
        turnToPoint(myPath.path[ii]);
        if((myPath.path[ii].y % 2 == 0) && (myPath.path[ii].x % 2 == 0))//if the next point is a cross
            useTwoPoints = true;
        else
            useTwoPoints = false;
        robot.movePoints(1,useTwoPoints);
/*if((robot.getMyDirection() == NEAST) || (robot.getMyDirection() == NWEST) ||(robot.getMyDirection() == SEAST) || (robot.getMyDirection() == SWEST))
{
  delay(300);
  robot.moveTicks(5,STRAIGHTSPEED);
  delay(300);
}*/

//testing stuff
        myLocation = myPath.path[ii];
        Serial.print(myLocation.x);
        Serial.print(myLocation.y);
        Serial.println();
        //showMaze();
//end of testing stuff
    }        
}

//returns stepValue for mapping the maze, used to set where nextNode is
short int getStepValue(short int inPointCoord, short int inMyLocCoord)
{
//stepValue is 2 or -2 because we skip the node on the line; we are just traversing the crosses
    short int stepValue;
    if(inPointCoord > inMyLocCoord)
        stepValue = 2;
    else
        stepValue = -2;
    return stepValue;
}

//returns direction to pass to maze.setTerrain() if there is a connection on our right
Direction getRightConnection(Direction inDir)
{
    Direction connection;
    switch(inDir)
    {
        case NORTH:
            connection = EAST;
            break;
        case SOUTH:
            connection = WEST;
            break;
        case EAST:
            connection = SOUTH;
            break;
        default:
            connection = NORTH;
            break;
    }
    return connection;
}

//returns direction to pass to maze.setTerrain() if there is a connection on our left
Direction getLeftConnection(Direction inDir)
{
    Direction connection;
    switch(inDir)
    {
        case NORTH:
            connection = WEST;
            break;
        case SOUTH:
            connection = EAST;
            break;
        case EAST:
            connection = NORTH;
            break;
        default:
            connection = SOUTH;
            break;
    }
    return connection;
}

//returns bridgeNode in order to set bridge locations 
Point getBridgeNode(bool inXDirection, Point nextNode, int stepValue)
{
    Point bridgeNode;
    if(inXDirection)
    {
        bridgeNode.y = nextNode.y;
        bridgeNode.x = nextNode.x - (stepValue/2);
    }
    else
    {
        bridgeNode.x = nextNode.x;
        bridgeNode.y = nextNode.y - (stepValue/2);
    }
    return bridgeNode;
}

//returns if the sensors might be seeing a bridge on the right
bool isPotentialRightBridge(unsigned int* sensors)
{
    //return true if sensors see: [X| | |O| | | |O]
    return ((sensors[Lvvv] > MIN_BLACK) && ((sensors[vvvR] < MIN_BLACK) && (sensors[vvvL] < MIN_BLACK)));
}

//returns if the sensors might be seeing a bridge on the left
bool isPotentialLeftBridge(unsigned int* sensors)
{
    //return true if sensors see: [O| | | |O| | |X]
    return ((sensors[vvvR] > MIN_BLACK) && ((sensors[Lvvv] < MIN_BLACK) && (sensors[Rvvv] < MIN_BLACK)));
}

//returns true if sensors are [O|O| | | | |O|O]
bool onCross(unsigned int* sensors)
{
    return ((sensors[Lvvv] < MIN_BLACK) && (sensors[vLvv] < MIN_BLACK) && (sensors[vvRv] < MIN_BLACK) && (sensors[vvvR] < MIN_BLACK));
}

bool isUnpassable()
{
    unsigned int sensors[8];//holds sensor info to see where the line is
    robot.gridSensors.readCalibrated(sensors,QTR_EMITTERS_ON);
    if((sensors[vvvL] > MIN_BLACKISH) || (sensors[Rvvv] > MIN_BLACKISH) || (sensors[vvLv] > MIN_BLACKISH) || (sensors[vRvv] > MIN_BLACKISH))
        return true;
    else
        return false;
}

bool overTime(unsigned long startTimeInMillis, int loopCount, const int NUMPOINTS)
{
    unsigned long currentTimeInMillis = millis();
        const long MAX_TIME_TO_MAP = 270000;
    if(((currentTimeInMillis - startTimeInMillis) > MAX_TIME_TO_MAP) && (loopCount > (NUMPOINTS - 3)))//if there are less than 30 seconds left AND we are not heading to the last node already
    {
        goal = startPoint;
        moveTo();//goal is global, no import needed
        return true;
    }
    else
        return false;
}

bool withinExpectedTicks(int startTicks, int scale)
{
    //return (((robot.getTicks() - startTicks) < (MAX_EXPECTED_TICKS / scale)) && ((robot.getTicks() - startTicks) > (MIN_EXPECTED_TICKS / scale)));
	return true;
}

//go backwards while adjusting for error, untested may not work does the error need to be negative? do the ticks go down when moving backwards? do we need a delay?
void reverse()
{
    while(robot.getTicks() > (EXPECTED_TICKS/2))
    {
        robot.motorBackward(APPROACHSPEED,robot.errorAdjustment());
    }
}


