/*
 * The Robot class is here to represent what a robot can do.
 * A robot is designed to have three IR Transceivers, two wheels, two grippers
 * and two QTR sensors.
 * The IR transcievers are to detect blocks.
 * One QTR sensor is to keep the robot on a grid line.
 * One QTR sensor is to determine if a block is black or white.
 *
 * The original code was written by James Lumsden, Yusuf Syaid, & Mike Hamer.
 * Tidied, converted into OO and continued by Todd Hurst & Zachary Oliver.
 * Started 10:22 10/12/2010
 * Last Updated 11:34 29/12/2010
 */


#ifndef Robot_h
#define Robot_h

#include <WProgram.h>
#include <PololuQTRSensors.h>
#include <PololuWheelEncoders.h>
#include "struct.h"

#define MotorControl Serial1
#define ServoControl Serial2
#define XBee Serial3

#define TURNSPEED 40  
#define STRAIGHTSPEED 30  
#define APPROACHSPEED 30  
#define GRABSPEED 30  
#define TURNANGLE 24
#define TURNANGLE360 19
#define TURNADJUST 1

// If the sensor returns a value geater than SENSORBLACK then we know 
// that what we're looking at is black.
#define SENSORBLACK 250

// The number of ticks here is about half the distance from one point to
// another. The reasoning here is that we will want to travel an estimate by
// ticks then continue till we hit the point by gridSensor.
#define DIAGONALTICKS 20//40//60
#define ORTHOGONALTICKS 20//40
#define AFEW 30

#define SERVO_LEFT 0
#define SERVO_RIGHT 1
#define OPENRIGHTANGLE 50
#define OPENLEFTANGLE 70
#define CLOSERIGHTANGLE 0//30, set 0 for demonstration 
#define CLOSELEFTANGLE 60//90, set 60 for demonstration

#define IR_LEFT 0
#define IR_CENTRE 1
#define IR_RIGHT 2

#define W 0
#define B 1
#define N -1

class Robot
{
  public:
    // Constructor
    Robot(PololuQTRSensorsRC createGridSensors,
        PololuQTRSensorsRC createBlockSensors,
        PololuWheelEncoders createEnc);

	//Query if sensors are in configuration sensorConfig, W = white, B = black, N = neither
	bool querySensors(int sensorConfig[8]);	
	
	//uses querySensors to detect if a point has been reached in moveTillPoint
	bool reachedPoint(bool useBothPoints);
	
    // turnTo will turn a robot to the direction we give it here.
    void turnTo(enum Direction newDirection);

    // getMyDirection returns the direction this robot is facing.
    enum Direction getMyDirection();

    // setMyDirection sets the direction of the robot without turning.
    void setMyDirection(enum Direction newDirection);

    // movePoints has the robot move the number of points told.
    // Positive is forward, negative is backwards.
    void movePoints(int numberOfPoints, bool useBothSensors = true);

    // Here we open the servo claws.
    void openClaws();
    
    // Here we close the servo claws.
    void closeClaws();
    
    // Here we calibrate the grid QTR.
    void calibrateGrid();

    // Here we calibrate the block QTR.
    void calibrateBlock();

    // Stop moving after slowing down a little.
    // Set 'backwards' to true when stopping from moving backwards.
    void stopMoving(int motorSpeed, bool backwards = false);

    //TODO THIS IS HERE TO GET INFO FOR DEBUGGING
    void gridSensorInfo();

    // This will approach a block, determine its colour and return 'mine or theirs' and then grab it or not grab it.
    //TODO This needs to be re-read and maybe re-coded to find out what is going on.
    //bool grabBlock(int *nextColour, bool *firstBlock, bool noColour = false);

    // This will attempt to drop a block
    void dropBlock();

    // This will scan the grid in direction specified, return if block.
    int blockScan(char dir);

    // This will send info by Serial
    void talkSend(String info);

    // This will receive info from serial
    String talkHear(void);

    void servoClose(void);

    // Move the number of ticks. We do not stop when we have hit the ticks.
    void moveTicks(int ticks, int motorSpeed);

    // Move till we hit the next white line. We do not stop when we have hit the line.
    void moveTillPoint(int motorSpeed, bool forward = true, bool useBothSensors = true);

    // Have the motor move forward.
    void motorForward(int motorSpeed, int error);
    // Have the motor move backward.
    void motorBackward(int motorSpeed, int error);
    // Have the motor stop.
    void motorStop();

    // Here we use both errorTickAdjustment() and errorSensorAdjustment()
    // to return a value used to adjust the movement speed of wheels.
    int errorAdjustment(bool onlyTick=false);
    
    //should be private, need wrapper
    PololuQTRSensorsRC gridSensors;
    
        // Here we return a value used to adjust the movement speed of wheels
    // based on the positioning of the robot on a line.
    int errorSensorAdjustment();
    
	//returns the average of the ticks on both wheels
	int getTicks();
	
  private:
    enum Direction myDirection;

    // Pololu Objects for sensors and wheels.

    PololuQTRSensorsRC blockSensors;
    PololuWheelEncoders wheelEnc;

    // This method is here to do an actual modulo rather than a '%'.
    int modulo(int value, int moduloBy);
    // Limits val between 0 and 127.
    int limit(int val);

    // These two are used when calculating an error.
    int lastError;
    int errorSum;


    // Will check the block colour, return black or white
    int checkColour();

    // Changes servo positions
    void servo(int num, int pos);


    float MeanNoOutliers(int*, int);

    // This varible is here to determine if we adjust for errors.
    // Todd Says "BE CAREFUL WHEN USING THIS!" Very Loudly.
    bool errorAdjust;
    // Here we return a value used to adjust the movement speed of wheels 
    // based on the number of ticks seen by each motor.
    int errorTickAdjustment();

  protected:
};

#endif

