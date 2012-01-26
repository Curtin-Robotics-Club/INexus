/* 
 * The Robot class is here to represent what a robot can do.
 * A robot is designed to have three IR Transceivers, two wheels, two grippers and two QTR sensors.
 * The IR transcievers are to detect blocks.
 * One QTR sensor is to keep the robot on a grid line.
 * One QTR sensor is to determine if a block is black or white.
 *
 * The original code was written by James Lumsden, Yusuf Syaid, & Mike Hamer.
 * Tidied, converted into OO and continued by Todd Hurst & Zachary Oliver.
 * Started 10:22 10/12/2010
 * Last Updated 12:42 10/08/2011
 */

#include "Robot.h"

Robot::Robot(const PololuQTRSensorsRC createGridSensors,
    const PololuQTRSensorsRC createBlockSensors,
    const PololuWheelEncoders createWheelEnc)
{
  //int restarted = digitalRead(40);   //Restart switch
  //if (restarted == HIGH)
  //{
  //  myDirection = NORTH;
  //}
  //else    
  //{
  //  myDirection = WEST;
  //}

  // For the open day, we always want the direction to start NORTH.
  myDirection = NORTH;

  gridSensors = createGridSensors;
  blockSensors = createBlockSensors;
  wheelEnc = createWheelEnc;
  wheelEnc.init(8,9,11,10); //swap second order for +ve forward

  lastError = 0;
  errorSum = 0;

  // MAKE SURE WE TURN errorAdjust ON!
  errorAdjust = true;

}

bool Robot::querySensors(int sensorConfig[8])
{
	unsigned int sensors[8];
    gridSensors.readCalibrated(sensors, QTR_EMITTERS_ON);
	for(int ii = 0; ii < 8; ii++)
	{
		if((sensorConfig[ii] == W) && (sensors[ii] > SENSORBLACK))
			return false;
		if((sensorConfig[ii] == B) && (sensors[ii] < SENSORBLACK))
			return false;
	}
	return true;
}


// turnTo will turn a robot to the direction we give it here.
void Robot::turnTo(enum Direction newDirection)
{
//this speeds up either the left or right motor when turning so the robot lines up better orthogonally
//changes to 1 for 360 degree turn
  int adjustMotors = 2;
  wheelEnc.getCountsAndResetM1();
  wheelEnc.getCountsAndResetM2();

  // Get the relative direction to turn to based apon myDirection being north.
  Direction relativeDirection = (enum Direction)modulo((newDirection - myDirection), 8);

  // Then decide the number of ticks to turn.
  int ticks = 0;
  switch (relativeDirection)
  {
    case SOUTH:
      ticks = 4*TURNANGLE360;//need the right ticks for 360 degrees
      adjustMotors = 1;//both motors going the same speed works fine for this
      break;

    case SEAST:
      ticks = 3*TURNANGLE;
      break;

    case EAST:
      ticks = 2*TURNANGLE;
      break;

    case NEAST:
      ticks = TURNANGLE;
      break;

    case NORTH:
      break;

    case SWEST:
      ticks = -3*TURNANGLE;
      break;

    case WEST:
      ticks = -2*TURNANGLE;
      break;

    case NWEST:
      ticks = -TURNANGLE;
      break;
  }

  // Start the motor.
  // If it's positive, turn right.
  if (ticks < 0)
  {
    MotorControl.write(0x88);//right forward
    MotorControl.write(TURNSPEED*adjustMotors);
    MotorControl.write(0x8E);//left back
    MotorControl.write(TURNSPEED); 
  }
  // If it's negative, turn left.
  else if (ticks > 0)
  {
    MotorControl.write(0x8A);//right back
    MotorControl.write(TURNSPEED);
    MotorControl.write(0x8C);//left forward
    MotorControl.write(TURNSPEED*adjustMotors);
  }
  // If it's equal, don't bother.

  bool turnComplete = false;
  int motorOne;
  int motorTwo;
  int motorOneSpeed = TURNSPEED;
  int motorTwoSpeed = TURNSPEED;

  // Turn till we've turned the ticks or until our gridSensor is on straight on line.
  // We want to turn at least half the number of ticks before we check for the grid.
  while (!turnComplete)
  {
    motorOne = abs(wheelEnc.getCountsM1());
    motorTwo = abs(wheelEnc.getCountsM2());

    // Here we are only using ticks in errorTickAdjustment to get a value as we only use ticks
    int error = errorTickAdjustment();

    // Calculate the speed that we should turn based on error.
    motorOneSpeed = TURNSPEED + error;
    motorTwoSpeed = TURNSPEED - error;

    // Slow down by a bit towards the end.
    if (motorOne > (abs(ticks) - 10) || motorTwo > (abs(ticks) - 10))
    {
      motorOneSpeed -= 20;
      motorTwoSpeed -= 20;
    }

    /*
    // If we've hit the number of ticks on either motor, or we've hit the 
    // grid line stop turning (after turning at least half the number of ticks).
    unsigned int sensors[8];
    gridSensors.readLine(sensors, QTR_EMITTERS_ON, true);
    if (((motorTwo > (ticks * 2 / 3)) || (motorOne > (ticks * 2 / 3))) 
        && ((newDirection == NORTH) || (newDirection == SOUTH)
          || (newDirection == EAST) || (newDirection == WEST))
        && ((sensors[3] < SENSORBLACK) || (sensors[4] < SENSORBLACK)
          || (sensors[2] < SENSORBLACK) || (sensors[5] < SENSORBLACK)))
    {
      motorStop();
      turnComplete = true;
    }
    else if (motorTwo > abs(ticks) || motorOne > abs(ticks))
*/
    if (motorTwo > abs(ticks) || motorOne > abs(ticks))
    {
      motorStop();
      turnComplete = true;    
    }
    // Otherwise, continue turning based on the calculated error speed.
    else
    {
      if (ticks < 0)
      {
        MotorControl.write(0x88);
        MotorControl.write(limit(motorOneSpeed*adjustMotors));
        MotorControl.write(0x8E);
        MotorControl.write(limit(motorTwoSpeed));
      }
      else
      {
        MotorControl.write(0x8A);
        MotorControl.write(limit(motorOneSpeed));
        MotorControl.write(0x8C);   
        MotorControl.write(limit(motorTwoSpeed*adjustMotors));
      }
    } 
    delay(1);
  }  
  wheelEnc.getCountsAndResetM1();
  wheelEnc.getCountsAndResetM2();
  // We are now facing 'newDirection'
  myDirection = newDirection;

  delay(100);
}

void Robot::setMyDirection(enum Direction newDirection)
{
  myDirection = newDirection;
}

// getMyDirection returns the direction this robot is facing.
enum Direction Robot::getMyDirection()
{
  return myDirection;
}

// movePoints has the robot move the number of points told.
// Positive is forward, negative is backwards.
// Slowing down does not happen until we have moved the number of points told.
// If you want to be safe, move one point. Todd suggests you take a risk.
void Robot::movePoints(int numberOfPoints, bool useBothSensors)
{
  bool forward = true;
  // Move the number of points. We're not going to slow down here.
  if (numberOfPoints < 0)
  {
    numberOfPoints = -numberOfPoints;
    forward = false;
  }

  for (int i = 0; i < numberOfPoints; i++)
  {
    switch (myDirection)
    {
      case NEAST:
      case SEAST:
      case NWEST:
      case SWEST:
        if (forward)
        {
          moveTicks(DIAGONALTICKS, STRAIGHTSPEED);
        }
        else
        {
          moveTicks(-DIAGONALTICKS, STRAIGHTSPEED);
        }
        break;

      default:
        if (forward)
        {
          moveTicks(ORTHOGONALTICKS, STRAIGHTSPEED);
        }
        else
        {
          moveTicks(-ORTHOGONALTICKS, STRAIGHTSPEED);
        }
        break;
    }
    moveTillPoint(STRAIGHTSPEED, forward, useBothSensors);
  }
  stopMoving(STRAIGHTSPEED);

  unsigned int sensors[8];
  gridSensors.readCalibrated(sensors, QTR_EMITTERS_ON);
  delay(20);

  // TODO WE NEED TO LOOK INTO THIS WAY MORE
  // This code is being written assuming that our gridSensors have the right most sensor
  // at sensor[0] and the left most sensor at sensor[8]  

  // Turn the error adjust off. 
  //errorAdjust = false;
/*  switch (myDirection)
  {
    // If we're facing north east
    case NEAST:
      // If our right sensor is on the line, but left is not.
      if ((sensors[0] < SENSORBLACK) && (sensors[7] > SENSORBLACK))
      {
        // Then we turn north.
//        turnTo(NORTH);
//        delay(10);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving
        stopMoving(APPROACHSPEED);
//        delay(10);
        // Turn east.
//        turnTo(EAST);
//        delay(10);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving.
        stopMoving(APPROACHSPEED);
        delay(10);
      }
      // If our left sensor is on the line, but right is not.
      else if ((sensors[0] > SENSORBLACK) && (sensors[7] < SENSORBLACK))
      {
        // Then we turn east.
        turnTo(EAST);
        delay(10);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving
        stopMoving(APPROACHSPEED);
        delay(10);
        // Turn north.
        turnTo(NORTH);
        delay(10);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving.
        stopMoving(APPROACHSPEED);
        delay(10);
      }
      // If both sensors are on the line.
      else
      {
        // Move forward a few ticks
        moveTicks(AFEW, APPROACHSPEED);
        stopMoving(APPROACHSPEED);
      }
      break;
    case SEAST:
      // If our right sensor is on the line, but left is not.
      if ((sensors[0] < SENSORBLACK) && (sensors[7] > SENSORBLACK))
      {
        // Then we turn south.
        turnTo(SOUTH);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving
        stopMoving(APPROACHSPEED);
        // Turn east.
        turnTo(EAST);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving.
        stopMoving(APPROACHSPEED);
      }
      // If our left sensor is on the line, but right
      // is not.
      else if ((sensors[0] > SENSORBLACK) && (sensors[7] < SENSORBLACK))
      {
        // Then we turn east.
        turnTo(EAST);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving
        stopMoving(APPROACHSPEED);
        // Turn south.
        turnTo(SOUTH);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving.
        stopMoving(APPROACHSPEED);
      }
      // If both sensors are on the line.
      else
      {
        // Move forward a few ticks
        moveTicks(AFEW, APPROACHSPEED);
        stopMoving(APPROACHSPEED);
      }    
      break;
    case NWEST:
      // If our right sensor is on the line, but left is not.
      if ((sensors[0] < SENSORBLACK) && (sensors[7] > SENSORBLACK))
      {
        // Then we turn north.
        turnTo(NORTH);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving
        stopMoving(APPROACHSPEED);
        // Turn west.
        turnTo(WEST);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving.
        stopMoving(APPROACHSPEED);
      }
      // If our left sensor is on the line, but right is not.
      else if ((sensors[0] > SENSORBLACK) && (sensors[7] < SENSORBLACK))
      {
        // Then we turn west.
        turnTo(WEST);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving
        stopMoving(APPROACHSPEED);
        // Turn north.
        turnTo(NORTH);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving.
        stopMoving(APPROACHSPEED);
      }
      // If both sensors are on the line.
      else
      {
        // Move forward a few ticks
        moveTicks(AFEW, APPROACHSPEED);
        stopMoving(APPROACHSPEED);
      }    
      break;
    case SWEST:
      // If our right sensor is on the line, but left is not.
      if ((sensors[0] < SENSORBLACK) && (sensors[7] > SENSORBLACK))
      {
        // Then we turn south.
        turnTo(SOUTH);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving
        stopMoving(APPROACHSPEED);
        // Turn west.
        turnTo(WEST);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving.
        stopMoving(APPROACHSPEED);
      }
      // If our left sensor is on the line, but right is not.
      else if ((sensors[0] > SENSORBLACK) && (sensors[7] < SENSORBLACK))
      {
        // Then we turn east.
        turnTo(WEST);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving
        stopMoving(APPROACHSPEED);
        // Turn north.
        turnTo(SOUTH);
        // Move till we hit the line.
        moveTillPoint(APPROACHSPEED, forward, useBothSensors);
        // Stop Moving.
        stopMoving(APPROACHSPEED);
      }
      // If both sensors are on the line.
      else
      {
        // Move forward a few ticks
        moveTicks(AFEW, APPROACHSPEED);
        stopMoving(APPROACHSPEED);
      }    
      break;
  }
  // Turn the error adjust back on.
  errorAdjust = true;*/
}

// motorForward sets the motor to move forward at the motorSpeed given.
void Robot::motorForward(int motorSpeed, int error)
{
  MotorControl.write(0x88);
  MotorControl.write(limit(motorSpeed + error));
  MotorControl.write(0x8C);
  MotorControl.write(limit(motorSpeed - error));

  // If we stop, we need to remove the results.
  if (motorSpeed == 0)
  {
    lastError = 0;
    errorSum = 0;
  }
}

// motorBackward sets the motor to move forward at the motorSpeed given.
void Robot::motorBackward(int motorSpeed, int error)
{
  MotorControl.write(0x8A);
  MotorControl.write(limit(motorSpeed + error));
  MotorControl.write(0x8E);
  MotorControl.write(limit(motorSpeed - error));

  // If we stop, we need to remove the results.
  if (motorSpeed == 0)
  {
    lastError = 0;
    errorSum = 0;
  }
}

// motorStop sets the motors to stop.
void Robot::motorStop()
{
  int zero = 0;
  MotorControl.write(0x88);
  MotorControl.write(zero);
  MotorControl.write(0x8E);
  MotorControl.write(zero);
}

// moveTillPoint moves till we hit the next point.
// moteTillPoint doesn't stop. Use stopMoving() to do so.

// TODO at the moment, this only works well for NORTH, SOUTH, EAST, WEST
// We'll try to sort it out for NEAST, SEAST, NWEST, SWEST later.
void Robot::moveTillPoint(int motorSpeed, bool forward, bool useBothSensors)
{
	wheelEnc.getCountsAndResetM1();
	wheelEnc.getCountsAndResetM2();

	//Moves forward or backward based on the boolean 'forward' until a line intersection.
	//unsigned int sensors[8];
	//gridSensors.readLine(sensors, QTR_EMITTERS_ON, true);
	int error;
	//bool atPoint = false;
	while(!reachedPoint(useBothSensors)/*!atPoint*/)
	{
	/*	gridSensors.readLine(sensors, QTR_EMITTERS_ON, true);
		if(reachedPoint(sensors, useBothSensors))
		{
			delay(5);
			gridSensors.readLine(sensors, QTR_EMITTERS_ON, true);
			if(reachedPoint(sensors, useBothSensors))
				atPoint = true;
		}*/
		error = errorAdjustment();
		if (forward)
		    motorForward(motorSpeed, error); 
		else
		    motorBackward(motorSpeed, error);
    }
    if(!useBothSensors)
      moveTicks(5,STRAIGHTSPEED);
}

bool Robot::reachedPoint(/*unsigned short int* sensors,*/bool useBothSensors)
{
	int leftSensorsConfig[8] = {W,W,W,W,W,N,N,N};
	int rightSensorsConfig[8] = {N,N,N,W,W,W,W,W};
	int crossConfig[8] = {W,N,N,N,N,N,N,W};
        int leftMidpointConfig[8] = {W,W,W,N,N,N,N,N};
        int rightMidpointConfig[8] = {N,N,N,N,N,W,W,W};
	if(((!useBothSensors && ((myDirection == NEAST) || (myDirection == SEAST) || (myDirection == NWEST) || (myDirection == SWEST))) && ((querySensors(leftSensorsConfig)) || (querySensors(rightSensorsConfig)))) || (useBothSensors && querySensors(crossConfig)) || (!useBothSensors && (querySensors(leftMidpointConfig) || querySensors(rightMidpointConfig))))
	{
		delay(5);
                int motorOne = abs(wheelEnc.getCountsM1());
                int motorTwo = abs(wheelEnc.getCountsM2());
                int averageTicks = (motorOne + motorTwo)/2;
		return ((averageTicks > 15) && (((!useBothSensors && ((myDirection == NEAST) || (myDirection == SEAST) || (myDirection == NWEST) || (myDirection == SWEST))) && ((querySensors(leftSensorsConfig)) || (querySensors(rightSensorsConfig)))) || (useBothSensors && querySensors(crossConfig)) || (!useBothSensors && (querySensors(leftMidpointConfig) || querySensors(rightMidpointConfig)))));
	}

}

// moveTicks moves the number of ticks given.
// A positive ticks number will go forward, a negative ticks number 
// will go backwards.
// We do not stop after hitting the number of ticks. Call stopMoving() to do so.
void Robot::moveTicks(int ticks, int motorSpeed)
{

  wheelEnc.getCountsAndResetM1();
  wheelEnc.getCountsAndResetM2();
  bool moveComplete = false;
  while(!moveComplete)
  {    
    int motorOne = abs(wheelEnc.getCountsM1());
    int motorTwo = abs(wheelEnc.getCountsM2());
    int error = errorAdjustment();

    if(motorTwo > abs(ticks) || motorOne > abs(ticks))
    {
      moveComplete = true;
    }
    else
    {
      if(ticks > 0)
      {
        // motorForward(adjustedMotorSpeed, error);
        motorForward(motorSpeed, error);
      }
      else
      {
        // motorBackward(adjustedMotorSpeed, error);
        motorBackward(motorSpeed, error);
      }
    } 
    delay(1);   
  }  
}

// stopMoving moves a short distance of 10 ticks to slow down, then stops.
// It is to be called whenever we want to stop.
// The movement errors used for adjustment are reset here.
// Use the motorSpeed that we would be travelling before stopping.
// This should be called after moveTicks() and moveTillPoint()
void Robot::stopMoving(int motorSpeed, bool backwards)
{
  // Just move 10 ticks to slow down first.
  if (backwards)
  {
    moveTicks(-5, motorSpeed - 20);
  }
  else
  {
    moveTicks(5, motorSpeed - 20);
  }
  motorStop();
  lastError = 0;
  errorSum = 0;
}

// modulo actually does what a modulo should do.
// Damn coders think that -a % b should be a negative value.
// It should only be in the range of [0,(b-1)].
int Robot::modulo(int value, int moduloBy)
{
  int temp = value % moduloBy;
  if (temp < 0)
  {
    temp = moduloBy + temp;
  }

  return temp;
}

// We make a limit of what a value can be.
// Limits it between 0 and 127.
int Robot::limit(int val)
{
  if (val < 0)
  {
    val = 0;
  }
  else if (val > 127)
  {
    val = 127; 
  }  

  return val;
}

// errorTickAdjustment returns a value used to adjust the movement speed
// of wheels based on the number of ticks seen by each motor.
// It will be between -5 and 5
int Robot::errorTickAdjustment()
{
  int motorOne = abs(wheelEnc.getCountsM1());
  int motorTwo = abs(wheelEnc.getCountsM2());
  int error = motorOne - motorTwo;

  error = constrain(error,-5,5);
  return error;
}

// errorSensorAdjustment returns a value used to adjust the movement speed
// of wheels based on the position of the sensors over the line.
// It will be between -25 and 25
int Robot::errorSensorAdjustment()
{
//3000 = maximum error
  unsigned int sensors[8];
  gridSensors.readCalibrated(sensors,QTR_EMITTERS_ON);
  int position = gridSensors.readLine(sensors, QTR_EMITTERS_ON, true);
  int error = position - 3000;//position = 0 to 6000 or so, give us -3000 to 3000
  error = ((15*error)/3000)-2;//-2 is to shift the robot a bit because the calculation is a little off (gives about -14 to 16), brackets are important cos it's using ints
  if((error<5)&&(error>-5))//amplify the error if it's too low
      error = error*2;
  error = constrain(error,-15,15);
  return error;
}

// errorAdjustment returns a value used to adjust the movement speed
// of wheels using both errorTickAdjustment and errorSensorAdjustment.
int Robot::errorAdjustment(bool onlyTick)
{
//now the errorSensorAdjustment() is fixed we might use it for diagonals, we'll see
  int tickError = errorTickAdjustment();
  int sensorError = errorSensorAdjustment();

  int error = 0;

  // If we only want to use the ticks for error adjustment (say for turning)
  if (onlyTick)
  {
    error = tickError;
  }
  else
  {
    switch (myDirection)
    {
      // If we're going diagonally, we only want to use the tickError.
      case NEAST:
      case SEAST:
      case NWEST:
      case SWEST:
        //error = tickError;
        error = sensorError + tickError;
        break;
        // Every time else, we want to use both tickError and position
      default:
        error = sensorError + tickError;
        break;
    }
  }
  return error;
}

void Robot::openClaws()
{
  servo(SERVO_RIGHT, OPENRIGHTANGLE);
  servo(SERVO_LEFT, OPENLEFTANGLE);
}

void Robot::calibrateBlock()
{
  digitalWrite(30, LOW);    //Enable colour detector
  digitalWrite(32, HIGH);    

  for (int i = 0; i < 25; i++)
  {
    blockSensors.calibrate();
    delay(20);
  }

  digitalWrite(32, LOW);    //Enable colour detector
  digitalWrite(30, HIGH);    

}

void Robot::calibrateGrid()
{
  int i = 0;
  for(i = 0; i < 25; i++)
  {
    gridSensors.calibrate();
    delay(20); 
  }

  turnTo(NWEST);
  delay(500);
  moveTicks(AFEW, APPROACHSPEED);
  stopMoving(APPROACHSPEED);

  for(i = 0; i < 25; i++)
  {
    gridSensors.calibrate();
    delay(20); 
  }

  moveTicks(-AFEW, APPROACHSPEED);
  stopMoving(APPROACHSPEED, true);
  delay(500);
  turnTo(NORTH);
  delay(500);
}

void Robot::gridSensorInfo()
{
  unsigned int sensors[8];
  gridSensors.readCalibrated(sensors, QTR_EMITTERS_ON);
  Serial.println("INFO");
  Serial.println(sensors[0]);
  Serial.println(sensors[1]);
  Serial.println(sensors[2]);
  Serial.println(sensors[3]);
  Serial.println(sensors[4]);
  Serial.println(sensors[5]);
  Serial.println(sensors[6]);
  Serial.println(sensors[7]);
  Serial.println("FIN");
  Serial.println("");
}

// TODO: Re-read and maybe re-code to find out what is going on.
/*
bool Robot::grabBlock(int *nextColour, bool *firstBlock, bool noColour)
{
  // Open grippers
  // Move to block
  // checkColour()
  // Grab or not grab
  // Back away
  int block_colour;
  int centre_range;
  int ticks_moved = 0;
  bool haveBlock;

  openClaws();

  unsigned int sensors[8];
  bool atPoint = false;

  centre_range = analogRead(IR_CENTRE);   //Move close to block
  while ((centre_range < 500) && !atPoint) 
  {
    moveTicks(2, 30);  
    ticks_moved = ticks_moved + wheelEnc.getCountsM1();
    centre_range = analogRead(IR_CENTRE);
    gridSensors.readCalibrated(sensors, QTR_EMITTERS_ON);
    if ((sensors[0] < SENSORBLACK) && (sensors[7] < SENSORBLACK) && ticks_moved >10)
    {
      atPoint = true;
    }
  }
  motorStop();
  delay(200);  //Comes to a stop b4 colour checking.
  block_colour = checkColour();

  if (block_colour == 2)
  {
    moveTicks(5, 30);
    motorStop();
    delay(200);
    block_colour = checkColour();
    ticks_moved = ticks_moved + wheelEnc.getCountsM1();  
  }

  if ((block_colour == *nextColour) || *firstBlock || noColour) 
  {
    closeClaws();
    delay(400);
    *firstBlock = false;
    if (block_colour == 1) 
    {
      *nextColour = 0;
    }
    else 
    {
      *nextColour = 1;
    }
    haveBlock = true;
  }
  else
  {
    haveBlock = false;
  }
  ticks_moved+=5;
  moveTicks(-ticks_moved, 30);
  motorStop();

  return haveBlock;
}
*/

// TODO: This function may need to be re-coded because it is too specific for
// the code we're on. It might be best to have something like this in the AI part.
// It might be best to just have openClaws and closeClaws.
void Robot::dropBlock()
{
  unsigned int sensors[8];
  bool atPoint = false;
  bool delivered = false;
  int ticks_moved=0;

  turnTo(SOUTH);
  movePoints(1);
  moveTicks(60, 25);
  stopMoving(25);
  openClaws();
  delay(300);
  moveTicks(-50, 25);
  stopMoving(25, true);
  delay(300);
  turnTo(NORTH);
  delay(300);
  movePoints(1);

  delay(500);


  /*while (!delivered)
    {
    ticks_moved = 0;
    while ((ticks_moved < 70) && !atPoint)
    {
  //Go Forward
  moveTicks(2, 25);  
  ticks_moved = ticks_moved + wheelEnc.getCountsM1();
  gridSensors.readCalibrated(sensors, QTR_EMITTERS_ON);

  if ((sensors[0] < SENSORBLACK) && (sensors[7] < SENSORBLACK) && (ticks_moved >10))
  {
  motorStop();
  delay(2000);
  atPoint = true;
  servo(SERVO_RIGHT,30);  //Check required values.
  servo(SERVO_LEFT,80);
  moveTicks(-ticks_moved, 20);
  delivered = true;
  }

  }
  motorStop();
  delay(2000);
  atPoint = false;
  moveTicks(-ticks_moved, 20);


  }*/
}

// This closes the claw;
void Robot::closeClaws()
{
  servo(SERVO_RIGHT, CLOSERIGHTANGLE);
  servo(SERVO_LEFT, CLOSELEFTANGLE); 
}

// TODO: Read through this to find out what is going on.
// I would like this re-coded because it does not fit the coding standard.
// I would also like this to return a struct for left, right, centre rather
// than asking for left, right, centre.
int Robot::blockScan(char dir)
{
  openClaws();
  int block = 0;
  int temp_left[20];     
  int temp_right[20];
  int temp_centre[20];

  if (dir == 'l')
  {
    //scan left sensor
    for (int i=0; i<20; i++)
    {
      temp_left[i] = analogRead(IR_LEFT);
      delay(4);
    }
    int left_ir ;
    left_ir = (int)MeanNoOutliers(temp_left, 20);
    if ((left_ir > 400) && (left_ir < 540))
    {
      block = 1;
    }
    else if ((left_ir > 200) && (left_ir < SENSORBLACK))
    {
      block = 2;
    }
    else
    {
      block = 0;
    }
  }
  else if (dir == 'c')
  {
    //scan centre sensor
    for (int i=0; i<20; i++)
    {
      temp_centre[i] = analogRead(IR_CENTRE);
      delay(4);
    }
    int centre_ir = (int)MeanNoOutliers(temp_centre, 20);
    if ((centre_ir > 100) /*&& (centre_ir < 180)*/)
    {
      block = 1;
    }
    /*  else if ((centre_ir > 45) && (centre_ir < 60))
        {
        block = 2;
        }*/
    else
    {
      block = 0;
    }
  }
  else
  {
    //scan right sensor
    for (int i=0; i<20; i++)
    {
      temp_right[i] = analogRead(IR_RIGHT);
      delay(4);
    }
    int right_ir = (int)MeanNoOutliers(temp_right, 20);
    if ((right_ir > 400) && (right_ir < 540))
    {
      block = 1;
    }
    else if ((right_ir > 200) && (right_ir < SENSORBLACK))
    {
      block = 2;
    }
    else
    {
      block = 0;
    }
  }
  return block;
}

// TODO Read through this and determine what it is doing and why it's here.
float Robot::MeanNoOutliers(int* data, int len)
{
  int i;
  int temp;
  float rawmean;
  float stdev;
  float newmean  = 0;
  float newcount = 0;

  for (i = 0; i < len; i++)
  {
    temp = data[i] + temp;
  }
  rawmean = temp/len;
  for(i=0; i<len; i++) {stdev+=(data[i]-rawmean)*(data[i]-rawmean)/len;}
  stdev = pow(stdev, 0.5);
  for(i=0; i<len; i++)
  {
    if (rawmean-stdev<data[i] && data[i]<rawmean+stdev)
    {
      newmean+=data[i];
      newcount++;
    }
  }
  newmean=(newcount>0?newmean/newcount:rawmean);
  return newmean;
}

void Robot::servo(int num, int pos)
{
  ServoControl.write(0x80);
  ServoControl.write(0x01);
  ServoControl.write(0x02); //set position, 1 byte
  ServoControl.write(num);
  ServoControl.write(pos);
}

void Robot::talkSend(String info)
{
  info = info.concat("|");
  XBee.print(info);
}

// TODO read through this and find out why it's got the 'concat('|')'.
String Robot::talkHear()
{
  String info;
  char temp;
  while (XBee.available() > 0)
  {
    temp = XBee.read();
    info = info.concat(temp);
  }

  return info;  
}

int Robot::getTicks()
{
    int motorOne = abs(wheelEnc.getCountsM1());
    int motorTwo = abs(wheelEnc.getCountsM2());
	return (motorOne + motorTwo)/2;
}

// TODO Probably re-code this so that it's standard with the rest of the code.
// Looks like it works pretty well though.
int Robot::checkColour()
{
  //Disable line sensor, enable colour sensor
  //Check its colour
  //Disable colour sensor, enable line sensor
  //Return its colour

  int block_colour;   //0 is white, 1 is black, 2 is error

  digitalWrite(30, LOW);    //Enable colour detector
  digitalWrite(32, HIGH);

  unsigned int sensors[8];
  delay(20);
  blockSensors.read(sensors, QTR_EMITTERS_ON);    //Read values from colour sensor



  // If the bottom sensor is white and top sensor is black
  // We have a black block
  if ((sensors[0] < 3900) && (sensors[6] > 3900))
  {
    block_colour = 1;
  }
  // If both sensors are white, we have white block
  else if ((sensors[0] < 3900) && (sensors[6] < 3900))
  {
    block_colour = 0;
  }
  // For anything else, we have an error
  else
  {
    block_colour = 2;
  }

  digitalWrite(32, LOW);    //Enable line detector again
  digitalWrite(30, HIGH);  
  return block_colour;
}
