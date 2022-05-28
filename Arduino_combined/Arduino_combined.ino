//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Updated by Allison Okamura 1.17.2018
//--------------------------------------------------------------------------
// Parameters that define what environment to render
//#define ENABLE_MASS_SPRING_DAMP 
bool ENABLE_MASS_SPRING_DAMP = false;

#define DEBUGGING 
#define TESTING 

//constants here
float unitsDivisor = 1000.0; 

#define ARDBUFFER 16 //for serial line printing

// Includes
#include <math.h>
#include <AS5048A.h>
// the sensor CSn pin is connected to pin 10
AS5048A angleSensor(10);

// generate constants
double a1 = 129; //mm
double a2 = 152; //mm
double a3 = 152; //mm
double a4 = 129; //mm
double a5 = 82; //mm

// Pin declares
int pwmPinR = 5; // PWM output pin for motor
int dirPinR = 8; // direction output pin for motor
int pwmPinL = 6; // PWM output pin for motor
int dirPinL = 7; // direction output pin for motor
int sensorPosPin = A4; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 575;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;

// Position tracking variables Hall effect sensor
long updatedPosHE = 0; // latest processed value of the Hall Effect sensor
int rawPosHE = 0;         // current raw reading from MR sensor
int lastRawPosHE = 0;     // last raw reading from MR sensor
int lastLastRawPosHE = 0; // last last raw reading from MR sensor
int flipNumberHE = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffsetHE = 0;
int rawDiffHE = 0;
int lastRawDiffHE = 0;
int rawOffsetHE = 0;
int lastRawOffsetHE = 0;
const int flipThreshHE = 10000;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flippedHE = false;
double OFFSETHE = 16300;
double OFFSET_NEGHE = 15;

// Kinematics
//variables
double xh = 0;
double yh = 0;
double dxh = 0;
double dxh_prev = 0;
double dyh = 0;
double dyh_prev = 0;

//theta 1
double theta1 = 0;
double theta1_prev;
double theta1_prev2;
double dtheta1;
double dtheta1_prev;
double dtheta1_prev2;
// theta 5
double theta5 = 0;
double theta5_prev;
double theta5_prev2;
double dtheta5;
double dtheta5_prev;
double dtheta5_prev2;

//angle calibration values
//calibration MR sensor, th5, right
double calPos5 = 518;
double m5 = -0.01369;
double b5 = 153.6599;
double adjustedb5 = b5;

//calibration HE sensor, th1, left
double calPos1 = 16110;
double m1 = 0.001532;
double b1 = 14.1984;
double adjustedb1 = b1;

// Force output variables
double forceX = 0;           // force at the handle
double forceY = 0;
float forceMultiplier = 0.03; //multiple user felt forces

double TR = 0;              // torque of the motor pulley
double TL = 0;              // torque of the motor pulley

double dutyR = 0;            // duty cylce (between 0 and 255)
unsigned int outputR = 0;    // output command to the motor
double dutyL = 0;            // duty cylce (between 0 and 255)
unsigned int outputL = 0;    // output command to the motor

//workspace setup
const float lengthWorkspace = 75.0 / unitsDivisor; //usable work length
const int points = 6; //number of points to be used (needs to be constant to initialize arrays)
const float lengthBetween = lengthWorkspace / (points - 1); //distance between points
const float startingDepth = 40.0 / unitsDivisor; //thickness of clay block when starting

// Kinematics variables
float yUser = 0.0;
float xUser = 0.0;
float dyUser = 0.0;

//mass spring dampners system 
//position variables
float ymass[points] = {0.0}; //based on x & y axis of pantograph
float xmass[points] = {0.0}; //based on x & y axis of pantograph

//force calculation
float claySpringForce[points] = {0.0};
float clayDampForce[points] = {0.0};
float clayTotalForce[points] = {0.0};

float kUser = 100.0;
float bUser = 2;
float bClay = 50;
float kClay = 0.0;
float massClay = 2.0;

//change in position variables
float velMass[points] = {0.0};
float accMass[points] = {0.0};
float velMassPrev[points] =  {0.0};
float accMassPrev[points] =  {0.0};

//loop time
unsigned long mSStart = 0;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  angleSensor.init();

  // Set up serial communication
  Serial.begin(38400);

  // Set PWM frequency
  setPwmFrequency(pwmPinR, 1);
  setPwmFrequency(pwmPinL, 1);

  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPinR, OUTPUT);  // PWM pin for motor
  pinMode(dirPinR, OUTPUT);  // dir pin for motor

  // Output pins
  pinMode(pwmPinL, OUTPUT);  // PWM pin for motor
  pinMode(dirPinL, OUTPUT);  // dir pin for motor

  // Initialize motor
  analogWrite(pwmPinR, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPinR, LOW);  // set direction

  // Initialize motor
  analogWrite(pwmPinL, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPinL, LOW);  // set direction

  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;

  // Initialize position variables
  angleSensor.getRawRotation(); // take reading in case first reading is spurious
  lastLastRawPosHE = angleSensor.getRawRotation(); //position from Hall Effect sensor
  lastRawPosHE = angleSensor.getRawRotation(); //position from Hall Effect sensor

  //setup virtual enviroment
  SetAllElements(&ymass[0],startingDepth);
//  ymass[0] = 0;
  InitializeXmass(&xmass[0]);
  //  Serial.println("Starting Arrays ymass xmass");
  //  PrintArray(ymass);
  //  PrintArray(xmass);

  //calibration MR sensor, th5, right
  adjustedb5 = 144.5 - lastLastRawPos * m5;

  //calibration HE sensor, th1, left
  adjustedb1 = 35.76  - lastLastRawPosHE * m1;

  //to reset Processing visuals
  Serial.println("327, 0, 0, 0, 0, 0, 0, 0, 0, 0");
  
}// end of setup loop

// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------

//for counting loops
int loopNumber = 0;

//for not printing every time
int doNotPrintEveryTime = 0;

//for calulating loop time
unsigned long mainLoopStart = millis();

void loop()
{
  doNotPrintEveryTime++;
  
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
   updatedPos = rawPos + flipNumber*OFFSET; // need to update pos based on what most recent offset is 


  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************
  
  // Get angle output by Hall Effect sensor 
  rawPosHE = int(angleSensor.getRawRotation()); //position from Hall Effect sensor 

    // Calculate differences between subsequent MR sensor readings
  rawDiffHE = rawPosHE - lastRawPosHE;          //difference btwn current raw position and last raw position
  lastRawDiffHE = rawPosHE - lastLastRawPosHE;  //difference btwn current raw position and last last raw position
  rawOffsetHE = abs(rawDiffHE);
  lastRawOffsetHE = abs(lastRawDiffHE);
  
  // Update position record-keeping vairables
  lastLastRawPosHE = lastRawPosHE;
  lastRawPosHE = rawPosHE;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffsetHE > flipThreshHE) && (!flippedHE)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiffHE > 0) {        // check to see which direction the drive wheel was turning
      flipNumberHE--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumberHE++;              // ccw rotation
    }
    flippedHE = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flippedHE = false;
  }
   updatedPosHE = rawPosHE + flipNumberHE*OFFSETHE; // need to update pos based on what most recent offset is 

 
  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

    // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  //double rh = ?;   //[m]
  // Step B.1: print updatedPos via serial monitor
  //Serial.println((float)updatedPosHE,5);
  //Serial.println((float)updatedPos,5);
  // Step B.6: double ts = ?; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  
  double theta5 = (PI / 180) * (updatedPos * m5 + adjustedb5);
  double theta1 = (PI / 180) * (updatedPosHE * m1 + adjustedb1);
  //  Serial.print((float)theta5,5);
  //  Serial.print("\t");
  //  Serial.print((float)theta1,5);
  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)
  // Step B.8: print xh via serial monitor

  // Calculate velocity with loop time estimation
  dtheta1 = (double)(theta1 - theta1_prev) / 0.001;
  // Record the position and velocity
  theta1_prev2 = theta1_prev;
  theta1_prev = theta1;
  theta1_prev2 = theta1_prev;
  theta1_prev = theta1;

  // Calculate velocity with loop time estimation
  dtheta5 = (double)(theta5 - theta5_prev) / 0.001;
  // Record the position and velocity
  theta5_prev2 = theta5_prev;
  theta5_prev = theta5;
  theta5_prev2 = theta5_prev;
  theta5_prev = theta5;

//    Serial.print("\t");
//    Serial.print((float)theta5 * 180 / PI,5);
//    Serial.print("\t");
//    Serial.println((float)theta1 * 180 / PI,5);

  //calculate positions 2 and 4
  double P2x = a1 * cos(theta1);
  double P2y = a1 * sin(theta1);
  double P4x = a4 * cos(theta5) - a5;
  double P4y = a4 * sin(theta5);


  // calculate norms
  double P42_norm = sqrt((P2x - P4x) * (P2x - P4x) + (P2y - P4y) * (P2y - P4y));
  double P2h_norm = (a2 * a2 - a3 * a3 + P42_norm * P42_norm) / (2 * P42_norm);
  double P3h_norm = sqrt(a2 * a2 - P2h_norm * P2h_norm);

  // calculate position h
  double Phx = P2x + (P2h_norm / P42_norm) * (P4x - P2x);
  double Phy = P2y + (P2h_norm / P42_norm) * (P4y - P2y);
  // calculate position 3
  double P3x = Phx + (P3h_norm / P42_norm) * (P4y - P2y);
  double P3y = Phy - (P3h_norm / P42_norm) * (P4x - P2x);

  xh = P3x;
  yh = P3y;

  //rename define for convenience
  double d = P42_norm;
  double b = P2h_norm;
  double h = P3h_norm;
  // calculate partial derivatives
  // d1 partial derivatives
  double d1x2 = -a1 * sin(theta1);
  double d1y2 = a1 * cos(theta1);
  double d1y4 = 0;
  double d1x4 = 0;
  double d1d = ( (P4x - P2x) * (d1x4 - d1x2) + (P4y - P2y) * (d1y4 - d1y2 ) ) / d;
  double d1b = d1d - ( d1d * (a2 * a2 - a3 * a3 + d * d)) / (2 * d * d);
  double d1h = -b * d1b / h;
  double d1yh = d1y2 + ( (d1b * d - d1d * b) / (d * d) ) * (P4y - P2y) + (b / d) * (d1y4 - d1y2);
  double d1xh = d1x2 + ( (d1b * d - d1d * b) / (d * d) ) * (P4x - P2x) + (b / d) * (d1x4 - d1x2);
  double d1y3 = d1yh - (h / d) * (d1x4 - d1x2) - (d1h * d - d1d * h) / (d * d) * (P4x - P2x);
  double d1x3 = d1xh + (h / d) * (d1y4 - d1y2) + (d1h * d - d1d * h) / (d * d) * (P4y - P2y);
  // d5 partial derivatives
  double d5x4 = -a4 * sin(theta5);
  double d5y4 = a4 * cos(theta5);
  double d5y2 = 0;
  double d5x2 = 0;
  double d5d = ( (P4x - P2x) * (d5x4 - d5x2) + (P4y - P2y) * (d5y4 - d5y2) ) / d;
  double d5b = d5d - ( d5d * (a2 * a2 - a3 * a3 + d * d)) / (2 * d * d);
  double d5h = -b * d5b / h;
  double d5yh = d5y2 + ( (d5b * d - d5d * b) / (d * d) ) * (P4y - P2y) + (b / d) * (d5y4 - d5y2);
  double d5xh = d5x2 + ( (d5b * d - d5d * b) / (d * d) ) * (P4x - P2x) + (b / d) * (d5x4 - d5x2);
  double d5y3 = d5yh - (h / d) * (d5x4 - d5x2) - (d5h * d - d5d * h) / (d * d) * (P4x - P2x);
  double d5x3 = d5xh + (h / d) * (d5y4 - d5y2) + (d5h * d - d5d * h) / (d * d) * (P4y - P2y);
  //calculate velocity from omega (dTheta{i})
  double dxh = (d1x3 * dtheta1 + d5x3 * dtheta5)*0.1+dxh_prev*0.9;
 
  double dyh = (d1y3 * dtheta1 + d5y3 * dtheta5)*0.1+dyh_prev*0.9;

  if (isnan(dxh)){
    dxh = 0;
  }
  
  if (isnan(dyh)){
    dyh = 0;
  }
  
  dxh_prev = dxh;
  dyh_prev = dyh;
  
  //calculate torque from force
  static int  myCount = 0;
  if (myCount == 100){

    myCount = 0;
  }
  else {
    myCount++;
  }

  //*************************************************************
  //*** Section 3.Rendering Algorithms: ************************* 
  //*** Assign a motor output force in Newtons ******************  
  //*************************************************************

  //ardprintf("th5, th1: %f %f...xh, yh: %f %f...b5, ab5: %f %f...xU, yU: %f %f",theta5 * 180 / PI, theta1 * 180, xh, yh, b5, adjustedb5,b1, adjustedb1, xUser, yUser);      

//      Serial.print(theta5 * 180 / PI,3);
//      Serial.print(",");
//      Serial.print(theta1 * 180 / PI,3);
//      Serial.print(" ");
//      Serial.print(xh,3);
//      Serial.print(",");
//      Serial.print(yh,3);
//      Serial.print(" ");
//      Serial.print(b5,3);
//      Serial.print(",");
//      Serial.print(adjustedb5,3);
//      Serial.print(" ");
//      Serial.print(b1,3);
//      Serial.print(",");
//      Serial.print(adjustedb1,3);
//      Serial.print(" ");
//      Serial.print(xUser,3);
//      Serial.print(",");
//      Serial.println(yUser,3);

//      Serial.println(rawPosHE);
//      Serial.print(updatedPos);
//      Serial.print(",");
//      Serial.print(updatedPosHE);
//      Serial.print(" ");

//for now, wait until in position before starting rendering //(millis()- mainLoopStart)/64 >= 1000*10
if ( yh > 245 && ENABLE_MASS_SPRING_DAMP == false ){
  ENABLE_MASS_SPRING_DAMP = true;
  //calculating loop time
  mSStart = 0;
}

if (ENABLE_MASS_SPRING_DAMP == true){

  ///****Recieve User Information****///
  
  #ifdef DEBUGGING
  
    xUser = - (xh - (-9)) / unitsDivisor; //xh and yh are in mm, virtual enviroment is in m - (xh - (-9))
    yUser = (yh - 190) / unitsDivisor; //(yh - 220)
  
  #endif //DEBUGGING  
  
    if (xUser <= xmass[0]){//handle case if xUser is = or surpasses min/max xMass
      xUser = xmass[0];
    }
    else if (xUser >= xmass[points - 1]){
        xUser = xmass[points - 1];
    }   
  
    ///****Collision Detection Setup****///
    
    //find which two points will be affected
    float xdiffUserMass[points] = {0}; //array containing difference between user position and all clay positions
    
    memcpy(xdiffUserMass, xmass, sizeof(xdiffUserMass));
    
    AddValue(xdiffUserMass, -xUser);
  
    int clayIndexClosest = points + 1; //should start as impossible index
  
    clayIndexClosest = indexMin(xdiffUserMass); //choose the clay element to interact with as the element with minimum xdiffUserMass 
  
    int clayIndexNext = points + 1; //should start as impossible index    
  
    //find 2nd element depending on if  xUser  > or < xMass
    if (xUser > xmass[clayIndexClosest]){ // inequality could be if xdiffUserMass[clayIndexClosest] <= 0 
      clayIndexNext = clayIndexClosest + 1;
    }
    else if (xUser < xmass[clayIndexClosest]){
      clayIndexNext = clayIndexClosest - 1;
    }
    else if (xUser == xmass[clayIndexClosest]){//if equal, check if at top or bottom
      if(clayIndexClosest == 0){
        clayIndexNext = clayIndexClosest + 1;
      }
      else if( clayIndexClosest == (points - 1) ){
          clayIndexNext = clayIndexClosest - 1;
      }
    }
    
    //find penetration distance
    
    //line equations
    int slopeLowerIndex = min(clayIndexClosest, clayIndexNext);
    
    int slopeHigherIndex = max(clayIndexClosest, clayIndexNext);
  
    float xLineWeight = 0.0;
    float yLineWeight = 0.0;
    float lineConstant = 0.0;
    
    yLineWeight = xmass[slopeHigherIndex] - xmass[slopeLowerIndex]; //b 
    xLineWeight = ymass[slopeLowerIndex] - ymass[slopeHigherIndex]; //a
    lineConstant = xmass[slopeLowerIndex] * ymass[slopeHigherIndex] - ymass[slopeLowerIndex] * xmass[slopeHigherIndex]; //c
  
    //determine if there is penetration by finding the depth at the user height to be on the line
    float yOnLineUser = - ( (xLineWeight * xUser + lineConstant) / yLineWeight ); //if depth is not less than this, then not inside the line
  
    ///****Test Collision and Set userForce onto the clay accordingly****///
  
    //initialize userForce as all zeroes before proving contact was made
    float userForce[points] = {0.0};
    float userForceMag = 0.0; //if no contact found forceX and forceY should be 0
    forceX = 0;
    forceY = 0;
  
    if (yUser < yOnLineUser){ //if there is contact, set the adjacent clay userForce not to zero  
  
      //force calculation variables 
        
        float d = abs ( xLineWeight * xUser + yLineWeight * yUser  + lineConstant ) / sqrt(xLineWeight*xLineWeight + yLineWeight*yLineWeight); //penetration distance

        //ardprintf("d xLineWeight yLineWeight lineConstant %f %d %d %d", d, xLineWeight, yLineWeight, lineConstant);
        
        userForceMag = kUser * d; 
        
        userForce[clayIndexClosest] = -userForceMag * abs( ( xUser - xmass[clayIndexNext] ) / lengthBetween ); //need to add negative such that clay is being pushed inwards
        userForce[clayIndexNext] = -userForceMag * abs( ( xUser - xmass[clayIndexClosest] ) / lengthBetween ); //see above

        //ardprintf("d userForceMag userForce[clayIndexClosest] userForce[clayIndexNext] %f %f %f %f", d, userForceMag, userForce[clayIndexClosest], userForce[clayIndexNext]);
              
         ///****Calculate force on the user****///
  
        //unit vector perpendicular to line components
        float unitDirectionY = ( ymass[slopeHigherIndex] - ymass[slopeLowerIndex] ) / sqrt(xLineWeight*xLineWeight + yLineWeight*yLineWeight); 
        float unitDirectionX = ( xmass[slopeHigherIndex] - xmass[slopeLowerIndex] ) / sqrt(xLineWeight*xLineWeight + yLineWeight*yLineWeight);
      
        //force at handle, if no contact found userForceMag will be 0 
        //force applied to the User should be perpendicular to the line
        double forceClayFrameX = - userForceMag * unitDirectionY; //need multiply by negative unitDirectionY weight to have the user direction point OUT of the clay
        double forceClayFrameY = userForceMag * unitDirectionX;
    
        forceX = -forceClayFrameX; //add clay force
        forceY = forceClayFrameY; //add clay force   
         //total affects user force calc
        if (abs(xmass[clayIndexClosest] - xUser) < lengthBetween*0.05){
          forceX = 0;
        }
        else  {
          forceX = (forceX - bUser * (dxh/1000)) * forceMultiplier; //add all other forces & multiply by force divider
        }
        forceY = (forceY - bUser * (dyh/1000)) * forceMultiplier; 
    
    }  

//        //total affects user force calc
//        if (abs(xmass[clayIndexClosest] - xUser) < lengthBetween*0.05){
//          forceX = 0;
//        }
//        else  {
//          forceX = (forceX - bUser * (dxh/1000)) * forceMultiplier; //add all other forces & multiply by force divider
//        }
//        forceY = (forceY - bUser * (dyh/1000)) * forceMultiplier; 

#ifdef TESTING
   
   ///****Calculate total force on clay****///
   
      UpdateClaySpringForce(&claySpringForce[0], &ymass[0]); //spring force
  
      UpdateClayDampForce(&clayDampForce[0], &velMass[0]);//dampner force
  
      UpdateTotalForce(clayTotalForce,clayDampForce,claySpringForce,userForce); //total force  
  
   ///****Calculate resulting motion****///
  
      //update all the masses velocities, accelerations, and forces
      //calculate acceleration from F = ma
      UpdateClayAccelerations(clayTotalForce, accMass);   
  
      //integrate to find velocity and postion
  
      //change in time
      float loopTime = (1.0 / 1000.0); //in SECONDS
  
      //calculated based on actual loop time (may need to use if becomes very slow)
      unsigned long mSTemp = millis();
      float milliLoopseconds = ( mSTemp - mSStart ) / (64.0); //divide by 64 to account for motor prescalar IN MILLISECONDS
      mSStart = mSTemp;
      //Serial.print("time per loop ");
      //Serial.println(milliLoopseconds,6);
  
      IntegratePrevious(velMass,accMass,accMassPrev, loopTime); //integrate for velocity
      IntegratePrevious(ymass,velMass,velMassPrev, loopTime); //integrate for position  
  
      //store acceleration and velocity from last time for use this time
      memcpy(accMassPrev, accMass, sizeof(accMassPrev));
      memcpy(velMassPrev, velMass, sizeof(velMassPrev)); 

#endif
    
      ///****Send User Information and Clay Information over Serial to Processing****///
  
        if( doNotPrintEveryTime % (10) == 0 ){
          //Serial.println(rawPosHE);
          //Serial.println(updatedPos);
          //Serial.println(updatedPosHE);
//          Serial.print(xUser,5);
//          Serial.print(",");
//          Serial.println(yUser,5);
       }
//    

      if ( !isnan(clayIndexClosest) && !isnan(clayIndexNext) && !isnan(ymass[clayIndexClosest]) && !isnan(xmass[clayIndexClosest])&& 
      !isnan(ymass[clayIndexNext]) && !isnan(xmass[clayIndexNext]) && !isnan(yUser) && !isnan(xUser) && !isnan(forceX) && !isnan(forceY) ){   
        
        Serial.print(clayIndexClosest);
        Serial.print(","); 
        Serial.print(ymass[clayIndexClosest],6);
        Serial.print(",");
        Serial.print(xmass[clayIndexClosest],6);
        Serial.print(",");
        Serial.print(clayIndexNext);
        Serial.print(","); 
        Serial.print(ymass[clayIndexNext],6);
        Serial.print(",");
        Serial.print(xmass[clayIndexNext],6);
        Serial.print(",");
        if (yUser < yOnLineUser){
          Serial.print(yOnLineUser,6);
        }
        else{
          Serial.print(yUser,6);
        }
        Serial.print(",");
        Serial.print(xUser,6);
        Serial.print(",");
        Serial.print(dutyR / 1000.0,6);
        Serial.print(",");
        Serial.print(dutyL / 1000.0,6);
        Serial.println(); 
        
      }
  
//      SendArrayOverSerial(xmass);
//      SendArrayOverSerial(ymass);
//      Serial.print(1000.0 * xUser,0);
//      Serial.print(",");
//      Serial.println(1000.0 * yUser,0);
//      Serial.print(",");
//      Serial.print(forceX,0);
//      Serial.print(",");
//      Serial.print(forceY,0);
//      Serial.println(); 
//      Serial.print(" "); 
//      Serial.print(xh,5);
//      Serial.print(",");
//      Serial.println(yh,5);
//      
    
}//#endif //ENABLE_MASS_SPRING_DAMP  
 
  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  
  // Step C.1: force = ?; // You can  generate a force by assigning this to a constant number (in Newtons) or use a haptic rendering / virtual environment
  // Step C.2: Tp = ?;    // Compute the require motor pulley torque (Tp) to generate that force using kinematics

  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  //VIRTUAL OBJECT HERE
  // calculate forceX
  // calculate forceY
  //forceX = 0.0;
  //forceY = 0.0;
  // FORCE SIMULATION HERE
//  TL = -1;
//  TR = -1;
  TL = (d1x3*forceX + d1y3*forceY) / 1000.0;
  TR = (d5x3*forceX + d5y3*forceY) / 1000.0; 

  if (isnan(TL)){
    TL = 0;
  }
  
  if (isnan(TR)){
    TR = 0;
  }
  
//  Serial.print(TL,5);
//  Serial.print(" : ");
//  Serial.println(TR,5);
  
  // Step C.1: force = ?; // You can  generate a force by assigning this to a constant number (in Newtons) or use a haptic rendering / virtual environment
  // Step C.2: Tp = ?;    // Compute the require motor pulley torque (Tp) to generate that force using kinematics
//  Serial.print(TR);
//  Serial.print("\t");
//  Serial.println(TL);
 
  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
//COMMENT THIS TO CHECK TORQUE VALUE AND SIGN
  // Determine correct direction for RIGHT motor torque
  if (TR > 0) {
    digitalWrite(dirPinR, HIGH);
  } else {
    digitalWrite(dirPinR, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  dutyR = sqrt(abs(TR) / 0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (dutyR > 1) {
    dutyR = 1;
  } else if (dutyR < 0) {
    dutyR = 0;
  }
  outputR = (int)(dutyR * 255);  // convert duty cycle to output signal
  analogWrite(pwmPinR, outputR); // output the signal


  // Determine correct direction for LEFT  motor torque
  if (TL > 0) {
    digitalWrite(dirPinL, HIGH);
  } else {
    digitalWrite(dirPinL, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  dutyL = sqrt(abs(TL) / 0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (dutyL > 1) {
    dutyL = 1;
  } else if (dutyL < 0) {
    dutyL = 0;
  }
  outputL = (int)(dutyL * 255);  // convert duty cycle to output signal
  analogWrite(pwmPinL, outputL); // output the signal
  //STOP COMMENT OUT HERE

  //ardprintf("d1x3: %f, d5x3: %f, d1y3: %f, d5y3: %f", d1x3, d5x3, d1y3, d5y3);
  //ardprintf("force x,y: %f %f torque R,L: %f %f duty R,L: %f %f", forceX, forceY, TR, TL, dutyR, dutyL);
}




// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

///****Function****///

//special for intializing the x values of the clay
void InitializeXmass(float xmass[]){
  for (int index = 1; index < points; xmass[index] += xmass[index-1] + lengthBetween, index++){}
  return;
}

//set all the elemts of an array to a single value
void SetAllElements(float ymass[], const float startingDepth){
  for (int index = 0; index < points; ymass[index] = startingDepth, index++){}
  return;
}

//add a float value to every element in an array
void AddValue(float targetArray[], float val2add){
  for (int index = 0; index < points; targetArray[index] += val2add, index++){}
  return;
}

//update the clay spring force on all clay points
void UpdateClaySpringForce(float claySpringForce[], float ymass[]){
  for (int index = 0; index < points; index++){    
    claySpringForce[index] = kClay * (ymass[index] - startingDepth);  //no negative sign so that the clay resists OUTWARDS, positive Y direction     
  }  
 return;
}

//update the clay dampening force on all clay points
void UpdateClayDampForce(float clayDampForce[], float velMass[]){
  for (int index = 0; index < points; index++){    
    clayDampForce[index] = -bClay * velMass[index];          
  }  
 return;
}

//update the total force on all the clay points, each seperate component force has to be updated before running this
void UpdateTotalForce(float clayTotalForce[], float clayDampForce[], float claySpringForce[], float userForce[]){
  for (int index = 0; index < points; index++){    
    clayTotalForce[index] = clayDampForce[index] + claySpringForce[index] + userForce[index];           
  }  
 return;
}

//update all the accelerations for all the clay points, clayTotalForce has to be updated before running this
void UpdateClayAccelerations(float clayTotalForce[], float accMass[]){
  
  for (int index = 0; index < points; index++){    
    accMass[index] = clayTotalForce[index] / massClay;           
  }  
 return;
 
}

//solve for a variable by integrating its derivative
void IntegratePrevious(float velMass[], float accMass[], float accMassPrev[], float loopTime){
  //for position integration yMass, velMass, vellMassPrev
  
  for (int index = 0; index < points; index++){    
    velMass[index] = velMass[index] + ( 0.5 * (accMassPrev[index] + accMass[index]) * (loopTime) ); //NOTE loop time estimation taken from arduino starter code 0.001;           
  } 
   
 return;
 
}

//print an entire array over serial
void PrintArray(float printArray[]){  
  for (int index = 0; index < points; index++){
    Serial.print(printArray[index],3);
    Serial.print(", ");
    }   
  Serial.println();
  return;    
}

//send an entire array over serial to Processing (needs special delinators)
void SendArrayOverSerial(float printArray[]){  
  for (int index = 0; index < points; index++){
    Serial.print(1000.0 * printArray[index],0);
    Serial.print(",");
    }   
  return;    
}

//find the index of the minimum value in an array
int indexMin(float targetArray[]){
  
    int minIndex = 0; //have to create outside forloop to pass it out
    
    for(int currentIndex = 1; currentIndex < points; currentIndex++){        
        if(abs(targetArray[currentIndex]) < abs(targetArray[minIndex]))
          minIndex = currentIndex;           
    }
    return minIndex;
}

//A printf function for serial communication from Arduino boards
//example syntax is ardprintf("test %d %l %c %s %f", l, k, s, j, f);
//https://arduino.stackexchange.com/questions/176/how-do-i-print-multiple-variables-in-a-string

int ardprintf(char *str, ...)
{
  int i, count=0, j=0, flag=0;
  char temp[ARDBUFFER+1];
  for(i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++;

  va_list argv;
  va_start(argv, count);
  for(i=0,j=0; str[i]!='\0';i++)
  {
    if(str[i]=='%')
    {
      temp[j] = '\0';
      Serial.print(temp);
      j=0;
      temp[0] = '\0';

      switch(str[++i])
      {
        case 'd': Serial.print(va_arg(argv, int));
                  break;
        case 'l': Serial.print(va_arg(argv, long));
                  break;
        case 'f': Serial.print(va_arg(argv, double));
                  break;
        case 'c': Serial.print((char)va_arg(argv, int));
                  break;
        case 's': Serial.print(va_arg(argv, char *));
                  break;
        default:  ;
      };
    }
    else 
    {
      temp[j] = str[i];
      j = (j+1)%ARDBUFFER;
      if(j==0) 
      {
        temp[ARDBUFFER] = '\0';
        Serial.print(temp);
        temp[0]='\0';
      }
    }
  };
  Serial.println();
  return count + 1;
}
