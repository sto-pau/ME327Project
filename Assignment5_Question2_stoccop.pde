// Pro_Graph2.pde
/*
 Based on the Arduining example which is based on the Tom Igoe example.
 Mofified by Cara Nunez 5/1/2019:
  -A wider line was used. strokeWeight(4);
  -Continuous line instead of vertical lines.
  -Bigger Window size 600x400.
-------------------------------------------------------------------------------
This program takes ASCII-encoded strings
from the serial port at 9600 baud and graphs them. It expects values in the
range 0 to 1023, followed by a newline, or newline and carriage return


*/

import processing.serial.*;

Serial myPort;        // The serial port

//initialize all variables
float inByte = 0; //current value of the first variable in the string
float lastByte = 0; //previous value of the first variable in the string
float inByte2 = 0; //current value of the second variable in the string
float lastByte2 = 0; //previous value of the second variable in the string

String receivedString  = null;

double xh = 0.0;
float virtualXh = 0.0;

float xmass = 0.5 / 100; 
float virtualXmass = map((float)xmass, -0.085, 0.051, 0.0, 600.0); 
float velMass = 0.0;
float accMass = 0.0;
float velMassPrev = 0.0;
float accMassPrev = 0.0;

float xwall = 45.0 / 1000; 
float virtualXwall = map((float)xwall, -0.085, 0.051, 0.0, 600.0);

float userForce = 0.0;
float springForce = 0.0;
float springEqX = 0.5 / 100.0;
float dampForce = 0.0;

float mass = 2.0;
float b = 1.0;
float k = 300.0;
float kUser = 1000.0;

int mSStart = 0;

void setup () {
  // set the window size:
  size(600, 400);        

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  myPort = new Serial(this, Serial.list()[0], 38400);  //make sure baud rate matches Arduino

  // A serialEvent() is generated when a newline character is received :
  //myPort.bufferUntil('\n');
  
  receivedString  = myPort.readStringUntil('\n');
  
  background(0);      // set inital background:
}
void draw () {
  // everything happens in the serialEvent()
  background(0); //uncomment if you want to control a ball
  stroke(127,34,255);     //stroke color
  strokeWeight(4);        //stroke wider
  
  //START EDITING HERE
  //stroke(r,g,b);     //stroke color
  //strokeWeight(num);        //stroke wider
  
  // Mass Spring Damper
  rectMode(CENTER);
  
  //draw the wall
  rect(virtualXwall, 200.0, 20.0, 400.0);
  //draw an ellipse to represent the user 
  
  // draw an ellipse to represent user position
  while (myPort.available () > 0) {  
    receivedString  = myPort.readStringUntil('\n');  
  } 
   
  if (receivedString != null){
    
    xh = Double.parseDouble(receivedString.trim());
    
    virtualXh = map((float)xh, -0.087, 0.050, 0.0, 600.0);
    
    if (virtualXh < ( virtualXmass - 10.0)){
      ellipse(virtualXh, 200.0, 20.0, 20.0);
    }
    else{
      ellipse(virtualXmass - 20.0, 200.0, 20.0, 20.0);
    }       
  }  
  
  //calculate current xmass position
  
  //forces
  if( (xh -xmass) > 0 ){//user applied force
    userForce = kUser * ((float)xh - xmass);
  }
  else{
  userForce = 0;
  }
  
  springForce = -k * (xmass - springEqX); //spring force
  //println(springForce);
    
  dampForce = -b * velMass; //dampner force  
  
  accMass = (userForce + springForce + dampForce) / mass; // calculate accleration from sum forces and mass
  //println(accMass);
  
  //change in time
  float seconds = ( millis() - mSStart ) /1000.0; 
  if (seconds < 0.01){
    seconds = seconds;
  }
  else{
    seconds = 0.01;
  }
    
  velMass = velMass + ( 0.5 * (accMassPrev + accMass) * 0.001); //NOTE loop time estimation taken from arduino starter code 0.001;
  xmass = xmass + ( 0.5 * (velMassPrev + velMass) * 0.001);
  
  //draw an ellipse to represent the mass of the spring-mass-damper
  virtualXmass = map((float)xmass, -0.085, 0.051, 0.0, 600.0); //set virtualXmass current position
  ellipse(virtualXmass, 200.0, 20.0, 20.0); 
  
  //draw a line from the wall to the xMass  
  rect(virtualXmass + (virtualXwall - virtualXmass) / 2 + 5 , 200.0, (virtualXwall - virtualXmass) - 10, 10.0); 
  
  mSStart = millis(); //for next time
  
  //store acceleration and velocity from last time for use this time
  accMassPrev = accMass;
  velMassPrev = velMass;

}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  
  //For Mass Spring Damper
  // make sure to match variable names with what has been declared above in lines 10-13 (or change the original variable names if you wish)
  
  // read the first part of the input string
  // HINT: use myPort.readStringUntil() with the appropriate argument
  // trim and convert string to a number
  // if: the number is NaN, set current value to previous value
  // otherwise: map the new value to the screen width
  //           & update previous value variable
  
  // HINT: use map(myValue, minValueIn, maxValueIn, minValueOut, maxValueOut) to map from units of your Arduino simulation to pixels 
  
    // draw an ellipse to represent user position

  
  // repeat for second part of the input string
  
  //STOP EDITING HERE
}
