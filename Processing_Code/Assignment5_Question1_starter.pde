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

//initialize variables
float inByte = 0;
float lastByte = 0;

// self initialized variables
String rxString;
String[] rxArray;
int idx1;
float newX1;
float newY1;
int idx2;
float newX2;
float newY2;

// constant values for graphics
float xMin = 0.01;
float xMax = 0.04;
float yMin = -0.02;
float yMax = 0.02;
float xMapMin = 0;
float xMapMax = 200;
float yMapMin = 0;
float yMapMax = 200;
float spaceBuffer = 10;

// array to store point positions
int numPoints = 10;
float[][] points;
// handle coordinates
float rawXh;
float rawYh;
float xh;
float yh;

void setup () {
  // set the window size:
  size(600, 400);        

  // initialize points
  points = new float[numPoints][2];
  // populate points
  for (int i = 0; i < numPoints; i += 1){
    points[i][0] = (i+1) * height/(numPoints+1);
    points[i][1] = width - 200;
  }

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].
  myPort = new Serial(this, Serial.list()[9], 38400);  //make sure baud rate matches Arduino

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
}

void draw () {
  // everything happens in the serialEvent()
  background(0);
  stroke(127,34,255);     //stroke color
  strokeWeight(4);        //stroke width
  
  // draw center line
  line(300, 0, 300, 400);
  // draw handle position
  ellipse(xh, yh, 30, 30);
  // draw points for pottery
  for (int i = 0; i < numPoints; i += 1) {
    point(points[i][0], points[i][1]);
  }
}

void serialEvent (Serial myPort) {
  // read input string
  rxString = myPort.readStringUntil('\n');
  // process received data
  if (rxString != null){
    // split into array of strings
    rxArray = split(rxString, ',');
    // convert values to ints/floats and update values
    idx1 = Integer.parseInt(rxArray[0]);
    newX1 = float(rxArray[1]);
    newY1 = float(rxArray[2]);
    idx2 = Integer.parseInt(rxArray[3]);
    newX2 = float(rxArray[4]);
    newY2 = float(rxArray[5]);
    rawXh = float(rxArray[6]);
    rawYh = float(rxArray[7]);
    // map and update values
    xh = map(rawXh, xMin, xMax, xMapMin, xMapMax);  // x handle position
    yh = map(rawYh, yMin, yMax, yMapMin, yMapMax);  // y handle position
    points[idx1][0] = map(newX1, xMin, xMax, xMapMin, xMapMax-spaceBuffer);  // first point x, TODO: check if mapping correct
    points[idx1][1] = map(newY1, yMin, yMax, yMapMin, yMapMax);  // first point y
    points[idx2][0] = map(newX2, xMin, xMax, xMapMin, xMapMax-spaceBuffer);  // first point x
    points[idx2][1] = map(newY2, yMin, yMax, yMapMin, yMapMax);  // first point y
  }
}
