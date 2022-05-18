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

//self initialized variables
String x_pos_str;
float x_pos_f;
float x_pos;
String y_pos_str;
float y_pos_f;
float y_pos;
//parameters for graphics
float x_min = 0.01;
float x_max = 0.04;
float y_min = -0.02;
float y_max = 0.02;
float x_map_min = 0;
float x_map_max = 200;
float y_map_min = 0;
float y_map_max = 200;


void setup () {
  // set the window size:
  size(600, 400);        

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
  background(0); //uncomment if you want to control a ball
  stroke(127,34,255);     //stroke color
  strokeWeight(4);        //stroke wider
  
  // START EDITING HERE
  
  // Virtual Wall
  // map the wall position from units of Arduino simulation to the screen width.
  // HINT: use map(myValue, minValueIn, maxValueIn, minValueOut, maxValueOut) to map from units of your Arduino simulation to pixels
  // draw the wall as a line
  // draw an ellipse to represent user position

}

void serialEvent (Serial myPort) {
  // read the input string as String
  x_pos_str = myPort.readStringUntil('\t');
  // convert to float
  x_pos_f = float(x_pos_str);
  // check if value is valid and update position
  if (x_pos != Float.NaN){
    x_pos = map(x_pos_f, x_min, x_max, x_map_min, x_map_max);
  }
  else {
    x_pos = 0.5;
  }
  
  // repeat for second part of the input string
  y_pos_str = myPort.readStringUntil('\n');
  y_pos_f = float(y_pos_str);
  if (y_pos_f != Float.NaN){
    y_pos = map(y_pos_f, y_min, y_max, y_map_min, y_map_max);
  }
  else {
    y_pos = 0.5;
  }
}
