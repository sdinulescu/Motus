/*
 * Listening OSC
 * Stejara Dinulescu
 * Program listens for OSC messages from motion capture webcam input and
 * uses it to drive sound
 */


/* CITATIONS: 
 * oscP5sendreceive by andreas schlegel
 * example shows how to send and receive osc messages.
 * oscP5 website at http://www.sojamo.de/oscP5
 *
 * sound example taken from processing.org 
 * examples show how to create sine waves, envelopes, tri oscillator
 * sound website at https://processing.org/tutorials/sound/
 */

import oscP5.*;
import netP5.*;
import processing.sound.*;

OscP5 oscP5;

int DEST_PORT = 8888;
String DEST_HOST = "127.0.0.1";

//Values for Osc

float angle = 0; 
float xAccel = 0;
float prevXAccel = 0;
float yAccel = 0;
float prevYAccel = 0;
float der1x =0;
float prevDer1x = 0;
float der1y = 0; 
float prevDer1y = 0;
float der2x =0;
float prevDer2x = 0;
float der2y = 0; 
float prevDer2y = 0;

float xPos = 0;
float prevXPos = 0;
float yPos = 0;
float prevYPos = 0;
float maxMotion = 0;
float prevMaxMotion = 0;


void setup() {
  size(1000, 800);
  background(255);
  
  /* start oscP5, listening for incoming messages at destination port */
  oscP5 = new OscP5(this, DEST_PORT);

  /* myRemoteLocation is a NetAddress. a NetAddress takes 2 parameters,
   * an ip address and a port number. myRemoteLocation is used as parameter in
   * oscP5.send() when sending osc packets to another computer, device, 
   * application. usage see below. for testing purposes the listening port
   * and the port of the remote location address are the same, hence you will
   * send messages back to this sketch.
   */
}

void oscEvent(OscMessage msg) { //checks for incoming osc messages
  String addr = msg.addrPattern();
  //set previous values?
  prevXAccel = xAccel;
  prevYAccel = yAccel;
  prevDer1x = der1x;
  prevDer1y = der1y;
  prevDer2x = der2x;
  prevDer2y = der2y;
  
  prevXPos = xPos;
  prevYPos = yPos;
  prevMaxMotion = maxMotion;
  if ( addr.contains("mocap/points") ) {
    //set current values
    xAccel = msg.get(0).floatValue();
    yAccel = msg.get(1).floatValue();
  } else if ( addr.contains("mocap/derivative") ) {
    der1x = msg.get(0).floatValue();
    der1y = msg.get(1).floatValue();
  } else if ( addr.contains("mocap/square") ) {
    maxMotion = msg.get(0).floatValue();
    xPos = msg.get(1).floatValue();
    yPos = msg.get(2).floatValue();
  }
  
  //println("### received an osc message: " + addr + " " + xAccel + " " + yAccel + " " + der1x + " " + der1y + " " + der2x + " " + der2y);
  //println(xPos + " " + yPos +  " " + maxMotion);
}

void draw() {
  ////mapping incoming blob values
  //xAccel = map(xAccel, 0, 1, 0, width);
  //prevXAccel = map(prevXAccel, 0, 1, 0, width);
  //yAccel = map(yAccel, 0, 1, 0, height);
  //prevYAccel = map(prevYAccel, 0, 1, 0, height);
  der1x = map(der1x, 0, 1000, 0, width/2);
  //prevDer1x = map(prevDer1x, 0, 1, 0, width);
  der1y = map(der1y, 0, 1000, 0, height/2);
  //prevDer1y = map(prevDer1y, 0, 1, 0, height);
    xPos = map(xPos, 0, 640, 0, width/2);
  prevXPos = map(prevXPos, 0, 640, 0, width/2);
  yPos = map(yPos, 0, 480, 0, height/2);
  prevYPos = map(prevYPos, 0, 480, 0, height/2);
  maxMotion = map(maxMotion, 0, 10000, 0, height/2);
  prevMaxMotion = map(prevMaxMotion, 0, 10000, 0, height/2);
  
  
  float colRed = map(maxMotion, 0, 10000, 0, 255);
  float colGreen = map(maxMotion, 0, 10000, 0, 255);
  float colBlue = map(maxMotion, 0, 10000, 0, 255);
  float colAlpha = map(maxMotion, 0, 1000, 0, 255);
  
  translate(maxMotion, maxMotion);
  rotate(angle);

  fill(colRed, colGreen, colBlue, colAlpha);
  bezier(0, 0, xPos, yPos, xAccel, yAccel, der1x, der1y);
  angle++;
}
