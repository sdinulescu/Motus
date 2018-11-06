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
float yAccel = 0;
float prevXAccel = 0;
float prevYAccel = 0;


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
  println("message!");
  if ( addr.contains("mocap") ) {
    //set previous values?
    prevXAccel = xAccel;
    prevYAccel = yAccel;
    
    //set current values
    xAccel = msg.get(0).floatValue() / 100;
    yAccel = msg.get(1).floatValue() / 100;
    println("### received an osc message: " + addr + " " + xAccel + " " + yAccel);
  }
}

void draw() {
  //mapping incoming blob values
  line(prevXAccel, prevYAccel, xAccel, yAccel);
}
