//Jabez Wilson : jabez.wilson@student.unsw.edu.au
//Mark Yeo: mark.yeo@student.unsw.edu.au
//Last Modified  :  13Apr15
//Magnometer (Adafruit HMC5883L) functions for the Magnetic Detumbler Demo

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

//constants for plus and minus which is returned by return_direction
#define MN_PLUS 1
#define MN_MINUS (-1)

//'measurement ready' pin
#define MN_READY 3//check this if code is freezing

//function to make sure that the magnometer is connected
//returns 1 if mag is initialized 0 otherwise
int MN_init(Adafruit_HMC5883_Unified mag);

//Stores strength of magnetic field in x,y,z dir's in eventOut
void MN_getEvent(Adafruit_HMC5883_Unified mag, int eventOut[3]);

