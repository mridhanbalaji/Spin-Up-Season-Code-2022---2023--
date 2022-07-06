/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Mridhan Balaji                                   */
/*    Created:      Mon Jun 27 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// DriveTrainLeftFront  motor         1               
// DriveTrainLeftBack   motor         20              
// DriveTrainRightFront motor         2               
// DriveTrainRightBack  motor         21              
// EncoderX1            encoder       A, B            
// EncoderX2            encoder       C, D            
// EncoderY1            encoder       G, H            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "iostream"
#include "vex.h"
#include <iostream>
#include <list>
#include <cmath>

using namespace vex;

//A global instance of Competition
competition Competition;


//Declares the Cordinates globaly
double X_Cord;
double Y_Cord;
double THETA_CORD;

int THETA_CORDINATE;
int X_CORDINATE;
int Y_CORDINATE;

bool OdometryENABLED = true;

const double RadToDeg = (180/M_PI);
const double TwoPI = 2 * M_PI;

//Odometry function
//PROVIDE THETA IN RADIANS
void odometry(double startX, double startY, double startTHETA) {

  //Assigns the the starting cordinates
  //CAN CHANGE ACORDING TO STARTING POSITION IN AUTON
  X_Cord = startX;
  Y_Cord = startY;
  THETA_CORD = startTHETA;

  // std::list<double> cordinates = {}; 

  //Initilises the oldposition variable
  double oldpositionX1 = 0;
  double oldpositionX2 = 0;
  double oldpositionY1 = 0;

  //Initilises the currentposition variable
  double currentpositionX1 = 0;
  double currentpositionX2= 0;
  double currentpositionY1= 0;

  //Sets the standerd values needed for calculaition
  //Change the L value to calibrate
  double L = 1;         // Distance between the 2 horizontal encoders (Cm)
  double B = 1;         // Distance between the center of the 2 horizontal encoders and the vertical encoder (Cm)
  double R = 4.1275;      // Radius of the small omni wheels in Cm(2 Inches)
  double C = (2.0 * M_PI * R);  //Creates a Constant CM per revolution
    
    //Loops the odometry to constantly track and calculate the values
    while(OdometryENABLED){

      //sets the old positionm to the currentposition at the begining of each iteraition
      oldpositionX1 = currentpositionX1;
      oldpositionX2 = currentpositionX2;
      oldpositionY1 = currentpositionY1;

      //Gets the current position of the Encoders in revolutions
      currentpositionX1 = EncoderX1.rotation(rev);
      currentpositionX2 = EncoderX2.rotation(rev);
      currentpositionY1= EncoderX2.rotation(rev);
      
      //IF NOT WORKING PROPERLLY USE THIS:
      // currentpositionX1 = -(EncoderX1.rotation(rev));
      // currentpositionX2 = -(EncoderX2.rotation(rev));
      // currentpositionY1= EncoderX2.rotation(rev);

      //Gets the encoder position change in revolutions
      double N1 = currentpositionX1 - oldpositionX1;
      double N2 = currentpositionX2 - oldpositionX2;
      double N3 = currentpositionY1 - oldpositionY1;

      //Calculates the distance change for the robot for X, Y, and Heading
      double dX = C * (N1 + N2) / 2.0;
      double dY =  C * (N3 - ((B * (N2 - N1)) / L));
      double dTHETA = C * ((N2 - N1) / L);

      //Calculates the robots cordinates relitive to the field by adding the the change in 
      //cordinates relitive to the direction
      double THETA = THETA_CORD + (dTHETA/2.0);
      X_Cord += dX *  cos(THETA) - dY * sin(THETA);
      Y_Cord += dY * sin(THETA) + dX * cos(THETA);
      THETA_CORD += dTHETA;


      //Reset the theat value to cvontain it between 0 and 360 degrees
      if (THETA_CORD > TwoPI){
        while(THETA_CORD > TwoPI){
          THETA_CORD -= TwoPI;
        }
      }
      else if(THETA_CORD < 0){
        while(THETA_CORD < 0){
          THETA_CORD +=TwoPI;
        }
      }

      THETA_CORDINATE = round(THETA_CORD * RadToDeg);
      X_CORDINATE = round(X_Cord);
      Y_CORDINATE = round(Y_Cord);

      //Sets the odometry to stop for 10 miliseconds to prevent overheating and crashing
      vex::task::sleep(6);
    }

}