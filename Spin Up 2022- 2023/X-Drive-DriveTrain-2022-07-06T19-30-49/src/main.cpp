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

//Creates an array to set the robots cordinates to later be used in Autonomous.
// int coordinates[3] =  {X_CORDINATE, Y_CORDINATE, THETA_CORDINATE};

//Rounding Function
float roundoff(float value, unsigned char prec){
  float pow_10 = pow(10.0f, (float)prec);
  return round(value * pow_10) / pow_10;
}


//GOTOPOSITION Function to take the robot to the set cordinate
void GoToPosition(int desiredX, int desiredY, int desiredHEADING, int variance){
  
  //Calculate the needeed distance to travel in the X and Y directions
  int TravelX = desiredX - X_CORDINATE;
  int TravelY = desiredY - X_CORDINATE;

  //Declare the angle variable for the robot to turn to get to the point
  float GOTO_angle;

  //Derive the angle using refrence angle calculaition, based on the quadrent
  //since this is a right triangle find the hypotnuse by using the arctangent of y/x
  if (TravelX > 0 && TravelY > 0){
    GOTO_angle = std::abs(atan(TravelY/TravelX));
  }
  else if (TravelX < 0 && TravelY > 0) {
    GOTO_angle = M_PI - std::abs(atan(TravelY/TravelX));
  }
  else if (TravelX < 0 && TravelY < 0) {
    GOTO_angle = M_PI + std::abs(atan(TravelY/TravelX));
  }
  else if (TravelX > 0 && TravelY < 0) {
    GOTO_angle =  TwoPI - std::abs(atan(TravelY/TravelX));
  }

  GOTO_angle = round(GOTO_angle * RadToDeg);

  //Variable to keep trag of how much the orientaition program has run
  int Orientaitionprogram = 0;

  //If loop to make sure the orientaition program only runs once 
  if (Orientaitionprogram == 0) {
    //Rotates the robot to travel to the point in the correct orientaition
    while(THETA_CORDINATE < GOTO_angle - 2 || THETA_CORDINATE > GOTO_angle + 2) {
      //Right Motor Spins
      DriveTrainRightFront.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
      DriveTrainRightBack.spin(vex::directionType::rev,  50, vex::velocityUnits::pct);

      //Left Motor Spins
      DriveTrainLeftFront.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
      DriveTrainLeftBack.spin(vex::directionType::fwd,  50, vex::velocityUnits::pct);
    }
    //Sets the orientaition program to one so it doesnt run again
    Orientaitionprogram = 1;
  }



  //If their is no change in the X axis, makes the robot use the Y cordinate as a refrence to move
  if (TravelX == 0){
    //Runs until the robot is at the right cordinate
    while(desiredY - variance > Y_CORDINATE || desiredY + variance < Y_CORDINATE){
      //Makes the robot travel to reach the correct X or Y cordinate
      while(THETA_CORDINATE > GOTO_angle - 2 || THETA_CORDINATE < GOTO_angle + 2) {
        //Right Motor Spins
        DriveTrainRightFront.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);
        DriveTrainRightBack.spin(vex::directionType::fwd,  75, vex::velocityUnits::pct);

        //Left Motor Spins
        DriveTrainLeftFront.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);
        DriveTrainLeftBack.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);
      }

      while(THETA_CORDINATE < GOTO_angle - 2 || THETA_CORDINATE > GOTO_angle + 2){
         //Corrects the angle by spining to the right if the robot angle is greater than the goto angle
         if(THETA_CORDINATE > GOTO_angle){
            //Right Motor Spins
            DriveTrainRightFront.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
            DriveTrainRightBack.spin(vex::directionType::rev,  50, vex::velocityUnits::pct);

            //Left Motor Spins
            DriveTrainLeftFront.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
            DriveTrainLeftBack.spin(vex::directionType::fwd,  50, vex::velocityUnits::pct);
         }
         else if(THETA_CORDINATE < GOTO_angle){
           //Right Motor Spins
            DriveTrainRightFront.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
            DriveTrainRightBack.spin(vex::directionType::fwd,  50, vex::velocityUnits::pct);

            //Left Motor Spins
            DriveTrainLeftFront.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
            DriveTrainLeftBack.spin(vex::directionType::rev,  50, vex::velocityUnits::pct);
         }
      }
    }
  }
  //Makes the robot use the X cordinate for any other senarios as a reference
  else{
     //Runs until the robot is at the right cordinate
    while(desiredX - variance > X_CORDINATE || desiredX + variance < X_CORDINATE) {
      //Makes the robot travel to reach the correct X or Y cordinate
      while(THETA_CORDINATE > GOTO_angle - 2 || THETA_CORDINATE < GOTO_angle + 2) {
        //Right Motor Spins
        DriveTrainRightFront.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);
        DriveTrainRightBack.spin(vex::directionType::fwd,  75, vex::velocityUnits::pct);

        //Left Motor Spins
        DriveTrainLeftFront.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);
        DriveTrainLeftBack.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);
      }

      while(THETA_CORDINATE < GOTO_angle - 2 || THETA_CORDINATE > GOTO_angle + 2){
         //Corrects the angle by spining to the right if the robot angle is greater than the goto angle
         if(THETA_CORDINATE > GOTO_angle){
            //Right Motor Spins
            DriveTrainRightFront.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
            DriveTrainRightBack.spin(vex::directionType::rev,  50, vex::velocityUnits::pct);

            //Left Motor Spins
            DriveTrainLeftFront.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
            DriveTrainLeftBack.spin(vex::directionType::fwd,  50, vex::velocityUnits::pct);
         }
         else if(THETA_CORDINATE < GOTO_angle){
           //Right Motor Spins
            DriveTrainRightFront.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
            DriveTrainRightBack.spin(vex::directionType::fwd,  50, vex::velocityUnits::pct);

            //Left Motor Spins
            DriveTrainLeftFront.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
            DriveTrainLeftBack.spin(vex::directionType::rev,  50, vex::velocityUnits::pct);
         }
      }
    }
  }

  //Rotates the robot to travel to the point in the correct orientaition
  while(THETA_CORDINATE < desiredHEADING - 2 || THETA_CORDINATE > desiredHEADING + 2) {
    //Right Motor Spins
    DriveTrainRightFront.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    DriveTrainRightBack.spin(vex::directionType::rev,  50, vex::velocityUnits::pct);

    //Left Motor Spins
    DriveTrainLeftFront.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
    DriveTrainLeftBack.spin(vex::directionType::fwd,  50, vex::velocityUnits::pct);
  }

}


void pre_auton(){
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  //Resets the encoder positions to 0
  EncoderX1.resetRotation();
  EncoderX2.resetRotation();
  EncoderY1.resetRotation();

  Brain.Screen.print(
    "███████████████████████████████████████████████████████████████████████████████████████"
    "█░░░░░░░░░░░░░░█░░░░░░░░░░░░░░█░░░░░░░░███░░░░░░░░░░░░░░█░░░░░░░░░░░░░░█░░░░░░░░░░░░░░█"
    "█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀░░███░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█"
    "█░░▄▀░░░░░░▄▀░░█░░▄▀░░░░░░░░░░█░░░░▄▀░░███░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█"
    "█░░▄▀░░██░░▄▀░░█░░▄▀░░███████████░░▄▀░░███████████░░▄▀░░█░░▄▀░░██░░▄▀░░█░░▄▀░░██░░▄▀░░█"
    "█░░▄▀░░░░░░▄▀░░█░░▄▀░░░░░░░░░░███░░▄▀░░███░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█"
    "█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░███░░▄▀░░███░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█"
    "█░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░███░░▄▀░░███░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█"
    "█████████░░▄▀░░█░░▄▀░░██░░▄▀░░███░░▄▀░░███████████░░▄▀░░█░░▄▀░░██░░▄▀░░█░░▄▀░░██░░▄▀░░█"
    "█░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█░░░░▄▀░░░░█░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█░░▄▀░░██░░▄▀░░█"
    "█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀░░██░░▄▀░░█"
    "█░░░░░░░░░░░░░░█░░░░░░░░░░░░░░█░░░░░░░░░░█░░░░░░░░░░░░░░█░░░░░░░░░░░░░░█░░░░░░██░░░░░░█"
    "███████████████████████████████████████████████████████████████████████████████████████"
  );


  // Brain.Screen.print(coordinates);
  wait(3, sec);
   
}

void autonomous(void){
  //AUTONOMOUS CODE
  //OdometryENABLED = true 
}


//Contoller program
void usercontrol(void){

  //Set the velocitiies at 100 percent
  // DriveTrainRightFront.setVelocity(100, percent);
  // DriveTrainLeftBack.setVelocity(100, percent); 
  // DriveTrainRightBack.setVelocity(100, percent);
  // DriveTrainLeftFront.setVelocity(100, percent);

  //Declare the variable for the velcoities
  int DriveTrainFrontLeftVelocity = 0;
  int DriveTrainFrontRightVelocity = 0;
  int DriveTrainBackRightVelocity = 0;
  int DriveTrainBackLeftVelocity = 0;

  //Whileloop to run infintlly to check for the controller axis position
  while(true){
    //Right Motor Velocities
    DriveTrainFrontRightVelocity = Controller1.Axis2.position() - Controller1.Axis4.position() - Controller1.Axis1.position();
    DriveTrainBackRightVelocity = Controller1.Axis2.position() - Controller1.Axis4.position() + Controller1.Axis1.position();

    //Left Motor Velocities
    DriveTrainFrontLeftVelocity = Controller1.Axis2.position() + Controller1.Axis4.position() + Controller1.Axis1.position();
    DriveTrainBackLeftVelocity = Controller1.Axis2.position() + Controller1.Axis4.position() - Controller1.Axis1.position();


    //Right Motor Spins
    DriveTrainRightFront.spin(vex::directionType::fwd, DriveTrainFrontRightVelocity, vex::velocityUnits::pct);
    DriveTrainRightBack.spin(vex::directionType::fwd,  DriveTrainBackRightVelocity , vex::velocityUnits::pct);

    //Left Motor Spins
    DriveTrainLeftFront.spin(vex::directionType::fwd, DriveTrainFrontLeftVelocity , vex::velocityUnits::pct);
    DriveTrainLeftBack.spin(vex::directionType::fwd,  DriveTrainBackLeftVelocity, vex::velocityUnits::pct);

    Brain.Screen.print(
    "███████████████████████████████████████████████████████████████████████████████████████"
    "█░░░░░░░░░░░░░░█░░░░░░░░░░░░░░█░░░░░░░░███░░░░░░░░░░░░░░█░░░░░░░░░░░░░░█░░░░░░░░░░░░░░█"
    "█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀░░███░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█"
    "█░░▄▀░░░░░░▄▀░░█░░▄▀░░░░░░░░░░█░░░░▄▀░░███░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█"
    "█░░▄▀░░██░░▄▀░░█░░▄▀░░███████████░░▄▀░░███████████░░▄▀░░█░░▄▀░░██░░▄▀░░█░░▄▀░░██░░▄▀░░█"
    "█░░▄▀░░░░░░▄▀░░█░░▄▀░░░░░░░░░░███░░▄▀░░███░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█"
    "█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░███░░▄▀░░███░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█"
    "█░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░███░░▄▀░░███░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█"
    "█████████░░▄▀░░█░░▄▀░░██░░▄▀░░███░░▄▀░░███████████░░▄▀░░█░░▄▀░░██░░▄▀░░█░░▄▀░░██░░▄▀░░█"
    "█░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█░░░░▄▀░░░░█░░░░░░░░░░▄▀░░█░░▄▀░░░░░░▄▀░░█░░▄▀░░██░░▄▀░░█"
    "█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀▄▀▄▀▄▀▄▀░░█░░▄▀░░██░░▄▀░░█"
    "█░░░░░░░░░░░░░░█░░░░░░░░░░░░░░█░░░░░░░░░░█░░░░░░░░░░░░░░█░░░░░░░░░░░░░░█░░░░░░██░░░░░░█"
    "███████████████████████████████████████████████████████████████████████████████████████"
    );

    // Brain.Screen.print(coordinates);
    //Prevent overhearting the Brain
    vex::task::sleep(20);
  }
}


int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}