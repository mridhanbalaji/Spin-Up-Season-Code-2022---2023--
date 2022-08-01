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



void pre_auton(){
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
   
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