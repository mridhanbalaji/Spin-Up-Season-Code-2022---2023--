/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Mridhan Balaji                                   */
/*    Created:      Wed Aug 17 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// EncoderY1            encoder       A, B            
// EncoderX2            encoder       C, D            
// EncoderX1            encoder       G, H            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

//A global instance of Competition
competition Competition;

void pre_auton(){
  EncoderX1.resetRotation();
  EncoderY1.resetRotation();
  EncoderX2.resetRotation();
  EncoderX1.setRotation(0, rotationUnits::raw);
  EncoderX2.setRotation(0, rotationUnits::raw);
  EncoderY1.setRotation(0, rotationUnits::raw);
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

void autonomous(void){ 
}

void usercontrol(void){

const double RadToDeg = (180/M_PI);
const double TwoPI = 2 * M_PI;
const double C = (TwoPI * 8.255/360);  //Creates a Constant CM per revolution

//Declares the Cordinates globaly
double X_Cord;
double Y_Cord;
double THETA_CORD;

int THETA_CORDINATE;
int X_CORDINATE;
int Y_CORDINATE;

//Odometry function
//PROVIDE THETA IN RADIANS

//Assigns the the starting cordinates
//CAN CHANGE ACORDING TO STARTING POSITION IN AUTON
X_Cord = 0;
Y_Cord = 0;
THETA_CORD = 0;

// std::list<double> cordinates = {}; 

//Initilises the oldposition variable
int oldpositionX1 = 0;
int oldpositionX2 = 0;
int oldpositionY1 = 0;

//Initilises the currentposition variable
int currentpositionX1 = 0;
int currentpositionX2= 0;
int currentpositionY1= 0;

//Sets the standerd values needed for calculaition
//Change the L value to calibrate
double L = 39.37;         // Distance between the 2 horizontal encoders (Cm)
double B = 18.6055;         // Distance between the center of the 2 horizontal encoders and the vertical encoder (Cm)
  
  //Loops the odometry to constantly track and calculate the values
  while(true){

    //sets the old positionm to the currentposition at the begining of each iteraition
    oldpositionX1 = currentpositionX1;
    oldpositionX2 = currentpositionX2;
    oldpositionY1 = currentpositionY1;

    //Gets the current position of the Encoders in revolutions
    currentpositionX1 = -1 * EncoderX1.rotation(rotationUnits::raw);
    currentpositionX2 = EncoderX2.rotation(rotationUnits::raw);
    currentpositionY1= EncoderY1.rotation(rotationUnits::raw);
    
    //IF NOT WORKING PROPERLLY USE THIS:
    // currentpositionX1 = -(EncoderX1.rotation(rev));
    // currentpositionX2 = -(EncoderX2.rotation(rev));
    // currentpositionY1= EncoderX2.rotation(rev);

    //Gets the encoder position change in revolutions
    int N1 = currentpositionX1 - oldpositionX1;
    int N2 = currentpositionX2 - oldpositionX2;
    int N3 = currentpositionY1 - oldpositionY1;

    //Calculates the distance change for the robot for X, Y, and Heading
    double dX = C * (N1 + N2) / 2.0;
    double dY =  C * (N3 - ((N2 - N1)  * B / L));
    double dTHETA = C * (N2 - N1) / L;

    //Calculates the robots cordinates relitive to the field by adding the the change in 
    //cordinates relitive to the direction
    double THETA = THETA_CORD + (dTHETA/2.0);
    X_Cord += dX *  cos(THETA) - dY * sin(THETA);
    Y_Cord += dX * sin(THETA) + dY * cos(THETA);
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

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(50,50,true,"Cordinates: %d, %d, %d", X_CORDINATE, Y_CORDINATE, THETA_CORDINATE); 
    //Sets the odometry to stop for 10 miliseconds to prevent overheating and crashing
    vex::task::sleep(20);
}
}

int main() {
   // Run the pre-autonomous function.
  pre_auton();

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

