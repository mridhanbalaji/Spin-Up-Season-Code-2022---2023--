

void GoToPosition(int desiredX, int desiredY, int desiredHEADING, int variance){
  //Runs this task, until the desired cordinates are met
  while(desiredY - variance > Y_CORDINATE || desiredY + variance < Y_CORDINATE) || desiredX - variance > X_CORDINATE || desiredX + variance < X_CORDINATE){
  
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

    while(THETA_CORDINATE < GOTO_angle - 2 || THETA_CORDINATE > GOTO_angle + 2) {
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

    //Right Motor Spins
    DriveTrainRightFront.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);
    DriveTrainRightBack.spin(vex::directionType::fwd,  75, vex::velocityUnits::pct);

    //Left Motor Spins
    DriveTrainLeftFront.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);
    DriveTrainLeftBack.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);

    wait(1500, msec);

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

