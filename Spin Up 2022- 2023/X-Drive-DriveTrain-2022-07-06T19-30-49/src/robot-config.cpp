#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor DriveTrainLeftFront = motor(PORT1, ratio18_1, false);
motor DriveTrainLeftBack = motor(PORT20, ratio18_1, false);
motor DriveTrainRightFront = motor(PORT2, ratio18_1, true);
motor DriveTrainRightBack = motor(PORT21, ratio18_1, true);
encoder EncoderX1 = encoder(Brain.ThreeWirePort.A);
encoder EncoderX2 = encoder(Brain.ThreeWirePort.C);
encoder EncoderY1 = encoder(Brain.ThreeWirePort.G);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}