using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor DriveTrainLeftFront;
extern motor DriveTrainLeftBack;
extern motor DriveTrainRightFront;
extern motor DriveTrainRightBack;
extern encoder EncoderX1;
extern encoder EncoderX2;
extern encoder EncoderY1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );