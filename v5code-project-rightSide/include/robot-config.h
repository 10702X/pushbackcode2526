using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor fl;
extern motor ml;
extern motor bl;
extern motor fr;
extern motor mr;
extern motor br;
extern motor topRoller;
extern motor middleRoller;
extern motor bottomRoller;
extern inertial GaryInertial;
extern digital_out diddy;
extern digital_out puncherR;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );