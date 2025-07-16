#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor fl = motor(PORT18, ratio6_1, true);
motor ml = motor(PORT19, ratio6_1, true);
motor bl = motor(PORT20, ratio6_1, true);
motor fr = motor(PORT17, ratio6_1, false);
motor mr = motor(PORT14, ratio6_1, false);
motor br = motor(PORT16, ratio6_1, false);
motor topRoller = motor(PORT10, ratio6_1, false);
motor middleRoller = motor(PORT15, ratio18_1, false);
motor bottomRoller = motor(PORT9, ratio18_1, false);
inertial GaryInertial = inertial(PORT8);
digital_out diddy = digital_out(Brain.ThreeWirePort.A);
digital_out puncherR = digital_out(Brain.ThreeWirePort.B);

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