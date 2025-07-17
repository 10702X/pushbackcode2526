#include "vex.h"
#include <iostream>
using namespace std;
/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

/**
 * The expected behavior is to return to the start position.
 */

void drive_test(){
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

/*******************Push Back Autons Start here *******/
/*************** declare in autons.h ******************/


/******************************************/
void AWP_solo()
{
  int startTime =Brain.Timer.time();
  fl.setPosition(0, degrees);
  ml.setPosition(0, degrees);
  bl.setPosition(0, degrees);
  fr.setPosition(0, degrees);
  mr.setPosition(0, degrees);
  br.setPosition(0, degrees);

  //move to the long goal on the right side
  chassis.set_drive_constants(10, 0.8, 0, 10, 0);
  chassis.set_heading_constants(6, .2, 0, 1, 0);
  //chassis.drive_distance(42, 270, 12, 7.1, 1, 300, 1200);//faster but not consistent
  chassis.drive_distance(43, 270, 8, 2.5, 1, 300, 1200); 

  //score on the long goal
  topRoller.spin(fwd,12,voltageUnits::volt); 
  wait(0.4,sec);
  topRoller.stop();

  //drive to the 3 balls on the right side
  chassis.drive_distance(-16, 270, 12, 6, 1, 300, 650); 
  chassis.turn_to_angle(224,12,1,300,400);

  // pick up the 3 balls
  bottomRoller.spin(fwd,12,voltageUnits::volt);  
  middleRoller.spin(vex::reverse,12,voltageUnits::volt);

  chassis.drive_distance(26, 224, 12, 6, 1, 300, 800); 
  //chassis.drive_distance(18, 224, 1.7, 6, 1, 300, 1700); 
  chassis.drive_distance(18, 224, 2, 6, 1, 300, 1700);
  bottomRoller.stop();
  middleRoller.stop();
  chassis.drive_distance(15, 224, 6, 6, 1, 300, 800);

  // score on the low center goal
  middleRoller.spin(fwd,5,voltageUnits::volt);
  bottomRoller.spin(vex::reverse,5,voltageUnits::volt); 
  wait(1.7, sec);

  //back out (moved to line 188)
  chassis.drive_distance(-18, 224, 6, 6, 1, 300, 700);
  chassis.turn_to_angle(180,12, 1, 300, 500);

  // drive to the 3 balls on the left side
  chassis.set_heading_constants(6, .6, 0, 1, 0);
  chassis.drive_distance(36, 180, 10, 6, 1, 300, 1200);
  bottomRoller.spin(fwd,12,voltageUnits::volt);  // pick up the 3 balls
  middleRoller.spin(vex::reverse,12,voltageUnits::volt);
  chassis.drive_distance(16, 180, 1.7, 6, 1, 300, 2000);
  
  //turn and drive to upper center goal
  chassis.turn_to_angle(321, 12, 1, 300, 600);
  
  bottomRoller.stop();
  middleRoller.stop();
  chassis.drive_distance(16, 321, 6, 6, 1, 300, 750);

  // shoot into upper center
  bottomRoller.spin(fwd,7,voltageUnits::volt);
  middleRoller.spin(fwd,10,voltageUnits::volt);
  topRoller.spin(vex::reverse,7,voltageUnits::volt);
  int finishTime = Brain.Timer.time();
  int timeUsed = finishTime-startTime;
  cout<<"time used = "<<timeUsed<<endl;
}

/**************right side auton****************/
/******score on right side long goal and lower center goal *********/
void rightSide_v1()
{
    fl.setPosition(0, degrees);
    ml.setPosition(0, degrees);
    bl.setPosition(0, degrees);
    fr.setPosition(0, degrees);
    mr.setPosition(0, degrees);
    br.setPosition(0, degrees);
    int startTime =Brain.Timer.time();

    //drive forward, turn and drive to long goal
    chassis.drive_distance(33, 0, 6, 6, 1, 300, 4000);
    chassis.turn_to_angle(270, 4, 1, 300, 700);
    chassis.drive_distance(20.7, 270, 6, 6, 1, 300, 1700);
    
    //score on top goal
    topRoller.spin(fwd,10,voltageUnits::volt); 
    wait(0.4,sec);
    topRoller.stop();
  
    //drive to the 3 balls on the right side
    chassis.drive_distance(-14.5, 270, 6, 6, 1, 300, 1000); 
    chassis.turn_to_angle(224,6,1,300,600);

  // pick up the 3 balls
  bottomRoller.spin(fwd,12,voltageUnits::volt);  
  middleRoller.spin(vex::reverse,12,voltageUnits::volt);

  chassis.drive_distance(24, 224, 6, 6, 1, 300, 800); 
  chassis.drive_distance(19, 224, 2, 6, 1, 300, 1700); 
  //bottomRoller.stop();
  //middleRoller.stop();
  chassis.drive_distance(10, 224, 3, 6, 1, 300, 1000);

  // score on the low center goal
  middleRoller.spin(fwd,5,voltageUnits::volt);
  bottomRoller.spin(vex::reverse,5,voltageUnits::volt); 
  wait(4, sec);

  middleRoller.stop();
  bottomRoller.stop();
  int finishTime = Brain.Timer.time();
  int timeUsed = finishTime-startTime;
  cout<<"time used = "<<timeUsed<<endl;

}

/**************left side auton****************/
/******score on left side long goal and upper center goal *********/
/******clear some blocks from the left side loader ***********/
void leftSide_v1()
{
    fl.setPosition(0, degrees);
    ml.setPosition(0, degrees);
    bl.setPosition(0, degrees);
    fr.setPosition(0, degrees);
    mr.setPosition(0, degrees);
    br.setPosition(0, degrees);
    int startTime =Brain.Timer.time();
    //drive forward and turn and drive to long goal

    //chassis.drive_distance(33, 0, 4, 6, 1, 300, 5000);
    chassis.drive_distance(33, 0, 10, 6, 1, 300, 1000, 1.0, 0, 10, 0, 0.4, 0, 1, 0 );
    chassis.turn_to_angle(90, 6, 1, 300, 550);
    chassis.drive_distance(21, 90, 8, 6, 1, 300, 850);
    
    //score on top goal
    topRoller.spin(fwd,12,voltageUnits::volt); 
    wait(0.4,sec);
    topRoller.stop();
    

    //drive to the 3 balls 
  chassis.drive_distance(-15, 90, 6, 6, 1, 300, 700); 
  cout<<"4\n";
  chassis.turn_to_angle(136,6,1,300,600);
  cout<<"5\n";

  // pick up the 3 balls
  bottomRoller.spin(fwd,12,voltageUnits::volt);  
  middleRoller.spin(vex::reverse,12,voltageUnits::volt);

  chassis.drive_distance(25, 136, 6, 6, 1, 300, 800);
  cout<<"6\n"; 
  chassis.drive_distance(10, 136, 1.7, 6, 1, 300, 1300); 
  cout<<"7\n";
  chassis.drive_distance(16, 136, 5, 6, 1, 300, 800);

  // score on the high center goal
  bottomRoller.spin(fwd,6.5,voltageUnits::volt);
  middleRoller.spin(fwd,9,voltageUnits::volt);
  topRoller.spin(vex::reverse,6.5,voltageUnits::volt);
  wait(1.5, sec);
  topRoller.stop();
  middleRoller.stop();
  bottomRoller.stop();

  //drive backwards to the matchloader
  chassis.drive_distance(-50, 136, 10, 6, 1, 300, 1200);
    
  chassis.turn_to_angle(-90, 6, 1, 300, 600);
  diddy.set(true);
  bottomRoller.spin(fwd,9,voltageUnits::volt);  
  middleRoller.spin(vex::reverse,9,voltageUnits::volt);
  chassis.drive_distance(13, -90, 3, 6, 1, 300, 800);
//  chassis.drive_distance(-1, -90, 10, 9, 1, 300, 200);
//  chassis.drive_distance(1, -90, 10, 9, 1, 300, 200);
  wait(0.5, sec);
  chassis.drive_distance(-13, -90, 6, 6, 1, 300, 800);
  cout<<"1\n";
  diddy.set(false);
  bottomRoller.stop();
  middleRoller.stop();
  
  chassis.turn_to_angle(90, 6, 1, 300, 800);
  chassis.drive_distance(12, 90, 8, 6, 1, 300, 1000);
    cout<<"2\n";
  bottomRoller.spin(fwd,9,voltageUnits::volt);
  middleRoller.spin(fwd,12,voltageUnits::volt);
  topRoller.spin(fwd,12,voltageUnits::volt);


  int finishTime = Brain.Timer.time();
  int timeUsed = finishTime-startTime;
  cout<<"time used = "<<timeUsed<<endl;
}//end of leftSide_v1()

void leftSide()
{
  fl.setPosition(0, degrees);
  ml.setPosition(0, degrees);
  bl.setPosition(0, degrees);
  fr.setPosition(0, degrees);
  mr.setPosition(0, degrees);
  br.setPosition(0, degrees);
  int startTime =Brain.Timer.time();


//drive to the loader
  chassis.drive_distance(34, 0, 8, 6, 1, 300, 1200, 1.0, 0, 10, 0, 0.4, 0, 1, 0 );
  chassis.turn_to_angle(-90, 6, 1, 300, 750);
  diddy.set(true);
  bottomRoller.spin(fwd,9,voltageUnits::volt);  
  middleRoller.spin(vex::reverse,9,voltageUnits::volt);
  chassis.drive_distance(8.5, -90, 3, 6, 1, 300, 700);
  chassis.drive_distance(-2, -90, 12, 6, 1, 300, 250);
  chassis.drive_distance(2, -90, 12, 6, 1, 300, 250);
  wait(0.8,sec);
  chassis.drive_distance(-13,-90, 8, 6, 1, 300, 700);
  diddy.set(false);
  //bottomRoller.stop();
  //middleRoller.stop();  
  
  //score to the long goal
  chassis.turn_to_angle(90, 6, 1, 300, 850);
  chassis.drive_distance(13, 90, 8, 6, 1, 300, 800);
  bottomRoller.spin(fwd,9,voltageUnits::volt);
  middleRoller.spin(fwd,12,voltageUnits::volt);
  topRoller.spin(fwd,12,voltageUnits::volt);
  wait(2.0,sec);
  bottomRoller.stop();
  middleRoller.stop();
  topRoller.stop();

  //back out and turn to the 3

  chassis.drive_distance(-16, 90, 6, 6, 1, 300, 700); 
  cout<<"4\n";
  chassis.turn_to_angle(136,6,1,300,700);
  cout<<"5\n";

  // pick up the 3 balls
  bottomRoller.spin(fwd,12,voltageUnits::volt);  
  middleRoller.spin(vex::reverse,12,voltageUnits::volt);

  chassis.drive_distance(25, 136, 6, 6, 1, 300, 800);
  cout<<"6\n"; 
  chassis.drive_distance(10, 136, 1.7, 6, 1, 300, 1300); 
  cout<<"7\n";
  chassis.drive_distance(16, 136, 5, 6, 1, 300, 700);

  // score on the high center goal
  bottomRoller.spin(fwd,6.5,voltageUnits::volt);
  middleRoller.spin(fwd,9,voltageUnits::volt);
  topRoller.spin(vex::reverse,6.5,voltageUnits::volt);
/*  wait(1.5, sec);
  topRoller.stop();
  middleRoller.stop();
  bottomRoller.stop();*/

  int finishTime = Brain.Timer.time();
  int timeUsed = finishTime-startTime;
  cout<<"time used = "<<timeUsed<<endl;
}


void matchLoadtest() {
  fl.setPosition(0, degrees);
    ml.setPosition(0, degrees);
    bl.setPosition(0, degrees);
    fr.setPosition(0, degrees);
    mr.setPosition(0, degrees);
    br.setPosition(0, degrees);
    int startTime =Brain.Timer.time();

    //drive forward, turn and drive to long goal
    chassis.drive_distance(33, 0, 6, 6, 1, 300, 1800);
    chassis.turn_to_angle(270, 4, 1, 300, 700);
    chassis.drive_distance(20.7, 270, 6, 6, 1, 300, 800);
    
    //score on top goal
    topRoller.spin(fwd,9,voltageUnits::volt); 
    wait(0.3,sec);
    topRoller.stop();
  
    //drive to the 3 balls on the right side
    chassis.drive_distance(-14.5, 270, 6, 6, 1, 300, 600); 
    chassis.turn_to_angle(224,6,1,300,600);

  // pick up the 3 balls
  bottomRoller.spin(fwd,12,voltageUnits::volt);  
  middleRoller.spin(vex::reverse,12,voltageUnits::volt);

  chassis.drive_distance(24, 224, 6, 6, 1, 300, 800); 
  chassis.drive_distance(19, 224, 1.7, 6, 1, 300, 1700); 
  //bottomRoller.stop();
  //middleRoller.stop();
  chassis.drive_distance(12.5, 224, 3, 6, 1, 300, 1400);

  // score on the low center goal
  middleRoller.spin(fwd,5,voltageUnits::volt);
  bottomRoller.spin(vex::reverse,5,voltageUnits::volt); 
  wait(2, sec);

  middleRoller.stop();
  bottomRoller.stop();
  

  chassis.drive_distance(-52, 224, 6, 6, 1, 300, 3500);
  chassis.turn_to_angle(90, 6, 1, 300, 600);
  diddy.set(true);
  bottomRoller.spin(fwd,9,voltageUnits::volt);  
  middleRoller.spin(vex::reverse,9,voltageUnits::volt);

  chassis.drive_distance(13, 90, 3, 6, 1, 300, 2300);
  chassis.drive_distance(-13, 90, 3, 6, 1, 300, 1500);
  
  int finishTime = Brain.Timer.time();
  int timeUsed = finishTime-startTime;
  cout<<"time used = "<<timeUsed<<endl;
}

// New version rightside
void rightSide() {
    fl.setPosition(0, degrees);
  ml.setPosition(0, degrees);
  bl.setPosition(0, degrees);
  fr.setPosition(0, degrees);
  mr.setPosition(0, degrees);
  br.setPosition(0, degrees);
  int startTime =Brain.Timer.time();


//drive to the loader
  chassis.drive_distance(34, 0, 8, 6, 1, 300, 1200, 1.0, 0, 10, 0, 0.4, 0, 1, 0 );
  chassis.turn_to_angle(90, 6, 1, 300, 750);
  diddy.set(true);
  bottomRoller.spin(fwd,9,voltageUnits::volt);
  middleRoller.spin(vex::reverse,9,voltageUnits::volt);
  chassis.drive_distance(8.5, 90, 3, 6, 1, 300, 700);
  chassis.drive_distance(-2, 90, 12, 6, 1, 300, 250);
  chassis.drive_distance(2, 90, 12, 6, 1, 300, 250);
  wait(0.8,sec);
  chassis.drive_distance(-13, 90, 8, 6, 1, 300, 700);
  diddy.set(false);
  //bottomRoller.stop();
  //middleRoller.stop();

  //score to the long goal
  chassis.turn_to_angle(-90, 6, 1, 300, 850);
  chassis.drive_distance(13, -90, 8, 6, 1, 300, 800);
  bottomRoller.spin(fwd,9,voltageUnits::volt);
  middleRoller.spin(fwd,12,voltageUnits::volt);
  topRoller.spin(fwd,12,voltageUnits::volt);
  wait(2.0,sec);
  bottomRoller.stop();
  middleRoller.stop();
  topRoller.stop();

  //back out and turn to the 3

  chassis.drive_distance(-14, -90, 6, 6, 1, 300, 700);
  cout<<"4\n";
  chassis.turn_to_angle(-136,6,1,300,700);
  cout<<"5\n";

  // pick up the 3 balls
  bottomRoller.spin(fwd,12,voltageUnits::volt);
  middleRoller.spin(vex::reverse,12,voltageUnits::volt);

  chassis.drive_distance(25, -136, 6, 6, 1, 300, 800);
  cout<<"6\n";
  chassis.drive_distance(10, -136, 1.7, 6, 1, 300, 1300);
  cout<<"7\n";
  chassis.drive_distance(16, -136, 5, 6, 1, 300, 700);

  // score on the high center goal
  bottomRoller.spin(fwd,6.5,voltageUnits::volt);
  middleRoller.spin(fwd,9,voltageUnits::volt);
  topRoller.spin(vex::reverse,6.5,voltageUnits::volt);
/*  wait(1.5, sec);
  topRoller.stop();
  middleRoller.stop();
  bottomRoller.stop();*/

  int finishTime = Brain.Timer.time();
  int timeUsed = finishTime-startTime;
  cout<<"time used = "<<timeUsed<<endl;
}

void FlagTest() {
  chassis.turn_to_angle(90, 6, 1, 300, 700, 15) // 15 flags
  chassis.drive_distance(10, 45, 6, 6, 1, 300, 700, 15) //also 15 flags
  // add more tests
}