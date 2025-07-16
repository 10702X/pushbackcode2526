//this file has all the functions used for button control

#include "vex.h"

//print each motor temperature on the brain
void printTemps() 
{     
   Brain.Screen.clearScreen();
  Brain.Screen.print(" fl T=");
  Brain.Screen.print(fl.temperature(celsius));
  Brain.Screen.print(" ml T=");
  Brain.Screen.print(ml.temperature(celsius));
  Brain.Screen.print(" bl T=");
  Brain.Screen.print(bl.temperature(celsius));
  Brain.Screen.newLine();
  Brain.Screen.print(" fr T=");
  Brain.Screen.print(fr.temperature(celsius));
  Brain.Screen.print(" mr T=");
  Brain.Screen.print(mr.temperature(celsius));
  Brain.Screen.print(" br T=");
  Brain.Screen.print(br.temperature(celsius));
  Brain.Screen.newLine();
  Brain.Screen.print(" bottom roller T=");
  Brain.Screen.print(bottomRoller.temperature(celsius));
  Brain.Screen.print(" middle roller T=");
  Brain.Screen.print(middleRoller.temperature(celsius));
  Brain.Screen.print(" top roller T=");
  Brain.Screen.print(topRoller.temperature(celsius));
}//end of printTemps()


bool diddystat = false;
bool isPuncherROut = false;

void toggleMatchload(){
  if(diddystat){
    diddy.set(false);
    diddystat = false;
  }
  else{
    diddy.set(true);
    diddystat = true;
  }
}

void togglePuncherR(){
  if(isPuncherROut){
    puncherR.set(false);
    isPuncherROut = false;
  }
  else{
    puncherR.set(true);
    isPuncherROut = true;
  }
}