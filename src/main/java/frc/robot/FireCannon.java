package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.CannonConstants;;

public class FireCannon {

  // private void fireTrio(Solenoid[] solonoidsToFire, Joystick joystick, int secondaryButton) {
  //   // Should be called every update loop, checks for butttons and may fire
  //   if (joystick.getRawButton(2) && joystick.getRawButton(secondaryButton)) {
  //     solonoidsToFire[0].set(true);
  //     solonoidsToFire[1].set(true);
  //     solonoidsToFire[2].set(true);
  //   } else {
  //     solonoidsToFire[0].set(false);
  //     solonoidsToFire[1].set(false);
  //     solonoidsToFire[2].set(false);
  //   }
  // }

  void fireGrouping(Joystick joystick , Solenoid[] solonoidToFire, boolean[] selected){
    if (joystick.getRawButton(1)) {
      for (int i = 0; i < CannonConstants.totalSolonoids; i++) {
        if(selected[i]){
          solonoidToFire[i].set(true);
          System.out.println("Firing Cannon :"+ i);
        }
      }
    } else {
      for (int i = 0; i < CannonConstants.totalSolonoids; i++) {
          if(!selected[i]==false){
            solonoidToFire[i].set(false);
          }
      }
    }
  }
  
}