package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class FireCannon {
    
  private int totalSolonoids = 6;

  public void fireCannon(Solenoid solonoidToFire, Joystick joystick, int secondaryButton) {
    //! Obsolete, do not use. 
    if (joystick.getTrigger() && joystick.getRawButton(1) && joystick.getRawButton(secondaryButton)) {
      solonoidToFire.set(true);
    } else {
      solonoidToFire.set(false);
    }
  }

  private void fireTrio(Solenoid[] solonoidsToFire, Joystick joystick, int secondaryButton) {
    // Should be called every update loop, checks for butttons and may fire
    if (joystick.getRawButton(2) && joystick.getRawButton(secondaryButton)) {
      solonoidsToFire[0].set(true);
      solonoidsToFire[1].set(true);
      solonoidsToFire[2].set(true);
    } else {
      solonoidsToFire[0].set(false);
      solonoidsToFire[1].set(false);
      solonoidsToFire[2].set(false);
    }
  }

  void fireGrouping(Joystick joystick , Solenoid[] solonoidToFire, boolean[] selected){
    if (joystick.getRawButton(1)) {
      for (int i = 0; i < totalSolonoids; i++) {
        if(selected[i]){
          solonoidToFire[i].set(true);
          System.out.println("Firing Cannon :"+ i);
        }
      }
    } else {
      for (int i = 0; i < totalSolonoids; i++) {
          if(!selected[i]==false){
            solonoidToFire[i].set(false);
          }
      }
    }
  }
  
}