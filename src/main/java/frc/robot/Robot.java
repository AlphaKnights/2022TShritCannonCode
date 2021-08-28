/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants.CannonConstants;
import frc.robot.Constants.ControlConstants;


/**
 * * This is the code for the T-Shirt Cannon. Works pretty good.
 * 
 * Basic info: Drives with logitech flight stick as stick 0, air compressor toggled with button 2,
 * press 7-12 to toggle which cannons to fire, selected cannons are shown on shuffleboard
 * pull trigger to fire cannons.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private SpeedControllerGroup leftSideSpeedControllerGroup;
  private SpeedControllerGroup rightSideSpeedControllerGroup;

  private Solenoid Solenoid_1;
  private Solenoid Solenoid_2;
  private Solenoid Solenoid_3;
  private Solenoid Solenoid_4;
  private Solenoid Solenoid_5;
  private Solenoid Solenoid_6;

  private Joystick m_stick;
  private Joystick t_stick;
  private Solenoid[] allSolonoids;

  private boolean[] selectedCannons = {false,false,false,false,false,false};

  private Compressor TestCompressor;

  public double integrative;
  public double derivitive;
  public double previousError;
  private FireCannon cannon;
  private ShuffleboardTab tab;

  private NetworkTableEntry Cannon1;
  private NetworkTableEntry Cannon2;
  private NetworkTableEntry Cannon3;
  private NetworkTableEntry Cannon4;
  private NetworkTableEntry Cannon5;
  private NetworkTableEntry Cannon6;
  private NetworkTableEntry[] cannons = {Cannon1, Cannon2, Cannon3, Cannon4, Cannon5, Cannon6};

  @Override
  public void robotInit() {
    cannon = new FireCannon();
    Solenoid_1 = new Solenoid(0);
    Solenoid_2 = new Solenoid(1);
    Solenoid_3 = new Solenoid(2);
    Solenoid_4 = new Solenoid(3);
    Solenoid_5 = new Solenoid(4);
    Solenoid_6 = new Solenoid(5);
    TestCompressor = new Compressor(0);
    TestCompressor.setClosedLoopControl(false);

    m_stick = new Joystick(ControlConstants.mJoystickChannel);
    t_stick = new Joystick(ControlConstants.tJoystickChannel);

    leftSideSpeedControllerGroup = new SpeedControllerGroup(new WPI_TalonSRX(1), new WPI_TalonSRX(2));
    rightSideSpeedControllerGroup = new SpeedControllerGroup(new WPI_TalonSRX(3), new WPI_TalonSRX(4));
    m_myRobot = new DifferentialDrive(leftSideSpeedControllerGroup, rightSideSpeedControllerGroup);
    allSolonoids = new Solenoid[] {Solenoid_1, Solenoid_2, Solenoid_3, Solenoid_4, Solenoid_5, Solenoid_6};

    tab = Shuffleboard.getTab("T-Shirt Cannon");
    Cannon1 = tab.addPersistent("Cannon 1:", false)
    .withWidget("Toggle Button")
    .getEntry();
    Cannon2 = tab.addPersistent("Cannon 2:", false)
    .withWidget("Toggle Button")
    .getEntry();
    Cannon3 = tab.addPersistent("Cannon 3:", false)
    .withWidget("Toggle Button")
    .getEntry();
    Cannon4 = tab.addPersistent("Cannon 4:", false)
    .withWidget("Toggle Button")
    .getEntry();
    Cannon5 = tab.addPersistent("Cannon 5:", false)
    .withWidget("Toggle Button")
    .getEntry();
    Cannon6 = tab.addPersistent("Cannon 6:", false)
    .withWidget("Toggle Button")
    .getEntry();

    System.out.println("init success");
  }

  @Override
  public void teleopPeriodic() {
    //? Only run the compressor if button 12 is held
    if (t_stick.getRawButtonPressed(2)){
      if(!TestCompressor.getClosedLoopControl()){
        TestCompressor.setClosedLoopControl(true);
        System.out.println("Pressurizing!");
      }
      else if(TestCompressor.getClosedLoopControl()){
        TestCompressor.setClosedLoopControl(false);
        System.out.println("Pressurizing!");
      }
    } 

    for (int i = 0; i < CannonConstants.totalSolonoids; i++) {
      if(t_stick.getRawButtonPressed(ControlConstants.buttonCannonMappings[i])){
        if(cannons[i].getBoolean(false)){
          cannons[i].setBoolean(false);
        }
        else{
          cannons[i].setBoolean(true);
        }
      }
      selectedCannons[i] = cannons[i].getBoolean(false);
    }

    cannon.fireGrouping(t_stick, allSolonoids, selectedCannons);

    // Make robit go
    double[] movementList = adjustJoystickInput(-m_stick.getY(), m_stick.getX(), m_stick.getThrottle());
    m_myRobot.arcadeDrive(movementList[0], movementList[1]);
    //System.out.println(m_gyro.getAngle());
  }

  @Override
  public void testInit() {
    System.out.println("Test Init!");
  }

  @Override
  public void testPeriodic() {
    double lSpeed = 0;
    double rSpeed = 0;
    double Kp = 0.1f;
    double I = 0;
    double D = 0;
    double min_command = 0.09f;

    edu.wpi.first.networktables.NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry targetx = table.getEntry("tx");
    NetworkTableEntry targetv = table.getEntry("tv");

    
    double tx = targetx.getDouble(0);
    double tv = targetv.getDouble(0);
    System.out.println(tx);
    double steering_adjust = 0.0f;
    double error = tx;

    if(m_stick.getRawButton(7) == true) {
      if(tv == 0){
          steering_adjust = 75;
        }
      else{
          integrative += (error*0.2);
          derivitive = (error - previousError) / 0.2;
          if(tx > 1.0){
              steering_adjust = (Kp*error + I*integrative + D*derivitive) - min_command;
            }
          else if(tx < 1.0){
              steering_adjust = (Kp*error + I*integrative + D*derivitive) + min_command;
            }
        }
      lSpeed = steering_adjust;
      rSpeed = -steering_adjust;

      m_myRobot.tankDrive(lSpeed, rSpeed);
      previousError = error;
    }
  }

  private double[] adjustJoystickInput(double yPower, double zPower, double Throttle) {
    double adjustedThrottle = Math.sqrt(((-1 * Throttle) + 1) / 2); // Seems like a pretty good throttle curve
    double yPowerOut = yPower * adjustedThrottle;
    double zPowerOut = zPower * adjustedThrottle * 0.85;

    double[] outputList = new double[] { yPowerOut, zPowerOut };
    return outputList;
  }
}