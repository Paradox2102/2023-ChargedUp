// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to 
 * statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Set controller to xbox or joystick
  public static final boolean k_xboxController = false;

  // Drive
  public static final int k_rightDrive = 8;
  public static final int k_rightFollower = 9;
  public static final int k_leftDrive = 11;
  public static final int k_leftFollower = 10;

  public static final double k_feetPerTick = 0.1;
  public static final double k_rampTimeSeconds = .1;
  public static final double k_maxSpeed = 19000;
  public static final double k_deadBand = .1;

  // Reach
  public static final int k_reachMotor = 0;
  public static final int k_canTimeOut = 30;
  public static final int k_topSwitch = 9;
  public static final int k_bottomSwitch = 0;

  // Arm
  public static final int k_armMotor = 19; // left
  public static final int k_armFollower = 4; // right
  // Wrist
  public static final int k_wristMotor = 1;
  
  // Intake
  public static final int k_leftIntakeMotor = 3;
  public static final int k_rightIntakeMotor = 2;

  // Arm Brakes
  public static final int k_rightArmBrake = 0;
  public static final int k_leftArmBrake = 0;
  
  public static class OperatorConstants {}
}
