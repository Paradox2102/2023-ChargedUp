// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e
 * . public static). Do not put anything functional in this class.
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

  public static final double k_feetPerTick = 18.0/204024;
  public static final double k_rampTimeSeconds = .1;
  public static final double k_maxSpeed = 19000;
  public static final double k_deadBand = .1;

  // Reach
  public static final int k_reachMotor = 0;
  public static final int k_canTimeOut = 30;
  public static final int k_topSwitch = 9;
  public static int k_bottomSwitch = 0;

  // Arm
  public static final int k_armMotor = 19; // left
  public static final int k_armFollower = 4; // right
  public static final int k_armBrake = 1;

  // Claw
  public static final int k_clawMotor = 1;
  
  // Intake
  public static final int k_leftIntakeMotor = 3;
  public static final int k_rightIntakeMotor = 2;
  public static final int k_claw = 0;
  public static final int k_openCloseMotor = 0; //New intake motor 

  public static final double k_wheelBase = 2.04;
  public static final double k_startAngleDegrees = -90;

  // LED
  public static final int k_leftLED = 1;
  public static final int k_rightLED = 0;
  public static final int k_leftLength = 40;
  public static final int k_rightLength = 40;
  // 66 red side


  
  //Arm positions
  public static final double k_groundPickupExtent = 0;
  public static final double k_groundPickupAngle = -115;
  public static final double k_midConeNodeExtent = 10;
  public static final double k_midConeNodeAngle = -62;
  public static final double k_topConeNodeAngle = -57;
  public static final double k_midCubeNodeExtent = 0;
  public static final double k_midCubeNodeAngle = -75;
  public static final double k_humanPlayerStationExtent = 13.5;
  public static final double k_humanPlayerStationAngle = -59;
  public static final double k_straightUpExtent = 0;
  public static final double k_straightUpAngle = 0;
  public static final double k_topConeExtent = 25;

  //Arm constants
  public static double k_armDegreesPerTick = 0.0895;
  public static double k_armZeroPoint = 202.04;
  public static final double k_armP = 0.02; // 0.01
  public static final double k_armI = 0;
  public static final double k_armD = 0.002;
  public static double k_armF = 0.002; //0.003
  public static final double k_armTicksToDegrees = 6;
  public static final double k_armStartingAngle = 0;
  public static final double k_maxArmPower = 0.4;
  public static final double k_armDeadZoneInDegrees = 5;
  public static final double k_armDeadZoneInDegreesBraked = 10;
  public static final double k_brakeEngageTime = 0.1;
  public static final double k_pivotHeight = 22.25;
  public static double k_armSpeedDeadzone = 5; // Degrees per second

  //Drive constants
  public static final double k_driveP = 0.08;
  public static final double k_driveI = 0.0001; 
  public static final double k_driveF = 0.045;
  public static final double k_DriveIZone = 300; 
  public static final int k_timeout = 30; 

  //Reach constants
  public static double k_minArmLength = 22; 
  public static double k_maxArmLength = 26.375; 
  public static final double k_reachTicksPerInch = 133514 / 26.375;

  //Camera constants
  public static double k_frontCameraAngle = 0.5; 
  public static double k_rearCameraAngle = 180;
  public static double k_xFrontCameraOffsetInches = - 6; 
  public static double k_xRearCameraOffsetInches = 6;

  public static boolean k_isCompetition = false;

  public Constants() {
    File f = new File("/home/lvuser/practice"); 
    if (!f.exists()) { //Competition
      //this is where code for competition robot goes
      k_isCompetition = true;
      k_armDegreesPerTick = 0.088;
      k_armZeroPoint = 0.733;
      k_bottomSwitch = 8;
      k_armF = 0.002;
      k_frontCameraAngle = -2.2; 
      k_rearCameraAngle = 180;
      k_xFrontCameraOffsetInches = 6.5; 
      k_xRearCameraOffsetInches = -6.5;
      k_maxArmLength = 28.5;
      SmartDashboard.putString("Robot name", "Updraft");
    } else { //Practice
      SmartDashboard.putString("Robot name", "Downfall");
    }
  }

  public static class OperatorConstants {}

}
