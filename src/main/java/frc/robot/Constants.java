// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e
 * . public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to
 * statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // The following values are either shared between competition and practice
  // robots,
  // or are for the practice robot only. Selected (non-final) values are modified
  // in the constructor for the competition robot.

  // Set controller to xbox or joystick
  public static final boolean k_xboxController = false;

  // Drive
  public static final int k_rightDrive = 8; // CAN id
  public static final int k_rightFollower = 9; // CAN id
  public static final int k_leftDrive = 11; // CAN id
  public static final int k_leftFollower = 10; // CAN id

  public static final double k_feetPerTick = 18.0 / 204024;
  public static final double k_rampTimeSeconds = .1;
  public static final double k_maxSpeed = 19000; // not used?
  public static final double k_deadBand = .1;

  // Reach
  public static final int k_reachMotor = 0; // CAN id
  public static final int k_canTimeOut = 30; // ms
  public static final int k_topSwitch = 9; // DIO port
  public static int k_bottomSwitch = 0; // DIO port

  // Arm
  public static final int k_armMotor = 19; // left CAN id
  public static final int k_armFollower = 4; // right CAN id
  public static final int k_armBrake = 1; // pneumatic channel

  // Claw
  public static final int k_clawMotor = 1; // CAN id

  // Old Intake
  public static final int k_leftIntakeMotor = 3; // CAN id
  public static final int k_rightIntakeMotor = 2; // CAN id
  public static final int k_claw = 0; // pneumatic channel
  public static final int k_openCloseMotor = 0; // New intake motor - Not used?
  public static double k_intakePower = -0.4; 
  public static double k_outakePower = 0.5; 

  public static final double k_wheelBase = 2.04; // feet
  public static final double k_startAngleDegrees = -90;

  // LED
  public static final int k_leftLED = 9; // PWM port
  public static final int k_rightLED = 0; // PWM port
  public static final int k_leftLength = 60;
  public static final int k_rightLength = 40;
  // 66 red side

  public static double k_autoOutake = .5;

  // New intake
  public static final int k_newIntakeMotor = 7;
  public static final int k_armEncoder = 3;   // This is a TalonSRX which is ONLY used to read the abs mag encoder for the arm
  public static final double k_intakeStallSpeed = 300;
  public static final double k_intakeMinPower = 0.075;

  // Wrist
  public static final int k_wristMotor = 1;
  public static final double k_wristResetPosition = 29.5;

  // Wrist Positions Cube
  public static double k_groundPickupWristCUBE = 17;
  public static double k_midNodeWristCUBE = 20;
  public static double k_topNodeWristCUBE = 5;
  public static double k_humanPlayerStationWristCUBE = 27;

  // Wrist Positions Cone
  public static double k_groundPickupWristCONE = -5;
  public static double k_midNodeWristCONE = 17;
  public static double k_topNodeWristCONE = 18;
  public static double k_humanPlayerStationWristCONE = 23;

  // Arm positions Cube PRACTICE ROBOT
  public static double k_groundPickupExtentCUBE = 0;
  public static double k_groundPickupAngleCUBE = -93;
  public static double k_midNodeExtentCUBE = 10;
  public static double k_midNodeAngleCUBE = -62;
  public static double k_topNodeExtentCUBE = 10;
  public static double k_topNodeAngleCUBE = -57;
  public static double k_midCubeNodeExtentCUBE = 0;
  public static double k_midCubeNodeAngleCUBE = -75;
  public static double k_humanPlayerStationExtentCUBE = 13.5;
  public static double k_humanPlayerStationAngleCUBE = -59;

  // Arm Positions Cone PRACTICE ROBOT
  public static double k_groundPickupExtentCONE = 0;
  public static double k_groundPickupAngleCONE = -93;
  public static double k_midNodeExtentCONE = 10;
  public static double k_midNodeAngleCONE = -62;
  public static double k_topExtentCONE = 25;
  public static double k_topNodeAngleCONE = -57;
  public static double k_midCubeNodeExtentCONE = 0;
  public static double k_midCubeNodeAngleCONE = -75;
  public static double k_humanPlayerStationExtentCONE = 11.5;
  public static double k_humanPlayerStationAngleCONE = -59;

  // Arm position Straight Up
  public static final double k_straightUpExtent = 0;
  public static final double k_straightUpAngle = 0;
  public static final double k_straightUpWrist = 0;


  // Arm constants
  public static double k_armDegreesPerTick = 0.0895;
  public static double k_armZeroPoint = 202.04;
  public static double k_armP = 0.02; // 0.01
  public static double k_armI = 0;
  public static double k_armD = 0.002;
  public static double k_armF = 0.002; // 0.003
  public static final double k_armTicksToDegrees = 6;
  public static final double k_armStartingAngle = 0;
  public static double k_maxArmPower = 0.4;
  public static double k_armDeadZoneInDegrees = 5;
  public static double k_armDeadZoneInDegreesBraked = 10;
  public static final double k_brakeEngageTime = 0.1; // seconds
  public static final double k_pivotHeight = 22.25; // inches
  public static double k_armSpeedDeadzone = 5; // Degrees per second

  // Drive constants
  public static final double k_driveP = 0.08;
  public static final double k_driveI = 0.0001;
  public static final double k_driveF = 0.045;
  public static final double k_DriveIZone = 300;
  public static final int k_timeout = 30;

  // Reach constants
  public static double k_minArmLength = 18; // inches; distance from pivot to CoM when extent=0 | 22
  public static double k_maxArmLength = 26.375; // inches; maximum extension beyond minimum
  public static final double k_reachTicksPerInch = 133514 / 26.375;

  // Camera constants
  public static double k_frontCameraAngle = 0.8;
  public static double k_rearCameraAngle = 180;
  public static double k_xFrontCameraOffsetInches = -8.8;
  public static double k_xRearCameraOffsetInches = 6;

  public static boolean k_isCompetition = false;

  public Constants() {
    File f = new File("/home/lvuser/practice");
    if (!f.exists()) { // Competition
      // this is where code for the competition robot goes
      k_isCompetition = true;
      k_armDegreesPerTick = 0.08731;
      k_armZeroPoint = 179.439;
      k_bottomSwitch = 8;
      k_armF = 0.004;
      k_frontCameraAngle = -3.2;
      k_rearCameraAngle = 179;
      k_xFrontCameraOffsetInches = 6.5;
      k_xRearCameraOffsetInches = -7.5;
      k_maxArmLength = 28.5;
      k_intakePower = 0.3; 
      k_outakePower = -0.6; 

      // Arm positions Cube
      k_groundPickupExtentCUBE = 0;
      k_groundPickupAngleCUBE = -93;
      k_midNodeExtentCUBE = 0;
      k_midNodeAngleCUBE = -10;
      k_topNodeExtentCUBE = 5;
      k_topNodeAngleCUBE = -30;
      k_midCubeNodeExtentCUBE = 0;
      k_midCubeNodeAngleCUBE = -75;
      k_humanPlayerStationExtentCUBE = 6;
      k_humanPlayerStationAngleCUBE = -21;

      // Arm Positions Cone
      k_groundPickupExtentCONE = 1;
      k_groundPickupAngleCONE = -115;
      k_midNodeExtentCONE = 7;
      k_midNodeAngleCONE = -47;
      k_topExtentCONE = 28;
      k_topNodeAngleCONE = -47;
      k_midCubeNodeExtentCONE = 0;
      k_midCubeNodeAngleCONE = -18;
      k_humanPlayerStationExtentCONE = 8.5;
      k_humanPlayerStationAngleCONE = -21;

      k_autoOutake = 1;

      // Arm PID
      k_armP = 0.02;
      k_armD = 0.002;

      k_armDeadZoneInDegrees = 6;
      k_armDeadZoneInDegreesBraked = 6;
      k_maxArmPower = 0.4;

      SmartDashboard.putString("Robot name", "Updraft");
    } else { // Practice
      SmartDashboard.putString("Robot name", "Downfall");
    }
  }

  public static class OperatorConstants {
  }

}
