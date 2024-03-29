// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * -120 back
 */

package frc.robot.subsystems;

import java.util.OptionalDouble;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.Logger;
import frc.ApriltagsCamera.PositionServer;
import frc.pathfinder.MathUtil;
import frc.robot.Constants;
import frc.robot.PositionTracker;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private boolean m_isCube = true;
  public enum ArmPosition {HIGH, MID, LOW, SUBSTATION2, SUBSTATION1, RESET}


  // Arm Motors
  CANSparkMax m_arm = new CANSparkMax(Constants.k_armMotor, MotorType.kBrushless);
  CANSparkMax m_armFollower = new CANSparkMax(Constants.k_armFollower, MotorType.kBrushless);

  // Combine motors
  // private MotorControllerGroup m_arm = new MotorControllerGroup(m_armMotor,
  // m_armFollower);

  // Encoders
  RelativeEncoder m_armEncoder = m_arm.getEncoder();

  // Limit Switches
  private SparkMaxLimitSwitch m_armForwardLimit;
  private SparkMaxLimitSwitch m_armReverseLimit;

  // Create Pneumatics
  // Solenoid m_brake = new Solenoid(PneumaticsModuleType.CTREPCM,
  // Constants.k_rightArmBrake);
  Solenoid m_brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.k_armBrake);

  private double m_armTargetAngleInDegrees = Constants.k_armStartingAngle;


  // Arm PID
  //private final double k_armP = 0.01; //started at .05
  //private final double k_armI = 0; // 0.003
  //private final double k_armD = 0.002; //started at .006
  PIDController m_armPID = new PIDController(Constants.k_armP, Constants.k_armI, Constants.k_armD);
  // ProfiledPIDController m_armPID = new ProfiledPIDController(k_armP, k_armI,
  // k_armD, new TrapezoidProfile.Constraints(450, 200));
  //private final double k_armF = .004;


  // Set zero position
  private double m_armZero = 0;
  SmartDashboard m_smartDashboard;
  private boolean m_isEnabled = false;

  private boolean m_override = false;

  ReachSubsystem m_reachSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  PositionServer m_positionServer;
  PositionTracker m_positionTracker;

  public ArmSubsystem(ReachSubsystem reachSubsystem, IntakeSubsystem intakeSubsystem, PositionTracker positionTracker) {
    m_reachSubsystem = reachSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_positionTracker = positionTracker;
    m_positionServer = m_positionTracker.m_posServer;

    m_armZero = SmartDashboard.getNumber("Arm Zero Angle", getRawArmAngle());
    SmartDashboard.putNumber("Arm Zero Angle", m_armZero);

    // Reset motors
    m_arm.restoreFactoryDefaults();
    m_armFollower.restoreFactoryDefaults();
    m_armFollower.follow(m_arm, true);

    // m_armFollower.setInverted(true);

    // Set Brake Mode
    m_arm.setIdleMode(IdleMode.kBrake);
    m_armFollower.setIdleMode(IdleMode.kBrake);
    setArmBrake(true);

    // Set arm limit switches
    m_armForwardLimit = m_arm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_armReverseLimit = m_arm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_armForwardLimit.enableLimitSwitch(true);
    m_armReverseLimit.enableLimitSwitch(true);

    // Convert ticks to degrees
    m_armEncoder.setPositionConversionFactor(Constants.k_armTicksToDegrees);
  }

  public void storeArmZeroReference() {
    SmartDashboard.setPersistent("Arm Zero Angle");

    m_armZero = getRawArmAngle();
    // System.out.println(String.format("Zero = %f, Angle = %f", m_azimuthZero,
    // getAngleDegrees()));
    SmartDashboard.putNumber("Arm Zero Angle", m_armZero);
    // System.out.println(SmartDashboard.getNumber(m_smartDashboardName, 0));
    SmartDashboard.setPersistent("Arm Zero Angle");
  }

  public void moveToAngle(double armAngleInDegrees) {
    m_armTargetAngleInDegrees = armAngleInDegrees;
    enableArm(true);
  }

  // The position server is returning node locations, but we need to adjust them slightly
  // to get the robot into the correct position.  These will be added to the calculated values.
  private double[] m_coneTargetHeights = {12/12.0, 12/12.0, 12/12.0}; // feet
  private double[] m_cubeTargetHeights = {18/12.0, 7/12.0, 10/12.0}; // feet

  private double[] m_coneTargetExtent = {-20, -7, -10}; // inches
  private double[] m_cubeTargetExtent = {-20, -15, -19}; // inches

  private double[] m_coneWristAngle = {0, 0, 0};
  private double[] m_cubeWristAngle = {5, 15, 15};

  public double[] computeTargetAngleInDegreesExtentInInches() {
    PositionServer.Target target = m_positionServer.getTarget();
    Pose2d pose = m_positionTracker.getPose2d();
    if (target == null) {
      // return OptionalDouble.empty();
      return null;
    }

    double targetY = target.m_y;
    double targetX = target.m_x;
    double robotY = pose.getY();
    double robotX = pose.getX();
    double targetHeight = target.m_h;
    double targetWristAngle = 0;
    // Magic numbers! And again below.  Should at least be hidden in a method.  -Gavin
    if (target.m_no == 1 || target.m_no == 4 || target.m_no == 7) {
      targetHeight += m_cubeTargetHeights[target.m_level];
    } else {
      targetHeight += m_coneTargetHeights[target.m_level];
    }
    SmartDashboard.putNumber("computeTargetAngleInDegrees.targetHeight", targetHeight);
    double distance = Math.sqrt(Math.pow((targetY - robotY), 2) + Math.pow((targetX - robotX), 2));
    SmartDashboard.putNumber("computeTargetAngleInDegrees.distance", distance);
    double pivotHeight = Constants.k_pivotHeight / 12; // feet
    double heightToTarget = targetHeight - pivotHeight; // feet
    SmartDashboard.putNumber("computeTargetAngleInDegrees.heightToTarget", heightToTarget);
    double targetAngleInDegrees = Math.toDegrees(Math.atan2(heightToTarget, distance));

    // Compute extent
    double armExtentInches = Math.sqrt((distance * distance) + (heightToTarget * heightToTarget))*12 - Constants.k_minArmLength - 4;

    Logger.log("ArmSubsystem", 1, String.format("Target No: %d", target.m_no));
    if (target.m_no == 1 || target.m_no == 4 || target.m_no == 7) {
      Logger.log("ArmSubsystem", 1, String.format("level=%d, extent=%f", target.m_level,
      m_cubeTargetExtent[target.m_level]));
      armExtentInches += m_cubeTargetExtent[target.m_level];
    } else {
      armExtentInches += m_coneTargetExtent[target.m_level];
    }

    if (armExtentInches < 0){
      armExtentInches = 0;
    }
    else if (armExtentInches > Constants.k_maxArmLength) {
      armExtentInches = Constants.k_maxArmLength;
    }

    if (target.m_no == 1 || target.m_no == 4 || target.m_no == 7) {
      targetWristAngle = m_cubeWristAngle[target.m_level];
    } else {
      targetWristAngle = m_coneWristAngle[target.m_level];
    }

    double[] ret = { 90 - targetAngleInDegrees, armExtentInches, targetWristAngle};

    return(ret);

    // return OptionalDouble.of(90 - targetAngleInDegrees);
  }

  public void setGamePiece(boolean isCube) {
    m_isCube = isCube;
  }

  public boolean isGamePieceCube() {
    return m_isCube;
  }

  public OptionalDouble computeTargetDistance() {
    PositionServer.Target target = m_positionServer.getTarget(); 
    Pose2d pose = m_positionTracker.getPose2d();
    if (target == null) {
      return OptionalDouble.empty();
    }
    double targetY = target.m_y;
    double targetX = target.m_x;
    double robotY = pose.getY();
    double robotX = pose.getX();
    double targetHeight = target.m_h;
    double distance = Math.sqrt(Math.pow((targetY - robotY), 2) + Math.pow((targetX - robotX), 2));
    SmartDashboard.putNumber("computeTargetDistance.distance", distance);
    double pivotHeight = Constants.k_pivotHeight / 12;
    double heightToTarget = targetHeight - pivotHeight;
    double targetDistance = Math.sqrt((distance * distance) + (heightToTarget * heightToTarget))*12;
    // if (target.m_no == 1 || target.m_no == 4 || target.m_no == 7) {
    //   targetDistance += m_cubeTargetExtent[target.m_level];
    // } else {
    //   targetDistance += m_coneTargetExtent[target.m_level];
    // }
    if (targetDistance < 0){
      targetDistance = 0;
    }
    else if (targetDistance > Constants.k_maxArmLength) {
      targetDistance = Constants.k_maxArmLength;
    }
    return OptionalDouble.of(targetDistance - Constants.k_minArmLength - 4);
  }

  public void enableArm(boolean enable) {
    m_isEnabled = enable;
    if (!enable) {
      setArmPower(0);
    }
  }

  public void setArmPower(double armPower) {
    m_arm.set(armPower);
  }

  // public void setArmAngle(double degrees) {
  // m_armEncoder.setPosition(degrees);
  // }

  public void setArmBrake(boolean brake) {
    m_brake.set(!brake);
    SmartDashboard.putNumber("Arm Brake", brake ? 1 : 0);
  }

  // Not converted by zero point
  public double getRawArmAngle() {
    return m_intakeSubsystem.getMagEncoderPosition();
  }

  public double getArmAngleDegrees() {
    if (Constants.k_isCompetition) {
      return MathUtil.normalizeDegrees(m_intakeSubsystem.getMagEncoderPosition() * Constants.k_armDegreesPerTick - Constants.k_armZeroPoint);
    } else {
      return -MathUtil.normalizeDegrees(m_intakeSubsystem.getMagEncoderPosition() * Constants.k_armDegreesPerTick - Constants.k_armZeroPoint);
    }
  }

  // Set arm angle to limit switch
  private void checkArmLimitSwitch() {
    // if (m_armForwardLimit.isPressed()) {
    // setArmAngle(k_armForwardLimitPos);
    // } else if (m_armReverseLimit.isPressed()) {
    // setArmAngle(k_armReverseLimitPos);
    // }
  }

  public void addToTargetAngle(double degrees, boolean switchSides) {
    m_armTargetAngleInDegrees += switchSides ? degrees : -degrees;
    m_override = true;
  } 

  public double getArmFeedforward() {
    double length = Constants.k_minArmLength + m_reachSubsystem.getExtentInInches(); // inches
    if (Constants.k_isCompetition) {
      return -Constants.k_armF * Math.sin(Math.toRadians(m_armTargetAngleInDegrees)) * length;
    }
    return -Constants.k_armF * Math.sin(Math.toRadians(m_armTargetAngleInDegrees)) * length;
  }

  private void applyArmPower() {
    double armPower = getArmFeedforward() + m_armPID.calculate(getArmAngleDegrees(), m_armTargetAngleInDegrees);
    armPower = Math.abs(armPower) > Constants.k_maxArmPower ? Constants.k_maxArmPower * Math.signum(armPower) : armPower;
    // armPower = Math.abs(armPower) < 0.05 ? 0.05 * Math.signum(armPower) : armPower;
    m_arm.set(armPower);
    SmartDashboard.putNumber("Arm Power", armPower);
  }

  private void runPID() {
    // applyArmPower();
    if (isArmOnTarget()) {
      if (Constants.k_isCompetition) {
        m_arm.set(0);
        setArmBrake(true);
      } else {
        applyArmPower(); 
      }
      // setArmBrake(true);
    } else {
        if (!isArmOnTargetBraked() || m_override) {
          setArmBrake(false);
          m_override = false;
        }
      applyArmPower();
    }
    
    // if (m_wasOnTarget) {
    //   if (isArmOnTarget()) {
    //     if (m_brakeTimer.get() > Constants.k_brakeEngageTime) {
    //       m_arm.set(0);
    //       SmartDashboard.putNumber("Arm Power", 0);
    //     } else {
    //       setArmBrake(false);
    //       applyArmPower();
    //     }
    //   } else {
    //     setArmBrake(false);
    //     m_wasOnTarget = false;
    //     applyArmPower();
    //   }
    // } else {
    //   if (isArmOnTarget()) {

    //     setArmBrake(true);
    //     m_wasOnTarget = true;
    //     m_brakeTimer.reset();
    //     applyArmPower();
    //   } else {
    //     setArmBrake(false);
    //     applyArmPower();
    //   }
    // }
  }

  public double getArmSpeed() {
    return m_intakeSubsystem.getMagEncoderVelocity() * Constants.k_armDegreesPerTick * 10;
  }

  public boolean isArmOnTarget() {
    SmartDashboard.putNumber("Arm Angle Error", getArmAngleDegrees() - m_armTargetAngleInDegrees);
    SmartDashboard.putNumber("Arm DeadZone In Degrees", Constants.k_armDeadZoneInDegrees);
    return Math.abs(getArmAngleDegrees() - m_armTargetAngleInDegrees) <= Constants.k_armDeadZoneInDegrees && Math.abs(getArmSpeed()) < Constants.k_armSpeedDeadzone;
  }

  public boolean isArmOnTargetBraked() {
    SmartDashboard.putNumber("Arm Angle Error", getArmAngleDegrees() - m_armTargetAngleInDegrees);
    SmartDashboard.putNumber("Arm DeadZone In Degrees Braked", Constants.k_armDeadZoneInDegreesBraked);
    return Math.abs(getArmAngleDegrees() - m_armTargetAngleInDegrees) <= Constants.k_armDeadZoneInDegreesBraked;
  }

  public void enable(boolean enable) {
    m_isEnabled = enable;
  }

  @Override
  public void periodic() {
    // Comment out later
    SmartDashboard.putNumber("Raw Arm Angle", getRawArmAngle());
    SmartDashboard.putNumber("Arm Angle", getArmAngleDegrees());
    SmartDashboard.putBoolean("Grab Cube", isGamePieceCube());
    // SmartDashboard.putNumber("Mag Encoder Position", m_intakeSubsystem.getMagEncoderPosition());
    // SmartDashboard.putBoolean("Arm Forward Limit", m_armForwardLimit.isPressed());
    // SmartDashboard.putBoolean("Arm Forward Limit", m_armForwardLimit.isPressed());
    // SmartDashboard.putBoolean("Arm Reverse Limit", m_armReverseLimit.isPressed());
    // SmartDashboard.putBoolean("Arm PID is enabled", m_isEnabled);
    // SmartDashboard.putBoolean("Is Arm On Target", isArmOnTarget());
    SmartDashboard.putNumber("Arm Speed", getArmSpeed());

    checkArmLimitSwitch();

    if (m_isEnabled) {
      runPID();
    }

    // This method will be called once per scheduler run
  }
}
