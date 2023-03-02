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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.PositionServer;
import frc.pathfinder.MathUtil;
import frc.robot.Constants;
import frc.robot.PositionTracker;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

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

  // Time for brakes to engage
  private Timer m_brakeTimer = new Timer();
  private boolean m_wasOnTarget = false;

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

  public OptionalDouble computeTargetAngleInDegrees() {
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
    SmartDashboard.putNumber("computeTargetAngleInDegrees.targetHeight", targetHeight);
    double distance = Math.sqrt(Math.pow((targetY - robotY), 2) + Math.pow((targetX - robotX), 2));
    SmartDashboard.putNumber("computeTargetAngleInDegrees.distance", distance);
    double pivotHeight = Constants.k_pivotHeight / 12;
    double heightToTarget = targetHeight - pivotHeight;
    SmartDashboard.putNumber("computeTargetAngleInDegrees.heightToTarget", heightToTarget);
    double targetAngleInDegrees = Math.toDegrees(Math.atan2(heightToTarget, distance));
    return OptionalDouble.of(90 - targetAngleInDegrees);
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
    double pivotHeight = Constants.k_pivotHeight / 12;
    double heightToTarget = targetHeight - pivotHeight;
    double targetDistance = Math.sqrt((distance * distance) + (heightToTarget * heightToTarget));
    return OptionalDouble.of(targetDistance * 12 - Constants.k_minArmLength - 4);
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
    SmartDashboard.putBoolean("Arm Brake", brake);
  }

  // Not converted by zero point
  public double getRawArmAngle() {
    return m_intakeSubsystem.getMagEncoderPosition();
  }

  public double getArmAngleDegrees() {
    return -MathUtil.normalizeDegrees(m_intakeSubsystem.getMagEncoderPosition() * Constants.k_armDegreesPerTick - Constants.k_armZeroPoint);
  }

  // Set arm angle to limit switch
  private void checkArmLimitSwitch() {
    // if (m_armForwardLimit.isPressed()) {
    // setArmAngle(k_armForwardLimitPos);
    // } else if (m_armReverseLimit.isPressed()) {
    // setArmAngle(k_armReverseLimitPos);
    // }
  }

  public double getArmFeedforward() {
    double length = Constants.k_minArmLength + m_reachSubsystem.getExtentInInches(); // inches
    return -Constants.k_armF * Math.sin(Math.toRadians(m_armTargetAngleInDegrees)) * length;
  }

  private void applyArmPower() {
    double armPower = getArmFeedforward() + m_armPID.calculate(getArmAngleDegrees(), m_armTargetAngleInDegrees);
    armPower = Math.abs(armPower) > Constants.k_maxArmPower ? Constants.k_maxArmPower * Math.signum(armPower) : armPower;
    m_arm.set(armPower);
    SmartDashboard.putNumber("Arm Power", armPower);
  }

  private void runPID() {
    if (isArmOnTarget()) {
      m_arm.set(0);
      setArmBrake(true);
    } else if (!isArmOnTargetBraked()) {
      setArmBrake(false);
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

  public boolean isArmOnTarget() {
    SmartDashboard.putNumber("Arm Angle Error", getArmAngleDegrees() - m_armTargetAngleInDegrees);
    return Math.abs(getArmAngleDegrees() - m_armTargetAngleInDegrees) <= Constants.k_armDeadZoneInDegrees;
  }

  public boolean isArmOnTargetBraked() {
    SmartDashboard.putNumber("Arm Angle Error", getArmAngleDegrees() - m_armTargetAngleInDegrees);
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
    SmartDashboard.putNumber("Mag Encoder Position", m_intakeSubsystem.getMagEncoderPosition());
    SmartDashboard.putBoolean("Arm Forward Limit", m_armForwardLimit.isPressed());
    SmartDashboard.putBoolean("Arm Forward Limit", m_armForwardLimit.isPressed());
    SmartDashboard.putBoolean("Arm Reverse Limit", m_armReverseLimit.isPressed());
    SmartDashboard.putBoolean("Arm PID is enabled", m_isEnabled);

    checkArmLimitSwitch();

    if (m_isEnabled) {
      runPID();
    }

    // This method will be called once per scheduler run
  }
}
