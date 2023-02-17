// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.pathfinder.MathUtil;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  // Arm Motors
  CANSparkMax m_arm = new CANSparkMax(Constants.k_armMotor, MotorType.kBrushless);
  CANSparkMax m_armFollower = new CANSparkMax(Constants.k_armFollower, MotorType.kBrushless);

  // Wrist Motors
  CANSparkMax m_wrist = new CANSparkMax(Constants.k_wristMotor, MotorType.kBrushless);

  // Combine motors
  // private MotorControllerGroup m_arm = new MotorControllerGroup(m_armMotor,
  // m_armFollower);

  // Encoders
  RelativeEncoder m_armEncoder = m_arm.getEncoder();
  RelativeEncoder m_wristEncoder = m_wrist.getEncoder();
  private final double k_armTicksToDegrees = 6;
  private final double k_wristTicksToDegrees = 0.325;

  // Limit Switches
  private SparkMaxLimitSwitch m_armForwardLimit;
  private SparkMaxLimitSwitch m_armReverseLimit;
  private SparkMaxLimitSwitch m_wristForwardLimit;
  private SparkMaxLimitSwitch m_wristReverseLimit;

  // Limit Switch Positions
  private static final double k_wristForwardLimitPos = 0;
  private static final double k_wristReverseLimitPos = 0;

  // Create Pneumatics
  // Solenoid m_brake = new Solenoid(PneumaticsModuleType.CTREPCM,
  // Constants.k_rightArmBrake);
  Solenoid m_brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.k_armBrake);

  private final double k_armStartingAngle = 0;
  private final double k_wristStartingAngle = 0;

  private final double k_armDeadZoneInDegrees = 5;
  private final double k_maxArmPower = 0.3;

  private final double k_wristDeadZoneInDegrees = 2;

  // Set arm angle member variables
  private double m_armTargetAngleInDegrees = k_armStartingAngle;
  private double m_wristTargetAngleInDegrees = k_wristStartingAngle;

  // Arm PID
  private final double k_armP = 0.01; //started at .05
  private final double k_armI = 0; // 0.003
  private final double k_armD = 0.002; //started at .006
  PIDController m_armPID = new PIDController(k_armP, k_armI, k_armD);
  // ProfiledPIDController m_armPID = new ProfiledPIDController(k_armP, k_armI,
  // k_armD, new TrapezoidProfile.Constraints(450, 200));
  private final double k_armF = .004;

  // Wrist PID
  private final double k_wristP = 0.04;
  private final double k_wristI = 0;
  private final double k_wristD = 0;
  PIDController m_wristPID = new PIDController(k_wristP, k_wristI, k_wristD);
  private final double k_wristF = 0.01;

  // Set zero position
  private double m_armZero = 0;
  SmartDashboard m_smartDashboard;
  private boolean m_isEnabled = false;

  // Time for brakes to engage
  private final double k_brakeEngageTime = 0.1;
  private Timer m_brakeTimer = new Timer();
  private boolean m_wasOnTarget = false;

  ReachSubsystem m_reachSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  public ArmSubsystem(ReachSubsystem reachSubsystem, IntakeSubsystem intakeSubsystem) {
    m_reachSubsystem = reachSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    resetAngles();

    m_armZero = SmartDashboard.getNumber("Arm Zero Angle", getRawArmAngle());
    SmartDashboard.putNumber("Arm Zero Angle", m_armZero);

    // Reset motors
    m_arm.restoreFactoryDefaults();
    m_armFollower.restoreFactoryDefaults();
    m_armFollower.follow(m_arm, true);

    // m_armFollower.setInverted(true);

    m_wrist.restoreFactoryDefaults();

    // Set Brake Mode
    m_arm.setIdleMode(IdleMode.kBrake);
    m_armFollower.setIdleMode(IdleMode.kBrake);
    m_wrist.setIdleMode(IdleMode.kBrake);

    // Set arm limit switches
    m_armForwardLimit = m_arm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_armReverseLimit = m_arm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_armForwardLimit.enableLimitSwitch(true);
    m_armReverseLimit.enableLimitSwitch(true);

    // Set wrist limit switches
    m_wristForwardLimit = m_wrist.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_wristReverseLimit = m_wrist.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_wristForwardLimit.enableLimitSwitch(true);
    m_wristReverseLimit.enableLimitSwitch(true);

    // Convert ticks to degrees
    m_armEncoder.setPositionConversionFactor(k_armTicksToDegrees);
    m_wristEncoder.setPositionConversionFactor(1.0 / k_wristTicksToDegrees);
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

  public void moveToAngle(double armAngleInDegrees, double wristAngleInDegrees) {
    m_armTargetAngleInDegrees = armAngleInDegrees;
    m_wristTargetAngleInDegrees = wristAngleInDegrees;
    enableArm(true);
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

  public void setWristPower(double wristPower) {
    m_wrist.set(wristPower);
  }

  public void setWristAngle(double degrees) {
    m_wristEncoder.setPosition(degrees);
  }

  // public void setArmAngle(double degrees) {
  // m_armEncoder.setPosition(degrees);
  // }

  public void setArmBrake(boolean brake) {
    m_brake.set(!brake);
    SmartDashboard.putBoolean("Arm Brake", brake);
  }

  // Run this in the beginning
  public void resetAngles() {
    setWristAngle(k_armStartingAngle);
    // setArmAngle(k_wristStartingAngle);
  }

  public double getRawWristAngle() {
    return m_wristEncoder.getPosition();
  }

  public double getWristAngleInDegrees() {
    return m_wristEncoder.getPosition();
  }

  // Not converted by zero point
  public double getRawArmAngle() {
    return m_intakeSubsystem.getMagEncoderPosition();
  }

  public double getArmAngleDegrees() {
    // return m_armEncoder.getPosition() - m_armZero;
    return -MathUtil.normalizeDegrees(m_intakeSubsystem.getMagEncoderPosition() * 0.0883 - 288.96);
  }

  // Set arm angle to limit switch
  private void checkArmLimitSwitch() {
    // if (m_armForwardLimit.isPressed()) {
    // setArmAngle(k_armForwardLimitPos);
    // } else if (m_armReverseLimit.isPressed()) {
    // setArmAngle(k_armReverseLimitPos);
    // }
  }

  // Set wrist angle to limit switch
  private void checkWristLimitSwitch() {
    if (m_wristForwardLimit.isPressed()) {
      setWristAngle(k_wristForwardLimitPos);
    } else if (m_wristReverseLimit.isPressed()) {
      setWristAngle(k_wristReverseLimitPos);
    }
  }

  public double getArmFeedforward() {
    // double length = m_reachSubsystem.getExtentInInches();
    double length = ReachSubsystem.k_minArmLength + m_reachSubsystem.getExtentInInches(); // inches
    return -k_armF * Math.sin(Math.toRadians(m_armTargetAngleInDegrees)) * length;
  }

  public double getWristFeedforward() {
    return k_wristF * Math.sin(Math.toRadians(m_wristTargetAngleInDegrees + getArmAngleDegrees()));
  }

  private void applyArmPower() {
    double armPower = getArmFeedforward() + m_armPID.calculate(getArmAngleDegrees(), m_armTargetAngleInDegrees);
    armPower = Math.abs(armPower) > k_maxArmPower ? k_maxArmPower * Math.signum(armPower) : armPower;
    m_arm.set(armPower);
    SmartDashboard.putNumber("Arm Power", armPower);
  }

  private void runPID() {
    if (m_wasOnTarget) {
      if (isArmOnTarget()) {
        if (m_brakeTimer.get() > k_brakeEngageTime) {
          m_arm.set(0);
          SmartDashboard.putNumber("Arm Power", 0);
        } else {
          applyArmPower();
        }
      } else {
        setArmBrake(false);
        m_wasOnTarget = false;
        applyArmPower();
      }
    } else {
      if (isArmOnTarget()) {
        setArmBrake(true);
        m_wasOnTarget = true;
        m_brakeTimer.reset();
        applyArmPower();
      } else {
        applyArmPower();
      }
    }

    double wristPower = m_wristPID.calculate(getRawWristAngle(), m_wristTargetAngleInDegrees);
    // m_wrist.set(wristPower);
    SmartDashboard.putNumber("Wrist Power", wristPower);
    // System.out.println(String.format("Arm Power = %f", armPower));
    // }
    // if (isWristOnTarget()) {
    // m_wrist.set(0);
    // } else {
    // double wristPower = m_wristPID.calculate(getWristAngle(),
    // m_wristTargetAngleInDegrees);
    // m_wrist.set(wristPower + getWristAngle());
  }

  public boolean isArmOnTarget() {
    return Math.abs(getArmAngleDegrees() - m_armTargetAngleInDegrees) <= k_armDeadZoneInDegrees;
  }

  public boolean isWristOnTarget() {
    return Math.abs(getWristAngleInDegrees() - m_wristTargetAngleInDegrees) <= k_wristDeadZoneInDegrees;
  }

  public void enable(boolean enable) {
    m_isEnabled = enable;
  }

  @Override
  public void periodic() {
    // Comment out later
    SmartDashboard.putNumber("Raw Wrist Angle", getRawWristAngle());
    SmartDashboard.putNumber("Wrist Angle in Degrees", getWristAngleInDegrees());
    SmartDashboard.putNumber("Raw Arm Angle", getRawArmAngle());
    SmartDashboard.putNumber("Arm Angle", getArmAngleDegrees());
    SmartDashboard.putNumber("Mag Encoder Position", m_intakeSubsystem.getMagEncoderPosition());
    SmartDashboard.putBoolean("Arm Forward Limit", m_armForwardLimit.isPressed());
    SmartDashboard.putBoolean("Arm Forward Limit", m_armForwardLimit.isPressed());
    SmartDashboard.putBoolean("Arm Reverse Limit", m_armReverseLimit.isPressed());
    SmartDashboard.putBoolean("Wrist Forward Limit", m_wristForwardLimit.isPressed());
    SmartDashboard.putBoolean("Wrist Reverse Limit", m_wristReverseLimit.isPressed());

    checkArmLimitSwitch();
    checkWristLimitSwitch();

    if (m_isEnabled) {
      runPID();
    }

    // This method will be called once per scheduler run
  }
}
