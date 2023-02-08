// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  // Arm Motors
  CANSparkMax m_arm = new CANSparkMax(Constants.k_armMotor, MotorType.kBrushless);
  CANSparkMax m_armFollower = new CANSparkMax(Constants.k_armFollower, MotorType.kBrushless);

  // Wrist Motors
  CANSparkMax m_wrist = new CANSparkMax(Constants.k_wristMotor, MotorType.kBrushless);

  // Combine motors
  // private MotorControllerGroup m_arm = new MotorControllerGroup(m_armMotor, m_armFollower);

  // Encoders
  RelativeEncoder m_armEncoder = m_arm.getEncoder();
  RelativeEncoder m_wristEncoder = m_wrist.getEncoder();
  private final double k_armTicksToDegrees = 1;
  private final double k_wristTicksToDegrees = 1;

  // Limit Switches
  private SparkMaxLimitSwitch m_armForwardLimit;
  private SparkMaxLimitSwitch m_armReverseLimit;
  private SparkMaxLimitSwitch m_wristForwardLimit;
  private SparkMaxLimitSwitch m_wristReverseLimit;

  // Limit Switch Positions
  private static final double k_armForwardLimitPos = 0;
  private static final double k_armReverseLimitPos = 0;
  private static final double k_wristForwardLimitPos = 0;
  private static final double k_wristReverseLimitPos = 0;

  // Create Pneumatics
//  Solenoid m_brake = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.k_rightArmBrake);
  Solenoid m_brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.k_rightArmBrake);


  private final double k_armStartingAngle = 0;
  private final double k_wristStartingAngle = 0;

  private final double k_armDeadZoneInDegrees = 2;
  private final double k_wristDeadZoneInDegrees = 2;

  // Set arm angle member variables
  private double m_armTargetAngleInDegrees = k_armStartingAngle;
  private double m_wristTargetAngleInDegrees = k_wristStartingAngle;

  // Arm PID
  private final double k_armP = 0;
  private final double k_armI = 0;
  private final double k_armD = 0;
  PIDController m_armPID = new PIDController(k_armP, k_armI, k_armD);
  private final double k_armF = 0;

  // Wrist PID
  private final double k_wristP = 0;
  private final double k_wristI = 0;
  private final double k_wristD = 0;
  PIDController m_wristPID = new PIDController(k_wristP, k_wristI, k_wristD);
  private final double k_wristF = 0;

  ReachSubsystem m_reachSubsystem;

  public ArmSubsystem(ReachSubsystem reachSubsystem) {
    m_reachSubsystem = reachSubsystem;

    // Reset motors
    m_arm.restoreFactoryDefaults();
    m_armFollower.restoreFactoryDefaults();
    m_armFollower.follow(m_arm);

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
    m_armEncoder.setPositionConversionFactor(1.0/k_armTicksToDegrees);
    m_wristEncoder.setPositionConversionFactor(1.0/k_wristTicksToDegrees);
  }

  public void moveToAngle(double armAngleInDegrees, double wristAngleInDegrees) {
    m_armTargetAngleInDegrees = armAngleInDegrees;
    m_wristTargetAngleInDegrees = wristAngleInDegrees;
  }

  public void setArmPower(double armPower) {
    m_arm.set(armPower);
  }

  public void setWristPower(double wristPower){
    m_wrist.set(wristPower);
  }

  public void setWristAngle(double degrees) {
    m_wristEncoder.setPosition(degrees);
  }

  public void setArmAngle(double degrees) {
    m_armEncoder.setPosition(degrees);
  }

  public void setArmBrake(boolean brake) {
    m_brake.set(brake);
  }

  // Run this in the beginning
  public void resetAngles() {
    setWristAngle(k_armStartingAngle);
    setArmAngle(k_wristStartingAngle);
  }

  public double getWristAngle() {
    return m_wristEncoder.getPosition();
  }

  public double getArmAngle() {
    return m_armEncoder.getPosition();
  }

  // Set arm angle to limit switch
  private void checkArmLimitSwitch() {
    if (m_armForwardLimit.isPressed()) {
      setArmAngle(k_armForwardLimitPos);
    } else if (m_armReverseLimit.isPressed()) {
      setArmAngle(k_armReverseLimitPos);
    }
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
    double length = m_reachSubsystem.getExtentInInches();
    return k_armF * Math.sin(Math.toRadians(m_armTargetAngleInDegrees) * length);
  }

  public double getWristFeedforward() {
    return k_wristF * Math.sin(Math.toRadians(m_wristTargetAngleInDegrees));
  }

  private void runPID() {
    if (isArmOnTarget()) {
      m_arm.set(0);
      setArmBrake(true);
    } else {
      setArmBrake(false);
      double armPower = m_armPID.calculate(getArmAngle(), m_armTargetAngleInDegrees);
      m_arm.set(armPower + getArmFeedforward());
    }
    if (isWristOnTarget()) {
      m_wrist.set(0);
    } else {
      double wristPower = m_wristPID.calculate(getWristAngle(), m_wristTargetAngleInDegrees);
      m_wrist.set(wristPower + getWristAngle());
    }
  }

  public boolean isArmOnTarget() {
    return Math.abs(getArmAngle() - m_armTargetAngleInDegrees) <= k_armDeadZoneInDegrees;
  }

  public boolean isWristOnTarget() {
    return Math.abs(getWristAngle() - m_wristTargetAngleInDegrees) <= k_wristDeadZoneInDegrees;
  }

  @Override
  public void periodic() {
    // Comment out later
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
    SmartDashboard.putNumber("Arm Angle", getArmAngle());
    SmartDashboard.putBoolean("Arm Forward Limit", m_armForwardLimit.isPressed());
    SmartDashboard.putBoolean("Arm Forward Limit", m_armForwardLimit.isPressed());
    SmartDashboard.putBoolean("Arm Reverse Limit", m_armReverseLimit.isPressed());
    SmartDashboard.putBoolean("Wrist Forward Limit", m_wristForwardLimit.isPressed());
    SmartDashboard.putBoolean("Wrist Reverse Limit", m_wristReverseLimit.isPressed());

    checkArmLimitSwitch();
    checkWristLimitSwitch();

    // runPID();
  
    // This method will be called once per scheduler run
  }
}
