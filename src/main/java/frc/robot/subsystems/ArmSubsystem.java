// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  // Arm Motors
  CANSparkMax m_armMotor = new CANSparkMax(Constants.k_armMotor, MotorType.kBrushless);
  CANSparkMax m_armFollower = new CANSparkMax(Constants.k_armFollower, MotorType.kBrushless);

  // Wrist Motors
  CANSparkMax m_wrist = new CANSparkMax(Constants.k_wristMotor, MotorType.kBrushless);

  // Combine motors
  MotorControllerGroup m_arm = new MotorControllerGroup(m_armMotor, m_armFollower);

  // Encoders
  RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
  RelativeEncoder m_wristEncoder = m_wrist.getEncoder();

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
  Solenoid m_rightBrake = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.k_rightArmBrake);
  Solenoid m_leftBrake = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.k_leftArmBrake);

  public ArmSubsystem() {

    // Reset motors
    m_armMotor.restoreFactoryDefaults();
    m_armFollower.restoreFactoryDefaults();

    m_armMotor.setInverted(false);
    m_armFollower.setInverted(false);

    // Set arm limit switches
    m_armForwardLimit = m_armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_armReverseLimit = m_armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_armForwardLimit.enableLimitSwitch(true);
    m_armReverseLimit.enableLimitSwitch(true);

    // Set wrist limit switches
    m_wristForwardLimit = m_wrist.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_wristReverseLimit = m_wrist.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_wristForwardLimit.enableLimitSwitch(true);
    m_wristReverseLimit.enableLimitSwitch(true);
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

  // Run this in the beginning
  public void resetAngles() {
    setWristAngle(0);
    setArmAngle(0);
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
  
    // This method will be called once per scheduler run
  }
}
