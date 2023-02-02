// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private CANSparkMax m_armMotor = new CANSparkMax(Constants.k_armMotor, MotorType.kBrushless);
  private CANSparkMax m_armFollower = new CANSparkMax(Constants.k_armFollower, MotorType.kBrushless);

  // Combine motors
  private MotorControllerGroup m_arm = new MotorControllerGroup(m_armMotor, m_armFollower);

  private RelativeEncoder m_armEncoder = m_armMotor.getEncoder();

  private SparkMaxLimitSwitch m_forwardLimit;
  private SparkMaxLimitSwitch m_reverseLimit;

  public ArmSubsystem() {

    // Reset motors
    m_armMotor.restoreFactoryDefaults();
    m_armFollower.restoreFactoryDefaults();

    m_armMotor.setInverted(false);
    m_armFollower.setInverted(false);

    // Set limit switches
    m_forwardLimit = m_armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_reverseLimit = m_armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }
  
  public void setPower(double armPower) {
    m_arm.set(armPower);
  }

  @Override
  public void periodic() {
  
    // This method will be called once per scheduler run
  }
}
