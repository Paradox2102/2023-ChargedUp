// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// Intake Subsystem 
public class IntakeSubsystem extends SubsystemBase {

  // Left and right motors for the Intake Subsystem 
  // private CANSparkMax m_leftIntakeMotor, m_rightIntakeMotor;
  // Limit switch for the Intake Subsystem 
  private DigitalInput m_intakeLimitSwitch;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  // m_leftMotor = new CANSparkMax();
  // m_rightMotor = ;
    m_intakeLimitSwitch = new DigitalInput(Constants.k_intakeLimitSwitch);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets power of left and right motors of Intake Subsystem
  public void setPower(double leftPower, double rightPower) {

  }
}
