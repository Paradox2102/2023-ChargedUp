// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// Intake Subsystem 
public class IntakeSubsystem extends SubsystemBase {

  // Left and right motors for the IntakeSubsystem 
  private TalonSRX m_leftIntakeMotor, m_rightIntakeMotor;
  // Limit switch for the IntakeSubsystem 
  private DigitalInput m_intakeLimitSwitch;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Declare instance variables
    m_leftIntakeMotor = new TalonSRX(Constants.k_leftIntakeMotor);
    m_rightIntakeMotor = new TalonSRX(Constants.k_rightIntakeMotor);
    m_intakeLimitSwitch = new DigitalInput(Constants.k_intakeLimitSwitch);
    // Set proper inversions
    m_leftIntakeMotor.setInverted(false);
    m_rightIntakeMotor.setInverted(false);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets power of left and right motors of IntakeSubsystem
  public void setPower(double leftPower, double rightPower) {
    // Limit switch is on, set speed to 0 
    if(m_intakeLimitSwitch.get())
    {
      m_leftIntakeMotor.set(ControlMode.PercentOutput, 0);
      m_rightIntakeMotor.set(ControlMode.PercentOutput, 0);
    }
    // Limit swtich is off, set speed to given speeds 
    else
    {
      m_leftIntakeMotor.set(ControlMode.PercentOutput, leftPower);
      m_rightIntakeMotor.set(ControlMode.PercentOutput, rightPower);
    }

  }
}
