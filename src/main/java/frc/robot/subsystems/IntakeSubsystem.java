// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// Intake Subsystem 
public class IntakeSubsystem extends SubsystemBase {

  // Left and right TalonSRX motors for the IntakeSubsystem 
  private TalonSRX m_leftIntakeMotor, m_rightIntakeMotor;
  private Solenoid m_brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.k_claw);
  private PowerDistribution m_powerDistribution = new PowerDistribution();
  // Digital Input Limit switch for the IntakeSubsystem 
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Initialize motors 
    m_leftIntakeMotor = new TalonSRX(Constants.k_leftIntakeMotor);
    m_rightIntakeMotor = new TalonSRX(Constants.k_rightIntakeMotor);
    m_leftIntakeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); 
    // Set proper inversions
    m_leftIntakeMotor.setInverted(false);
    m_rightIntakeMotor.setInverted(true);
    m_leftIntakeMotor.setNeutralMode(NeutralMode.Brake);
    m_rightIntakeMotor.setNeutralMode(NeutralMode.Brake);
    // m_leftIntakeMotor.configContinuousCurrentLimit(1);
    // m_rightIntakeMotor.configContinuousCurrentLimit(1);
    // m_leftIntakeMotor.enableCurrentLimit(true);
    // m_rightIntakeMotor.enableCurrentLimit(true);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Intake Current", m_powerDistribution.getCurrent(Constants.k_leftIntakeMotor));
    SmartDashboard.putNumber("Right Intake Current", m_powerDistribution.getCurrent(Constants.k_rightIntakeMotor));
  }

  // Sets power of left and right motors of IntakeSubsystem
  public void setPower(double leftPower, double rightPower) {
    m_leftIntakeMotor.set(ControlMode.PercentOutput, leftPower);
    m_rightIntakeMotor.set(ControlMode.PercentOutput, rightPower);
  }

  public void setClaw(boolean closed) {
    m_brake.set(closed);
  }

  public double getMagEncoderPosition() {
    return m_leftIntakeMotor.getSelectedSensorPosition(); 
  }

}
