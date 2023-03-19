// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class NewIntakeSubsystem extends IntakeSubsystem {
  TalonFX m_intakeMotor = new TalonFX(Constants.k_newIntakeMotor);
  // The following TalonSRX in ONLY used to read the abs encoder for the arm
  // Ideally, this should be declared in the ArmSubsyste, but for compatibility
  // reasons between the practice and competition robots, it is here.
  TalonSRX m_armEncoder = new TalonSRX(Constants.k_armEncoder);


  /** Creates a new NewIntakeSubsystem. */
  public NewIntakeSubsystem() {
    m_armEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }

  @Override
  public void setPower(double power) {
    m_intakeMotor.set(ControlMode.PercentOutput, power);
  }

  public void setClaw(ClawPosition position) {
    // This function is a no-op form the new intake
  }

  public double getMagEncoderPosition()
  {
    return m_armEncoder.getSelectedSensorPosition(); 
  }

  public double getMagEncoderVelocity()
  {
    return m_armEncoder.getSelectedSensorVelocity(); 
  }

  public void stop()
  {
    setPower(0);
  }

  public void setPowerAutoPeriod(double power)
  {
    setPower(power);
  }
  
  public void setClawPower(double clawPower)
  {
    // This is a no-op for the new intake
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
