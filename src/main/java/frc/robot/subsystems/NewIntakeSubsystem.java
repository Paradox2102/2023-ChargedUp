// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class NewIntakeSubsystem extends IntakeSubsystem {
  TalonFX m_intakeMotor = new TalonFX(Constants.k_newIntakeMotor);
  // The following TalonSRX in ONLY used to read the abs encoder for the arm
  // Ideally, this should be declared in the ArmSubsyste, but for compatibility
  // reasons between the practice and competition robots, it is here.
  TalonSRX m_armEncoder = new TalonSRX(Constants.k_armEncoder);

  private Timer m_stallTimer = new Timer();
  private double m_power = 0;


  /** Creates a new NewIntakeSubsystem. */
  public NewIntakeSubsystem() {
    m_armEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    m_intakeMotor.setInverted(true);
    m_stallTimer.reset();
    m_stallTimer.start();
  }

  @Override
  public void setPower(double power) {
    m_intakeMotor.set(ControlMode.PercentOutput, power);
    m_power = power;
    // SmartDashboard.putNumber("Intake Power", power);
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
    m_power = power;
    m_intakeMotor.set(ControlMode.PercentOutput, power);
  }
  
  public void setClawPower(double clawPower)
  {
    // This is a no-op for the new intake
  }

  private boolean isIntakeStalled() {
    if (Math.abs(m_intakeMotor.getSelectedSensorVelocity()) < Constants.k_intakeStallSpeed && m_power > 0) {
      if (m_stallTimer.get() > .1) {
        return true;
      } else {
        return false;
      }
    } else {
      m_stallTimer.reset();
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPower(m_power);
    if (isIntakeStalled()) {
      m_power = Constants.k_intakeMinPower;
      setPower(Constants.k_intakeMinPower);
    }
    SmartDashboard.putNumber("Intake Speed", m_intakeMotor.getSelectedSensorVelocity());
  }
}
