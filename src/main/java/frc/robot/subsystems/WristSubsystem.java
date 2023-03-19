// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  CANSparkMax m_wristMotor = new CANSparkMax(Constants.k_wristMotor, MotorType.kBrushless);
  RelativeEncoder m_encoder = m_wristMotor.getEncoder(); 
  double m_position = 0; 
  boolean m_enabled = false; 
  static final double k_p = 2.0/60; 
  static final double k_minPower = 0.1; 
  static final double k_deadzone = 1; 
  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    m_wristMotor.restoreFactoryDefaults();
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_encoder.setPosition(0);
  }

  public void setPower(double power) {
    m_enabled = false; 
    m_wristMotor.set(power);
  }

  public void setPosition(double position) {
    m_enabled = true;
    m_position = position; 
  }

  @Override
  public void periodic() {
    if (m_enabled) {
      double error = m_encoder.getPosition() - m_position;
      double power = -k_p * error; 
      if (Math.abs(error) < k_deadzone) {
        power = 0; 
      } else if (Math.abs(power) < k_minPower) {
        power = k_minPower * Math.signum(power); 
      }
      m_wristMotor.set(power); 
      SmartDashboard.putNumber("Wrist Power", power); 
    }
    SmartDashboard.putNumber("Wrist Position", m_encoder.getPosition());
    // This method will be called once per scheduler run
  }
}
