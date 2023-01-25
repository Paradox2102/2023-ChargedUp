// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReachSubsystem extends SubsystemBase {
  TalonFX m_reachMotor = new TalonFX(Constants.k_reachMotor);
  private static final double m_ticsPerInch = 1000 / 6.0;

  /** Creates a new ReachSubsystem. */
  public ReachSubsystem() {
    m_reachMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        Constants.k_canTimeOut);
    m_reachMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        Constants.k_canTimeOut);
  }

  public double getExtentInInches() {
    double rawPosition = m_reachMotor.getSelectedSensorPosition();
    return rawPosition / m_ticsPerInch;
  }

  public void setPower(double power) {
    m_reachMotor.set(ControlMode.PercentOutput, power);
  }

  public void resetEncoder(double value) {
    m_reachMotor.setSelectedSensorPosition(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
