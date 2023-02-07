// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReachSubsystem extends SubsystemBase {
  TalonFX m_reachMotor = new TalonFX(Constants.k_reachMotor);
  private static final double k_ticsPerInch = 1000 / 6.0;
  DigitalInput m_topSwitch = new DigitalInput(Constants.k_topSwitch);
  DigitalInput m_bottomSwitch = new DigitalInput(Constants.k_bottomSwitch);


  /** Creates a new ReachSubsystem. */
  public ReachSubsystem() {
    // Set limit switches
    // m_reachMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
    //     Constants.k_canTimeOut);
    // m_reachMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
    //     Constants.k_canTimeOut);
    setBrakeMode(true);
  }

  public double getExtentInInches() {
    double rawPosition = m_reachMotor.getSelectedSensorPosition();
    return rawPosition / k_ticsPerInch;
  }

  public void setBrakeMode(boolean brake){
    NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
    m_reachMotor.setNeutralMode(mode);
  }

  public void setPower(double power) {
    if ((!m_topSwitch.get() && power < 0) || (!m_bottomSwitch.get() && power > 0)) {
      m_reachMotor.set(ControlMode.PercentOutput, 0);
    } else {
      m_reachMotor.set(ControlMode.PercentOutput, power);
    }
  }
  

  public void resetEncoder(double value) {
    m_reachMotor.setSelectedSensorPosition(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Top Switch", m_topSwitch.get());
    SmartDashboard.putBoolean("Bottom Switch", m_bottomSwitch.get());
  }
}
