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
  DigitalInput m_topSwitch = new DigitalInput(Constants.k_topSwitch);
  DigitalInput m_bottomSwitch = new DigitalInput(Constants.k_bottomSwitch);

  private double m_armZero; 

  private double m_power;

  private final double k_deadZone = 1;
  private static final double k_p = 0.1;

  private boolean m_runP;

  private double m_extentInInches = 0;


  /** Creates a new ReachSubsystem. */
  public ReachSubsystem() {
    m_reachMotor.setInverted(true);
    m_armZero = m_reachMotor.getSelectedSensorPosition();
    // Set limit switches
    // There's no point in using a timeout unless you're going to check the return value. -Gavin
    m_reachMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        Constants.k_canTimeOut);
    m_reachMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        Constants.k_canTimeOut);
    setBrakeMode(true);
  }

  public double getExtentInInches() {
    double rawPosition = m_reachMotor.getSelectedSensorPosition() - m_armZero;
    return rawPosition / Constants.k_reachTicksPerInch;
  }

  public void setBrakeMode(boolean brake){
    NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
    m_reachMotor.setNeutralMode(mode);
  }

  public void setPower(double power) {
    m_power = power;
  }
  

  public void resetEncoder(double value) {
    m_reachMotor.setSelectedSensorPosition(value);
  }

  public void runP(double distance) {
    if (Math.abs(distance) < k_deadZone) {
      setPower(0);
    } else {
      double power = distance * k_p;
      if (Math.abs(power) < 0.2)
      {
        power = 0.2 * Math.signum(power);
      }
      setPower(power);
    }
  }

  public void isRunP(boolean runP) {
    m_runP = runP;
  }

  public void setExtentInInches(double extent) {
    m_extentInInches = extent;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Top Switch", m_topSwitch.get());
    // SmartDashboard.putBoolean("Bottom Switch", m_bottomSwitch.get());
    // SmartDashboard.putNumber("Reach Position", m_reachMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Extent Inches", getExtentInInches());
    // SmartDashboard.putBoolean("Top Switch", m_topSwitch.get());
    // SmartDashboard.putBoolean("Bottom Switch", m_bottomSwitch.get());

    if (!m_bottomSwitch.get()) {
      m_armZero = m_reachMotor.getSelectedSensorPosition(); 
    }

    double currentPosition = getExtentInInches();
    double distance = m_extentInInches - currentPosition;

    if (m_runP) {
      runP(distance);
    }

    // run reach motor -----
    double power = m_power;
    if (power > 0) {
      if (!m_topSwitch.get()) {
        power = 0;
      }
    } else if (power < 0) {
      if (!m_bottomSwitch.get()) {
        power = 0;
      }
    }
    m_reachMotor.set(ControlMode.PercentOutput, power);
  }
  // ------------------------
}
