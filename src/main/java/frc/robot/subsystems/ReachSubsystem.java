// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReachSubsystem extends SubsystemBase {
  TalonFX m_reachMotor = new TalonFX(Constants.k_reachMotor);


  /** Creates a new ReachSubsystem. */
  public ReachSubsystem() {
    m_reachMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.k_bottomSensor);
    m_reachMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.k_topSensor);

  }

  //move pls

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
