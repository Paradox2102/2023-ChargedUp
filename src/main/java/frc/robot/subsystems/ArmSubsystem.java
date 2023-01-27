// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  CANSparkMax m_armMotor = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax m_armFollower = new CANSparkMax(0, MotorType.kBrushless);

  MotorControllerGroup m_arm = new MotorControllerGroup(m_armMotor, m_armFollower);

  RelativeEncoder m_armEncoder = m_armMotor.getEncoder();

  public ArmSubsystem() {}
  
  public void setPower(double armPower) {
    m_armMotor.set(armPower);
  }

  @Override
  public void periodic() {
  
    // This method will be called once per scheduler run
  }
}
