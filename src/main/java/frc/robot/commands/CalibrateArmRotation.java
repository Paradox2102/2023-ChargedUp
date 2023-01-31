// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class CalibrateArmRotation extends CommandBase {
  private ArmSubsystem m_subsystem;
  private Encoder m_armEncoder;
  private PIDController m_pid;
  private static final double k_p = 0;
  private static final double k_i = 0;
  private static final double k_d = 0;

  /** Creates a new CalibrateArmRotation. */
  public CalibrateArmRotation(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    //m_armEncoder = m_subsystem.getEncoder();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pid = new PIDController(k_p, k_i, k_d);
    //m_subsystem.setPower(0.5, 0.5);
  }

  public Encoder getEncoder() {
    return m_armEncoder;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Position", m_armEncoder.getDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
