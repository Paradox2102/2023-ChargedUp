// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  IntakeSubsystem m_subsystem;
  double m_power;
  boolean m_isAuto;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, double power, boolean isAuto) {
    m_subsystem = intakeSubsystem;
    m_power = power;
    m_isAuto = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("IntakeCommand", 1, "initialize");
    if (!m_isAuto) {
      m_subsystem.setPower(m_power);
    } else {
      m_subsystem.setPowerAutoPeriod(m_power);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("IntakeCommand", 1, "end");
    if (!m_isAuto) {
      m_subsystem.setPower(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isAuto;
  }
}
