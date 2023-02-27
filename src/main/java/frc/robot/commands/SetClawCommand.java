// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ClawPosition;

public class SetClawCommand extends CommandBase {
  IntakeSubsystem m_subsystem;
  ClawPosition m_pos;
  /** Creates a new SetClawCommand. */
  public SetClawCommand(IntakeSubsystem intakeSubsystem, ClawPosition pos) {
    m_subsystem = intakeSubsystem;
    m_pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setClaw(m_pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_subsystem.setClaw(IntakeSubsystem.ClawPosition.OPEN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
