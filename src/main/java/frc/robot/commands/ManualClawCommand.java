// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualClawCommand extends CommandBase {
  IntakeSubsystem m_subsystem;
  /** Creates a new ManualClawCommand. */
  public ManualClawCommand(IntakeSubsystem intakeSubsystem) {
    m_subsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ManualClawCommand", 1, "initialize");
    m_subsystem.setClaw(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ManualClawCommand", 1, "end");
    m_subsystem.setClaw(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
