// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PositionTracker;
import frc.robot.subsystems.DriveSubsystem;

public class DeleteMeCommand extends CommandBase {
  PositionTracker m_tracker;
  /** Creates a new DeleteMeCommand. */
  public DeleteMeCommand(DriveSubsystem driveSubsystem) {
    m_tracker = driveSubsystem.getTracker();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // findTargetAngle(23.0/12, -9.0/12);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
