// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */

  DriveSubsystem m_subsystem;
  boolean m_isFinished = false;

  public AutoBalanceCommand(DriveSubsystem driveSubsystem) {
    m_subsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The code in execute() leaves the speed unchanged if ½°<|pitch|⩽2¾°.  What happens if we're in this range when the command starts?  -Gavin
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentRobotPitch = m_subsystem.getPitch();

    // If we've reached the middle and |pitch| <= ½°, stop.
    // For simplicity, could move this test to isFinished().  -Gavin
    if (Math.abs(currentRobotPitch) <= .5) {
      m_subsystem.setSpeedFPS(0, 0);
      m_isFinished = true;
    }
    // If pitch < 2¾°, go forwards at 1FPS
    else if (currentRobotPitch < -2.75) { 
      m_subsystem.setSpeedFPS(1, 1);
    } 
    // If pitch > 2¾°, go backwards at 1FPS
    else if (currentRobotPitch > 2.75) {
      m_subsystem.setSpeedFPS(-1, -1);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Should call stop() instead for clarity. -Gavin
    m_subsystem.setSpeedFPS(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
