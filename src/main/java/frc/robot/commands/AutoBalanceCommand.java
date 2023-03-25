// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */

  DriveSubsystem m_subsystem;
  boolean m_isFinished = false;
  double m_previousRobotPitch = 0;
  double m_currentSpeed = 0;
  boolean m_balanced = false;

  public AutoBalanceCommand(DriveSubsystem driveSubsystem) {
    m_subsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("AutoBalanceCommand", 1, "initialize");
    m_previousRobotPitch = 0;
    m_currentSpeed = 0;
    m_balanced = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentRobotPitch = m_subsystem.getPitch();

    // If we've reached the middle and |pitch| <= ½°, stop.
    // For simplicity, could move this test to isFinished().  -Gavin
    // If pitch < 2¾°, go forwards at 1FPS
    if (currentRobotPitch < -10 && !m_balanced) { 
      m_subsystem.setSpeedFPS(0.75, 0.75);
      m_currentSpeed = .75;
    } 
    // If pitch > 2¾°, go backwards at 1FPS
    else if (currentRobotPitch > 10 && !m_balanced) {
      m_subsystem.setSpeedFPS(-0.75, -0.75);
      m_currentSpeed = -.75;
    } 
    else if (Math.abs(currentRobotPitch) < Math.abs(m_previousRobotPitch) && !m_balanced){
      double speed = Math.signum(m_currentSpeed) * -2;
      // m_subsystem.setSpeedFPS(speed , speed);
      m_balanced = true;
      // m_isFinished = true;
    } else if (Math.abs(currentRobotPitch) < 3) {
      m_subsystem.setSpeed(0, 0);
    }
    m_previousRobotPitch = currentRobotPitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Should call stop() instead for clarity. -Gavin
    Logger.log("AutoBalanceCommand", 1, "end");
    m_subsystem.setSpeedFPS(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
