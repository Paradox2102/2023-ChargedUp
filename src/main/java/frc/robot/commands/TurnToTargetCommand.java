// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.ParadoxField;
import frc.robot.PositionTracker;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTargetCommand extends CommandBase {
  DriveSubsystem m_subsystem;
  PositionTracker m_tracker;
  double[] m_target;
  private final double k_p = .005;
  private final double k_minSpeed = .1;
  double m_targetAngle;
  double m_distance;
  private final double k_deadZone = 3;
  /** Creates a new TurnToTargetCommand. */
  public TurnToTargetCommand(DriveSubsystem driveSubsystem, double targetAngle, double[] target) {
    m_subsystem = driveSubsystem;
    m_target = target;
    m_tracker = m_subsystem.getTracker();
    m_targetAngle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TurnToTargetCommand", 1, "initialize");
    // double beta = Math.atan2(m_tracker.getPose2d().getY() - m_target[1], m_tracker.getPose2d().getX() - m_target[0]);
    //double robotAngle = m_tracker.getPose2d().getRotation().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentRobotAngle = m_tracker.getPose2d().getRotation().getDegrees();
    m_distance =  currentRobotAngle - m_targetAngle;
    m_distance = ParadoxField.normalizeAngle(m_distance);
    double power = m_distance * k_p;
    power = Math.abs(power) > k_minSpeed ? power : k_minSpeed * Math.signum(power);
    m_subsystem.setPower(power, -power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TurnToTargetCommand", 1, "end");
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_distance) < k_deadZone;
  }
}
