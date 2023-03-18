// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.ParadoxField;
import frc.robot.PositionTracker;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTargetCommand extends CommandBase {
  DriveSubsystem m_subsystem;
  PositionTracker m_tracker;
  // double[] m_target;
  private final double k_p = .005; // Unitless power per degree
  private final double k_minSpeed = .1; // Unitless power level in [-1,+1]
  double m_targetAngle; // angle in degrees
  double m_distance; // angle in degrees
  private final double k_deadZone = 3; // angle in degrees
  private boolean m_invalid = true; // indicates that we do not have a valid target
  private BooleanSupplier m_switchSides; // if true, indicates that we should reverse the front and back of the robot.

  /** Creates a new TurnToTargetCommand. */
  public TurnToTargetCommand(DriveSubsystem driveSubsystem, BooleanSupplier switchSides) {
    m_subsystem = driveSubsystem;
    m_switchSides = switchSides;
    // m_target = target;
    m_tracker = m_subsystem.getTracker();
    // m_targetAngle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_targetAngle = m_subsystem.findTargetAngleDegrees(); 
    Logger.log("TurnToTargetCommand", 1, String.format("initialize: angle = %f", m_targetAngle));
    m_invalid = false; // set in execute, then tested in isFinished
    // double beta = Math.atan2(m_tracker.getPose2d().getY() - m_target[1], m_tracker.getPose2d().getX() - m_target[0]);
    //double robotAngle = m_tracker.getPose2d().getRotation().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    OptionalDouble angle = m_subsystem.findTargetAngleDegrees(); 
    if (angle.isPresent()) { // We have a valid target
      Logger.log("TurnToTargetCommand", 1, "isPresent"); 
      m_targetAngle = angle.getAsDouble() + (m_switchSides.getAsBoolean() ? 0 : 180); 
      double currentRobotAngle = m_tracker.getPose2d().getRotation().getDegrees();
      m_distance =  currentRobotAngle - m_targetAngle;
      m_distance = ParadoxField.normalizeAngle(m_distance);
      double power = m_distance * k_p; // Unitless power [-1,+1]
      power = Math.abs(power) > k_minSpeed ? power : k_minSpeed * Math.signum(power);
      m_subsystem.setPower(power, -power);
    } else {
      Logger.log("TurnToTargetCommand", 1, "notPresent"); 
      m_invalid = true; // give up
    }
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
    // on target or no valid target
    return Math.abs(m_distance) < k_deadZone || m_invalid;
  }
}
