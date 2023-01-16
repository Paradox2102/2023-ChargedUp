// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.pathfinder.Pathfinder;
import frc.pathfinder.Pathfinder.Path;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.subsystems.DriveSubsystem;

public class PathFollowingCommand extends CommandBase {
  DriveSubsystem m_subsystem;
  DoubleSupplier m_speed;
  Path m_path;

  private static final int k_nPoints = 1000;
  private static final double k_dt = 0.020000;
  private static final double k_maxSpeed = 4.000000;
  private static final double k_maxAccel = 11.000000;
  private static final double k_maxDecl = 11.000000;
  private static final double k_maxJerk = 100.000000;
  private static final double k_wheelbase = 1.812500;
  /*
  0, 10, 0
  3.3, 10, 0
  */
  final static Waypoint[] waypoints = { 
      new Waypoint(0, 0, Math.toRadians(90)), 
      new Waypoint(10, 10, Math.toRadians(90)),
      new Waypoint(0, 20, Math.toRadians(90)) };

      
  /** Creates a new PathFollowingCommand. */
  public PathFollowingCommand(DriveSubsystem driveSubsystem, DoubleSupplier speed) {
    m_subsystem = driveSubsystem;
    m_speed = speed;

    m_path = Pathfinder.computePath(waypoints, k_nPoints, k_dt, k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk, k_wheelbase);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("PathFollowingCommand", 1, "initialize");
    m_subsystem.startPath(m_path, false, true, m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("PathFollowingCommand", 1, "execute");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("PathFollowingCommand", 2, "end"); 
    m_subsystem.endPath();
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isPathFinished() && (m_speed == null || Math.abs(m_speed.getAsDouble()) < .1);
  }
}
