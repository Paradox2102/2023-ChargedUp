// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.pathfinder.Pathfinder;
import frc.pathfinder.Pathfinder.Path;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.subsystems.DriveSubsystem;

public class CreatePathCommand extends CommandBase {
  private DriveSubsystem m_driveSubsystem; 
  private boolean m_setPosition; 
  private boolean m_reversed; 
  private String m_name; 
  private PurePursuitData m_data; 
  private Path m_path; 
  private static final int k_nPoints = 1000; 
  private static final double k_dt =0.02; 
  private static final double k_wheelbase = 1.8125; 

  /** Creates a new CreatePathCommand. */
  private void init(DriveSubsystem driveSubsystem, Waypoint[] waypoints, boolean setPosition, boolean reversed, String name, PurePursuitData data, double lookAheadTime){
    Logger.log(name, 3, "CreatPathCommand");
    m_driveSubsystem = driveSubsystem; 
    m_setPosition = setPosition; 
    m_reversed = reversed; 
    m_name = name; 
    m_data = data;

    m_path = Pathfinder.computePath(waypoints, k_nPoints, k_dt, data.k_maxSpeed, data.k_maxAccel, data.k_maxDecl, data.k_maxJerk, k_wheelbase);
    m_path.setLookAheadTime(lookAheadTime);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  public CreatePathCommand(DriveSubsystem driveSubsystem, Waypoint[] waypoints, boolean setPosition, boolean reversed, String name, PurePursuitData data, double lookAheadTime) {
    init(driveSubsystem, waypoints, setPosition, reversed, name, data, lookAheadTime);
  }

  public CreatePathCommand(DriveSubsystem driveSubsystem, Waypoint[] waypoints, boolean setPosition, boolean reversed, String name, PurePursuitData data) {
    init(driveSubsystem, waypoints, setPosition, reversed, name, data, 1);
  }

  public CreatePathCommand(DriveSubsystem driveSubsystem, Waypoint[] waypoints, boolean setPosition, boolean reversed, String name){
    init(driveSubsystem, waypoints, setPosition, reversed, name, new PurePursuitData(), 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log(m_name, 2, "initialize"); 
    m_driveSubsystem.startPath(m_path, m_reversed, m_setPosition, null);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log(m_name, 2, "end"); 
    m_driveSubsystem.endPath();
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveSubsystem.isPathFinished();
  }
}