// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Place cube on low, engage charge station

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Auto_4LS extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  ArmSubsystem m_armSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  // m_tippedStation becomes true once the robot has gotten near the middle of the charge station.
  // This is important to know because we don't want to respond to the changes in pitches that will
  // happen when we start to climb up on the charge station and it initially tips. 
  private boolean m_tippedStation = false;
  // m_start is true for a short initial period to allow us to deliver a game piece before we start moving.
  private boolean m_start = true;
  private double m_previousRobotPitch = 0; 
  private double m_currentRobotPitch = 0;
  private Timer m_timer = new Timer();
  /** Creates a new DriveStationAuto. */
  public Auto_4LS(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_armSubsystem = armSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    addRequirements(m_armSubsystem);
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("Auto_4LS", 1, "initialize");
    // Move the arm down to knock off gamepiece
    m_armSubsystem.moveToAngle(-80);
    m_driveSubsystem.resetEncoders();
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // PROBLEM: Ideally, we should only set the speed once per call to execute.  This code potentially calls it up to five times.  This could lead to jittering.  -Gavin

    m_currentRobotPitch = m_driveSubsystem.getPitch();
    // Wait a second for the game piece to fall off and then start moving at 3FPS
    if (m_start && m_timer.get() > 1) {
      m_driveSubsystem.setSpeedFPS(3, 3);
      m_start = false;
    }
    // If |pitch| <= 10° and we have't reached the middle, slow down to 2FPS
    if (Math.abs(m_driveSubsystem.getPitch()) >= 10 && m_tippedStation == false) {
      m_driveSubsystem.setSpeedFPS(2, 2);
    }
    // I figured out at the speed the robot runs at consistently has it go to the same spot even if the wheels slip
    // If we've gone 7.5 ft on either side, then we've reached the middle.  Stop.
    if (m_driveSubsystem.getLeftPos() >= 7.5 || m_driveSubsystem.getRightPos() >= 7.5) {
      m_tippedStation = true;
      m_driveSubsystem.setSpeedFPS(0, 0); // .5, .5
    }
    // If we've reached the middle and |pitch| <= ½°, stop.
    if (Math.abs(m_currentRobotPitch) <= .5 && m_tippedStation) {
      m_driveSubsystem.setSpeedFPS(0, 0);
    }
    // PROBLEM: This test will never be true because of the call to abs(). -Gavin
    else if (m_tippedStation && Math.abs(m_currentRobotPitch) < -2.75) { 
      m_driveSubsystem.setSpeedFPS(1, 1);
    } 
    // If we've reached the middle and |pitch| > 2¾°, go backwards at 1FPS
    // PROBLEM: It's strange that you want to call abs() here. -Gavin
    else if (m_tippedStation && Math.abs(m_currentRobotPitch) > 2.75) {
      m_driveSubsystem.setSpeedFPS(-1, -1);
    }
    // If we've gone more than 9 ft (on either side), stop.  Runaway robot!
    if (m_driveSubsystem.getLeftPos() >= 9 || m_driveSubsystem.getRightPos() >= 9) {
      m_driveSubsystem.setSpeedFPS(0, 0);
    }
    m_previousRobotPitch = m_currentRobotPitch; // not used?
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("Auto_4LS", 1, "end");
    m_driveSubsystem.setSpeedFPS(0, 0); // Should call stop() -Gavin
    m_intakeSubsystem.setClaw(IntakeSubsystem.ClawPosition.OPEN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
