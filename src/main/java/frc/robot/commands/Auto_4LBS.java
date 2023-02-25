// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Place cube low, pick up cube, engage charge station

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Auto_4LBS extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  ArmSubsystem m_armSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  // Checks which phase of the auto the robot is in
  private boolean m_tippedStation = false;
  private boolean m_crossedStation = false;
  private boolean m_grabbingGamePiece = false;
  private boolean m_startEngage = false;

  private double m_previousRobotRoll = 0;
  private double m_currentRobotRoll = 0;
  private double m_startingRobotPos = 0; // robot's left encoder pos
  private double m_currentRobotPos = 0;  // robot's left encoder pos

  /** Creates a new Auto_4LBS. */
  public Auto_4LBS(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    addRequirements(m_intakeSubsystem);
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("Auto_4LBS", 1, "initialize");
    m_armSubsystem.moveToAngle(-95);
    m_intakeSubsystem.setClaw(IntakeSubsystem.ClawPosition.CONE);
    m_driveSubsystem.resetEncoders();
    m_driveSubsystem.setSpeedFPS(5, 5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentRobotRoll = m_driveSubsystem.getPitch();
    m_currentRobotPos = m_driveSubsystem.getLeftPos();
    // Grab cube
    // Crossed charge station?
    if (m_currentRobotRoll >= 5) {
      m_crossedStation = true;
    }
    // Go to game piece (cube) if crossed charge station
    if (m_crossedStation && Math.abs(m_currentRobotRoll) <= 2) {
      m_driveSubsystem.setSpeedFPS(4, 4);
      m_armSubsystem.moveToAngle(115);
      m_intakeSubsystem.setClaw(IntakeSubsystem.ClawPosition.OPEN);
      m_intakeSubsystem.setPower(-.25);
      m_startingRobotPos = m_currentRobotPos;
      m_grabbingGamePiece = true;
      m_crossedStation = false;
    }
    // Is it at the game piece? If yes, go dock
    if (Math.abs(m_currentRobotPos) - Math.abs(m_startingRobotPos) >= 3.5 && m_grabbingGamePiece) {
      m_startingRobotPos = m_currentRobotPos;
      m_driveSubsystem.setSpeedFPS(-3, -3);
      m_armSubsystem.moveToAngle(110);
      m_startEngage = true;
      m_grabbingGamePiece = false;
    }
    // Engage charge station
    if (Math.abs(Math.abs(m_currentRobotPos) - Math.abs(m_startingRobotPos)) >= 9.5 && m_startEngage) {
      m_tippedStation = true;
      m_driveSubsystem.setSpeedFPS(-.5, -.5);
    }
    if (m_tippedStation && m_currentRobotRoll < m_previousRobotRoll && m_startEngage) {
      m_driveSubsystem.setSpeedFPS(-1, -1);

    }
    if (Math.abs(Math.abs(m_previousRobotRoll) - Math.abs(m_currentRobotRoll)) <= .5 && m_tippedStation && m_startEngage) {
      m_driveSubsystem.stop();
    }
    m_previousRobotRoll = m_currentRobotRoll;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("Auto_4LBS", 1, "end");
    m_intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
