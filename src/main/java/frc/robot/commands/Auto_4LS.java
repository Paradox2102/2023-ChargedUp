// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Place cube on low, engage charge station

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Auto_4LS extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  ArmSubsystem m_armSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  private boolean m_tippedStation = false;
  private double m_previousRobotPitch = 0;
  private double m_currentRobotPitch = 0;
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
    m_armSubsystem.moveToAngle(-100);
    m_intakeSubsystem.setClaw(IntakeSubsystem.ClawPosition.CONE);
    m_driveSubsystem.resetEncoders();
    m_driveSubsystem.setSpeedFPS(3, 3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_currentRobotPitch = m_driveSubsystem.getPitch();
    if (m_driveSubsystem.getLeftPos() >= 7.5 || m_driveSubsystem.getRightPos() >= 7.5) {
      m_tippedStation = true;
      m_driveSubsystem.setSpeedFPS(.5, .5);
    }
    if (m_tippedStation && m_currentRobotPitch < m_previousRobotPitch) {
      m_driveSubsystem.setSpeedFPS(1, 1);

    }
    if (Math.abs(m_previousRobotPitch - m_currentRobotPitch) <= .5 && m_tippedStation) {
      m_driveSubsystem.stop();
    }
    m_previousRobotPitch = m_currentRobotPitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("Auto_4LS", 1, "end");
    m_driveSubsystem.stop();
    m_intakeSubsystem.setClaw(IntakeSubsystem.ClawPosition.OPEN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
