// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Place cube on low, engage charge station

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReachSubsystem;

public class Auto_4MS extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  ArmSubsystem m_armSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ReachSubsystem m_reachSubsystem;
  private boolean m_tippedStation = false;
  private boolean m_start = true;
  private double m_previousRobotPitch = 0;
  private double m_currentRobotPitch = 0;
  private Timer m_timer = new Timer();
  /** Creates a new DriveStationAuto. */
  public Auto_4MS(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ReachSubsystem reachSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_armSubsystem = armSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_reachSubsystem = reachSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    addRequirements(m_armSubsystem);
    addRequirements(m_intakeSubsystem);
    addRequirements(m_reachSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("Auto_4LS", 1, "initialize");
    m_intakeSubsystem.setClaw(IntakeSubsystem.ClawPosition.CONE);
    m_armSubsystem.moveToAngle(Constants.k_midConeNodeAngle);
    m_reachSubsystem.setExtentInInches(Constants.k_midConeNodeExtent);
    m_reachSubsystem.isRunP(true);
    m_driveSubsystem.resetEncoders();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentRobotPitch = m_driveSubsystem.getPitch();
    if (m_start && m_timer.get() > 1 && m_timer.get() < 2) {
      m_intakeSubsystem.setClaw(IntakeSubsystem.ClawPosition.CUBE);
    }
    if (m_start && m_timer.get() > 2) {
      m_driveSubsystem.setSpeedFPS(3, 3);
      m_start = false;
    }
    if (Math.abs(m_armSubsystem.getArmAngleDegrees()) <= 5) {
      m_armSubsystem.moveToAngle(-100);
    }
    if (m_driveSubsystem.getLeftPos() >= 7.25 || m_driveSubsystem.getRightPos() >= 7.25) { // 7.5
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