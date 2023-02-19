// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// 11 ticks till end

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStationAuto extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  ArmSubsystem m_armSubsystem;
  private boolean m_tippedStation = false;
  private boolean m_balancedStation = false;
  /** Creates a new DriveStationAuto. */
  public DriveStationAuto(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.moveToAngle(-110);
    m_driveSubsystem.resetEncoders();
    m_driveSubsystem.setSpeedFPS(1, 1);
    // m_driveSubsystem.setSpeed(1.0/Constants.k_feetPerTick, 1.0/Constants.k_feetPerTick);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_driveSubsystem.getLeftPos()) > 7.5 || Math.abs(m_driveSubsystem.getRightPos()) > 7.5) {
      m_tippedStation = true;
      m_driveSubsystem.setSpeedFPS(-1, -1);
    }
    if (m_tippedStation && (m_driveSubsystem.getLeftPos() == 7 || m_driveSubsystem.getRightPos() == 7)) {
      m_driveSubsystem.setSpeedFPS(.5, .5);
      m_balancedStation = true;
    }
    if (m_balancedStation) {
      m_driveSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
    m_armSubsystem.moveToAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
