// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  DriveSubsystem m_subsystem;
  DoubleSupplier m_getX;
  DoubleSupplier m_getY;
  BooleanSupplier m_goingForward;
  SlewRateLimiter m_filter = new SlewRateLimiter(1.0 / Constants.k_rampTimeSeconds);

  /** Creates a new ArcadeDriveCommand. */
  public ArcadeDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier getX, DoubleSupplier getY, BooleanSupplier goingForward) {
    m_subsystem = driveSubsystem;
    m_getX = getX;
    m_getY = getY;
    m_goingForward = goingForward;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize Arcade Drive");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = m_getX.getAsDouble();
    double drive = -m_getY.getAsDouble();
    boolean goingForward = m_goingForward.getAsBoolean();

    turn = turn * turn * turn / 5;
    if (!goingForward) {
      drive = -drive;
    }
    drive = drive * drive * drive;
    drive = m_filter.calculate(drive);
    // m_subsystem.setPower(drive+turn, drive-turn);
    m_subsystem.setSpeed(drive+turn, drive-turn);
    // System.out.println(String.format("Drive %f Turn %f", drive, turn));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
