// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  DriveSubsystem m_subsystem;
  DoubleSupplier m_getX;
  DoubleSupplier m_getY;
  BooleanSupplier m_goingForward;

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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_getX.getAsDouble();
    double y = -m_getY.getAsDouble();
    boolean goingForward = m_goingForward.getAsBoolean();

    if (goingForward) {
      m_subsystem.setPower(y+x, y-x);
    } else {
      m_subsystem.setPower(-y+x, -y-x);
    }
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
