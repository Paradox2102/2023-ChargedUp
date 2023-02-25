// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;

public class ManualWristCommand extends CommandBase {
  ArmSubsystem m_subsystem;
  private double m_power;
  // private DoubleSupplier m_angleInDegrees;
  /** Creates a new ManualWristCommand. */
  public ManualWristCommand(ArmSubsystem armSubsystem, double power, DoubleSupplier angleInDegrees) {
    m_subsystem = armSubsystem;
    m_power = power;
    // m_angleInDegrees = angleInDegrees; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ManualWristCommand", 1, String.format("Initialize: power=%f", m_power));
    m_subsystem.setWristPower(m_power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_subsystem.moveToAngle(90, 60 * m_angleInDegrees.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setWristPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
