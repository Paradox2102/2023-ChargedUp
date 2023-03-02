// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmCommand extends CommandBase {
  ArmSubsystem m_subsystem;
  DoubleSupplier m_angleInDegrees;
  /** Creates a new ManualArmCommand. */
  public ManualArmCommand(ArmSubsystem armSubsystem, DoubleSupplier angleInDegrees) {
    m_subsystem = armSubsystem;
    m_angleInDegrees = angleInDegrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setArmBrake(false);
    Logger.log("ManualArmCommand", 2, "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_angleInDegrees.getAsDouble();
    // power = power * power * power * .5;
    m_subsystem.setArmPower(angle);
    // m_subsystem.moveToAngle(angle * 120);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ManualArmCommand", 2, "end");
    m_subsystem.enableArm(false);
    m_subsystem.setArmBrake(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
