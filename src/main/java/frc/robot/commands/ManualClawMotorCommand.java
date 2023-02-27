// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualClawMotorCommand extends CommandBase {
  IntakeSubsystem m_intakeSubsystem; 
  private double m_power; 
  /** Creates a new OpenCloseIntakeCommand. */
  public ManualClawMotorCommand(IntakeSubsystem intakeSubsystem, double power) {
    m_intakeSubsystem = intakeSubsystem; 
    m_power = power; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ClawMotorCommand", 1, String.format("initialize: power = %f ", m_power));
    m_intakeSubsystem.setClawPower(m_power); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ClawMotorCommand", 1, "end");
    m_intakeSubsystem.setClawPower(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
