// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;

public class SetBrakeCommand extends CommandBase {
  /** Creates a new SetBrakeCommand. */
  ArmSubsystem m_armSubsystem;


  public SetBrakeCommand(ArmSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SetBrakeCommand", 1, "initialize");
    m_armSubsystem.setArmBrake(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SetBrakeCommand", 1, "end");
    m_armSubsystem.setArmBrake(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
