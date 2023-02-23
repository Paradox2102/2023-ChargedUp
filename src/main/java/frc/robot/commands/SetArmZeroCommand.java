// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.htm
public class SetArmZeroCommand extends InstantCommand {
  ArmSubsystem m_armSubsystem;
  public SetArmZeroCommand(ArmSubsystem armSubsystem) {
    m_armSubsystem = armSubsystem; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SetArmZeroCommand", 1, "initialize"); 
    m_armSubsystem.storeArmZeroReference();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true; 
  }
}
