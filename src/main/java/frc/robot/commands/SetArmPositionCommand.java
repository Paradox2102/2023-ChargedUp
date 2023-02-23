// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Must Fix All

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmPositionCommand extends InstantCommand {
  ArmSubsystem m_armSubsystem; 
  private double m_armAngle = 0; 
  // private double m_wristAngle = 0; 

  public SetArmPositionCommand(ArmSubsystem armSubsystem, double armAngle) {
    m_armSubsystem = armSubsystem; 
    m_armAngle = armAngle;
 
    // m_wristAngle = wristAngle; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.moveToAngle(m_armAngle); 
    Logger.log("SetArmPositionCommand", 1, "initialize");
  }
}
