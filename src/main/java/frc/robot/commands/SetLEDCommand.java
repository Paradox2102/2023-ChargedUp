// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDCommand extends CommandBase {
  LEDSubsystem m_subsystem;
  String m_state;
  /** Creates a new SetLEDCommand. */
  public SetLEDCommand(LEDSubsystem subsystem, String state) {
    m_subsystem = subsystem;
    m_state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (m_state == "idle") {
    //   m_subsystem.setStateIdle();
    // } else if (m_state == "cone") {
    //   m_subsystem.setStateCone();
    // } else if (m_state == "cube") {
    //   m_subsystem.setStateCube();
    // }
    m_subsystem.setRed();
  }

  @Override 
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
