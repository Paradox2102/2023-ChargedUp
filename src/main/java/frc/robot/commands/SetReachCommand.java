// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ReachSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetReachCommand extends InstantCommand {
  private ReachSubsystem m_subsystem;
  private double m_extent;
  public SetReachCommand(ReachSubsystem reachSubsystem, double extent) {
    m_subsystem = reachSubsystem;
    m_extent = extent;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setExtentInInches(m_extent);
    m_subsystem.isRunP(true);
  }
}
